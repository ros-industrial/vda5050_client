# Client Adapter

This guide explains how to connect an existing robot stack to a VDA5050 master control
using the C++ client adapter provided by `vda5050_core::client::adapter`.
The client adapter acts as a high level bridge by managing VDA5050 communication over MQTT,
validation of incoming messages, order processing and state update.

It **does not** directly control the hardware - you provide the callbacks that forward
commands to your robot's lower-level drivers (ROS 2, vendor SDK, REST, etc.)

You can find a [complete integration checklist](#complete-integration-checklist) at the end.

## Table of Contents

1. [Start from the Existing Example](#1-start-from-the-existing-example)
2. [Build and Run the Packaged Example](#2-build-and-run-the-packaged-example)
3. [Create Your Own Robot Integration](#3-create-your-own-robot-integration)
    - 3.1. [Change MQTT Configuration and Robot Identity](#31-change-mqtt-configuration-and-robot-identity)
    - 3.2. [Replace Simulated Navigation and Report Navigation Completion](#32-replace-simulated-navigation-and-report-navigation-completion)
    - 3.3. [Replace Simulated Actions](#33-replace-simulated-actions)
    - 3.4. [Connect Localization and Map Calibration](#34-connect-localization-and-map-calibration)
    - 3.5. [Replace Simulated State with Real Robot State](#35-replace-simulated-state-with-real-robot-state)
    - 3.6. [Configure the Factsheet](#36-configure-the-factsheet)
    - 3.7. [Adapter Lifecycle and Entrypoint](#37-adapter-lifecycle-and-entrypoint)
    - 3.8. [Linking with CMake](#38-linking-with-cmake)
4. [Build and Test Your Robot Integration](#4-build-and-test-your-robot-integration)

## 1. Start from the Existing Example

Use the following file as the starting template: `examples/client/adapter_example.cpp`

The example already handles:
- MQTT communication
- VDA5050 order processing
- adapter startup and shutdown
- navigation callbacks
- action callbacks
- localization callbacks
- robot state reporting

To integrate a real robot, copy the example into your robot integration package and replace the simulated behavior with the robot's control API.

> Calls such as `robot_driver.navigate_to()` in this guide are placeholders. They are not part of `vda5050_core`.

## 2. Build and Run the Packaged Example

Build the package with examples enabled:

```
colcon build \
  --packages-select vda5050_core \
  --cmake-args -DBUILD_EXAMPLES=ON
```

Start an MQTT broker:

```
mosquitto -d
```

Source the workspace:

```
source install/setup.bash
```

Run the packaged example:

```
ros2 run vda5050_core adapter_example
```

Run this example first to confirm that the MQTT connection and client-adapter flow work before connecting a physical robot.

## 3. Create Your Own Robot Integration

After making sure the packaged example works, copy `adapter_example.cpp` into the directory of your choice and start working!

### 3.1 Change MQTT Configuration and Robot Identity

Set broker endpoints and align manufacturer strings with master control:

```cpp
auto mqtt_client =
  vda5050_core::transport::create_default_client_unique(
    "tcp://192.168.1.10:1883",
    "robot_1_vda5050_adapter");

auto protocol_adapter = ProtocolAdapter::make(
  std::move(mqtt_client),
  "uagv",          // Interface domain
  "2.0.0",         // VDA5050 specification version
  "MyCompany",     // Manufacturer string
  "AGV-001"        // Serial number
);

auto adapter = client::adapter::Adapter::make(protocol_adapter);
```

### 3.2 Replace Simulated Navigation and Report Navigation Completion

Forward navigation requests to local controllers. If a transformation for
`map_id` exists, the incoming position is automatically transformed
to local AGV coordinates:

```cpp
std::shared_ptr<OrderExecution> active_navigation;

adapter->on_navigate(
  [&](NodeRequest node_request,
      std::optional<EdgeRequest> edge_request,
      std::shared_ptr<OrderExecution> execution)
  {
    const auto position = node_request.node_position();

    if (!position.has_value())
    {
      execution->failed("Requested node does not contain a position");
      return;
    }

    active_navigation = std::move(execution);
    state_manager->set_driving(true);

    // Coordinates are automatically in local AGV frame if a transform exists!
    robot_driver.navigate_to(
      position->x,
      position->y,
      position->theta.value_or(0.0),
      position->map_id);
  });

// In main loop or event listener
if (active_navigation && robot_driver.navigation_completed())
{
  state_manager->set_driving(false);
  active_navigation->finished();
  active_navigation.reset();
}

if (active_navigation && robot_driver.navigation_failed())
{
  state_manager->set_driving(false);
  active_navigation->failed(robot_driver.navigation_failure_reason());
  active_navigation.reset();
}
```

Every navigation request must end with either:

```cpp
execution->finished();
```

or:

```cpp
execution->failed("Failure reason");
```

Do not call `execution->finished()` immediately after sending the
navigation command instead store the execution handle until the robot
reaches the destination or reports a failure.

The optional `EdgeRequest` may contain additional movement constraints,
such as speed or trajectory information. Use it only when required by the robot.

The adapter example copies the requested node position into
`StateManager` because movement is simulated.
A real integration should report position using the robot's localization
or odometry data.

### 3.3 Replace Simulated Actions

Map VDA5050 action types to hardware commands:

```cpp
std::shared_ptr<ActionExecution> active_action;

adapter->on_action(
  [&](ActionRequest request,
      std::shared_ptr<ActionExecution> execution)
  {
    if (request.action_type() == "customActionA")
    {
      execution->running();
      active_action = std::move(execution);

      // Replace with the robot action interface.
      robot_driver.start_custom_action_a();
      return;
    }

    execution->failed("Unsupported action: " + request.action_type());
  });

// For long-running actions, keep the execution handle
// and report the result later
if (active_action && robot_driver.action_completed())
{
  active_action->finished();
  active_action.reset();
}

if (active_action && robot_driver.action_failed())
{
  active_action->failed(robot_driver.action_failure_reason());

  active_action.reset();
}
```

### 3.4 Connect Localization and Map Calibration

Handle `on_localize` requests from master control to calibrate frame offsets.
There are two pathways to initialize position in `StateManager`:

#### Method A: Direct Initialization using `initialize_position`

Use when the local frame matches the VDA5050 world frame and no
transformation is required:

```cpp
// Sets position in StateManager and flags position_initialized = true
adapter->on_localize(
  [state_manager](
    LocalizationRequest request,
    std::shared_ptr<ActionExecution> execution)
  {
    const bool accepted = robot_driver.set_initial_pose(
      request.x(), request.y(), request.theta(), request.map_id());

    if (accepted)
    {
      const auto agv_pose = robot_driver.get_local_pose();
      state_manager->initialize_position(
        agv_pose.x(), agv_pose.y(), agv_pose.theta(), agv_pose.map_id());

      execution->finished("Localization calibrated and accepted");
    }
    else
    {
      execution->failed("Robot rejected localization request");
    }

    execution->finished();
  });
```

#### Method B: Adding a Transformation into `StateManager`

Use when local odometry/map coordinates differ from master world coordinates:

```cpp
adapter->on_localize(
  [state_manager](LocalizationRequest request,
      std::shared_ptr<ActionExecution> execution)
  {
    execution->running();

    // Calibrate transformation matrix
    Pose2D agv_pose = robot_driver.get_local_pose();
    Pose2D world_pose{request.x(), request.y(), request.theta()};
    std::string map_id = request.map_id();

    // Calibrate and store transformation in StateManager
    auto tf = Transformation::calibrate(world_pose, agv_pose);
    state_manager->set_transformation(tf, map_id);

    // Calling set_position converts local pose to world pose
    // AND automatically marks position_initialized = true
    state_manager->set_position(
      agv_pose.x(), agv_pose.y(), agv_pose.theta(), map_id);
  });
```

### 3.5 Replace Simulated State with Real Robot State

Store the state manager and continuously feed
local position and status metrics into it.

```cpp
auto state_manager = adapter->state_manager();

void update_robot_state()
{
  // Position: Pass local AGV coordinates (x_agv, y_agv, theta_agv).
  // If a transform exists for map_id, StateManager converts local pose
  // back to world pose for MQTT state messages automatically.
  const auto pose = robot_driver.current_local_pose();
  state_manager->set_position(pose.x, pose.y, pose.theta, pose.map_id);

  state_manager->set_driving(robot_driver.is_moving());
  state_manager->set_operating_mode(vda5050_core::types::OperatingMode::AUTOMATIC);

  vda5050_core::types::BatteryState battery{};
  battery.battery_charge = robot_driver.battery_percentage();
  battery.charging = robot_driver.is_charging();
  state_manager->set_battery_state(battery);
}
```

The integration may also update:

- velocity through `set_velocity(...)`
- paused state through `set_paused(...)`
- safety state through `set_safety_state(...)`
- distance since the last node through `set_distance_since_last_node(...)`
- errors through `add_error(...)`, `set_errors(...)`, `clear_errors(...)`
- loads through `add_load(...)`, `set_loads(...)`, `clear_loads(...)`, `remove_loads(...)`
- information through `add_information(...)`, `set_information(...)`, `remove_information(...)`


Order-related fields such as the current order, node states, edge states, and last reached node are managed by the adapter.

### 3.6  Configure the Factsheet

Create a factsheet describing the real robot:

```cpp
vda5050_core::types::Factsheet factsheet{};

// Populate the supported robot capabilities.
adapter->set_factsheet(factsheet);
```

The factsheet may include:

- supported actions
- robot dimensions
- load capabilities
- velocity limits
- acceleration limits
- supported protocol features

If a factsheet is not supplied during adapter initialization, a default
one will be created for you.

### 3.7 Adapter Lifecycle and Entrypoint

The startup and shutdown section of the example can remain mostly unchanged.
Make sure to register the callbacks first.

```cpp
// Adapter initialization
adapter->on_navigate(...);
adapter->on_action(...);
adapter->on_localize(...);

// Start the adapter
adapter->start();

// Lifecycle Loop
while (running) {
  update_robot_state();
  update_navigation_status();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

// Stop the adapter
adapter->stop();
```

### 3.8 Linking with CMake

```
find_package(vda5050_core REQUIRED)

target_link_libraries(robot_vda5050_adapter
  PRIVATE
    vda5050_core::client
    vda5050_core::transport
    vda5050_core::logger
)
```

When using with `colcon`, add the package dependency to `package.xml`:

```xml
<depend>vda5050_core</depend>
```

## 4. Build and Test Your Robot Integration

Build and run your integration package:

```bash
colcon build --packages-select robot_vda5050_adapter
```

## Complete Integration Checklist
Use this checklist to track your progress when replacing the template implementation with real robot integration code:

- [ ] 1. MQTT & Identity Configuration

    - Update MQTT broker URL and port (`tcp://<broker-ip>:1883`).

    - Set unique MQTT client ID.

    - Set `manufacturer` and `serialNumber` strings to match master control expectations.

- [ ] 2. Navigation Dispatching and Completion

    - Hook `on_navigate` to robot drivers navigation function.

    - Verify local target coordinates from `node_request.node_position()`.

    - Report completion through `execution->finished()` or failure through `execution->failed(...)`.

- [ ] 3. Action Execution and Status Update

    - Map supported VDA 5050 action types in `on_action`.

    - Transition execution states properly (`running()`, `finished()`, `failed()`, etc.).

    - Reject unsupported action types explicitly.

- [ ] 4. Localization and Coordinate Transforms

    - Register `on_localize` handler to calibrate world-to-AGV frame transformation using `Transformation::calibrate()`.

    - Store transform in `StateManager` using `set_transformation()`.

    - Pass calibrated pose to local robot controller if required.

- [ ] 5. State Reporting

    - Stream local pose into `state_manager->set_position(...)` (automatically converted to world coordinates if transforms available).

    - Continuously update driving status (`set_driving`), operating mode (`set_operating_mode`), and battery metrics (`set_battery_state`).

- [ ] 6. CMake Setup

    - Update `CMakeLists.txt` executable targets and link against `vda5050_core::client`, `vda5050_core::transport` and `vda5050_core::logger`.
