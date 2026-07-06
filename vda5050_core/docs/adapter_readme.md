# VDA5050 Adapter — Quick-Start Guide

The `Adapter` class is the AGV-side integration facade provided by this package.
It handles MQTT connectivity, VDA5050 order parsing, and order-execution sequencing
so that robot integrators only need to implement two callbacks: **navigate** and **cancel**.

---

## Source files

| Role | Path |
|---|---|
| Public API | `include/vda5050_core/adapter/adapter.hpp` |
| Navigation state API | `include/vda5050_core/adapter/navigation_manager.hpp` |
| Implementation | `src/adapter/adapter.cpp`, `src/adapter/navigation_manager.cpp` |
| Runnable example | `examples/adapter_example.cpp` |

The `adapter_example` executable is built from `examples/adapter_example.cpp` and linked against
`vda5050_core::adapter`, `vda5050_core::transport`, and `vda5050_core::logger`
(see `CMakeLists.txt` lines 254–259).

---

## Build and run

```bash
# from the workspace root
cd vda5050
source install/setup.bash
ros2 run vda5050_core adapter_example <broker_url> <client_id>
```

**Arguments**

| Position | Name | Default | Description |
|---|---|---|---|
| 1 | `broker_url` | `tcp://localhost:1883` | MQTT broker URI |
| 2 | `client_id` | `vda5050_adapter_example` | MQTT client ID (arbitrary, must be unique per broker) |

**Example — connect to a local broker as AGV `my_agv`:**

```bash
ros2 run vda5050_core adapter_example tcp://localhost:1883 my_agv
```

The adapter will log the exact order topic it is listening on, e.g.:

```
Adapter started on tcp://localhost:1883 — order topic: uagv/v2/Manufacturer/S001/order
```

---

## Hardcoded identity in the example

`adapter_example.cpp` uses a fixed AGV identity:

```cpp
constexpr const char* k_interface        = "uagv";
constexpr const char* k_protocol_version = "2.0.0";
constexpr const char* k_manufacturer     = "Manufacturer";
constexpr const char* k_serial_number    = "S001";
```

This means the order topic is always `uagv/v2/Manufacturer/S001/order`.
When writing your own adapter, pass your own values to `ProtocolAdapter::make(...)`.

---

## Testing with mosquitto_pub

Start the adapter first, then publish an order from another terminal:

```bash
ORDER='{"headerId":1,"timestamp":"2026-05-22T08:15:30.000Z","version":"2.0.0","manufacturer":"Manufacturer","serialNumber":"S001","orderId":"order_test_correct_topic","orderUpdateId":0,"nodes":[{"nodeId":"node_a","sequenceId":0,"released":true,"nodePosition":{"x":5.0,"y":10.0,"theta":0.0,"mapId":"map1"},"actions":[]}],"edges":[]}'

mosquitto_pub -h localhost -t "uagv/v2/Manufacturer/S001/order" -m "$ORDER"
```

Expected adapter output:

```
Navigate to node [node_a] at [5, 10]
Simulated arrival at node node_a
```

The example simulates a 2-second travel time before calling `node_reached()`, which
unblocks the execution engine and causes a VDA5050 `state` message to be published back
to `uagv/v2/Manufacturer/S001/state`.

---

## Writing your own adapter

### 1. Create an MQTT client and ProtocolAdapter

```cpp
#include <vda5050_core/adapter/adapter.hpp>
#include <vda5050_core/execution/protocol_adapter.hpp>
#include <vda5050_core/transport/mqtt_client_interface.hpp>

auto mqtt_client = vda5050_core::transport::create_default_client_unique(
    "tcp://localhost:1883", "my_robot_client_id");

auto protocol_adapter = vda5050_core::execution::ProtocolAdapter::make(
    std::move(mqtt_client),
    "uagv",        // interface (VDA5050 spec)
    "2.0.0",       // protocol version
    "MyCompany",   // manufacturer
    "AGV-001");    // serial number
```

### 2. Create the Adapter and get the NavigationManager

```cpp
auto adapter = vda5050_core::adapter::Adapter::make(protocol_adapter);
auto nav     = adapter->navigation_manager();
```

### 3. Register callbacks

```cpp
// Called once per node in the order sequence
adapter->on_navigate([&nav](vda5050_core::types::Node node) {
    // Drive your robot to node.node_position
    nav->set_driving(true);

    // When the robot physically arrives:
    nav->node_reached(node.node_id, node.sequence_id);
    nav->set_driving(false);
});

// Called when a cancel/stop is requested
adapter->on_cancel([]() {
    // Stop your robot
});
```

### 4. Start / stop

```cpp
adapter->start();   // connects MQTT, subscribes to orders
// ... main loop ...
adapter->stop();    // publishes OFFLINE state, disconnects
```

### 5. Report optional state updates

```cpp
nav->set_position(x, y, theta);
nav->set_battery(charge_percent, is_charging);
nav->set_operating_mode(vda5050_core::types::OperatingMode::AUTOMATIC);
nav->add_error(error);
nav->clear_errors();
```

---

## API reference

### `Adapter`

| Method | Description |
|---|---|
| `Adapter::make(protocol_adapter)` | Factory — returns `shared_ptr<Adapter>` |
| `on_navigate(callback)` | Register navigate callback — called once per node |
| `on_cancel(callback)` | Register cancel callback |
| `navigation_manager()` | Returns the `NavigationManager` for state reporting |
| `start()` | Connect MQTT, subscribe to orders, start execution loop |
| `stop()` | Stop execution loop, publish OFFLINE, disconnect |

### `NavigationManager`

| Method | Description |
|---|---|
| `node_reached(node_id, sequence_id)` | Report arrival — **must** be called to advance the order |
| `set_driving(bool)` | Update the `driving` field in published state |
| `set_position(x, y, theta, initialized)` | Update AGV position |
| `set_battery(charge, charging)` | Update battery state |
| `set_operating_mode(mode)` | Update operating mode |
| `add_error(error)` | Append an error to the state message |
| `clear_errors()` | Clear all errors from the state message |

---

## MQTT topics (VDA5050 v2)

| Direction | Topic pattern |
|---|---|
| Inbound (orders) | `{interface}/v2/{manufacturer}/{serialNumber}/order` |
| Outbound (state) | `{interface}/v2/{manufacturer}/{serialNumber}/state` |
| Outbound (connection) | `{interface}/v2/{manufacturer}/{serialNumber}/connection` |

With the example defaults: `uagv/v2/Manufacturer/S001/order` and `uagv/v2/Manufacturer/S001/state`.
