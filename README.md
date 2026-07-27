# VDA5050 Library and Support Tools

`vda5050_core` is a modern C++ library designed for implementing the [VDA5050 specification](https://github.com/VDA5050/VDA5050) across AGVs, AMRs and fleet control systems.

It provides native JSON serialization/deserialization, specification validation, an asynchronous execution framework, MQTT transport abstractions and high-level client adapter
and master control APIs.

The library is **framework-independent** and can be embedded directly into standalone native C++ drivers, ROS 2 packages, or Python-based systems.

```mermaid
flowchart LR
    Shared["<b>vda5050_core</b><br/>• MQTT Transport Layer<br/>• C++ Data Models<br/>• JSON Parsing & Validation<br/>• Execution Engine"]

    subgraph MasterTrack["Master Control Application"]
        direction LR
        MasterAPI["<b>vda5050_core::master</b><br/>Master Control API"]
        MasterApp["Fleet Controller"]
        MasterAPI --> MasterApp
    end

    subgraph ClientTrack["AGV/AMR Application"]
        direction LR
        ClientAPI["<b>vda5050_core::client</b><br/>AGV Client Adapter"]
        ClientApp["Robot Software"]
        ClientAPI --> ClientApp
    end

    Shared ---> MasterAPI
    Shared --> ClientAPI

    MasterApp <===>|"<b>MQTT</b> (uagv/v2/...)"| ClientApp
```

> [!NOTE]
> This project is under active development. API stability is guaranteed across minor releases.


## Features

- **Specification Compliant Data Structures:** Native C++17 representations for all VDA5050 message types.
- **Serialization and Validation:** Fast JSON parsing (`nlohmann/json`) with standard compliance validation
- **Asynchronous Execution Framework:** Reactive execution engine for managing non-blocking robot state transitions, node execution and instant actions.
- **High-Level Client Adapter API:** Pre-built abstraction layer wrapping navigation, action execution and automated state reporting.
- **Layout Interchange Format (LIF):** Native support for loading and validating VDMA define Layout Interchange Format.
- **Multi-Ecosystem Support:** Standalone CMake and `ament_cmake` build integration, optional ROS 2 (`vda5050_interfaces`) support and Python bindings via `pybind11`.

## Overview

| Guide                                                              | Description                                               |
| ------------------------------------------------------------------ | --------------------------------------------------------- |
| **[Client Adapter Guide](docs/client-adapter.md)**                 | Step-by-step integration guide for AGV/AMR                |
| **[Master Guide](docs/master.md)**                                 | Step-by-step guide to building a master control           |
| **[Master API Reference](docs/master-api.md)**                     | Master commands, types and callbacks                      |
| **[Types and Serialization Guide](docs/types.md)**                 | Message structures, validation rules and JSON conversion  |
| **[Validation Guide](docs/validation.md)**                         | Validator checks, required inputs and results             |
| **[Open-RMF Migration Guide](docs/rmf-migration.md)**              | Migrating an Open-RMF fleet adapter to a VDA5050 Adapter  |
| **[Architecture and Design](docs/design.md)**                      | Architecture and design rationale                         |

To connect an existing robot SDK, REST API or ROS 2 navigation system, start with the [Client Adapter Guide](docs/client-adapter.md).

To build a master control, or integrate one into an existing application, start with the [Master Guide](docs/master.md).

## Getting Started

### Requirements

- **C++ Compiler:** C++17 or higher
- **Build System:** CMake $\ge 3.8$, `colcon` (optional for ROS 2 workspaces)
- **System Libraries:** `nlohmann-json3-dev`, `libfmt-dev`, `libpaho-mqtt-dev`, `libpaho-mqttpp-dev`
- **Optional:** ROS 2 (Humble/Jazzy) for `vda5050_interfaces`, `pybind11` for Python bindings.

### Build

1. Install the required MQTT dependencies:

```bash
sudo apt update
sudo apt install libpaho-mqtt-dev libpaho-mqttpp-dev
```

2. Create a workspace, clone the repository and build the package:

```bash
mkdir -p ~/vda5050_ws/src
cd ~/vda5050_ws/src

git clone https://github.com/ros-industrial/vda5050_core.git

cd ~/vda5050_ws
colcon build --packages-select vda5050_core
source install/setup.bash
```

#### Build Options

Pass these flags through `colcon build --cmake-args -D<OPTION>=<VALUE>` or directly in CMake.

| Option           | Default | Effect                                                  |
| ---------------- | ------- | ------------------------------------------------------- |
| `ENABLE_ROS2`    | `OFF`   | Enables support for ROS 2 `vda5050_interfaces` messages |
| `BUILD_PYTHON`   | `ON`    | Builds the Python bindings                              |
| `BUILD_EXAMPLES` | `ON`    | Builds the examples                                     |
| `BUILD_TESTING`  | `ON`    | Builds the tests and configured linters                 |

### Quick Examples

#### AGV Client Integration

The following example shows the basic setup for an AGV-side client.

It creates an MQTT transport and a VDA5050 client adapter, then registers a navigation callback.
In a real application, the callback should forward the request to the robot's navigation system.

```cpp
#include <iostream>

#include "vda5050_core/client/adapter/adapter.hpp"
#include "vda5050_core/execution/protocol_adapter.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"

using namespace vda5050_core;

int main()
{
  auto mqtt_client = transport::create_default_client_unique(
    "tcp://localhost:1883",
    "agv_1");

  auto protocol_adapter = execution::ProtocolAdapter::make(
    std::move(mqtt_client),
    "uagv",
    "2.0.0",
    "Manufacturer",
    "S001");

  auto adapter = client::adapter::Adapter::make(protocol_adapter);

  adapter->on_navigate(
    [](auto node_request, auto edge_request, auto execution)
    {
      // Forward the request to the robot navigation system.
      //
      // This demonstration reports completion immediately.
      // A real integration should only report completion after
      // the robot reaches the requested node.
      execution->finished();
    });

  adapter->start();

  // Keep processing orders until Enter is pressed.
  std::cin.get();

  adapter->stop();
  return 0;
}
```

##### Linking with CMake

```cmake
find_package(vda5050_core REQUIRED)

target_link_libraries(agv_application
  PRIVATE
    vda5050_core::client
    vda5050_core::transport
    vda5050_core::logger
)
```

For a complete integration covering navigation, actions, localization, cancellation and state reporting,
see the [Client Adapter Guide](docs/client-adapter.md) and a preconfigured
[example](vda5050_core/examples/client/adapter_example.cpp).

#### Master Control Integration

The following example shows the basic setup for a master.

It creates an MQTT transport and a master, onboards one AGV, and assigns it a two-node order once
the AGV reports itself ready. In a real application, the completion callback assigns the next order.

This assumes an AGV that is already localized and reporting state. See the
[Master Guide](docs/master.md) for bringing an unlocalized vehicle up with an
`initPosition` instant action.

```cpp
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "vda5050_core/logger/logger.hpp"
#include "vda5050_core/master/master.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"

using namespace vda5050_core;

int main()
{
  auto mqtt_client = transport::create_default_client_shared(
    "tcp://localhost:1883",
    "master_1");

  auto master = master::VDA5050Master::make(mqtt_client);

  master->on_order_complete(
    [](const std::string& agv_id, const std::string& order_id)
    {
      VDA5050_INFO("[{}] completed order [{}]", agv_id, order_id);

      // A real integration would assign this AGV's next order here, with a
      // new order id, or return the AGV to its task queue.
    });

  master->connect();
  master->onboard_agv("uagv", "Manufacturer", "S001");

  // Wait until the AGV is online, localized and idle.
  auto agv = master->get_agv("Manufacturer", "S001");
  while (agv->get_operational_state() != master::AGVState::AVAILABLE)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  // A simple order: drive from node N0 to node N1.
  // Nodes take even sequence ids, the edges between them the odd ones.
  types::Order order;
  order.order_id = "order-1";
  order.order_update_id = 0;
  order.nodes = {{"N0", 0, true}, {"N1", 2, true}};
  order.edges = {{"E0", 1, "N0", "N1", true}};

  auto result = master->assign_order("Manufacturer", "S001", order);

  if (result.decision != master::OrderAssignmentDecision::ASSIGNED)
  {
    // A real integration should read result.decision and result.errors to
    // decide whether to retry, hand the task to another AGV, or raise it to
    // an operator.
    VDA5050_WARN(
      "Order [{}] not assigned ({} error(s))", order.order_id,
      result.errors.size());
  }

  // Keep the master running until Enter is pressed.
  std::cin.get();

  master->disconnect();
  return 0;
}
```

##### Linking with CMake

```cmake
find_package(vda5050_core REQUIRED)

target_link_libraries(master_application
  PRIVATE
    vda5050_core::master
    vda5050_core::transport
    vda5050_core::logger
)
```

For a complete integration covering order construction, validation, event handling and multi-AGV
dispatch, see the [Master Guide](docs/master.md) and a preconfigured
[example](vda5050_core/examples/master/master_example.cpp).

## Examples

You can launch a local MQTT broker to test the included examples.

```bash
mosquitto -v -p 1883
```

| Example                                                   | Demonstrates                                     |
| --------------------------------------------------------- | ------------------------------------------------ |
| `vda5050_core/examples/client/adapter_example.cpp`        | AGV client-adapter integration                   |
| `vda5050_core/examples/master/order_publisher.cpp`        | Continuously dispatching a growing VDA5050 order |
| `vda5050_core/examples/master/master_example.cpp`         | Continuously assigning orders with a new id upon completion, via the master API |

## Directory Layout

```bash
.
└── vda5050_core
    ├── docs                 # Guides and architectural documentation
    ├── examples             # Ready-to-run executables
    ├── include
    │   └── vda5050_core
    │       ├── client       # High-level AGV client adapter
    │       ├── errors       # Error definitions
    │       ├── execution    # Reactive execution framework
    │       ├── json_utils   # JSON serialization and traits
    │       ├── layout       # Layout Interchange Format (LIF) support and tools
    │       ├── logger       # Logging utilities
    │       ├── master       # Master control components
    │       ├── transport    # MQTT client interface and default implementation
    │       ├── types        # VDA5050 message structs
    │       └── validation   # VDA5050 specification compliance checks
    ├── python               # pybind11 modules and migration tools
    └── test                 # Unit and integration tests
```

## Testing

Run the unit and integration tests using `colcon`.

```bash
colcon test --event-handlers console_direct+ --packages-select vda5050_core
```

> Note: Some integration tests require an active MQTT broker listening on `localhost:1883`.

## Contributing

Contributions are welcome!

See [CONTRIBUTING.md](CONTRIBUTING.md) for development and contribution guidelines.

Commits must include a `Signed-off-by` line certifying the [Developer Certificate of Origin](https://developercertificate.org/).

## License

Licensed under the Apache License 2.0. See [LICENSE](LICENSE) for details.
