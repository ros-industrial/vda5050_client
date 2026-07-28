# Types and Serialization

This document explains how `vda5050_core` converts VDA5050 message
types to and from JSON.

## 1. Overview

The library explicitly separates VDA5050 data structures from their
JSON conversion logic.

- `vda5050_core::types` contains C++ structs representing VDA5050 messages.
- `vda5050_core::json_utils` provides functions that convert these structs
to and from JSON.

```cpp
#include "vda5050_core/types/order.hpp"               // C++ struct only
#include "vda5050_core/json_utils/serialization.hpp"  // JSON Converters
```

This separation ensures core types remain lightweight and can be used without
requiring JSON support. The JSON conversion can be enabled after including the
serialization header.

`ProtocolAdapter` utilizes `json_utils` internally.
High-level methods like `publish<T>()` and `subscribe<T>()` convert structs
to and from JSON payloads automatically.

## 2. Naming Convention

The VDA5050 specification uses `lowerCamelCase` for JSON field names
whereas the C++ types use standard `snake_case`. Serializers convert between
the two naming styles automatically using ADL.

Some example fields,

| C++ Struct             | JSON           |
| ---------------------- | -------------- |
| `header.serial_number` | `serialNumber` |
| `order.order_id`       | `orderId`      |
| `action.blocking_type` | `blockingType` |
| `state.agv_position`   | `agvPosition`  |

## 3. Create and Serialize an Order

The following example constructs an `Order` with two nodes and one
connecting edge, serializes it to JSON and verifies round-trip conversion.

#### Example

```cpp
#include <chrono>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>

#include "vda5050_core/json_utils/serialization.hpp"
#include "vda5050_core/types/order.hpp"

int main()
{
  using vda5050_core::types::Edge;
  using vda5050_core::types::Node;
  using vda5050_core::types::NodePosition;
  using vda5050_core::types::Order;

  Order order{};
  order.header.header_id = 1;
  order.header.timestamp = std::chrono::system_clock::now();
  order.header.version = "2.0.0";
  order.header.manufacturer = "ROS-I";
  order.header.serial_number = "S001";
  order.order_id = "order-001";
  order.order_update_id = 0;

  Node start{};
  start.node_id = "start";
  start.sequence_id = 0;
  start.released = true;
  start.node_position = NodePosition{};
  start.node_position->x = 0.0;
  start.node_position->y = 0.0;
  start.node_position->theta = 0.0;
  start.node_position->map_id = "map1";

  Node goal{};
  goal.node_id = "goal";
  goal.sequence_id = 2;
  goal.released = true;
  goal.node_position = NodePosition{};
  goal.node_position->x = 5.0;
  goal.node_position->y = 2.0;
  goal.node_position->map_id = "map1";

  Edge route{};
  route.edge_id = "start-to-goal";
  route.sequence_id = 1;
  route.start_node_id = start.node_id;
  route.end_node_id = goal.node_id;
  route.released = true;

  order.nodes = {start, goal};
  order.edges = {route};

  // Serialize to a JSON value and then to a compact string.
  //
  // Option A: Single line dump using ADL conversion
  const std::string compact_payload = nlohmann::json(order).dump();
  //
  // Option B: Explicit json assignment for logging
  const nlohmann::json json_order = order;
  const std::string log_payload  = json_order.dump(2);

  std::cout << "Serialized JSON:\n" << log_payload << std::endl;

  // Deserialize the payload and verify the round trip.
  const Order decoded = nlohmann::json::parse(log_payload);
  return decoded == order ? 0 : 1;
}
```

## 4. Deserialize an Incoming Payload

Thanks to C++ ADL, you can deserialize JSON text straight into
C++ types in a single step without verbose multi-line parsing.

#### Syntax options

```cpp
// Option 1: Direct assignment using ADL (Recommended)
vda5050_core::types::Order order = nlohmann::json::parse(payload);

// Option 2: Explicit template get<T>()
auto order = nlohmann::json::parse(payload).get<vda5050_core::types::Order>();

// Option 3: Two-step parsing
nlohmann::json data = nlohmann::json::parse(payload);
auto order = data.get<vda5050_core::types::Order>();
```

#### Example

```cpp
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>

#include "vda5050_core/json_utils/serialization.hpp"
#include "vda5050_core/types/order.hpp"

void handle_order_payload(const std::string& payload)
{
  try
  {
    // Single-line direct parsing via ADL
    const vda5050_core::types::Order order = nlohmann::json::parse(payload);

    std::cout << "Successfully parsed order ID: " << order.order_id << std::endl;
  }
  catch (const nlohmann::json::parse_error& error)
  {
    std::cerr << "Malformed JSON payload: " << error.what() << std::endl;
  }
  catch (const nlohmann::json::exception& error)
  {
    std::cerr << "VDA 5050 Schema mismatch: " << error.what() << std::endl;
  }
}
```

#### Common exceptions

Deserialization throws an nlohmann::json::exception when:

- A mandatory VDA 5050 field is absent in the JSON payload.
- A value type mismatch occurs (e.g., passing a string where a double is expected).
- A timestamp fails ISO 8601 formatting checks.
- An enum string fails to map to a recognized enum value.

## 5. Required and Optional Fields

The serializer distinguishes between required values, optional values
and empty arrays.

#### Required Fields

They are read using json::at(). If the key is missing from the JSON input,
an exception is thrown. Examples of some required fields include `header`,
`orderId`, `orderUpdateId`, `nodes`, `edges`.

#### Optional Fields (`std::optional`)

They are stored using `std::optional<T>`.
- If a value is present value then they are serialized to JSON.
- In the absence of a value the field is omitted entirely from output JSON.

```cpp
vda5050_core::types::Order order{};
order.zone_set_id = "warehouse-zone";  // Serializes as "zoneSetId": "warehouse-zone" in JSON
order.zone_set_id.reset();             // "zoneSetId" is completed omitted
```

#### Required Arrays

An empty required array is different from an absent optional field.
Empty C++ vectors that represent mandatory arrays are always emitted
as empty JSON arrays, not omitted or written as null

For example:

```
{
  "nodes": [],
  "edges": []
}
```

## 6. JSON Conversion and Message Validation
Deserialization success verifies structural compliance
(types and mandatory fields match), but does not verify semantic
correctness according to VDA5050 rules.

For example, an order may deserialize successfully but still contain,

- empty identifiers
- invalid node or edge sequences
- incorrect graph relationships
- unsupported actions
- conflicting action definitions

To prevent logic failures, always pass parsed structs
into the content validation methods.

```cpp
#include "vda5050_core/json_utils/serialization.hpp"
#include "vda5050_core/types/order.hpp"
#include "vda5050_core/validation/content_validator.hpp"

void process_incoming_message(const std::string& raw_payload)
{
  // Step 1: Parse structure
  const auto order = nlohmann::json::parse(raw_payload).get<vda5050_core::types::Order>();

  // Step 2: Validate VDA5050 constraints
  const auto result = vda5050_core::validation::validate_order_content(order);

  if (!result)
  {
    for (const auto& error : result.fatal_errors())
    {
      std::cerr << "Validation Error: " << error.message << std::endl;
    }
    return;
  }

  // Order is safe for state machine processing
}
```

Additional validators are available in `vda5050_core/validation/` and can be
used to check graph structure, action conflicts, protocol limits, traversability
or master-side pre-send conditions.

## 7. CMake Integration

```cmake
find_package(vda5050_core REQUIRED)

# Option A: Full JSON support
target_link_libraries(custom_adapter
  PRIVATE
    vda5050_core::types
    vda5050_core::json_utils
)

# Option B: Plain C++ structs only
target_link_libraries(example_internal_logic
  PRIVATE
    vda5050_core::types
)
```
