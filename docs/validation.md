# Validation

This is the reference for `vda5050_core::validation`: what each validator
checks, what data it needs, and how to read the result.

## 1. Overview

Every validator is a free function in `vda5050_core::validation` returning
`vda5050_core::errors::ValidationResult`. They hold no state, so concurrent
calls are safe as long as the caller is not mutating the message or context
being validated.

Findings come at two levels:

- **Fatal**: makes the result evaluate to `false`. The caller would normally
  reject the message.
- **Warning**: advisory. It does not affect that, and what to do about it is
  up to the caller.

```cpp
#include "vda5050_core/validation/content_validator.hpp"

const auto result = vda5050_core::validation::validate_order_content(order);

if (!result)
{
  // at least one fatal finding
}

if (result.has_warnings())
{
  // advisory findings; inspect the entries for details
}
```

## 2. ValidationResult

| Member | Returns |
| --- | --- |
| `add_error(error)` | adds an entry according to its error level |
| `fatal_errors()` | entries classified as fatal |
| `warnings()` | advisory entries |
| `has_fatal()` | `true` if any fatal entry |
| `has_warnings()` | `true` if any warning entry |
| `operator bool` | `true` when there are **no fatal** entries |

Each entry is a `types::Error` carrying an error type and level; the description
and field references are optional. There is no merge operation, so a caller
running several validators decides how to combine the results.

> **`operator bool` ignores warnings.** `if (result)` is `true` for a result
> holding warnings and no fatal errors. `has_warnings()` only tells you some
> warning exists. Read `warnings()` and check the entry's type to tell a
> skipped check from a completed one.

## 3. The validators

One header per concern, in `include/vda5050_core/validation/`.

### `content_validator.hpp`

Required-content checks per message type: supported header version, non-empty
manufacturer and serial number, non-empty identifiers, and every action
carrying an `action_id` and `action_type`.

| Function | Validates |
| --- | --- |
| `validate_order_content` | Order |
| `validate_instant_actions_content` | InstantActions |
| `validate_state_content` | State |
| `validate_connection_content` | Connection |
| `validate_factsheet_content` | Factsheet |
| `validate_visualization_content` | Visualization |

All findings are fatal. These need only the message, with no cached AGV state,
factsheet, or loaded layout.

### `pre_send_validator.hpp`

`validate_pre_send` is the AGV-readiness gate. It takes a `PreSendContext`
snapshot (connection status, last State, last Factsheet, operational state,
loaded graph) and fails, fatally, when the AGV:

- is not `ONLINE`
- is not in operational state `AVAILABLE`
- has not yet reported any State
- reports a mode other than `AUTOMATIC` or `SEMIAUTOMATIC`
- is paused
- has its e-stop engaged
- reports no position, or `positionInitialized: false`

The traversability, capability, and protocol-limit checks read the same
`PreSendContext`, so it is built once and passed to each.

### `order_graph_validator.hpp`

`is_valid_graph` checks that an order's nodes and edges form a valid graph:
sequence numbering, node/edge alternation, released-versus-horizon consistency.

`is_valid_update(base_order, next_order)` checks a sparse update against the
order it extends: same `orderId`, a newer `orderUpdateId`, and a first node
that matches the base's decision point.

Both produce only fatal findings, so `operator bool` is enough to gate on.

The two suit different callers. `is_valid_update` needs both orders in full, so
it fits a receiver holding the order it is about to extend. A sender that
combines the two itself can skip it and run `is_valid_graph` on the combined
order, which validates what actually goes on the wire.

### `traversability_validator.hpp`

`validate_traversability` checks that this AGV can actually drive the route:
first-node reachability, and, when a layout is loaded, node and edge
existence, map id, and edge direction.

Fatal. With no State reported it fails hard rather than skipping. With no layout
loaded it runs reachability only and warns that the layout checks were skipped.

### `capability_validator.hpp`

`validate_capability` checks the message's actions against the AGV factsheet:
action type supported, required scope, blocking type, declared and required
parameters. Overloads exist for Order and InstantActions; only the
InstantActions overload exempts the predefined action types.

Fatal, but it needs a factsheet. See
[checks that may be skipped](#4-checks-that-may-be-skipped).

### `protocol_limits_validator.hpp`

`validate_protocol_limits` checks array sizes against the factsheet's declared
limits: `order_nodes` and `order_edges` for the route, `node_actions` and
`edge_actions` per element, `actions_actions_parameters` per action, and
`instant_actions` per message. Overloads exist for Order and InstantActions.

Fatal, but it needs a factsheet. See
[checks that may be skipped](#4-checks-that-may-be-skipped). The limits are
per-order, so when an order is built by stitching an update onto a base, the
subject is the combined order rather than the update alone.

### `instant_action_mode_validator.hpp`

`validate_instant_action_mode` gates instant actions on the AGV's operating
mode. In `AUTOMATIC` or `SEMIAUTOMATIC` everything passes. In any other mode,
or when no State has been received, only exempt action types get through.
`is_mode_exempt_action_type` reports which.

All findings are fatal.

### `action_conflict_validator.hpp`

`validate_action_conflict` checks an instant action against the AGV's running
actions and driving state:

| Blocking type | Rejected when |
| --- | --- |
| `NONE` | never |
| `SOFT` | driving |
| `HARD` | driving, or any action is active |

Two exceptions: `initPosition` is rejected while driving whatever its blocking
type, and `cancelOrder` / `startPause` / `stopPause` bypass the check entirely.

Fatal. With no cached State the whole check passes.

### `factsheet_alignment.hpp`

`check_factsheet_alignment` compares the loaded layout's edge speeds against the
AGV factsheet's speed envelope. **Warning** level throughout. It is a
diagnostic to run once a factsheet is available, not a gate on any message.

### `operating_mode_control.hpp` and `predefined_action_types.hpp`

Helpers that answer a question rather than validate:

| Helper | Returns |
| --- | --- |
| `is_master_in_control(mode)` | mode is `AUTOMATIC` or `SEMIAUTOMATIC` |
| `is_mode_exempt_action_type(type)` | action passes the operating-mode gate |
| `is_capability_exempt_action_type(type)` | action needs no factsheet entry (`charge` is not exempt) |
| `is_motion_exempt_action_type(type)` | action may be issued mid-motion |
| `is_position_init_action_type(type)` | action rewrites the pose |

## 4. Checks that may be skipped

Three checks need data that may not have arrived:

| Check | Needs | Warns with |
| --- | --- | --- |
| `validate_capability` | the AGV factsheet | `CapabilityCheckSkipped` |
| `validate_protocol_limits` | the AGV factsheet | `ProtocolLimitCheckSkipped` |
| `validate_traversability` (layout part) | a loaded layout | `GraphIntegrityCheckSkipped` |

When the data is missing the check does not run and a warning records that.
Since warnings do not set `operator bool`, a result with no fatal errors is not
proof the check ran. Request a factsheet on connect if you rely on either
factsheet-gated check.

## 5. Error types

Each entry in a `ValidationResult` carries an error type naming the check that
produced it.

| Error type | Raised by |
| --- | --- |
| `ContentValidationError` | the content validators |
| `PreSendValidationError` | `validate_pre_send` |
| `GraphValidationError` | `is_valid_graph` |
| `OrderUpdateError` | `is_valid_update`, order stitching |
| `TraversabilityValidationError` | `validate_traversability` |
| `CapabilityValidationError` | `validate_capability` |
| `ProtocolLimitError` | `validate_protocol_limits` |
| `ModeValidationError` | `validate_instant_action_mode` |
| `ActionBlockedByDrivingError`, `HardActionBlockedError` | `validate_action_conflict` |
| `ValidationError` | callers, for conditions outside any single validator |

Most entries also carry field references identifying what failed, such as
`RefOrderId`, `RefNodeId`, `RefSequenceId`, or `RefActionId`.
