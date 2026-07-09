# vda5050_core::master

The master API is the fleet-control side of the library: a fleet-management
system subclasses `VDA5050Master`, overrides the `on_*` hooks to react to AGV
messages, and drives the fleet through the call surface (onboard, assign
orders, assign instant actions). It is the counterpart to the AGV-side
execution API in [`usage.md`](usage.md) — the fleet side, not used together
with it.

> Status: evolving. Order lifecycle, stitching, and instant actions are
> implemented. Cancel/pause/resume actions and some validators are still in
> progress — the surface below is stable but will grow.

## 1. A minimal master

`VDA5050Master` is an abstract base. Derive from it, override the hooks you
care about, and construct it with `make_shared` (it relies on
`enable_shared_from_this`).

```cpp
#include "vda5050_core/master/master.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"

class MyMaster : public vda5050_core::master::VDA5050Master
{
public:
  using VDA5050Master::VDA5050Master;  // inherit the constructor

  void on_state(
    const std::string& agv_id,
    const vda5050_core::types::State& state) override
  {
    VDA5050_INFO_STREAM(agv_id << " last node: " << state.last_node_id);
  }

  void on_node_reached(
    const std::string& agv_id, const std::string& node_id) override
  {
    VDA5050_INFO_STREAM(agv_id << " reached " << node_id);
  }
};

int main()
{
  auto mqtt = vda5050_core::transport::create_default_client_shared(
    "tcp://localhost:1883", "master");
  auto master = std::make_shared<MyMaster>(mqtt);

  master->connect();
  master->onboard_agv("ROS-I", "S001");  // manufacturer, serial_number

  // connect() returns once the broker handshake completes; it does NOT run a
  // loop. Callbacks arrive on the transport thread, so main must block itself:
  std::promise<void>().get_future().wait();  // or your own signal-driven wait

  master->disconnect();
}
```

Onboarding subscribes to that AGV's `state`, `connection`, `factsheet`, and
`visualization` topics; every inbound message updates the per-AGV cache and
then invokes the matching `on_*` override.

## 2. Threading and lifecycle

- **One inbound thread for the whole fleet.** All `on_*` hooks fire on the
  single transport (MQTT) callback thread, serialized across every AGV. A slow
  override for one AGV therefore delays message processing for all of them —
  keep hooks prompt; offload heavy work to your own thread.
- **Calling back into the master from a hook is safe.** No master lock is held
  while a hook runs, so the common pattern — `on_node_reached` →
  `assign_order` for the next task, or `get_agv(...)` to read state — works
  without deadlock.
- **Lifetime.** The master must stay alive (hold the `shared_ptr`) until after
  `disconnect()` returns, because callbacks run on the transport thread. Don't
  let the owning `shared_ptr` drop while connected.

## 3. Commanding AGVs

### 3.1 Orders

An `Order` is a graph of nodes and edges. The caller sets the header identity,
an `order_id`, and a base of released nodes/edges with strictly increasing
sequence ids (nodes even, edges odd). Orders are validated against the loaded
layout (see §5) — load it before assigning.

```cpp
vda5050_core::types::Order order;
order.header.version = "2.0.0";
order.header.manufacturer = "ROS-I";
order.header.serial_number = "S001";
order.order_id = "order-1";
order.order_update_id = 0;

vda5050_core::types::Node n0;
n0.node_id = "N0"; n0.sequence_id = 0; n0.released = true;
vda5050_core::types::Node n1;
n1.node_id = "N1"; n1.sequence_id = 2; n1.released = true;
vda5050_core::types::Edge e0;
e0.edge_id = "E0"; e0.sequence_id = 1;
e0.start_node_id = "N0"; e0.end_node_id = "N1"; e0.released = true;
order.nodes = {n0, n1};
order.edges = {e0};
```

`assign_order` runs the pre-flight chain (schema, readiness, graph,
traversability, capability, stitch decision) and, on success, queues the order
for publish. It returns an `AssignmentResult` you inspect synchronously.

```cpp
auto res = master->assign_order("ROS-I", "S001", order);
if (res)  // true iff ASSIGNED
{
  VDA5050_INFO_STREAM("order queued");
}
else if (res.decision ==
         vda5050_core::master::AssignmentDecision::STITCH_QUEUED)
{
  // Update accepted but held until the AGV reaches the stitch point. It goes
  // on the wire automatically on a later State; poll agv->pending_update_count()
  // (0 = drained) to observe it.
}
else
{
  for (const auto& e : res.errors)  // rejection diagnostics
  {
    VDA5050_WARN_STREAM(
      "rejected: " << e.error_description.value_or(e.error_type));
  }
}
```

`AssignmentDecision` covers the outcomes: `ASSIGNED`, `AGV_NOT_ONBOARDED`,
`AGV_OFFLINE`, `AGV_NOT_READY`, `AGV_MODE_NOT_AUTO`,
`AGV_POSITION_NOT_INITIALIZED`, `AGV_NO_STATE_YET`, `STITCH_REJECTED`,
`STITCH_QUEUED`, `DUPLICATE_IGNORED`. `operator bool` is `true` only for
`ASSIGNED` (a `STITCH_QUEUED` order has not gone on the wire yet).

### 3.2 Instant actions

Instant actions skip the operational-readiness gate that orders apply, so a
`stateRequest` still dispatches when the AGV is degraded. They are **not**
fully ungated, though: an offline AGV is rejected (`AGV_OFFLINE`), and a
non-instant-allowed action type outside automatic mode is rejected
(`AGV_MODE_NOT_AUTO_FOR_ACTION`). Build them with `ActionFactory`, then assign.

```cpp
#include "vda5050_core/master/actions/action_factory.hpp"

using vda5050_core::master::ActionFactory;

vda5050_core::types::InstantActions batch;
batch.actions.push_back(
  ActionFactory::build_state_request(ActionFactory::generate_action_id()));

auto res = master->assign_instant_actions("ROS-I", "S001", batch);
// res.decision on failure: AGV_OFFLINE, DUPLICATE_ACTION_ID, AGV_QUEUE_FULL,
// HARD_ACTION_BLOCKED, ACTION_BLOCKED_BY_DRIVING, AGV_MODE_NOT_AUTO_FOR_ACTION,
// AGV_CANNOT_PERFORM_ACTION, INVALID_CONTENT.
```

`ActionFactory::build_custom` builds an arbitrary action; `build_state_request`
and `build_factsheet_request` are canned helpers. Action ids must be unique —
`generate_action_id()` returns a UUIDv4.

## 4. Reacting to AGV events (override surface)

Beyond the raw message hooks, the master detects edges in the State/Connection
stream and dispatches named hooks. All default to empty; override the ones you
need. All fire on the single transport thread (see §2).

| Hook | Fires when |
|---|---|
| `on_state` / `on_connection` / `on_factsheet` / `on_visualization` | a raw message arrives (after caching) |
| `on_node_reached` | the AGV reports a previously-unreached released node |
| `on_errors_appeared` / `on_errors_resolved` | the State's error list gains / loses entries |
| `on_new_base_requested` | `new_base_request` rises false→true |
| `on_mode_changed` | `operating_mode` changes (leaving master control drains the queue to a resumable buffer) |
| `on_paused` / `on_driving` / `on_loads_changed` | the corresponding State field changes |
| `on_connect` / `on_offline` / `on_connection_broken` | connection transitions to ONLINE / OFFLINE / CONNECTIONBROKEN |
| `on_state_timeout` / `on_state_resumed` | the State heartbeat lapses (30s) / recovers |
| `on_broker_disconnected` / `on_broker_reconnected` | the master's own broker link drops / re-establishes |

## 5. Topology

Load a LIF layout so traversability and graph validation run against a known
map. Orders are validated against it at publish time.

```cpp
auto result = master->load_layout_from_config("map.json");
if (!result) { /* result.errors describes the parse failure */ }
```

`set_graph(...)` installs an already-built graph (tests / custom loaders);
`get_loaded_graph()` returns the current one.

## 6. Onboarding AGVs

Onboard/offboard one AGV at a time with `onboard_agv` / `offboard_agv`, or a
whole set at once with `onboard_agv_batch` / `offboard_agv_batch`. The batch
calls are idempotent — re-onboarding a present AGV is a no-op — so you can
replay the full membership list on every change without tracking deltas
yourself.

```cpp
std::vector<vda5050_core::master::VDA5050Master::OnboardSpec> fleet = {
  {"ROS-I", "S001"}, {"ROS-I", "S002"}};
auto result = master->onboard_agv_batch(fleet);
// result.onboarded / result.skipped_already_onboarded / result.failed
```

## 7. Reading AGV state

`get_agv` returns a handle to a managed `AGV` (or `nullptr` if not onboarded)
for direct, thread-safe queries.

```cpp
if (auto agv = master->get_agv("ROS-I", "S001"))
{
  auto snapshot = agv->get_status_snapshot();  // coherent State/Conn/Factsheet
  auto pose = agv->get_pose_view();            // fused State/Visualization pose
  bool busy = agv->has_active_order();
}
```

Broker health is available fleet-wide via `get_broker_status()`.
