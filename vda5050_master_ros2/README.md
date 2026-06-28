# vda5050_master_ros2

ROS 2 binding for the VDA5050 fleet master. Wraps
`vda5050_core::master::VDA5050Master` and exposes the master's per-AGV data
and order/membership intake over ROS 2 **topics** — the observe + command
surface an FMS builds on.

This package is the **topic/message layer**. Several surfaces are intentionally
out of scope here and land in follow-up PRs:
- **Services** (sync queries / dispatch) — command goes through the
  `assign_order_request` topic instead.
- **Device Manager integration** (`FleetRoster` / `MasterConnection`) — deferred.
- **Sample apps** (`example_master`, `example_client`, `mock_fms`,
  `mock_client`) live in the reference fork.

## What's in the package

| Component | Purpose |
|---|---|
| `VDA5050MasterROS2` library | Opt-in subclass of `VDA5050Master` that publishes per-AGV topics + accepts order dispatch over topics. 6 per-AGV topics + 2 global topics. |

## Endpoints

Default `<ns>` = `vda5050_master`; override via the `VDA5050MasterROS2`
ctor's `topic_namespace` argument for multi-master deployments.

### Per-AGV topics (6)

Created on first `Connection ONLINE` from the AGV. Continuous publish
streams — async observers subscribe without polling.

| Topic | Type | Purpose |
|---|---|---|
| `/<ns>/<mfg>/<serial>/state` | `vda5050_interfaces/State` | Raw VDA5050 State at AGV's publish rate |
| `/<ns>/<mfg>/<serial>/connection` | `vda5050_interfaces/Connection` | Connection state edges: `ONLINE` / `OFFLINE` / `CONNECTION_BROKEN` |
| `/<ns>/<mfg>/<serial>/factsheet` | `vda5050_interfaces/Factsheet` | AGV capability declaration (typeSpec, physicalParameters, etc.) |
| `/<ns>/<mfg>/<serial>/device_status` | `vda5050_master_ros2/DeviceStatus` | Combined snapshot — single-subscription convenience instead of three topics |
| `/<ns>/<mfg>/<serial>/order_status` | `vda5050_master_ros2/OrderStatus` | Master's lifecycle view: phase + last_node + base/horizon counts + action_states + errors |
| `/<ns>/<mfg>/<serial>/pose_view` | `vda5050_master_ros2/PoseView` | Fused pose at a fixed, reconfigurable rate (default 1 Hz, ctor `pose_view_rate_hz`): driving + position + velocity, freshest of State / optional Visualization (latest-wins by AGV timestamp), with `source` + `data_age`. Not the VDA5050 `visualization` message. |

#### ROS 2 topic-name sanitization for `<mfg>` and `<serial>`

VDA5050 allows `serialNumber` characters `A-Z a-z 0-9 _ . : -`, and places
no character restriction on `manufacturer`. ROS 2 topic name *segments*
are stricter — each must match `^[A-Za-z_][A-Za-z0-9_]*$`. When a vendor
identity contains a leading digit or any of `. : -` (or other non-
`[A-Za-z0-9_]` chars), master rewrites the **ROS 2 topic path only**.
MQTT subscriptions and the master's internal AGV cache always use the raw
`(manufacturer, serial_number)` values.

Rule (applies to each `<mfg>` / `<serial>` segment independently):

1. Replace any character outside `[A-Za-z0-9_]` with `_`.
2. If the result then starts with a digit, prepend `_`.

| Raw (mfg / serial) | ROS 2 segment |
|---|---|
| `KION` | `KION` |
| `S001` | `S001` |
| `001` | `_001` |
| `KION-001` | `KION_001` |
| `agv.42` | `agv_42` |
| `3M` | `_3M` |

Example: an AGV with `manufacturer="KION"`, `serial_number="001"` is
published at `/vda5050_master/KION/_001/order_status`, while its MQTT topic
remains `uagv/v2/KION/001/order_status`. Distinct raw identities that
sanitize to the same ROS 2 segment are not detected at onboarding;
deployments should choose serials that remain unambiguous after
sanitization. Master logs a one-shot `INFO` the first time each identity is
sanitized.

### Global topics (2)

| Topic | Direction | QoS | Purpose |
|---|---|---|---|
| `/<ns>/assign_order_request` | external → master | RELIABLE / VOLATILE | Wire-async order dispatch. Caller publishes `AssignOrderRequest` with a caller-generated `assignment_id` UUID |
| `/<ns>/assignment_results` | master → external | RELIABLE / VOLATILE | Per-`assignment_id` outcome. Decision enum: `ACCEPTED` / `QUEUED` / `REJECTED_PREFLIGHT` / `REJECTED_POSTFLIGHT` |

## Build & test

```bash
source /opt/ros/jazzy/setup.bash
cd <workspace>
colcon build --packages-up-to vda5050_master_ros2 --cmake-args -DENABLE_ROS2=ON
colcon test --packages-select vda5050_master_ros2
```
