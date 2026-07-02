# DeviceStatus — combined snapshot of one AGV's State + Connection +
# Factsheet, plus master-side cache freshness.
#
# Topic: /<topic_namespace>/<manufacturer>/<serial_number>/device_status
# QoS:   RELIABLE + TRANSIENT_LOCAL (latched so a late subscriber gets
#        the current snapshot immediately, without waiting for the next
#        State arrival).
# Direction: master -> downstream consumers (RES, MAPF, dashboards).
#
# Streaming equivalent of GetDeviceStatus.srv. Master publishes on every
# State / Connection / Factsheet arrival from the AGV. Cadence is
# dominated by State at ~1 Hz active / 30 s heartbeat. Each publish
# carries the latest cached values for all three components;
# `Type[<=1]` fields are empty when that component has not yet been
# received from the AGV.

# Identity.
string manufacturer
string serial_number

# Master wall-clock time the snapshot was built.
builtin_interfaces/Time stamp

# Optional payloads — bounded array carries 0 or 1 element to express
# the ROS 2 idiomatic optional. Empty until the AGV has sent that
# component at least once.
vda5050_interfaces/State[<=1]      state
vda5050_interfaces/Connection[<=1] connection
vda5050_interfaces/Factsheet[<=1]  factsheet

# Master-side cache freshness, one per payload. Empty when the matching
# payload above is empty; otherwise the master wall-clock time of the
# most recent receive for that component.
builtin_interfaces/Time[<=1] state_received_at
builtin_interfaces/Time[<=1] connection_received_at
builtin_interfaces/Time[<=1] factsheet_received_at
