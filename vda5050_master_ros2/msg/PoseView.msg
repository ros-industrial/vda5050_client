# Per-AGV fused pose stream (relay mode).
#
# Master publishes at a fixed, reconfigurable rate (default 1 Hz) to
# /<namespace>/<manufacturer>/<serial_number>/pose_view.
#
# Carries the freshest of the AGV's State / Visualization position and velocity
# (latest-wins by the AGV's own header timestamp), plus driving echoed from
# State. NOT the VDA5050 `visualization` message — that one is optional in
# v2.0.0 and published at a vendor-defined frequency; this stream is a
# master-derived view at a master-controlled rate.

# Identity of the AGV this pose describes.
string manufacturer
string serial_number

# Master wall-clock time when this sample was published.
builtin_interfaces/Time stamp

# Which cached message supplied agv_position / velocity below.
uint8 SOURCE_NONE=0           # no position received yet (payload empty)
uint8 SOURCE_VISUALIZATION=1
uint8 SOURCE_STATE=2
uint8 SOURCE_EXTRAPOLATED=3   # reserved for a future extrapolation mode
uint8 source

# Age of the underlying real sample at publish time (master receive clock).
builtin_interfaces/Duration data_age

# Pose + velocity, echoed from the chosen source (same source for both).
# Empty (length 0) until the first position sample arrives, i.e. when
# source == SOURCE_NONE. map_id and position_initialized ride inside
# AGVPosition. Callers check via `.empty()` and access via `[0]`.
vda5050_interfaces/AGVPosition[<=1] agv_position
vda5050_interfaces/Velocity[<=1] velocity

# Movement flag, always echoed from State (Visualization carries no driving).
bool driving
