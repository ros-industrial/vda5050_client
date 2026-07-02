# Per-AGV order execution status.
#
# Master publishes on every State arrival to the topic
# /<namespace>/<manufacturer>/<serial_number>/order_status.
#
# Pure-state fields are echoed from the AGV's most recent State message;
# lifecycle fields are master derivations.

# Identity of the AGV this status describes.
string manufacturer
string serial_number

# Cross-channel correlation key. UUID provided by the caller in the
# original AssignOrderRequest that produced this order. Same value
# echoed in the corresponding AssignmentResult and in every OrderStatus
# publish for the lifetime of the assignment (including stitched
# updates that share the order lifecycle). Empty when the master has
# no active assignment for this AGV (e.g. PHASE_NO_ORDER).
string assignment_id

# Master wall-clock time when this snapshot was built.
builtin_interfaces/Time stamp

# Order identification. Echoed from State; empty / 0 when there is no
# active order tracked by the master.
string order_id
uint32 order_update_id

# Master lifecycle phase. Sequential — future values appended.
# CANCELLING / CANCELLED are reserved for a future cancelOrder capability
# and are not emitted yet.
uint8 PHASE_NO_ORDER=0    # AGV onboarded; master tracking no order
uint8 PHASE_ACCEPTED=1    # order published; AGV not yet at first node
uint8 PHASE_RUNNING=2     # AGV traversing nodes / executing actions
uint8 PHASE_COMPLETED=3   # AGV reached final base node
uint8 PHASE_FAILED=4      # any FATAL error present in State.errors
uint8 phase

# Progress (echoed from State).
string last_node_id
uint32 last_node_sequence_id

# Master-side counts of nodes in the active order. Stable — these do
# NOT shrink when horizon is released to base. As released-horizon
# becomes base, nodes_in_horizon decreases by N and nodes_in_base
# increases by N, total stays constant. Both 0 when no active order.
uint32 nodes_in_base
uint32 nodes_in_horizon

# Live execution flags (mirror State). Orthogonal to phase.
# paused / new_base_request are optional in State: empty = the AGV did not
# report the field, single element = the reported value.
bool driving
bool[<=1] paused
bool[<=1] new_base_request

# Master queue depth — order updates waiting for stitch conditions in
# OrderLifecycleManager. 0 when no pending updates.
uint32 pending_stitch_count

# Action states from State.action_states. The AGV keeps action_states
# until a new order is received, preserving the audit trail — FMS clients
# filter by action_status as needed.
vda5050_interfaces/ActionState[] action_states

# Errors from State.errors. The AGV keeps errors until it resolves them.
vda5050_interfaces/Error[] errors
