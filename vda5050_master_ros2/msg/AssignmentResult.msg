# AssignmentResult — outcome of one AssignOrderRequest.
#
# Topic:     /<topic_namespace>/assignment_results
# QoS:       RELIABLE + VOLATILE, history KEEP_LAST(10).
# Direction: master -> caller.
#
# Multiplicity:
#   - Pre-flight rejection: master emits exactly one AssignmentResult
#     with decision = REJECTED_PREFLIGHT. No order lifecycle is created;
#     no OrderStatus follows.
#   - Accepted / queued: master emits one AssignmentResult with
#     decision = ACCEPTED or QUEUED, then the OrderStatus topic begins
#     (or continues) streaming for the AGV with the matching
#     `assignment_id`.
#   - Post-flight rejection (factsheet change, mode flip, stitch
#     timeout, etc.): master MAY emit a follow-up AssignmentResult with
#     decision = REJECTED_POSTFLIGHT. Callers treat the latest message
#     for an assignment_id as authoritative.

# Echoed from the request.
string assignment_id
string order_id
uint32 order_update_id

# Decision enum.
uint8 ACCEPTED=0             # Validators passed; order handed to AGV queue.
uint8 QUEUED=1               # Validators passed but stitch not yet possible;
                             # held in lifecycle pending_updates and will be
                             # drained as the AGV reports progress.
uint8 REJECTED_PREFLIGHT=2   # Synchronous validator failed at dispatch time.
                             # No order lifecycle entered.
uint8 REJECTED_POSTFLIGHT=3  # Earlier ACCEPTED / QUEUED has been invalidated
                             # by later state (e.g. factsheet change, stitch
                             # timeout). This enum value is reserved; the
                             # master does not yet emit it.
uint8 decision

# Diagnostic errors. Empty on ACCEPTED / QUEUED. Populated for
# REJECTED_* with the master's reason codes.
vda5050_interfaces/Error[] errors

# Wall-clock time the master decided this result.
builtin_interfaces/Time stamp
