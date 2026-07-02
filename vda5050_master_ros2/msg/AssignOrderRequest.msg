# AssignOrderRequest — async order dispatch from RES / MAPF / FMS to the
# master.
#
# Topic:     /<topic_namespace>/assign_order_request
# QoS:       RELIABLE + VOLATILE, history KEEP_LAST(10).
# Direction: caller -> master.
#
# Async order-dispatch entry point.
# The master subscribes, runs the full validator chain (schema, pre-send,
# graph, traversability, stitcher), then publishes the outcome on the
# `assignment_results` topic. Multiple requests can be in flight; results
# are correlated by `assignment_id`.

# Caller-generated UUID for correlation across topics. Must be unique
# per dispatch attempt. Retries of the same logical
# (order_id, order_update_id) SHOULD reuse the same assignment_id so a
# future de-duplication layer can collapse them. The master does not
# de-duplicate; every request produces one or more AssignmentResult
# messages.
string assignment_id

# Target AGV identity. Must match an onboarded AGV's
# {manufacturer, serial_number}; otherwise the master rejects with
# AGV_NOT_ONBOARDED in AssignmentResult.
string manufacturer
string serial_number

# The actual order payload.
vda5050_interfaces/Order order

# Wall-clock time when the caller built this request. Diagnostic only;
# the master does not validate against it.
builtin_interfaces/Time stamp
