# Offer or first valid response receipt of one request, published by its robot agent.
# deadline_ns is the robot's own effective deadline when it offered the request, before any group
# tightening. At an observation horizon H the request is on time if received by deadline_ns, late if
# received after it but by H, missing if unreceived and deadline_ns < H, and pending otherwise.
# Group release and chunk acceptance never change that outcome. Times are ROS time in integer ns.

uint8 EVENT_OFFERED=0
uint8 EVENT_RECEIVED=1

uint8 event
uint32 robot_id
uint64 job_id
int64 stamp_ns
int64 deadline_ns
