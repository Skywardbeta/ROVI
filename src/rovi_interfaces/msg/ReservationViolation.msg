# Logical service the link refused because a payload did not match its immutable reservation.
# A refused payload receives no service and is never delivered; the job's later reservations stay
# committed and unused until the window passes them. Times are ROS time in integer nanoseconds and
# sizes are integer bytes. robot_id, worker_id, and bytes are as claimed by the payload.

uint8 DIRECTION_UPLINK=0
uint8 DIRECTION_DOWNLINK=1

uint8 REASON_LATE_SEND=0
uint8 REASON_SIZE_MISMATCH=1
uint8 REASON_IDENTITY_MISMATCH=2
uint8 REASON_DUPLICATE=3
uint8 REASON_UNLEASED=4
uint8 REASON_WRONG_DIRECTION=5

uint64 job_id
uint32 robot_id
uint64 worker_id
uint8 direction
uint8 reason
int64 bytes
int64 sent_ns
int64 detected_ns
bool has_lease
int64 service_start_ns
int64 arrival_ns
int64 authorized_bytes
ServiceSource service_source
