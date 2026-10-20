# Admission decision for one job, published by the scheduler.
# Status and reason values mirror rovi_core DecisionStatus and ReasonCode.

uint8 STATUS_REJECTED=0
uint8 STATUS_DEFERRED=1
uint8 STATUS_ADMITTED=2

uint8 REASON_NONE=0
uint8 REASON_INVALID_SNAPSHOT=1
uint8 REASON_INVALID_PAYLOAD_SIZE=2
uint8 REASON_INVALID_TIMING=3
uint8 REASON_DUPLICATE_JOB=4
uint8 REASON_ARITHMETIC_OVERFLOW=5
uint8 REASON_NO_ELIGIBLE_WORKER=6
uint8 REASON_PREDECESSOR_INCOMPLETE=7
uint8 REASON_UPLINK_SERVICE_UNAVAILABLE=8
uint8 REASON_NO_FEASIBLE_CHAIN=9
uint8 REASON_DEADLINE_MISS=10
uint8 REASON_STALE_RESULT=11
uint8 REASON_GROUP_MEMBER_UNAVAILABLE=12
uint8 REASON_GROUP_SKEW_EXCEEDED=13
uint8 REASON_QUEUE_FULL=14
uint8 REASON_MALFORMED_REQUEST=15

uint64 job_id
uint32 robot_id
uint32 model_class
uint64 cycle
int64 decided_ns
uint8 status
uint8 reason
ServiceSource service_source
bool has_reservation
Reservation reservation
