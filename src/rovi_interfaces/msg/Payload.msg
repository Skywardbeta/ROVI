# Synthetic observation or action-chunk payload carried by the network service.
# sent_ns is stamped by the sender; delivered_ns and service_source are stamped by the delivering network service.

uint8 KIND_OBSERVATION=0
uint8 KIND_ACTION_CHUNK=1

uint8 kind
uint64 job_id
uint32 robot_id
uint64 worker_id
int64 bytes
int64 observation_time_ns
int64 sent_ns
int64 delivered_ns
ServiceSource service_source
