# Observation or action-chunk payload carried by the network service.
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
# Real observation content, empty for synthetic payloads: joint positions in policy units and one
# row-major RGB8 image.
float32[] joint_positions
uint32 image_width
uint32 image_height
uint8[] image_rgb8
# Real chunk content, empty for synthetic payloads: action_steps rows of action_dim values in policy
# units, action_period_ns apart, from the policy whose SHA-256 ID is policy_id.
uint32 action_steps
uint32 action_dim
float32[] actions
int64 action_period_ns
string policy_id
