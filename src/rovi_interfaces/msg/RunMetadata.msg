# Identity of one run, published once by the scheduler with transient-local durability.
# Compatibility: fields are only appended within one INTERFACE_VERSION; any other change increments it.

uint32 INTERFACE_VERSION=1

uint8 POLICY_LEAST_SLACK=0
uint8 POLICY_EARLIEST_DEADLINE=1
uint8 POLICY_FIRST_IN_FIRST_OUT=2

uint32 interface_version
string run_id
uint64 seed
uint64 config_digest
string git_revision
string network_profile_id
ServiceSource service_source
uint8 policy
int64 epoch_ns
int64 slot_length_ns
int64 horizon_slots
