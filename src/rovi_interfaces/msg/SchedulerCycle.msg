# Telemetry for one scheduler cycle. runtime_ns is host steady-clock time and never feeds a decision.

uint64 cycle
int64 now_ns
int64 window_start_ns
int64 window_end_ns
uint32 pending
uint32 admitted
uint32 deferred
uint32 rejected
uint64 decision_digest
int64 runtime_ns
