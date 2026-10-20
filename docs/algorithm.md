# Scheduling algorithm

ROVI's scheduler is a **deterministic heuristic**: rolling-horizon, least-slack, transactional reservation of upload, inference, and action-chunk download. This document does not claim optimality, hard real-time behavior, or performance superiority.

## One cycle

`schedule(snapshot, pending, completed, now, policy)` in `rovi_core/scheduler.hpp` runs the following steps.

1. **Validate the snapshot** (`is_consistent`). If it is inconsistent, reject every job with `InvalidSnapshot`.
2. **Partition the pending jobs:**
   - Duplicate IDs are rejected.
   - Jobs whose predecessors are not in `completed` are deferred. Readiness depends on *actual* completion only, never on admission within the same cycle.
   - Jobs that share a `synchronization_group` form one atomic unit, ordered by `(sequence, id)`. If any member is blocked, the whole group is deferred.
3. **Plan each unit against the cycle's starting snapshot.** Each member goes through single-job admission on a copy-on-write trial snapshot. If any member fails, the unit fails. For a group, the spread of result-ready times must not exceed the smallest `maximum_result_ready_skew` among its members. The unit's slack is the smallest member slack, and its deadline is the earliest member deadline. Units that fail here are decided immediately, because within a cycle capacity only shrinks.
4. **Order the feasible units by policy:**
   - `kLeastSlack` (ROVI): slack, then deadline, then sequence, then first job ID.
   - `kEarliestDeadline` (baseline): deadline, then sequence, then first job ID.
   - `kFirstInFirstOut` (baseline): earliest release, then sequence, then first job ID.
5. **Commit units in that order.** Each unit is planned again against the *working* snapshot, which already holds every earlier commit. It is committed all-or-none, or discarded with its reason. Committed reservations never move.
6. **Report.** Decisions are sorted by `(sequence, id)`. The result carries the final snapshot, the snapshot's service-source label, and an FNV-1a 64-bit **decision digest** over every decision and reservation field.

The baselines use the same chain search and transactional commit, so comparing them isolates ordering *within* linked planning; they are not independent controls. The policy comparison below adds independent-stage controls.

## Rolling planning window

`rovi_core/window.hpp` maintains the planning window between cycles.

- **Opening.** `open_window(profile, workers, epoch, slot_count)` builds the uplink, downlink, and optional shared-medium calendars for absolute profile slots `[0, slot_count)` from the epoch. It also creates one empty lane per worker. It rejects duplicate workers, duplicate models, non-positive durations, malformed segments, and a horizon that cannot be represented.
- **Advancing.** `advance_window(current, profile, epoch, now)` slides the window so that it starts at the slot containing `now`:
  - Slots that are still in the window keep their capacity and their committed reservations. `CapacityCalendar::advance` drops the leading slots and appends new ones.
  - Appended slots get their capacity from the profile at their absolute slot index (`service_capacity`), so outages and handovers stay anchored to run time.
  - If the jump passes the whole horizon, a fresh window opens at the current slot.
  - Compute intervals that ended at or before the new origin are pruned.
  - Delays, the dispatch lead, and the source label are carried over unchanged.
  - The result must still pass `is_consistent`.
- **No movement.** Committed reservations never move. The ROS scheduler calls `advance_window` at the start of every cycle.

## Single-job chain search (`admit`)

- The upload uses the earliest slot service from `now` plus `dispatch_lead`, bounded by the uplink and by the shared medium if one is configured.
- Each eligible worker gives an earliest-fit compute interval starting at or after the observation has arrived plus `dispatch_lead`.
- The download uses the earliest slot service from compute end plus `dispatch_lead`, bounded by the downlink and the medium.
- `dispatch_lead` keeps each reserved stage start actionable by the component that must react to the preceding event.
- The candidate with the earliest delivery wins, and ties go to the lowest worker ID.

Each decision carries a status and a reason code (`rovi_core/admission.hpp`). Single-job earliest delivery is checked against a brute-force enumeration oracle on 300 seeded tiny instances.

## Invariants and how they are checked

`audit_schedule()` (`rovi_core/audit.hpp`) recomputes the following from first principles, without using the planner's code paths:

- per-slot uplink, downlink, and medium usage against capacity
- stage ordering and delivery consistency
- effective deadlines
- compute length against the worker profile
- lane non-overlap
- predecessor completion
- group atomicity and skew
- the final snapshot
- the digest

Tests run it on 300 seeded random instances under every policy. Tamper tests show it detects each violation class.

## Policy comparison

`run_comparison()` (`rovi_core/comparison.hpp`) runs one policy over a fixed offered cohort on `Engine`, a ledger of actual per-slot uplink and downlink bytes and one non-preemptive compute lane. Every policy sees a request only from its release and acts only through the engine.

- **Linked policies** (`linked_least_slack`, `linked_earliest_deadline`, `linked_first_in_first_out`) admit whole chains with the planner above, then execute each reservation exactly.
- **Independent-stage controls** (`independent_earliest_deadline`, `independent_equal_share`) decide each stage from what has already happened and never reserve a later stage.
- **Endpoint caps** optionally bound each robot's service per slot and direction, summed over its requests, in both admission (`EndpointBudget`) and the engine.
- **Strict execution** is the default: a reservation the data misses is lost, never shifted.
- **Best-effort recovery** (`RecoveryRules`) serves rejected requests and lost reservations from service the commitments leave unused. Reserved slices on both links are served first. Recovery compute needs a declared worst-case bound and runs only in lane gaps no reservation protects; without a bound it is never dispatched. Optional age, supersession, and backlog rules drop work with an explicit reason.
- **Outcomes** are judged against original deadlines: each request is on time, late, missing, or pending at the horizon, and paired runs count recovered and harmed requests.

`rovi_compare` prints a deterministic YAML report of these runs.

## Known limitations

- **Greedy transfers.** Each slot gives a transfer all of its remaining capacity. There are no service-quantum or fairness limits.
- **No skew repair.** Group skew is checked, but members are not delayed to repair it.
- **No preemption or movement.** There is no preemption, batching, migration, or route selection, and committed reservations never move.
- **Window length bounds admission.** A chain whose download cannot complete inside the window is deferred with `NoFeasibleChain`, and the job is planned again in a later cycle. The window must be longer than the longest action horizon.
- **Cost.** Planning costs O(units × members × workers × slots) per pass, with two passes. This has not been profiled yet.
