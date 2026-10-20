# Logical wireless-service model

ROVI schedules an **abstract, end-to-end network service**. It does **not** schedule Wi-Fi airtime, OFDMA resource units, 5G resource blocks, or a radio MAC. Wireless is the access environment being modeled. The model is a set of deterministic integer capacity calendars.

## Elements

| Element | Representation |
|---|---|
| Uplink and downlink | Two separate `CapacityCalendar`s with identical slot geometry (origin, slot length, slot count) |
| Shared access medium (optional) | A third calendar. Every uplink and downlink slice reserves it too, so per slot, uplink plus downlink never exceeds the medium. |
| Usable capacity per slot | `bytes_per_slot(rate, slot_length, reduction)`: floor(rate × slot / 1 s), then the `reduction` permille removed, then whole bytes. Every step rounds down. |
| Loss, retry overhead, and uncertainty margin | Folded into `reduction` (0–1000 permille) as a conservative planning margin |
| Time variation, handover, reassociation | `build_service_calendar` takes piecewise-constant `ServiceSegment`s. A zero-rate segment is an outage; a later segment with a different rate is reassociation. |
| Fixed path delay | `uplink_delay` and `downlink_delay`, applied between stages and not carried as payload |
| Contention | Every job reserves bytes in the same shared calendars |
| Access site or AP association | Not modeled per job. Each snapshot describes one access path. |
| Provenance | `ServiceSource` on the snapshot (`kLogical`, `kEnforced`, `kMeasured`, `kReplayed`), copied unchanged into every `ScheduleResult` |

## Requested, reserved, and measured service

- **Requested:** the job's `upload_bytes` and `action_chunk_bytes`.
- **Modeled-reserved:** the `Transfer` slices committed in the calendars.
- **Delivered (logical):** the `rovi_network` link releases each observation and action chunk at its reserved arrival time if it was sent by its reserved service start with the authorized robot, worker, and byte count. Any other payload is refused and published as a `ReservationViolation`; it is never shifted onto unreserved service. The link records its release poll time in `Payload.delivered_ns` and labels the payload `ServiceSource.LOGICAL`. This is the logical model executing its own plan, not a measurement of a network.
- **Measured-delivered:** not produced yet; an enforced data path would label it `MEASURED`.

A logical reservation is never an enforced or measured guarantee.

## Profiles in use

| Profile ID | Where | Shape |
|---|---|---|
| `synthetic-single-path-v1` | `vertical_slice.yaml` | Uplink 100 Mbit/s and downlink 50 Mbit/s, each with a 10% reduction; 2 ms path delay each way |
| `synthetic-edge-handover-v1` | `sim_one_robot.yaml`, `arm_workcell.yaml`, `thirty_arms.yaml` | As above until 6 s after the scheduler epoch; a full outage from 6 s to 7.5 s; then reassociation at 80 and 40 Mbit/s. Time-indexed, not coupled to robot position. |

## Profile provenance

Every profile used in tests or scenarios is a **controlled synthetic input**, chosen so the arithmetic can be checked by hand, for example 1 Gbit/s × 1 ms = 125,000 bytes per slot. None of them reproduces a particular Wi-Fi or 5G deployment; calibrated profiles need measurements or a validated network simulator. Scenario files live in `src/rovi_bringup/config/`.
