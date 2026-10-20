#!/usr/bin/env bash
# Checks that rovi_compare reports hand-derived outcomes, repeats byte for byte, and refuses bad input.
set -euo pipefail

compare="$1"
work="$(mktemp -d)"
trap 'rm -rf -- "$work"' EXIT
fail() {
  echo "FAIL: $*" >&2
  exit 1
}
has() {
  grep -qxF -- "$1" "$2" || fail "missing line '$1' in $(basename "$2")"
}

# Four robots release together with 40 ms deadlines (see ABurstOfFourHasHandDerivedLinkedAndIndependentOutcomes).
burst=(--robots 4 --requests 1 --deadline-ns 40000000 --slots 100
  --policies "independent_earliest_deadline,linked_least_slack")
"$compare" "${burst[@]}" >"$work/first.yaml"
"$compare" "${burst[@]}" >"$work/again.yaml"
cmp -s "$work/first.yaml" "$work/again.yaml" || fail "identical inputs gave different reports"
has "reference_policy: independent_earliest_deadline" "$work/first.yaml"
has "  task_success: false" "$work/first.yaml"
has "  independent: work-conserving; a stage never drops a request whose deadline has passed" "$work/first.yaml"
has "  equal_share: a link-sharing stress diagnostic, not a competitive policy" "$work/first.yaml"
has "  linked_execution: strict; a missed lease is lost, never replanned or shifted" "$work/first.yaml"
has "  offered: the whole cohort, including requests released after the horizon, which stay pending" "$work/first.yaml"
has "  latency: receipt time minus release time" "$work/first.yaml"
has "  offered: 4" "$work/first.yaml"
has "    outcomes: {offered: 4, on_time: 2, late: 2, missing: 0, pending: 0}" "$work/first.yaml"
has "    outcomes: {offered: 4, on_time: 2, late: 0, missing: 2, pending: 0}" "$work/first.yaml"
has "    commitments: {none: 0, admitted: 2, rejected: 2, undecided: 0, leases_missed: 0}" "$work/first.yaml"
has "      late: {upload_bytes: 200000, compute_ns: 20000000, committed_compute_ns: 20000000, response_bytes: 20000}" \
  "$work/first.yaml"
has "    receipt_latency_ns: {received: 4, minimum: 21000000, median: 31000000, p95: 51000000, maximum: 51000000}" \
  "$work/first.yaml"
has "  - candidate: linked_least_slack" "$work/first.yaml"
has "    net_delivery_change: 0/4 (+0.00 pp)" "$work/first.yaml"

# A staggered, overloaded example.
"$compare" --robots 6 --requests 20 --period-ns 50000000 --phase-spread-ns 30000000 --deadline-ns 45000000 \
  --upload-bytes 60000 --seed 3 --slots 1100 >"$work/staggered.yaml"
has "  digest: 76412c8da040f759" "$work/staggered.yaml"
has "    outcomes: {offered: 120, on_time: 14, late: 95, missing: 11, pending: 0}" "$work/staggered.yaml"
has "    outcomes: {offered: 120, on_time: 101, late: 0, missing: 19, pending: 0}" "$work/staggered.yaml"
has "    net_delivery_change: 87/120 (+72.50 pp)" "$work/staggered.yaml"

# The same example with a 6,000-byte per-robot uplink cap: with a declared 10 ms compute
# bound, best effort delivers 7 of the 19 rejected requests late, and the on-time count is unchanged;
# without a bound, recovery compute is deferred and nothing more is delivered.
capped=(--robots 6 --requests 20 --period-ns 50000000 --phase-spread-ns 30000000 --deadline-ns 45000000
  --upload-bytes 60000 --seed 3 --slots 1100 --endpoint-uplink-bytes 6000 --policies linked_least_slack)
"$compare" "${capped[@]}" >"$work/capped_strict.yaml"
"$compare" "${capped[@]}" --best-effort 1 --recovery-compute-bound-ns 10000000 >"$work/capped_recovery.yaml"
"$compare" "${capped[@]}" --best-effort 1 >"$work/capped_deferred.yaml"
has "    outcomes: {offered: 120, on_time: 101, late: 0, missing: 19, pending: 0}" "$work/capped_strict.yaml"
has "    outcomes: {offered: 120, on_time: 101, late: 7, missing: 12, pending: 0}" "$work/capped_recovery.yaml"
has "    recovery_counts: {entered: 19, delivered: 7, expired: 0, superseded: 0, backlog_full: 0}" \
  "$work/capped_recovery.yaml"
has "    outcomes: {offered: 120, on_time: 101, late: 0, missing: 19, pending: 0}" "$work/capped_deferred.yaml"
has "    recovery_counts: {entered: 19, delivered: 0, expired: 0, superseded: 0, backlog_full: 0}" \
  "$work/capped_deferred.yaml"

"$compare" >"$work/default.yaml"
[[ "$(grep -c '^  - policy: ' "$work/default.yaml")" == 5 ]] || fail "the default run did not compare every policy"
[[ "$(grep -c '^  - candidate: ' "$work/default.yaml")" == 4 ]] || fail "the default run did not pair every candidate"

"$compare" --policies least_slack >"$work/alias.yaml"
has "  - policy: linked_least_slack" "$work/alias.yaml"

expect_status() {
  local expected="$1"
  shift
  local status=0
  "$compare" "$@" >/dev/null 2>&1 || status=$?
  [[ "$status" == "$expected" ]] || fail "'$*' exited with $status instead of $expected"
}
expect_status 2 --unknown 1
expect_status 2 --policies optimal
expect_status 2 --robots many
expect_status 2 --robots -1
expect_status 2 --robots
expect_status 1 --robots 0
expect_status 1 --slots 0
expect_status 2 --policies least_slack,
expect_status 2 --policies least_slack,linked_least_slack
expect_status 2 --robots 4 --robots 5
expect_status 0 --seed 18446744073709551615 --robots 1 --requests 1 --slots 60
expect_status 2 --seed 18446744073709551616
# A capped, recovering run declares its rules and counts its recovery.
"$compare" --robots 2 --requests 2 --deadline-ns 15000000 --slots 120 --endpoint-uplink-bytes 6000 \
  --best-effort 1 --supersede 1 --policies "independent_earliest_deadline,linked_least_slack" >"$work/recovery.yaml"
has "recovery: {best_effort: true, compute_bound_ns: none, maximum_age_ns: none, supersede: true, backlog: none}" "$work/recovery.yaml"
has "  endpoint_uplink_bytes: 6000" "$work/recovery.yaml"
has "  linked_execution: best_effort; a missed lease is never replanned or shifted; that request, like a rejected one, gets only unreserved service and is never computed, since no compute bound is declared" "$work/recovery.yaml"
has "  independent: work-conserving; a request is dropped only by the declared recovery rules, never because its deadline has passed" "$work/recovery.yaml"
has "  cancellation: maximum age, supersession, and backlog apply only to work served without a reservation" "$work/recovery.yaml"
! grep -q "^  cancellation:" "$work/first.yaml" || fail "a run without recovery rules declared cancellation"
grep -q "^    recovery_counts: {entered: [1-9]" "$work/recovery.yaml" || fail "the linked run did not report recovery"
expect_status 2 --best-effort 2
expect_status 2 --backlog -1
expect_status 2 --recovery-compute-bound-ns 0
expect_status 1 --recovery-compute-bound-ns 9999999
echo "rovi_compare checks passed"
