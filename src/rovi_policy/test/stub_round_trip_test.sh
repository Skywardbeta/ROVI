#!/usr/bin/env bash
# Runs the C++ probe against the Python adapter's stub server, so both wire implementations must agree, and
# checks that the server's announced identity follows its adapter source and task.
set -euo pipefail

adapter="$1"
probe="$2"
work="$(mktemp -d)"
servers=()
cleanup() {
  for server in "${servers[@]}"; do
    kill "$server" 2>/dev/null || true
    wait "$server" 2>/dev/null || true
  done
  rm -rf -- "$work"
}
trap cleanup EXIT
fail() {
  echo "FAIL: $*" >&2
  exit 1
}

# start_stub NAME ADAPTER_DIRECTORY TASK: serves on a free port and writes the port to $work/NAME.port.
start_stub() {
  python3 "$2/rovi_policy_server.py" --stub --host 127.0.0.1 --port 0 --port-file "$work/$1.port" \
    --task "$3" --image-width 4 --image-height 2 >"$work/$1.log" 2>&1 &
  servers+=("$!")
  for _ in $(seq 100); do
    [[ -s "$work/$1.port" ]] && return 0
    sleep 0.1
  done
  fail "the $1 stub did not start"
}

python3 -I -c '
import sys
sys.path.insert(0, sys.argv[1])
import rovi_wire
rovi_wire.write_observation_file(sys.argv[2], 1, 2, [10.0, 20.0, 30.0, 40.0, 50.0, 60.0], 4, 2, bytes(range(24)))
' "$adapter" "$work/observation.rovobs"
stub_id="$(python3 -I -c 'import hashlib; print(hashlib.sha256(b"rovi-stub-policy").hexdigest())')"
probe_at() {
  local port
  port="$(<"$work/$1.port")"
  shift
  "$probe" --host 127.0.0.1 --port "$port" --observation "$work/observation.rovobs" "$@"
}

start_stub original "$adapter" test
output="$(probe_at original --policy "$stub_id" --task test --repeat 2)"
grep -q "^policy stub $stub_id: 50 x 6 actions every 33.3 ms$" <<<"$output" || fail "unexpected hello"
grep -q "^request 2 (.*): valid, .* first action \[10.0, 20.0, 30.0, 40.0, 50.0, 60.0\]$" <<<"$output" ||
  fail "unexpected chunk"
original_adapter="$(sed -n 's/^server adapter \([0-9a-f]\{64\}\), task "test", .*/\1/p' <<<"$output")"
[[ -n "$original_adapter" ]] || fail "the server did not announce its adapter and task"

if probe_at original --policy "$(printf '%064d' 0)" >"$work/other_policy.log"; then
  fail "a client expecting another policy was served"
fi
grep -q "^request 1: identity" "$work/other_policy.log" || fail "another policy was not an identity failure"
if probe_at original --policy "$stub_id" --task place >"$work/other_task.log"; then
  fail "a client expecting another task was served"
fi
grep -q '^request 1: identity (the server.s task is "test")$' "$work/other_task.log" ||
  fail "another task was not an identity failure"

cp -r -- "$adapter" "$work/edited"
echo "# edited" >>"$work/edited/rovi_policy_server.py"
start_stub edited "$work/edited" test
edited_adapter="$(probe_at edited --policy "$stub_id" | sed -n 's/^server adapter \([0-9a-f]\{64\}\), .*/\1/p')"
[[ -n "$edited_adapter" && "$edited_adapter" != "$original_adapter" ]] ||
  fail "an edited adapter announced the same identity"
