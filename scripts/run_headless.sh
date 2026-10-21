#!/usr/bin/env bash
# Runs a ROVI scenario headlessly for a fixed wall-clock duration, records it to MCAP, and verifies the bag.
set -euo pipefail

usage() {
  echo "usage: $0 [--install-base <prefix>] [--params <file>]" \
    "<vertical_slice|sim_one_robot|arm_workcell|thirty_arms> <seconds> [bag_directory]" >&2
  exit 2
}

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
install_base="$repo_root/install"
params=""
while [[ "${1:-}" == --install-base || "${1:-}" == --params ]]; do
  [[ $# -ge 2 ]] || usage
  case "$1" in
    --install-base) install_base="$2" ;;
    --params) params="$(realpath -- "$2")" ;;
  esac
  shift 2
done
[[ $# -ge 2 && $# -le 3 ]] || usage
scenario="$1"
duration="$2"
[[ "$duration" =~ ^[1-9][0-9]*$ ]] || usage
case "$scenario" in
  vertical_slice)
    launch_args=()
    required_topics=(/rovi/run_metadata /rovi/downlink/delivered)
    ;;
  sim_one_robot)
    launch_args=(foxglove:=false)
    required_topics=(/rovi/run_metadata /rovi/downlink/delivered /robot_00/odom /tf)
    ;;
  arm_workcell)
    launch_args=(foxglove:=false gui:=false record:=true)
    required_topics=(/rovi/run_metadata /rovi/downlink/delivered /robot_00/joint_states
      /robot_00/arm/panda_joint1/position)
    ;;
  thirty_arms)
    launch_args=(foxglove:=false gui:=false record:=true)
    required_topics=(/rovi/run_metadata /rovi/downlink/delivered /rovi/coordination/releases
      /rovi/task_progress /robot_00/joint_states /robot_29/joint_states)
    ;;
  *)
    usage
    ;;
esac

launch_args+=("params_file:=${params:-$repo_root/src/rovi_bringup/config/${scenario}.yaml}")
bag_dir="$(realpath -m -- "${3:-$repo_root/bags/${scenario}_$(date -u +%Y%m%dT%H%M%SZ)}")"
if [[ -e "$bag_dir" ]]; then
  echo "refusing to overwrite $bag_dir" >&2
  exit 1
fi
mkdir -p -- "$(dirname -- "$bag_dir")"

revision="git-$(git -C "$repo_root" rev-parse HEAD)"
if [[ -n "$(git -C "$repo_root" status --porcelain)" ]]; then
  revision="${revision}-dirty"
fi
provenance="$(mktemp -d)"
trap 'rm -rf -- "$provenance"' EXIT
manifest_digest="$("$repo_root/scripts/write_manifest.sh" --install-base "$install_base" --duration "$duration" \
  "$scenario" "$provenance" "${launch_args[@]}")"

set +u
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source "$install_base/setup.bash"
set -u

export GZ_IP=127.0.0.1
GZ_PARTITION="rovi_$(od -An -N8 -tx1 /dev/urandom | tr -d ' \n')"
export GZ_PARTITION
echo "recording ${duration}s of ${scenario} from $install_base to $bag_dir ($revision, manifest $manifest_digest)"
status=0
ROVI_GIT_REVISION="$revision" timeout --signal=INT --kill-after=10 "$duration" \
  ros2 launch rovi_bringup "${scenario}.launch.xml" bag_uri:="$bag_dir" "${launch_args[@]}" ||
  status=$?
if [[ $status -ne 0 && $status -ne 124 ]]; then
  echo "launch failed with status $status" >&2
  exit "$status"
fi
mv -- "$provenance/manifest.yaml" "$provenance/params.yaml" "$bag_dir/"
info="$(ros2 bag info "$bag_dir")"
echo "$info"
for topic in "${required_topics[@]}"; do
  if ! grep -Eq "Topic: ${topic} \| Type: [^|]+\| Count: [1-9][0-9]* \|" <<<"$info"; then
    echo "the bag has no messages on ${topic}" >&2
    exit 1
  fi
done
if grep -Eq "Topic: /rovi/run/invalidations \| Type: [^|]+\| Count: [1-9][0-9]* \|" <<<"$info"; then
  echo "ROS time moved backward during the run; the bag is invalid and the run must be restarted" >&2
  exit 1
fi
