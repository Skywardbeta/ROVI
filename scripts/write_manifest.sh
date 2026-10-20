#!/usr/bin/env bash
# Writes manifest.yaml and the parameter file that a ROVI scenario run uses into a directory, and prints the manifest digest.
set -euo pipefail
export LC_ALL=C

usage() {
  echo "usage: $0 --install-base <prefix> --duration <seconds>" \
    "<vertical_slice|sim_one_robot|arm_workcell|thirty_arms> <directory> [launch_argument...]" >&2
  exit 2
}
die() {
  echo "$*" >&2
  exit 1
}

install_base=""
duration=""
while [[ $# -gt 0 && "$1" == --* ]]; do
  [[ $# -ge 2 ]] || usage
  case "$1" in
    --install-base) install_base="$2" ;;
    --duration) duration="$2" ;;
    *) usage ;;
  esac
  shift 2
done
[[ $# -ge 2 && -n "$install_base" && "$duration" =~ ^[1-9][0-9]*$ ]] || usage
scenario="$1"
out_dir="$2"
shift 2
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
installed="$(cd -- "$install_base" 2>/dev/null && pwd)" || die "no install base at $install_base"
install_base="$installed"
shown_install="${install_base/#"$repo_root/"/<repo>/}"

params="$repo_root/src/rovi_bringup/config/${scenario}.yaml"
sim_time=true
needs_model=false
case "$scenario" in
  vertical_slice)
    sim_time=false
    actions="none (no robot actuation)"
    task_outcome="none"
    ;;
  sim_one_robot)
    actions="predefined waypoints (no policy output)"
    task_outcome="none"
    ;;
  arm_workcell | thirty_arms)
    actions="predefined joint goals (no policy output)"
    task_outcome="joint-goal cycle count, not an object-placement detector"
    needs_model=true
    ;;
  *)
    usage
    ;;
esac

launch_list=""
for argument in "$@"; do
  case "$argument" in
    params_file:=*) params="${argument#params_file:=}" ;;
    use_sim_time:=*) sim_time="${argument#use_sim_time:=}" ;;
  esac
  shown="${argument//"$repo_root/"/<repo>/}"
  launch_list+="'${shown//\'/\'\'}', "
done
case "${sim_time,,}" in
  true | 1) timing="ros_sim_time, free-running (not lock-stepped)" ;;
  false | 0) timing="ros_wall_time, free-running" ;;
  *)
    echo "unrecognized use_sim_time value: $sim_time" >&2
    exit 2
    ;;
esac
[[ "$timing" != ros_sim_time* || "$scenario" == vertical_slice ]] ||
  timing="gazebo_sim_time, free-running (not lock-stepped)"

digest_of() { sha256sum | cut -d' ' -f1; }
git_in() { git -C "$repo_root" "$@"; }
share_of() {
  local share
  for share in "$install_base/$1/share/$1" "$install_base/share/$1"; do
    if [[ -f "$share/package.xml" || -L "$share/package.xml" ]]; then
      echo "$share"
      return 0
    fi
  done
  return 1
}

# --symlink-install links each file an ament package installs from its source tree back to that
# file. Every installed counterpart of a source file the source hashes cover (tracked, or untracked
# and not ignored) must therefore be such a link, so the launch files, worlds, and configurations ROS
# loads are the hashed ones. rovi_core is plain CMake, so it is copied, and it is compiled
# statically into the ament components checked here.
[[ -f "$install_base/setup.bash" ]] || die "$install_base has no setup.bash"
declare -A ament_share=()
for source_manifest in "$repo_root"/src/*/package.xml; do
  package="$(basename -- "$(dirname -- "$source_manifest")")"
  share="$(share_of "$package")" || die "$package is not installed in $install_base"
  if grep -q '<build_type>ament_cmake</build_type>' "$source_manifest"; then
    ament_share[$package]="$share"
  fi
done
unlinked="$(git_in ls-files -z --cached --others --exclude-standard -- src |
  while IFS= read -r -d '' file; do
    package="${file#src/}"
    package="${package%%/*}"
    share="${ament_share[$package]:-}"
    installed="$share/${file#src/"$package"/}"
    if [[ -n "$share" && (-e "$installed" || -L "$installed") &&
      "$(readlink -f -- "$installed")" != "$(readlink -f -- "$repo_root/$file")" ]]; then
      echo "$file"
    fi
  done)"
[[ -z "$unlinked" ]] ||
  die "$install_base is not a --symlink-install build of $repo_root; these installed files differ:"$'\n'"$unlinked"
launch="$(share_of rovi_bringup)/launch/${scenario}.launch.xml"
for input in "$params" "$launch"; do
  [[ -f "$input" ]] || die "missing input file: $input"
done
model_digest=none
if [[ "$needs_model" == true ]]; then
  model="$(share_of rovi_sim)/models/rovi_panda_fuel_v5"
  [[ -f "$model/model.sdf" ]] || die "the Panda model is missing from $model: run scripts/fetch_models.sh, then rebuild"
  [[ -z "$(find -L "$model" -type l -print -quit)" ]] || die "the installed Panda model has dangling links: rebuild"
  model_digest="$(cd "$model" && find -L . -type f -print0 | sort -z | xargs -0 -r sha256sum | digest_of)"
fi

source_block="$("$repo_root/scripts/print_source_provenance.sh")"
params_digest="$(digest_of <"$params")"
launch_digest="$(digest_of <"$launch")"
libraries="$(cd "$install_base" && find -L . -path '*/lib/*' -name '*.so' -type f -print0 | sort -z |
  xargs -0 -r sha256sum | digest_of)"
hashes=("$params_digest" "$launch_digest" "$libraries")
[[ "$model_digest" == none ]] || hashes+=("$model_digest")
for value in "${hashes[@]}"; do
  [[ "$value" =~ ^[0-9a-f]{64}$ ]] || die "could not hash every input"
done

body="$(
  cat <<EOF
manifest_version: 1
scenario: $scenario
launch_arguments: [${launch_list%, }]
timing_mode: $timing
observation_window:
  requested_duration_s: $duration
  clock_domain: host wall clock around ros2 launch, from launch start including startup, not ROS or simulation time
synthetic:
  network_service: logical reservation execution, labeled LOGICAL, not measured packets
  observation_payload: metadata only, no sensor data
  inference: configured duration plus slowdown, no model
  actions: $actions
  task_outcome: $task_outcome
resolved_component_settings: recorded in the bag on /rovi/run/components
$source_block
install:
  prefix: $shown_install
  libraries_sha256: $libraries
inputs:
  params_sha256: $params_digest
  launch_sha256: $launch_digest
  fetched_model_sha256: $model_digest
EOF
)"
digest="$(printf '%s\n' "$body" | digest_of)"

mkdir -p -- "$out_dir"
printf '%s\ndigest_sha256: %s\n' "$body" "$digest" >"$out_dir/manifest.yaml"
cp -- "$params" "$out_dir/params.yaml"
echo "$digest"
