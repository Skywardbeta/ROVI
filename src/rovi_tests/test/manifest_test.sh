#!/usr/bin/env bash
# Checks that a run manifest is reproducible, what changes its digest, and which install bases and inputs it refuses.
set -euo pipefail

writer="$1"
repo="$(cd "$(dirname "$writer")/.." && pwd)"
params="$repo/src/rovi_bringup/config/vertical_slice.yaml"
work="$(mktemp -d)"
trap 'rm -rf -- "$work"' EXIT
fail() {
  echo "FAIL: $*" >&2
  exit 1
}

# A --symlink-install layout of this repository's manifests and launch files with a fetched model, and one built
# from another checkout.
install="$work/install"
stale="$work/stale"
for source_manifest in "$repo"/src/*/package.xml; do
  package="$(basename "$(dirname "$source_manifest")")"
  mkdir -p "$install/$package/share/$package" "$stale/$package/share/$package" "$work/elsewhere/$package"
  ln -s "$source_manifest" "$install/$package/share/$package/package.xml"
  cp "$source_manifest" "$work/elsewhere/$package/package.xml"
  ln -s "$work/elsewhere/$package/package.xml" "$stale/$package/share/$package/package.xml"
done
launches="$install/rovi_bringup/share/rovi_bringup/launch"
mkdir -p "$launches"
ln -s "$repo"/src/rovi_bringup/launch/*.launch.xml "$launches/"
touch "$install/setup.bash" "$stale/setup.bash"
model="$install/rovi_sim/share/rovi_sim/models/rovi_panda_fuel_v5"
mkdir -p "$model"
echo '<sdf version="1.9"/>' >"$model/model.sdf"
so101="$install/rovi_sim/share/rovi_sim/models/so101_new_calib"
mkdir -p "$so101"
echo '<model/>' >"$so101/model.config"

write_for() {
  local seconds="$1"
  shift
  "$writer" --install-base "$install" --duration "$seconds" "$@"
}
write() { write_for 60 "$@"; }

first="$(write vertical_slice "$work/first")"
again="$(write vertical_slice "$work/again")"
[[ "$first" == "$again" ]] || fail "identical inputs gave different digests"
cmp -s "$work/first/manifest.yaml" "$work/again/manifest.yaml" || fail "identical inputs gave different manifests"
grep -qx "digest_sha256: $first" "$work/first/manifest.yaml" || fail "the manifest does not record its digest"
grep -qx "timing_mode: ros_wall_time, free-running" "$work/first/manifest.yaml" || fail "wrong timing mode"
grep -qx "  requested_duration_s: 60" "$work/first/manifest.yaml" || fail "the requested duration was not recorded"
grep -q "^  clock_domain: host wall clock" "$work/first/manifest.yaml" || fail "the duration's clock was not recorded"
grep -qx "  prefix: $install" "$work/first/manifest.yaml" || fail "the install base was not recorded"
cmp -s "$params" "$work/first/params.yaml" || fail "the manifest did not keep the parameter file it hashed"

longer="$(write_for 61 vertical_slice "$work/longer")"
[[ "$longer" != "$first" ]] || fail "a duration change left the digest unchanged"

cp -- "$params" "$work/params.yaml"
copied="$(write vertical_slice "$work/copied" "params_file:=$work/params.yaml")"
sed -i 's/slowdown_ns: 0$/slowdown_ns: 1/' "$work/params.yaml"
slowed="$(write vertical_slice "$work/slowed" "params_file:=$work/params.yaml")"
[[ "$copied" != "$slowed" ]] || fail "a worker slowdown change left the digest unchanged"
cmp -s "$work/params.yaml" "$work/slowed/params.yaml" || fail "the slowed parameter file was not kept"

simulated="$(write vertical_slice "$work/simulated" use_sim_time:=True)"
[[ "$simulated" != "$first" ]] || fail "a timing mode change left the digest unchanged"
grep -q "^timing_mode: ros_sim_time" "$work/simulated/manifest.yaml" || fail "sim time not recorded"

spaced="$(write vertical_slice "$work/with space")"
[[ "$spaced" == "$first" ]] || fail "the output directory changed the digest"
write vertical_slice "$work/quoted" "note:=it's" "params_file:=$params" >/dev/null
grep -qxF "launch_arguments: ['note:=it''s', 'params_file:=<repo>/src/rovi_bringup/config/vertical_slice.yaml']" \
  "$work/quoted/manifest.yaml" || fail "launch arguments were not quoted or relativized"
write vertical_slice "$work/sibling" "note:=$repo-old/x" >/dev/null
grep -qxF "launch_arguments: ['note:=$repo-old/x']" "$work/sibling/manifest.yaml" ||
  fail "a sibling of the repository was shown as <repo>"

arm="$(write arm_workcell "$work/arm")"
grep -Eq "^  fetched_model_sha256: [0-9a-f]{64}$" "$work/arm/manifest.yaml" || fail "the installed model was not hashed"
echo '<!-- edited -->' >>"$model/model.sdf"
edited="$(write arm_workcell "$work/edited")"
[[ "$edited" != "$arm" ]] || fail "an installed model change left the digest unchanged"
printf 'frame' >"$work/frame.rovobs"
policy="$(write so101_policy "$work/policy" "observation_file:=$work/frame.rovobs")"
grep -Eq "^  observation_sha256: [0-9a-f]{64}$" "$work/policy/manifest.yaml" || fail "the recorded observation was not hashed"
grep -qx "  inference: real policy on the inference host, timed per request on /rovi/inference/records" \
  "$work/policy/manifest.yaml" || fail "real inference was not declared"
grep -qx "  observation_sha256: none" "$work/first/manifest.yaml" || fail "a synthetic run declared an observation"
sim="$(write so101_sim "$work/sim" "observation_file:=$work/frame.rovobs")"
grep -Eq "^  fetched_model_sha256: [0-9a-f]{64}$" "$work/sim/manifest.yaml" || fail "the SO-101 model was not hashed"
grep -Eq "^  observation_sha256: [0-9a-f]{64}$" "$work/sim/manifest.yaml" || fail "the simulated run's observation was not hashed"
grep -q "^timing_mode: gazebo_sim_time" "$work/sim/manifest.yaml" || fail "the SO-101 simulation's timing was not recorded"
grep -qx "  actions: policy action chunks played open loop onto a simulated SO-101" "$work/sim/manifest.yaml" ||
  fail "open-loop simulated playback was not declared"
echo '<!-- edited -->' >>"$so101/model.config"
resimulated="$(write so101_sim "$work/resimulated" "observation_file:=$work/frame.rovobs")"
[[ "$resimulated" != "$sim" ]] || fail "an SO-101 model change left the digest unchanged"
printf 'other' >"$work/frame.rovobs"
reframed="$(write so101_policy "$work/reframed" "observation_file:=$work/frame.rovobs")"
[[ "$reframed" != "$policy" ]] || fail "a changed recorded observation left the digest unchanged"
mkdir -p "$install/rovi_network/lib"
echo build-1 >"$install/rovi_network/lib/librovi_network_component.so"
built="$(write vertical_slice "$work/built")"
echo build-2 >"$install/rovi_network/lib/librovi_network_component.so"
rebuilt="$(write vertical_slice "$work/rebuilt")"
[[ "$built" != "$rebuilt" ]] || fail "a changed installed library left the digest unchanged"

for scenario in so101_policy so101_sim; do
  if write "$scenario" "$work/no_observation_$scenario" 2>/dev/null; then
    fail "$scenario was accepted without a recorded observation"
  fi
done
if write vertical_slice "$work/missing" params_file:=/nonexistent.yaml 2>/dev/null; then
  fail "a missing parameter file was accepted"
fi
if GIT_DIR=/nonexistent write vertical_slice "$work/no_git" 2>/dev/null; then
  fail "an unreadable source revision was accepted"
fi
if write vertical_slice "$work/odd" use_sim_time:=maybe 2>/dev/null; then
  fail "an unrecognized use_sim_time value was accepted"
fi
if "$writer" --install-base "$install" vertical_slice "$work/no_duration" 2>/dev/null; then
  fail "a run without a requested duration was accepted"
fi
if write_for 0 vertical_slice "$work/zero" 2>/dev/null; then
  fail "a zero duration was accepted"
fi
if "$writer" --install-base "$stale" --duration 60 vertical_slice "$work/stale_run" 2>/dev/null; then
  fail "an install built from another checkout was accepted"
fi
cp -a -- "$install" "$work/replaced"
replaced="$work/replaced/rovi_bringup/share/rovi_bringup/launch/arm_workcell.launch.xml"
rm -- "$replaced"
cp -- "$repo/src/rovi_bringup/launch/arm_workcell.launch.xml" "$replaced"
echo "<!-- replaced after the build -->" >>"$replaced"
if "$writer" --install-base "$work/replaced" --duration 60 arm_workcell "$work/replaced_run" 2>/dev/null; then
  fail "an install whose launch file was replaced was accepted"
fi
cp -a -- "$install" "$work/unlaunched"
rm -- "$work/unlaunched/rovi_bringup/share/rovi_bringup/launch/vertical_slice.launch.xml"
if "$writer" --install-base "$work/unlaunched" --duration 60 vertical_slice "$work/unlaunched_run" 2>/dev/null; then
  fail "an install without the scenario launch file was accepted"
fi
cp -a -- "$install" "$work/partial"
rm -r -- "$work/partial/rovi_worker"
if "$writer" --install-base "$work/partial" --duration 60 vertical_slice "$work/partial_run" 2>/dev/null; then
  fail "an install without every package was accepted"
fi
ln -s "$work/nonexistent" "$model/meshes"
if write arm_workcell "$work/dangling" 2>/dev/null; then
  fail "an installed model with a dangling link was accepted"
fi
rm -- "$model/meshes" "$model/model.sdf" "$so101/model.config"
for scenario in arm_workcell thirty_arms so101_sim; do
  if write "$scenario" "$work/no_model_$scenario" "observation_file:=$work/frame.rovobs" 2>/dev/null; then
    fail "$scenario was accepted without its installed model"
  fi
done
for refused in no_observation_so101_policy no_observation_so101_sim missing no_git odd no_duration zero stale_run \
  partial_run replaced_run unlaunched_run dangling no_model_arm_workcell no_model_thirty_arms no_model_so101_sim; do
  [[ ! -e "$work/$refused" ]] || fail "the refused run $refused left a manifest"
done
echo "manifest provenance checks passed"
