#!/usr/bin/env bash
# Downloads the pinned SO-101 arm (TheRobotStudio/SO-ARM100) into src/rovi_sim/models and verifies it.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
models_dir="$repo_root/src/rovi_sim/models"
name=so101_new_calib
target="$models_dir/$name"
commit=a758567c3978dfeefe282ede0500085a48fe8f78
base="https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/$commit"
max_bytes=20000000

# Installed path and SHA-256; each file comes from Simulation/SO101/ in the repository, except LICENSE.
pins=(
  "LICENSE c71d239df91726fc519c6eb72d318ec65820627232b2f796219e87dcf35d0ab4"
  "so101_new_calib.urdf 3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c"
  "assets/base_motor_holder_so101_v1.stl 8cd2f241037ea377af1191fffe0dd9d9006beea6dcc48543660ed41647072424"
  "assets/base_so101_v2.stl bb12b7026575e1f70ccc7240051f9d943553bf34e5128537de6cd86fae33924d"
  "assets/motor_holder_so101_base_v1.stl 31242ae6fb59d8b15c66617b88ad8e9bded62d57c35d11c0c43a70d2f4caa95b"
  "assets/motor_holder_so101_wrist_v1.stl 887f92e6013cb64ea3a1ab8675e92da1e0beacfd5e001f972523540545e08011"
  "assets/moving_jaw_so101_v1.stl 785a9dded2f474bc1d869e0d3dae398a3dcd9c0c345640040472210d2861fa9d"
  "assets/rotation_pitch_so101_v1.stl 9be900cc2a2bf718102841ef82ef8d2873842427648092c8ed2ca1e2ef4ffa34"
  "assets/sts3215_03a_no_horn_v1.stl 75ef3781b752e4065891aea855e34dc161a38a549549cd0970cedd07eae6f887"
  "assets/sts3215_03a_v1.stl a37c871fb502483ab96c256baf457d36f2e97afc9205313d9c5ab275ef941cd0"
  "assets/under_arm_so101_v1.stl d01d1f2de365651dcad9d6669e94ff87ff7652b5bb2d10752a66a456a86dbc71"
  "assets/upper_arm_so101_v1.stl 475056e03a17e71919b82fd88ab9a0b898ab50164f2a7943652a6b2941bb2d4f"
  "assets/waveshare_mounting_plate_so101_v2.stl e197e24005a07d01bbc06a8c42311664eaeda415bf859f68fa247884d0f1a6e9"
  "assets/wrist_roll_follower_so101_v1.stl 4b17b410a12d64ec39554abc3e8054d8a97384b2dc4a8d95a5ecb2a93670f5f4"
  "assets/wrist_roll_pitch_so101_v2.stl 6c7ec5525b4d8b9e397a30ab4bb0037156a5d5f38a4adf2c7d943d6c56eda5ae"
)

model_config() {
  printf '%s\n' '<?xml version="1.0"?>' '<model>' "  <name>$name</name>" '  <version>1.0</version>' \
    '  <sdf version="1.9">so101_new_calib.urdf</sdf>' '</model>'
}

verified() {
  local dir="$1" pin expected actual
  expected="$( (echo model.config; for pin in "${pins[@]}"; do echo "${pin%% *}"; done) | LC_ALL=C sort)"
  actual="$(cd "$dir" && find . ! -type d -printf '%P\n' | LC_ALL=C sort)"
  [[ "$actual" == "$expected" ]] || return 1
  [[ -z "$(find "$dir" ! -type d ! -type f -print -quit)" ]] || return 1
  model_config | cmp -s - "$dir/model.config" || return 1
  for pin in "${pins[@]}"; do
    echo "${pin##* }  $dir/${pin%% *}"
  done | sha256sum --check --quiet --strict -
}

if [[ -e "$target" ]]; then
  if verified "$target"; then
    echo "$name is already present and verified in $target"
    exit 0
  fi
  echo "$target does not match the pinned files; remove it and re-run" >&2
  exit 1
fi

mkdir -p -- "$models_dir"
stage="$(mktemp -d "$models_dir/.${name}.XXXXXX")"
trap 'rm -rf -- "$stage"' EXIT
mkdir -p -- "$stage/model/assets"
echo "downloading $name from $base"
for pin in "${pins[@]}"; do
  path="${pin%% *}"
  remote_path="Simulation/SO101/$path"
  [[ "$path" == LICENSE ]] && remote_path=LICENSE
  curl --proto '=https' --proto-redir '=https' --tlsv1.2 --max-filesize "$max_bytes" --max-time 300 \
    -sSfL -o "$stage/model/$path" "$base/$remote_path"
done
model_config > "$stage/model/model.config"
if ! verified "$stage/model"; then
  echo "the downloaded files do not match the pinned checksums; nothing was installed" >&2
  exit 1
fi
mv -T -- "$stage/model" "$target"
echo "installed $name into $target"
