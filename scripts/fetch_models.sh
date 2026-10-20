#!/usr/bin/env bash
# Downloads the pinned Gazebo Fuel Panda into src/rovi_sim/models, strips remote textures, verifies it.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
models_dir="$repo_root/src/rovi_sim/models"
name=rovi_panda_fuel_v5
target="$models_dir/$name"
url="https://fuel.gazebosim.org/1.0/OpenRobotics/models/Panda%20with%20Ignition%20position%20controller%20model/5/Panda%20with%20Ignition%20position%20controller%20model.zip"
zip_sha256=86f3a9b45ee2a057ce3ff43f624fceb113e1554f6f2fb762dc6b52d85d497298
tree_sha256=48aae2c8350c7a6a90680f63a65add34a3bf94b94fcd6af3c0bf8d51c12c3908
max_bytes=100000000

tree_digest() {
  (cd "$1" && find . -type f -print0 | LC_ALL=C sort -z | xargs -0 sha256sum) | sha256sum | cut -d' ' -f1
}

if [[ -e "$target" ]]; then
  if [[ "$(tree_digest "$target")" == "$tree_sha256" ]]; then
    echo "$name is already present and verified in $target"
    exit 0
  fi
  echo "$target does not match the pinned tree checksum; remove it and re-run" >&2
  exit 1
fi

mkdir -p -- "$models_dir"
stage="$(mktemp -d "$models_dir/.${name}.XXXXXX")"
trap 'rm -rf -- "$stage"' EXIT
echo "downloading $name from $url"
curl --proto '=https' --proto-redir '=https' --tlsv1.2 --max-filesize "$max_bytes" --max-time 600 \
  -sSfL -o "$stage/model.zip" "$url"
if ! echo "$zip_sha256  $stage/model.zip" | sha256sum --check --quiet -; then
  echo "archive checksum mismatch for $name; nothing was installed" >&2
  exit 1
fi
unzip -q "$stage/model.zip" -d "$stage/model"
if [[ -n "$(find "$stage/model" ! -type f ! -type d -print -quit)" ]]; then
  echo "the archive contains links or special files; nothing was installed" >&2
  exit 1
fi
sed -i '/<environment_map>https:\/\/[^<]*<\/environment_map>/d' "$stage/model/model.sdf"
if [[ "$(tree_digest "$stage/model")" != "$tree_sha256" ]]; then
  echo "the unpacked model does not match the pinned tree checksum; nothing was installed" >&2
  exit 1
fi
mv -T -- "$stage/model" "$target"
echo "installed $name into $target"
