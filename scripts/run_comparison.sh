#!/usr/bin/env bash
# Runs rovi_compare from an install and writes its report and a provenance manifest into a new directory, then prints the manifest digest.
set -euo pipefail
export LC_ALL=C

usage() {
  echo "usage: $0 --install-base <prefix> <directory> [rovi_compare argument...]" >&2
  exit 2
}
die() {
  echo "$*" >&2
  exit 1
}

[[ $# -ge 3 && "$1" == --install-base ]] || usage
install_base="$2"
out_dir="$3"
shift 3
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[[ ! -e "$out_dir" ]] || die "refusing to overwrite $out_dir"

compare=""
for candidate in "$install_base/rovi_core/bin/rovi_compare" "$install_base/bin/rovi_compare"; do
  if [[ -x "$candidate" ]]; then
    compare="$candidate"
    break
  fi
done
[[ -n "$compare" ]] || die "no rovi_compare executable in $install_base"

digest_of() { sha256sum | cut -d' ' -f1; }
source_block="$("$repo_root/scripts/print_source_provenance.sh")"
binary_digest="$(digest_of <"$compare")"
report="$("$compare" "$@")"
report_digest="$(printf '%s\n' "$report" | digest_of)"
for value in "$binary_digest" "$report_digest"; do
  [[ "$value" =~ ^[0-9a-f]{64}$ ]] || die "could not hash the binary and its report"
done
arguments=""
for argument in "$@"; do
  shown="${argument//"$repo_root/"/<repo>/}"
  arguments+="'${shown//\'/\'\'}', "
done
shown_binary="$(cd "$(dirname "$compare")" && pwd)/rovi_compare"
shown_binary="${shown_binary/#"$repo_root/"/<repo>/}"

body="$(
  cat <<EOF
manifest_version: 1
harness: rovi_compare
arguments: [${arguments%, }]
binary: $shown_binary
binary_sha256: $binary_digest
report: report.yaml
report_sha256: $report_digest
$source_block
EOF
)"
digest="$(printf '%s\n' "$body" | digest_of)"

mkdir -p -- "$out_dir"
printf '%s\n' "$report" >"$out_dir/report.yaml"
printf '%s\ndigest_sha256: %s\n' "$body" "$digest" >"$out_dir/manifest.yaml"
echo "$digest"
