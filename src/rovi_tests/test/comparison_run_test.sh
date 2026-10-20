#!/usr/bin/env bash
# Checks that run_comparison.sh keeps the report it hashed, reproduces its digest, and refuses bad runs.
set -euo pipefail

runner="$1"
work="$(mktemp -d)"
trap 'rm -rf -- "$work"' EXIT
fail() {
  echo "FAIL: $*" >&2
  exit 1
}
digest_of() { sha256sum | cut -d' ' -f1; }

# An install whose rovi_compare is a stub that echoes its arguments and fails like a usage error on --bad.
install="$work/install"
stub="$install/rovi_core/bin/rovi_compare"
mkdir -p "$(dirname "$stub")"
cat >"$stub" <<'EOF'
#!/usr/bin/env bash
for argument in "$@"; do
  if [[ "$argument" == --bad ]]; then
    exit 2
  fi
done
echo "harness: rovi_compare"
echo "arguments: $*"
EOF
chmod +x "$stub"

first="$("$runner" --install-base "$install" "$work/first" --robots 4 "note=it's")"
again="$("$runner" --install-base "$install" "$work/with space" --robots 4 "note=it's")"
[[ "$first" == "$again" ]] || fail "identical runs gave different digests"
manifest="$work/first/manifest.yaml"
grep -qx "digest_sha256: $first" "$manifest" || fail "the manifest does not record its digest"
grep -qx "arguments: --robots 4 note=it's" "$work/first/report.yaml" || fail "the report was not kept"
grep -qxF "arguments: ['--robots', '4', 'note=it''s']" "$manifest" || fail "arguments were not quoted"
grep -qx "report_sha256: $(digest_of <"$work/first/report.yaml")" "$manifest" ||
  fail "the report hash does not match the kept report"
grep -qx "binary_sha256: $(digest_of <"$stub")" "$manifest" || fail "the binary was not hashed"
grep -Eq "^  revision: [0-9a-f]{40}$" "$manifest" || fail "the source revision was not recorded"
grep -Eq "^  untracked_sha256: [0-9a-f]{64}$" "$manifest" || fail "the untracked source was not hashed"

other="$("$runner" --install-base "$install" "$work/other" --robots 5)"
[[ "$other" != "$first" ]] || fail "different arguments gave the same digest"

if "$runner" --install-base "$install" "$work/first" --robots 4 2>/dev/null; then
  fail "an existing output directory was overwritten"
fi
grep -qx "digest_sha256: $first" "$manifest" || fail "a refused run changed an existing manifest"
if "$runner" --install-base "$work/empty" "$work/no_binary" 2>/dev/null; then
  fail "an install without rovi_compare was accepted"
fi
if "$runner" --install-base "$install" "$work/failed" --bad 2>/dev/null; then
  fail "a failed comparison was accepted"
fi
if GIT_DIR=/nonexistent "$runner" --install-base "$install" "$work/no_git" 2>/dev/null; then
  fail "an unreadable source revision was accepted"
fi
status=0
"$runner" >/dev/null 2>&1 || status=$?
[[ "$status" == 2 ]] || fail "a run without arguments exited with $status instead of 2"
for refused in no_binary failed no_git; do
  [[ ! -e "$work/$refused" ]] || fail "the refused run $refused left output"
done
echo "comparison provenance checks passed"
