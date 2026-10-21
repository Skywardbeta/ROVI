#!/usr/bin/env bash
# Prints the source block of a ROVI run manifest: the checkout's revision, dirty flag, and hashes of its uncommitted and untracked source.
set -euo pipefail
export LC_ALL=C

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
digest_of() { sha256sum | cut -d' ' -f1; }
git_in() { git -C "$repo_root" "$@"; }

revision="$(git_in rev-parse --verify HEAD)"
dirty=false
status="$(git_in status --porcelain)"
[[ -z "$status" ]] || dirty=true
tracked="$(git_in -c core.quotepath=off diff HEAD --binary --full-index --no-ext-diff --no-textconv \
  --no-color --src-prefix=a/ --dst-prefix=b/ -- src scripts adapters | digest_of)"
untracked="$(git_in ls-files --others --exclude-standard -z -- src scripts adapters | sort -z |
  (cd "$repo_root" && xargs -0 -r sha256sum) | digest_of)"
for value in "$tracked" "$untracked"; do
  if [[ ! "$value" =~ ^[0-9a-f]{64}$ ]]; then
    echo "could not hash the source" >&2
    exit 1
  fi
done
cat <<EOF
source:
  revision: $revision
  dirty: $dirty
  tracked_diff_sha256: $tracked
  untracked_sha256: $untracked
EOF
