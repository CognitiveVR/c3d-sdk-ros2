#!/usr/bin/env bash
# Refresh, or verify, the Cap'n Proto schemas vendored under core/capnp/.
#
# Cognitive3D maintains the canonical copies of these schemas. This SDK needs them
# to serialise the camera and lidar bodies and the platform needs them to read
# those bodies back, so a divergence between the two is a wire break that no test
# on either side can see alone. The bytes are pinned here, and the pin is checked
# in CI (tools/test_capnp_vendored.py). For maintainers with the canonical copies.
#
#   tools/sync_capnp.sh <schema-dir>            copy in, rewrite CHECKSUMS
#   tools/sync_capnp.sh --check <schema-dir>    report drift, exit 1 if any
set -euo pipefail

# The schemas this SDK writes, and nothing else the canonical directory holds. The
# list is explicit: a schema joins it in the change that starts writing it, and
# test_capnp_vendored.py names the same files. Alphabetical, so CHECKSUMS is
# byte-stable across machines and shells.
SCHEMAS=(camera_payload.capnp common_payload.capnp java.capnp lidar_payload.capnp pose_payload.capnp)

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENDOR_DIR="$REPO_ROOT/core/capnp"

usage() {
  sed -n '2,11p' "${BASH_SOURCE[0]}" | sed -e 's/^#$//' -e 's/^# //'
  exit 2
}

# shasum is the portable spelling (macOS has no sha256sum); both print "<hash>  <name>",
# so CHECKSUMS stays readable by `shasum -a 256 -c` from inside core/capnp/.
sha256() {
  if command -v shasum >/dev/null 2>&1; then
    shasum -a 256 "$@"
  else
    sha256sum "$@"
  fi
}

listed() {
  local f="$1" s
  for s in "${SCHEMAS[@]}"; do
    [ "$f" = "$s" ] && return 0
  done
  return 1
}

MODE=sync
if [ "${1:-}" = "--check" ]; then
  MODE=check
  shift
elif [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
  usage
fi

SOURCE_DIR="${1:-}"
[ -n "$SOURCE_DIR" ] || usage
if [ ! -d "$SOURCE_DIR" ]; then
  echo "not a directory: $SOURCE_DIR" >&2
  exit 1
fi

missing=0
for f in "${SCHEMAS[@]}"; do
  if [ ! -f "$SOURCE_DIR/$f" ]; then
    echo "no $f in $SOURCE_DIR" >&2
    missing=1
  fi
done
if [ "$missing" -ne 0 ]; then
  echo "$SOURCE_DIR is not the canonical schema directory, or it no longer offers a schema this SDK writes" >&2
  exit 1
fi

if [ "$MODE" = check ]; then
  drift=0

  # The vendored bytes against the recorded hashes: catches a hand edit here.
  if [ -f "$VENDOR_DIR/CHECKSUMS" ]; then
    if ! (cd "$VENDOR_DIR" && sha256 -c CHECKSUMS >/dev/null 2>&1); then
      echo "core/capnp/ does not match core/capnp/CHECKSUMS:" >&2
      (cd "$VENDOR_DIR" && sha256 -c CHECKSUMS 2>&1 | grep -v ': OK$' >&2) || true
      drift=1
    fi
  else
    echo "core/capnp/CHECKSUMS is missing; run tools/sync_capnp.sh <schema-dir> to write it" >&2
    drift=1
  fi

  # The vendored bytes against the canonical copies: catches a change made there.
  for f in "${SCHEMAS[@]}"; do
    if [ ! -f "$VENDOR_DIR/$f" ]; then
      echo "not vendored: $SOURCE_DIR/$f has no copy in core/capnp/" >&2
      drift=1
    elif ! cmp -s "$SOURCE_DIR/$f" "$VENDOR_DIR/$f"; then
      echo "drift: $f differs from $SOURCE_DIR/$f" >&2
      drift=1
    fi
  done

  # And the mirror of that: a schema vendored here that the list no longer names.
  for path in "$VENDOR_DIR"/*.capnp; do
    [ -f "$path" ] || continue
    f="$(basename "$path")"
    if ! listed "$f"; then
      echo "stale: core/capnp/$f is not a schema this SDK vendors" >&2
      drift=1
    fi
  done

  if [ "$drift" -ne 0 ]; then
    echo >&2
    echo "A schema change starts in the canonical copy. Change it there, then re-run" >&2
    echo "tools/sync_capnp.sh <schema-dir> and commit the result here." >&2
    exit 1
  fi
  echo "core/capnp/ matches $SOURCE_DIR and its own CHECKSUMS (${#SCHEMAS[@]} files)"
  exit 0
fi

mkdir -p "$VENDOR_DIR"
for f in "${SCHEMAS[@]}"; do
  cp "$SOURCE_DIR/$f" "$VENDOR_DIR/$f"
done
(cd "$VENDOR_DIR" && sha256 "${SCHEMAS[@]}" > CHECKSUMS)

echo "vendored ${#SCHEMAS[@]} files from $SOURCE_DIR into core/capnp/, CHECKSUMS rewritten:"
cat "$VENDOR_DIR/CHECKSUMS"
