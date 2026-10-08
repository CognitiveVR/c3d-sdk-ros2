#!/usr/bin/env bash
# Rewrite, or check, every declaration of this SDK's version.
#
#   tools/bump_version.sh 1.1.0           rewrite every declaration from the current version to 1.1.0
#   tools/bump_version.sh --check 1.1.0   exit non-zero unless every declaration says 1.1.0
#
# The declarations are `kSdkVersion` in core/include/c3d/version.hpp, which is authoritative
# because it is what reaches the wire as `c3d.version`, and the package's own <version> in every
# package.xml directly under the repository root. Those are FOUND, not listed, so a package added
# later is covered without anyone remembering it. core/CMakeLists.txt derives project(VERSION)
# from version.hpp and is not a declaration. Nothing else is touched: not the docs, and not
# CHANGELOG.md.
#
# The order is bump, merge, then tag. The release workflow runs `--check` against the tag.
set -euo pipefail

usage() {
  echo "usage: $0 <x.y.z> | --check <x.y.z>" >&2
  exit 2
}

MODE=bump
if [ "${1:-}" = --check ]; then
  MODE=check
  shift
fi
[ $# -eq 1 ] || usage
NEW="$1"

# Three integers: package.xml's <version> and CMake's project(VERSION) accept nothing else.
if ! printf '%s\n' "$NEW" | grep -Eq '^[0-9]+\.[0-9]+\.[0-9]+$'; then
  echo "'$NEW' is not a version of the form x.y.z" >&2
  exit 2
fi

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HEADER="$REPO/core/include/c3d/version.hpp"
HEADER_PREFIX='constexpr const char* kSdkVersion = "'

# The declaration line itself, anchored at the line start, so a comment that mentions the
# constant cannot be read instead.
header_version() {
  awk -v prefix="$HEADER_PREFIX" '
    index($0, prefix) == 1 {
      rest = substr($0, length(prefix) + 1)
      end = index(rest, "\"")
      if (end > 0) { print substr(rest, 1, end - 1); found = 1; exit }
    }
    END { if (!found) exit 1 }' "$1"
}

# The FIRST <version> element, which in a package.xml is the package's own: a dependency's
# version constraint is an attribute and can never be read as this.
manifest_version() {
  awk '
    {
      start = index($0, "<version>")
      if (start > 0) {
        rest = substr($0, start + length("<version>"))
        end = index(rest, "</version>")
        if (end > 0) { print substr(rest, 1, end - 1); found = 1; exit }
      }
    }
    END { if (!found) exit 1 }' "$1"
}

[ -f "$HEADER" ] || { echo "no $HEADER" >&2; exit 1; }
MANIFESTS=()
for manifest in "$REPO"/*/package.xml; do
  [ -f "$manifest" ] && MANIFESTS+=("$manifest")
done
if [ ${#MANIFESTS[@]} -eq 0 ]; then
  echo "no package.xml found under $REPO: refusing to report on declarations it cannot see" >&2
  exit 1
fi

CURRENT="$(header_version "$HEADER")" || { echo "$HEADER does not declare kSdkVersion" >&2; exit 1; }

# Every declaration, relative path and value, one per line.
declarations() {
  printf '%s\t%s\n' "core/include/c3d/version.hpp" "$CURRENT"
  local manifest value
  for manifest in "${MANIFESTS[@]}"; do
    value="$(manifest_version "$manifest")" || value="(none)"
    printf '%s\t%s\n' "${manifest#"$REPO"/}" "$value"
  done
}

if [ "$MODE" = check ]; then
  status=0
  while IFS="$(printf '\t')" read -r path value; do
    if [ "$value" != "$NEW" ]; then
      echo "$path declares $value, not $NEW" >&2
      status=1
    fi
  done < <(declarations)
  if [ "$status" -eq 0 ]; then
    echo "every declaration says $NEW (version.hpp and ${#MANIFESTS[@]} package.xml)"
  else
    echo "kSdkVersion is authoritative. Bump with tools/bump_version.sh, merge, and only then tag." >&2
  fi
  exit "$status"
fi

# A bump moves one agreed version to another. A tree that already disagrees is reported, not
# silently made consistent: which value was meant is a question for whoever made the change.
disagree=0
while IFS="$(printf '\t')" read -r path value; do
  [ "$value" = "$CURRENT" ] || disagree=1
done < <(declarations)
if [ "$disagree" -ne 0 ]; then
  echo "the declarations disagree before the bump; fix them by hand first:" >&2
  declarations | sed 's/^/  /' >&2
  exit 1
fi
if [ "$CURRENT" = "$NEW" ]; then
  echo "every declaration already says $NEW: nothing to bump" >&2
  exit 1
fi

# Replace exactly the declaration, writing through a temporary file so an interrupted run leaves
# no half-written file behind.
rewrite_header() {
  local file="$1" tmp
  tmp="$(mktemp "$file.XXXXXX")"
  awk -v prefix="$HEADER_PREFIX" -v old="$CURRENT" -v new="$NEW" '
    !done && $0 == prefix old "\";" { print prefix new "\";"; done = 1; next }
    { print }
    END { if (!done) exit 1 }' "$file" > "$tmp" || { rm -f "$tmp"; echo "could not rewrite $file" >&2; exit 1; }
  mv "$tmp" "$file"
}

rewrite_manifest() {
  local file="$1" tmp
  tmp="$(mktemp "$file.XXXXXX")"
  awk -v old="<version>$CURRENT</version>" -v new="<version>$NEW</version>" '
    !done {
      at = index($0, old)
      if (at > 0) {
        $0 = substr($0, 1, at - 1) new substr($0, at + length(old))
        done = 1
      }
    }
    { print }
    END { if (!done) exit 1 }' "$file" > "$tmp" || { rm -f "$tmp"; echo "could not rewrite $file" >&2; exit 1; }
  mv "$tmp" "$file"
}

# mktemp creates 0600, and mv keeps that mode: restore the original's.
keep_mode() {
  local file="$1" mode
  mode="$(stat -c %a "$file" 2>/dev/null || stat -f %Lp "$file")"
  "$2" "$file"
  chmod "$mode" "$file"
}

keep_mode "$HEADER" rewrite_header
for manifest in "${MANIFESTS[@]}"; do keep_mode "$manifest" rewrite_manifest; done

"$BASH" "${BASH_SOURCE[0]}" --check "$NEW"
echo "bumped $CURRENT -> $NEW. Add the CHANGELOG.md section, merge, then tag v$NEW."
