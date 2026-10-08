#!/usr/bin/env bash
# Print one version's section of CHANGELOG.md, without its heading: the release notes for it.
#
#   tools/release/changelog_section.sh 1.0.0                 # from the repository's CHANGELOG.md
#   tools/release/changelog_section.sh 1.0.0 path/to/CHANGELOG.md
#   tools/release/changelog_section.sh --release https://github.com/<owner>/<repo> 1.0.0
#   tools/release/changelog_section.sh --release https://github.com/<owner>/<repo> --allow-undated 1.0.0
#
# The file is in Keep a Changelog form: a version's section starts at `## [x.y.z]`, optionally
# followed by ` - <date>`, and runs to the next `## ` heading or to the link reference
# definitions at the foot of the file. A version with no section, or an empty one, is an error,
# so a release cannot go out with blank notes.
#
# --release prints the notes as a GitHub release carries them, and refuses a section a release
# must not ship:
#   - a heading not dated `## [x.y.z] - YYYY-MM-DD`, such as `- Unreleased`. The tag's tarball
#     would carry that heading for good, and only moving the tag could change it.
#   - an HTML comment anywhere in the section. A release page hides it, so a note left for the
#     editor would ship unseen.
#   - a link out of the repository with `../`.
# A release page resolves a relative link against the release, not the tree, so every relative
# link is rewritten to <repository-url>/blob/v<x.y.z>/<path>, and a bare `#anchor` to one in
# CHANGELOG.md. Links in code, fenced or inline, are left as they are.
#
# --allow-undated, after --release, drops the date requirement and nothing else. It is for a dry
# run before release day; a tag never passes it.
set -euo pipefail

usage() {
  echo "usage: $0 [--release <repository-url> [--allow-undated]] <x.y.z> [CHANGELOG.md]" >&2
  exit 2
}

RELEASE_URL=""
ALLOW_UNDATED=0
if [ "${1:-}" = --release ]; then
  [ $# -ge 2 ] || usage
  RELEASE_URL="${2%/}"
  shift 2
  case "$RELEASE_URL" in
    https://?*) ;;
    *) echo "--release takes the repository's https:// URL, got '$RELEASE_URL'" >&2; exit 2 ;;
  esac
  if [ "${1:-}" = --allow-undated ]; then
    ALLOW_UNDATED=1
    shift
  fi
fi
[ "${1:-}" != --allow-undated ] || usage
[ $# -ge 1 ] && [ $# -le 2 ] || usage
VERSION="$1"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CHANGELOG="${2:-$REPO/CHANGELOG.md}"
[ -f "$CHANGELOG" ] || { echo "no $CHANGELOG" >&2; exit 1; }

# The heading is matched as a literal prefix, never as a pattern, so the dots in a version cannot
# match any character and 1.1.0 cannot select 1.1.0.1 or 11.1.0.
awk -v version="$VERSION" -v release_url="$RELEASE_URL" -v allow_undated="$ALLOW_UNDATED" '
  function absolute(target) {
    if (target ~ /^[A-Za-z][A-Za-z0-9+.-]*:/) return target
    if (target ~ /^#/) return blob "/CHANGELOG.md" target
    sub(/^\.\//, "", target)
    sub(/^\//, "", target)
    if (target ~ /^\.\.\//) escaped = escaped " " target
    return blob "/" target
  }
  function rewrite_links(text,    out) {
    out = ""
    while (match(text, /\]\([^)]*\)/)) {
      out = out substr(text, 1, RSTART + 1) absolute(substr(text, RSTART + 2, RLENGTH - 3)) ")"
      text = substr(text, RSTART + RLENGTH)
    }
    return out text
  }
  # Outside inline code only: the segments between backticks alternate text, code, text.
  function rewrite_line(line,    n, parts, i, out) {
    n = split(line, parts, "`")
    out = ""
    for (i = 1; i <= n; i++) {
      out = out (i % 2 == 1 ? rewrite_links(parts[i]) : parts[i])
      if (i < n) out = out "`"
    }
    return out
  }
  BEGIN { heading = "## [" version "]"; blob = release_url "/blob/v" version }
  inside && /^## / { exit }
  inside && /^\[[^]]+\]: / { exit }
  inside { lines[++n] = $0; next }
  index($0, heading) == 1 {
    rest = substr($0, length(heading) + 1)
    if (rest == "" || substr(rest, 1, 3) == " - ") { inside = 1; found = 1; found_heading = $0 }
  }
  END {
    if (!found) { print "no section for " version > "/dev/stderr"; exit 1 }
    first = 1
    while (first <= n && lines[first] ~ /^[[:space:]]*$/) first++
    last = n
    while (last >= first && lines[last] ~ /^[[:space:]]*$/) last--
    if (first > last) { print "the section for " version " is empty" > "/dev/stderr"; exit 1 }
    if (release_url != "") {
      if (allow_undated != 1 &&
          found_heading !~ /^## \[[^]]+\] - [0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9]$/) {
        print "the heading \"" found_heading "\" is not dated: a release needs \"" heading \
              " - YYYY-MM-DD\"" > "/dev/stderr"
        exit 1
      }
      for (i = first; i <= last; i++) {
        if (index(lines[i], "<!--") > 0) {
          print "the section for " version " holds an HTML comment, which a release page would " \
                "hide: " lines[i] > "/dev/stderr"
          exit 1
        }
      }
      fence = 0
      for (i = first; i <= last; i++) {
        if (lines[i] ~ /^[ ]*(```|~~~)/) fence = !fence
        else if (!fence) lines[i] = rewrite_line(lines[i])
      }
      if (escaped != "") {
        print "the section for " version " links out of the repository:" escaped > "/dev/stderr"
        exit 1
      }
    }
    for (i = first; i <= last; i++) print lines[i]
  }' "$CHANGELOG"
