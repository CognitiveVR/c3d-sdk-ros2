#!/usr/bin/env bash
# Build this repository's .debs for one ROS 2 distro, with bloom, inside that distro's ros-base
# image, for the architecture of the machine it runs on.
#
#   tools/release/build_debs.sh humble <out-dir>
#   tools/release/build_debs.sh jazzy  <out-dir>
#
# The debs are written to <out-dir> and nowhere else: the repository is mounted read-only and
# built from a copy inside the container. ROS_IMAGE overrides the base image.
#
# The recipe is bloom's own: `bloom-generate rosdebian`, then `fakeroot debian/rules binary`, one
# package at a time in dependency order, installing each deb before the next package is
# generated, because bloom resolves a package's dependencies to installed system packages.
# tools/release/rosdep.yaml supplies the rosdep keys for this repository's own packages, which no
# public rosdep source has. Cap'n Proto has no rosdep key either, so it is installed by name.
#
# One step is not bloom's: each package's dependency on another package of this repository is
# pinned to the exact version of the deb built for it, so apt never pairs debs of two releases.
# A package.xml cannot say this, because bloom writes `version_eq` as the bare version, which no
# deb versioned `<version>-0<codename>` satisfies; tools/release/pin_control.py writes it instead.
#
# The packages are built from a full checkout: cognitive3d_ros builds core/ as a CMake
# subproject and installs files from tools/ and the repository root, so a package directory on
# its own does not configure.
set -euo pipefail

usage() {
  echo "usage: $0 <humble|jazzy> <out-dir>" >&2
  exit 2
}

if [ "${1:-}" = --in-container ]; then
  IN_CONTAINER=1
  shift
else
  IN_CONTAINER=0
fi

DISTRO="${1:-}"
case "$DISTRO" in
  humble) UBUNTU=jammy ;;
  jazzy) UBUNTU=noble ;;
  *) usage ;;
esac

if [ "$IN_CONTAINER" = 0 ]; then
  [ $# -eq 2 ] || usage
  REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
  mkdir -p "$2"
  OUT="$(cd "$2" && pwd)"
  IMAGE="${ROS_IMAGE:-ros:$DISTRO-ros-base-$UBUNTU}"
  echo "=== building the $DISTRO debs in $IMAGE into $OUT ==="
  exec docker run --rm \
    -v "$REPO":/src:ro \
    -v "$OUT":/out \
    -e OUT_OWNER="$(id -u):$(id -g)" \
    "$IMAGE" bash /src/tools/release/build_debs.sh --in-container "$DISTRO"
fi

# --- inside the container from here on ---

[ "$ROS_DISTRO" = "$DISTRO" ] || { echo "image is $ROS_DISTRO, asked for $DISTRO" >&2; exit 1; }
export DEBIAN_FRONTEND=noninteractive
step() { echo; echo "=== $* ($(date -u +%H:%M:%S))"; }

step "build tools"
apt-get update -qq
apt-get install -y -qq --no-install-recommends \
  python3-bloom python3-rosdep fakeroot dpkg-dev debhelper \
  capnproto libcapnp-dev > /tmp/apt.log 2>&1 || { tail -30 /tmp/apt.log; exit 1; }

step "stage the source"
WORK=/work
mkdir -p "$WORK"
packages=0
for manifest in /src/*/package.xml; do
  [ -f "$manifest" ] || continue
  cp -r "$(dirname "$manifest")" "$WORK/"
  packages=$((packages + 1))
done
[ "$packages" -gt 0 ] || { echo "no package.xml under /src" >&2; exit 1; }
# What the packages read from outside their own directories.
for extra in core tools LICENSE THIRD_PARTY_NOTICES.md; do
  cp -r "/src/$extra" "$WORK/"
done
# A deb is built from what the manifests declare, so stale bloom output must not be reused.
rm -rf "$WORK"/*/debian "$WORK"/*/.obj-*

step "rosdep"
echo "yaml file://$WORK/tools/release/rosdep.yaml" > /etc/ros/rosdep/sources.list.d/10-cognitive3d.list
rosdep update --rosdistro "$DISTRO" > /tmp/rosdep_update.log 2>&1 || { tail -30 /tmp/rosdep_update.log; exit 1; }
rosdep install --from-paths "$WORK" --ignore-src -y --rosdistro "$DISTRO" \
  > /tmp/rosdep_install.log 2>&1 || { tail -40 /tmp/rosdep_install.log; exit 1; }

# Dependency order, from the manifests: each package's deb must be installed before a package
# that depends on it is generated. Each line also names the run dependencies the package has on
# this repository's other packages, or `-` for none.
ORDER="$(python3 -c '
import os, sys
from catkin_pkg.topological_order import topological_order
ordered = topological_order(sys.argv[1])
ours = {package.name for _, package in ordered}
for path, package in ordered:
    package.evaluate_conditions(os.environ)
    run = package.exec_depends + package.build_export_depends + package.buildtool_export_depends
    deps = sorted({d.name for d in run if d.name in ours and d.evaluated_condition is not False})
    print(path, package.name, ",".join(deps) or "-")
' "$WORK")"
echo "$ORDER"

deb_name_of() { printf 'ros-%s-%s' "$DISTRO" "$(printf '%s' "$1" | tr '_' '-')"; }

built=()
declare -A built_version=()
while read -r path name deps; do
  step "$name"
  pins=()
  if [ "$deps" != - ]; then
    IFS=, read -r -a dep_names <<< "$deps"
    for dep in "${dep_names[@]}"; do
      [ -n "${built_version[$dep]:-}" ] || { echo "$name needs $dep, which is not built" >&2; exit 1; }
      pins+=("$(deb_name_of "$dep")=${built_version[$dep]}")
    done
  fi
  (
    cd "$WORK/$path"
    bloom-generate rosdebian --ros-distro "$DISTRO" > "/tmp/$name.bloom.log" 2>&1 ||
      { tail -30 "/tmp/$name.bloom.log"; exit 1; }
    if [ "${#pins[@]}" -gt 0 ]; then
      python3 "$WORK/tools/release/pin_control.py" debian/control "${pins[@]}"
    fi
    # nocheck: bloom's rules would build the test suites and then ignore their result. The suites
    # are run by tools/build_and_test.sh, not here.
    DEB_BUILD_OPTIONS=nocheck fakeroot debian/rules binary > "/tmp/$name.build.log" 2>&1 ||
      { tail -60 "/tmp/$name.build.log"; exit 1; }
  )
  deb="$(ls "$WORK/$(deb_name_of "$name")_"*.deb)"
  apt-get install -y -qq "$deb" > "/tmp/$name.install.log" 2>&1 ||
    { tail -30 "/tmp/$name.install.log"; exit 1; }
  built+=("$deb")
  built_version[$name]="$(dpkg-deb -f "$deb" Version)"
done <<< "$ORDER"

[ "${#built[@]}" -eq "$packages" ] || { echo "built ${#built[@]} debs for $packages packages" >&2; exit 1; }

step "debs"
for deb in "${built[@]}"; do
  cp "$deb" /out/
  echo "$(basename "$deb")  $(du -k "$deb" | cut -f1) KB"
  dpkg-deb -f "$deb" Depends | tr ',' '\n' | sed 's/^ */    /'
done
[ -z "${OUT_OWNER:-}" ] || chown "$OUT_OWNER" /out/*.deb
