#!/usr/bin/env bash
# Build and test everything, in a container, on a host that has no ROS.
#
# `core/` is built standalone with plain CMake -- that is the ROS-free guarantee
# exercised on every run, not merely asserted in a comment. The two
# ament packages are then built with colcon. Both build OUT of the source tree so
# nothing lands in the repo.
#
#   tools/build_and_test.sh              # everything
#   tools/build_and_test.sh core         # core only (fast: no ROS packages)
#   tools/build_and_test.sh ros          # the ament packages only
#   tools/build_and_test.sh synthetic    # the capture tier only
#   tools/build_and_test.sh humble       # core and the ament packages on ROS 2 Humble
#
# Every other selector runs on Jazzy. `humble` exists because the two distros'
# APIs differ in ways a Jazzy build cannot see:
# `LifecycleNode::get_fully_qualified_name()` compiles on Jazzy and does not exist
# on Humble. It builds its own image, from the same Dockerfile with a Humble
# base, and runs core and every ament package's tests there. It does NOT run the
# capture tier.
#
# The synthetic tier runs in a container of its OWN -- it is the one that stands
# up a ROS graph and a stub gateway -- so `all` builds the workspace twice. That
# is the price of the read-only source mount, and it buys the only check in the
# tree that can see what reaches the wire: the ament tests cannot decode a
# Cap'n Proto payload, by design, so the pose a laserscan record carries is
# observable there and nowhere else.
#
# The image is built from docker/Dockerfile.test on first use and cached after
# that. It resolves its ROS dependencies with `rosdep install` against this
# repo's own package manifests, so an undeclared dependency fails the image
# build rather than passing here and failing on a partner's clean checkout.
#
# The tag is derived from a hash of the Dockerfile and the package manifests, so
# a manifest edit rebuilds on its own. IMAGE overrides the whole thing;
# REBUILD_IMAGE=1 forces a rebuild of whatever tag is in play.
#
# Superseded tags stay TAGGED, so `docker image prune` will not reclaim them.
# Clear them out with:
#   docker images 'c3d-sdk-ros2-test' --format '{{.Repository}}:{{.Tag}}' | xargs -r docker rmi
set -euo pipefail

WHAT="${1:-all}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# A selector this script does not recognise must not be a successful no-op.
# `build_and_test.sh cores` matching no branch, running nothing and exiting 0
# would print a completion banner over work that never happened.
case "$WHAT" in
  all|core|ros|synthetic|humble) ;;
  *) echo "unknown selector '$WHAT' (want: all | core | ros | synthetic | humble)" >&2; exit 2 ;;
esac

# Jazzy unless the selector says otherwise. BUILD_ARGS feeds both the image build
# and the tag hash, so the Humble image can never be mistaken for the Jazzy one.
IMAGE_REPO=c3d-sdk-ros2-test
BUILD_ARGS=()
if [ "$WHAT" = humble ]; then
  IMAGE_REPO=c3d-sdk-ros2-test-humble
  BUILD_ARGS=(--build-arg ROS_IMAGE=ros:humble-ros-base-jammy --build-arg CAPTURE_TOOLS=0)
fi

# The tag carries a hash of everything the image is built FROM, so editing a
# manifest mints a new tag and rebuilds automatically. A plain "does the tag
# exist" check would defeat the point of resolving dependencies from the
# manifests: add `<depend>` to a package.xml, keep the stale image, and colcon
# fails with "not found" as though the manifest were still wrong.
# `shasum` is Perl's Digest::SHA front end and Debian-family images ship
# perl-base without it, while `sha256sum` is absent on macOS -- so a script whose
# whole purpose is "a host that has no ROS", i.e. a container or a CI runner,
# needs both. GitHub's ubuntu runners happen to carry perl, so CI alone does not
# show that either one is missing.
digest() {
  if command -v shasum >/dev/null 2>&1; then shasum -a 256; else sha256sum; fi
}

if [ -z "${IMAGE:-}" ]; then
  # An array, not a space-joined string: a partner cloning into
  # "~/My Projects/c3d-sdk-ros2" is exactly the case this script exists for, and
  # word-splitting there fails with `cat: .../My: No such file`, which reads as a
  # broken checkout rather than a quoting bug.
  IMAGE_INPUTS=("$REPO/docker/Dockerfile.test")
  while IFS= read -r manifest; do IMAGE_INPUTS+=("$manifest"); done < <(
    find "$REPO" -mindepth 2 -maxdepth 2 -name package.xml -not -path '*/.git/*' | sort
  )
  # The build args join the hash only when there are any, so the Jazzy tag stays the
  # one run_synthetic_integration.sh computes on its own from the same inputs.
  IMAGE_TAG="$({ cat "${IMAGE_INPUTS[@]}"
                 [ ${#BUILD_ARGS[@]} -eq 0 ] || printf '%s\n' "${BUILD_ARGS[@]}"; } |
               digest | cut -c1-12)"
  IMAGE="$IMAGE_REPO:$IMAGE_TAG"
fi

if [ -n "${REBUILD_IMAGE:-}" ] || ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "=== building $IMAGE (first run takes a few minutes) ==="
  # The `+` form, because bash 3.2 -- macOS's -- treats an empty array as unset
  # under `set -u`.
  docker build ${BUILD_ARGS[@]+"${BUILD_ARGS[@]}"} -f "$REPO/docker/Dockerfile.test" \
    -t "$IMAGE" "$REPO"
fi

# The capture tier is its own entry point and resolves this same image, so it is
# run from out here rather than from inside the container below: a container has
# no docker to start one with.
synthetic() {
  echo "=== synthetic integration: the ROS graph, the stub gateway and the captures ==="
  IMAGE="$IMAGE" bash "$REPO/tools/run_synthetic_integration.sh"
}

if [ "$WHAT" = synthetic ]; then
  synthetic
  exit 0
fi

# The source mount is read-only on purpose: a build that needs to write into the
# source tree is a build a partner cannot run from a package.
docker run --rm -v "$REPO":/src:ro -e WHAT="$WHAT" "$IMAGE" bash -lc '
set -euo pipefail

# Runs a command with an EMPTY environment and a PATH that contains no ROS.
#
# Not sourcing setup.bash is not enough. The base image exports AMENT_PREFIX_PATH
# and puts /opt/ros/jazzy/bin on PATH, and CMake derives a package search prefix
# from every PATH entry ending in /bin -- so `find_package(rclcpp REQUIRED)` inside
# core/ CONFIGURES SUCCESSFULLY in a shell that never sourced ROS. Adding that
# line to core/CMakeLists.txt passes a check that only skips setup.bash, and
# fails this one.
ros_free() {
  env -i PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin HOME=/root "$@"
}

if [ "$WHAT" = all ] || [ "$WHAT" = core ] || [ "$WHAT" = humble ]; then
  echo "=== core: standalone CMake, with NO ROS in the environment ==="
  ros_free cmake -S /src/core -B /tmp/b-core -DCMAKE_BUILD_TYPE=Release >/dev/null
  ros_free cmake --build /tmp/b-core -j"$(nproc)" >/dev/null
  ( cd /tmp/b-core && ros_free ctest --output-on-failure )
fi

if [ "$WHAT" = all ] || [ "$WHAT" = ros ] || [ "$WHAT" = humble ]; then
  echo "=== ament packages: colcon, ROS 2 $ROS_DISTRO ==="
  # -u off across the source only: setup.bash reads AMENT_TRACE_SETUP_FILES
  # unguarded and aborts the script under nounset.
  set +u
  source "/opt/ros/$ROS_DISTRO/setup.bash"
  set -u
  mkdir -p /tmp/ws/src
  # Every ament package, FOUND not listed -- a hardcoded list goes stale when a
  # package is added, and the failure reads as a missing dependency rather than a
  # missing copy. core/ comes along because cognitive3d_ros builds it as a
  # CMake subproject; its own COLCON_IGNORE keeps colcon from building it twice.
  # tools/ and the notices file are installed from beside the packages, docs/ is
  # checked by test_parameter_reference, and tools/ is no package at all.
  find /src -mindepth 2 -maxdepth 2 -name package.xml -not -path "*/.git/*" \
    -exec dirname {} \; | xargs -I{} cp -r {} /tmp/ws/src/
  cp -r /src/core /src/tools /src/docs /src/THIRD_PARTY_NOTICES.md /tmp/ws/src/
  touch /tmp/ws/src/tools/COLCON_IGNORE
  cd /tmp/ws
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
  # test-result runs either way. Under `set -e` a failing `colcon test` would end
  # the script here, so the one run that needs the per-test detail would print
  # only "1 package had test failures".
  colcon test --return-code-on-test-failure && ament_status=0 || ament_status=$?
  colcon test-result --verbose
  [ "$ament_status" -eq 0 ] || exit "$ament_status"
fi
'

if [ "$WHAT" = all ]; then synthetic; fi
