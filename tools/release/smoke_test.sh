#!/usr/bin/env bash
# Install one distro's debs into a fresh ros-base container and check what a robot would get.
#
#   tools/release/smoke_test.sh humble <deb-dir>
#   tools/release/smoke_test.sh jazzy  <deb-dir>
#
# <deb-dir> is what tools/release/build_debs.sh wrote. Every check runs and reports before the
# exit code is decided, so one failure does not hide the next. Nothing here contacts a
# Cognitive3D backend: the node is only constructed, never configured, and every tool is asked
# for its usage. ROS_IMAGE overrides the base image.
set -euo pipefail

usage() {
  echo "usage: $0 <humble|jazzy> <deb-dir>" >&2
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
  [ -d "$2" ] || { echo "no such directory: $2" >&2; exit 2; }
  DEBS="$(cd "$2" && pwd)"
  HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  IMAGE="${ROS_IMAGE:-ros:$DISTRO-ros-base-$UBUNTU}"
  echo "=== smoke-testing the $DISTRO debs in $DEBS on a fresh $IMAGE ==="
  exec docker run --rm -v "$DEBS":/debs:ro -v "$HERE":/release:ro \
    "$IMAGE" bash /release/smoke_test.sh --in-container "$DISTRO"
fi

# --- inside the container from here on ---

export DEBIAN_FRONTEND=noninteractive
PREFIX="/opt/ros/$DISTRO"
failures=0
pass() { echo "  ok    $1"; }
fail() { echo "  FAIL  $1"; failures=$((failures + 1)); }
check() {
  local label="$1"
  shift
  if "$@"; then pass "$label"; else fail "$label"; fi
}
step() { echo; echo "=== $*"; }

DEB_FILES=()
for deb in /debs/ros-"$DISTRO"-*.deb; do
  [ -f "$deb" ] && DEB_FILES+=("$deb")
done
[ "${#DEB_FILES[@]}" -gt 0 ] || { echo "no ros-$DISTRO-*.deb in the deb directory" >&2; exit 1; }
deb_for() {
  local f
  for f in /debs/ros-"$DISTRO"-"$1"_*.deb; do [ -f "$f" ] && { echo "$f"; return; }; done
}

step "the debs"
for pkg in cognitive3d-ros-msgs cognitive3d-ros cognitive3d-ros-nav2; do
  check "ros-$DISTRO-$pkg is present" test -n "$(deb_for "$pkg")"
done

# A runtime install must not pull build tooling: a -dev package or pkg-config in a deb's own
# Depends means a build dependency was declared as a runtime one.
no_build_deps() {
  local found
  found="$(dpkg-deb -f "$1" Depends | tr ',|' '\n\n' | sed 's/^ *//; s/ .*//' |
           grep -E -- '-dev$|^pkg-config$' || true)"
  [ -z "$found" ] || { echo "        $(basename "$1") depends on: $(tr '\n' ' ' <<< "$found")"; return 1; }
}
for deb in "${DEB_FILES[@]}"; do
  check "$(basename "$deb"): no build-only package in Depends" no_build_deps "$deb"
done

# A deb names each of its siblings at exactly the version of the sibling built with it, so apt
# cannot pair debs of two releases: the nav2 build of the node links the base package's library,
# whose ABI changes between releases. <required> is the sibling the deb must depend on.
pins_siblings() {
  local deb="$1" required="$2" sibling entry version seen="" status=0
  for sibling in cognitive3d-ros-msgs cognitive3d-ros cognitive3d-ros-nav2; do
    entry="$(dpkg-deb -f "$deb" Depends | tr ',' '\n' | sed 's/^ *//; s/ *$//' |
             grep -E "^ros-$DISTRO-$sibling( |\$)" || true)"
    [ -n "$entry" ] || continue
    seen="$seen $sibling "
    version="$(dpkg-deb -f "$(deb_for "$sibling")" Version 2>/dev/null || true)"
    if [ "$entry" != "ros-$DISTRO-$sibling (= $version)" ]; then
      echo "        $(basename "$deb"): '$entry', want 'ros-$DISTRO-$sibling (= $version)'"
      status=1
    fi
  done
  case "$seen" in
    *" $required "*) ;;
    *) echo "        $(basename "$deb") does not depend on ros-$DISTRO-$required"; status=1 ;;
  esac
  return "$status"
}
check "cognitive3d-ros depends on exactly its cognitive3d-ros-msgs" \
  pins_siblings "$(deb_for cognitive3d-ros)" cognitive3d-ros-msgs
check "cognitive3d-ros-nav2 depends on exactly its cognitive3d-ros" \
  pins_siblings "$(deb_for cognitive3d-ros-nav2)" cognitive3d-ros

step "install into a fresh $DISTRO image"
apt-get update -qq
# Reported, not checked: what a robot without Nav2 pulls for the two debs it needs.
echo "  footprint of msgs + ros alone, without recommends:"
apt-get install --no-install-recommends --assume-no \
  "$(deb_for cognitive3d-ros-msgs)" "$(deb_for cognitive3d-ros)" 2>/dev/null |
  grep -E 'newly installed|Need to get|After this operation' | sed 's/^/        /' || true
if apt-get install -y --no-install-recommends "${DEB_FILES[@]}" > /tmp/install.log 2>&1; then
  pass "apt-get install"
  grep -E 'newly installed|Need to get|After this operation' /tmp/install.log | sed 's/^/        /'
  echo "  build-only packages that arrive anyway, through another package's dependencies:"
  dpkg -l libcurl4-openssl-dev libzstd-dev nlohmann-json3-dev pkg-config libopencv-dev 2>/dev/null |
    awk '/^ii/ { print "        " $2 }' || true
else
  fail "apt-get install"
  tail -30 /tmp/install.log
  echo "smoke test: install failed, nothing further to check"
  exit 1
fi

step "what is installed"
no_core_archive() {
  ! dpkg -L "ros-$DISTRO-cognitive3d-ros" "ros-$DISTRO-cognitive3d-ros-nav2" | grep -q libc3d_core
}
check "no static core archive" no_core_archive
check "no top-level include/c3d" test ! -e "$PREFIX/include/c3d"
check "node headers under include/cognitive3d_ros" test -f "$PREFIX/include/cognitive3d_ros/cognitive3d_ros/node.hpp"
check "core headers under include/cognitive3d_ros" test -f "$PREFIX/include/cognitive3d_ros/c3d/config.hpp"
check "nav2 headers under include/cognitive3d_ros_nav2" \
  test -f "$PREFIX/include/cognitive3d_ros_nav2/cognitive3d_ros_nav2/register.hpp"
for pkg in cognitive3d_ros cognitive3d_ros_nav2; do
  for file in LICENSE THIRD_PARTY_NOTICES.md; do
    check "share/$pkg/$file" test -s "$PREFIX/share/$pkg/$file"
  done
done
for pkg in cognitive3d-ros-msgs cognitive3d-ros cognitive3d-ros-nav2; do
  check "ros-$DISTRO-$pkg copyright names the licence" \
    grep -q 'LicenseRef-Cognitive3D-SDK' "/usr/share/doc/ros-$DISTRO-$pkg/copyright"
done
TOOLS="$PREFIX/lib/cognitive3d_ros"
for tool in fetch_application_key.py verify_gaze_contract.py verify_dynamics_level.py \
            export_world_gltf.sh gltf_bbox.py collada_zup_repair.py; do
  check "integrator tool $tool" test -x "$TOOLS/$tool"
done
check "the c3d_api package beside them" test -f "$TOOLS/c3d_api/__init__.py"
for harness in stub_gateway.py capnp_dump.py verify_scan_plane.py fake_tf_publisher.py; do
  check "test harness $harness is not installed" test ! -e "$TOOLS/$harness"
done

set +u
# shellcheck disable=SC1091
source "$PREFIX/setup.bash"
set -u
export ROS_DOMAIN_ID=73

# The node answers get_state over its own service, which needs no ros2 daemon and so cannot be
# fooled by a daemon's cached view of a node that has already gone.
state_of() {
  timeout 30 ros2 service call "/$1/get_state" lifecycle_msgs/srv/GetState 2>/dev/null |
    grep -o "label='[a-z]*'" | head -1
}
is_unconfigured() { [ "$(state_of "$1")" = "label='unconfigured'" ]; }

# Job control, so that each background command below leads its own process group. Without it a
# background job ignores SIGINT, and the node a `ros2 run` wrapper started would outlive the
# wrapper and be found by a later check. The group is interrupted, then whatever is left is killed.
set -m
stop() {
  kill -INT -- "-$1" 2>/dev/null || true
  ( sleep 15; kill -KILL -- "-$1" 2>/dev/null ) &
  local watchdog=$!
  wait "$1" 2>/dev/null || true
  kill -KILL -- "-$1" 2>/dev/null || true
  kill -- "-$watchdog" 2>/dev/null || true
  wait "$watchdog" 2>/dev/null || true
}

step "the node"
ros2 run cognitive3d_ros cognitive3d_node --ros-args -r __node:=smoke_plain > /tmp/plain.log 2>&1 &
pid=$!
check "cognitive3d_ros's node comes up unconfigured" is_unconfigured smoke_plain
stop "$pid"

ros2 run cognitive3d_ros_nav2 cognitive3d_node --ros-args -r __node:=smoke_nav2 > /tmp/nav2.log 2>&1 &
pid=$!
check "cognitive3d_ros_nav2's node comes up unconfigured" is_unconfigured smoke_nav2
stop "$pid"

ros2 run rclcpp_components component_container --ros-args -r __node:=smoke_container \
  > /tmp/container.log 2>&1 &
pid=$!
# Retried for up to a minute: the CLI refuses a load, without trying it, until it has discovered the
# container, which a container started a moment ago may not yet be.
load_into() {
  local container="$1" log="$2"
  shift 2
  local deadline=$((SECONDS + 60))
  until timeout 60 ros2 component load "/$container" "$@" > "$log" 2>&1; do
    if [ "$SECONDS" -ge "$deadline" ]; then
      tail -5 "$log" | sed 's/^/        /'
      return 1
    fi
    sleep 1
  done
}
load_component() {
  load_into smoke_container /tmp/load.log cognitive3d_ros cognitive3d_ros::Cognitive3DNode \
    -n smoke_component
}
check "a component container loads the plugin" load_component
check "the loaded component is unconfigured" is_unconfigured smoke_component
stop "$pid"

# A container of its own, so this load does not depend on what the one above loaded.
ros2 run rclcpp_components component_container --ros-args -r __node:=smoke_nav2_container \
  > /tmp/nav2_container.log 2>&1 &
pid=$!
load_nav2_component() {
  load_into smoke_nav2_container /tmp/load_nav2.log cognitive3d_ros_nav2 \
    cognitive3d_ros_nav2::Cognitive3DNode -n smoke_nav2_component
}
check "a component container loads the nav2 plugin" load_nav2_component
check "the loaded nav2 component is unconfigured" is_unconfigured smoke_nav2_component
stop "$pid"

PARAMS="$PREFIX/share/cognitive3d_ros/config/params.minimal.yaml"
ros2 launch cognitive3d_ros cognitive3d.launch.py params_file:="$PARAMS" > /tmp/launch.log 2>&1 &
pid=$!
check "the launch file brings the node up unconfigured" is_unconfigured cognitive3d
# /proc rather than pgrep, which a ros-base image need not carry. The bracket keeps this grep's
# own command line from matching.
launched_nav2() { grep -a -q -l '[l]ib/cognitive3d_ros_nav2/cognitive3d_node' /proc/[0-9]*/cmdline 2>/dev/null; }
check "with the nav2 package installed, the launch file runs its node" launched_nav2
stop "$pid"

step "the integrator tools run from the install"
runs() {
  local expect_rc="$1" pattern="$2"
  shift 2
  local out rc=0
  out="$(timeout 30 ros2 run cognitive3d_ros "$@" 2>&1)" || rc=$?
  if [ "$rc" -ne "$expect_rc" ] ||
     { [ -n "$pattern" ] && ! printf '%s\n' "$out" | grep -q -- "$pattern"; } ||
     printf '%s\n' "$out" | grep -q -e Traceback -e 'No such file'; then
    printf '%s\n' "$out" | tail -5 | sed 's/^/        /'
    return 1
  fi
}
check "fetch_application_key.py --help" runs 0 usage: fetch_application_key.py --help
check "verify_gaze_contract.py --help" runs 0 usage: verify_gaze_contract.py --help
check "verify_dynamics_level.py --help" runs 0 usage: verify_dynamics_level.py --help
check "collada_zup_repair.py prints its usage" runs 2 usage: collada_zup_repair.py
check "export_world_gltf.sh prints its usage" runs 2 usage: export_world_gltf.sh
check "export_world_gltf.sh --help" runs 0 usage: export_world_gltf.sh --help
check "gltf_bbox.py prints its usage" runs 2 usage: gltf_bbox.py
# The minimal file's placeholder scene id refuses configure, so this contacts nothing.
check "doctor checks the build the launch file runs" \
  runs 1 "build .*cognitive3d_ros_nav2's node.*nav2_msgs/msg/BehaviorTreeLog" \
  doctor --ros-args --params-file "$PARAMS"

echo
if [ "$failures" -eq 0 ]; then
  echo "smoke test: $DISTRO passed"
else
  echo "smoke test: $DISTRO FAILED ($failures checks)"
  exit 1
fi
