# Install

The Cognitive3D SDK for ROS 2 is three ROS 2 packages. Install the prebuilt Debian packages from
GitHub Releases, or build them from source with colcon. Both give you the same node, the same
`doctor` preflight command and the same integrator tools.

## Supported platforms

| | ROS 2 Humble | ROS 2 Jazzy |
| -- | -- | -- |
| Ubuntu | 22.04 (Jammy) | 24.04 (Noble) |
| Architectures | amd64, arm64 | amd64, arm64 |
| `cognitive3d_ros_msgs` | yes | yes |
| `cognitive3d_ros` | yes | yes |
| `cognitive3d_ros_nav2` | yes | yes |

Install the packages for the distro your robot runs. ROS 2 does not support mixing distros in one
graph, so every machine that talks to the node needs the same one. On Humble, Nav2's behavior-tree
log carries less information than on Jazzy, which changes one Nav2 event; see
[Recording](recording.md).

## Packages

| package | what it contains | install it |
| -- | -- | -- |
| `cognitive3d_ros_msgs` | The `StartSession` and `EndSession` services and the `Event` message | Always. Also on any other machine whose nodes start sessions or publish events |
| `cognitive3d_ros` | The node (`cognitive3d_node`), its component plugin, `doctor`, the launch file, the reference parameter files and the integrator tools | Always |
| `cognitive3d_ros_nav2` | The Nav2 behavior-tree translators, and a build of the node and of its component plugin with them included | Only on a robot that runs Nav2. It depends on `nav2_msgs` |

When `cognitive3d_ros_nav2` is installed, `cognitive3d.launch.py` runs its build of the node
instead of the plain one. You do not change the launch command. A component container loads
whichever component you name: on a Nav2 robot, load `cognitive3d_ros_nav2::Cognitive3DNode`, because
`cognitive3d_ros::Cognitive3DNode` does not have the Nav2 translators. Load either into
`component_container_isolated` or a container of its own;
[Running in a container or under systemd](sessions.md#running-in-a-container-or-under-systemd)
says why.

## Install the prebuilt packages

Each [GitHub Release](https://github.com/CognitiveVR/c3d-sdk-ros2/releases) carries one `.deb` per
package, distro and architecture, plus a `SHA256SUMS` file. The files are named
`ros-<distro>-<package>_<version>-0<ubuntu-codename>_<architecture>.deb`, for example
`ros-humble-cognitive3d-ros_1.1.0-0jammy_arm64.deb`.

Before you start:

- Install ROS 2 from the ROS apt repository, as the ROS 2 installation guide describes. The debs
  take their dependencies from it.
- Refresh the package index with `sudo apt update`. Nothing needs upgrading first: the debs name the
  ROS packages they depend on without a version, so apt installs the ones the robot lacks and
  leaves the installed ones as they are. Only the SDK's own packages require an exact version, of
  one another.
- Keep Ubuntu's `universe` component enabled. The Cap'n Proto runtime library and `ffmpeg` come
  from it, and stock Ubuntu and the official ROS images enable it. The `cognitive3d-ros` deb
  depends on `ffmpeg`, the camera encoder, so apt installs it with the SDK. On a machine that
  already has OpenCV, which `cv_bridge` brings, it adds about 100 MB.

This downloads the three packages for the machine you run it on, checks them against `SHA256SUMS`,
previews the install and then installs them:

```bash
source /opt/ros/jazzy/setup.bash      # or humble; sets ROS_DISTRO
SDK_VERSION=1.1.0
UBUNTU="$(. /etc/os-release && echo "$VERSION_CODENAME")"
ARCH="$(dpkg --print-architecture)"
BASE="https://github.com/CognitiveVR/c3d-sdk-ros2/releases/download/v$SDK_VERSION"

mkdir -p ~/cognitive3d-debs && cd ~/cognitive3d-debs
for package in cognitive3d-ros-msgs cognitive3d-ros cognitive3d-ros-nav2; do
  curl -fLO "$BASE/ros-$ROS_DISTRO-${package}_$SDK_VERSION-0${UBUNTU}_$ARCH.deb"
done
curl -fLO "$BASE/SHA256SUMS"
sha256sum --check --ignore-missing SHA256SUMS

apt-get install -s --no-install-recommends ./ros-"$ROS_DISTRO"-cognitive3d-ros*_"$ARCH".deb
sudo apt install --no-install-recommends ./ros-"$ROS_DISTRO"-cognitive3d-ros*_"$ARCH".deb
```

`sha256sum` prints `OK` for each file it checked. Do not install a file it reports as `FAILED`.

`SHA256SUMS` lists every deb in the release. If you mirror the debs, on a package server of your
own or in an offline bundle for the robot, copy `SHA256SUMS` beside them unchanged: it is what the
check runs against, and `--ignore-missing` checks only the debs you copied. If debs reach you
without it, do not install them: download `SHA256SUMS` from the same release's GitHub Release page,
put it beside them, and run the check.

`apt-get install -s` is a preview: it changes nothing and needs no `sudo`. Read its summary line
before you run the real install. On a robot that already runs ROS 2, it reads like this:

```text
0 upgraded, 280 newly installed, 0 to remove and 0 not upgraded.
```

`newly installed` is the SDK and the dependencies the robot lacks, mostly OpenCV (see
[How much it installs](#how-much-it-installs)). `not upgraded` counts installed packages that have
newer versions, which this install leaves alone. Any package it would upgrade or remove is listed
above the summary, under `The following packages will be upgraded:` and
`The following packages will be REMOVED:`.

If the preview would upgrade or remove any of the robot's ROS packages, do not install yet. The
SDK's debs require no particular version of any ROS package, so the upgrade comes from another
package the install pulls in. On a robot under change control, make that upgrade as a change of its
own, check the robot, and then install the SDK. A `cognitive3d` package listed for removal means
another SDK release is installed; see the upgrade note below.

In a script or a provisioning tool, where nobody answers apt's prompt, run the real install without
one, once you have read the preview for that robot image:

```bash
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends ./ros-"$ROS_DISTRO"-cognitive3d-ros*_"$ARCH".deb
```

The debs are built against the ROS packages current at their release. On a robot whose ROS packages
are older, the node can fail to start with an `undefined symbol` error naming a ROS library.
Upgrading the package that provides that library fixes it, as a change of its own like any other.

On a robot without Nav2, leave out the Nav2 package so `nav2_msgs` is not installed:

```bash
sudo apt install --no-install-recommends \
  ./ros-"$ROS_DISTRO"-cognitive3d-ros-msgs_*_"$ARCH".deb \
  ./ros-"$ROS_DISTRO"-cognitive3d-ros_*_"$ARCH".deb
```

If you downloaded the release assets by hand, keep only one distro and one architecture in the
directory, or keep the `_<architecture>.deb` suffix in the pattern. apt refuses a package built for
another architecture.

The packages require one another at exactly the same version. To upgrade, run the same commands
with the new `SDK_VERSION`, for every package the robot has. Given only some of them, apt offers to
remove the package left at the old version rather than mix two releases.

### How much it installs

Almost all of the install is OpenCV. `cognitive3d_ros` depends on `cv_bridge`, and `cv_bridge`
depends on the OpenCV development packages. Measured on arm64, on top of a `ros-base`
installation, with `--no-install-recommends`:

| distro | `cognitive3d_ros_msgs` and `cognitive3d_ros` | all three packages |
| -- | -- | -- |
| Humble | about 280 packages, 181 MB download, 721 MB on disk | 738 MB on disk |
| Jazzy | about 280 packages, 192 MB download, 811 MB on disk | 852 MB on disk |

These figures were measured before `cognitive3d_ros` depended on `ffmpeg`, the camera encoder. Measured by apt simulation on arm64, it adds 85 MB on Ubuntu 22.04, 106 MB on Ubuntu 24.04
and 108 MB on Debian trixie when OpenCV is already installed, as it is here, because only its
filter and device libraries are new; on a bare OS it adds 387 to 676 MB.

Plan for roughly 800 to 960 MB of free space. On a robot that boots from an SD card, check the
free space before you install. If your robot already has `cv_bridge` installed, for example for a
camera driver, most of that space is already used and the SDK adds little beyond `ffmpeg`'s 85 to 108 MB. The three debs
themselves are under 1 MB together.

Leave out `--no-install-recommends` and apt also installs every recommended package, which is
larger still.

## Build from source

Build from source to run an unreleased version, or on a platform with no prebuilt package. You need
ROS 2 Humble or Jazzy, colcon and rosdep.

Cap'n Proto has no rosdep key, so rosdep cannot install it. Install it by name first:

```bash
sudo apt update
sudo apt install capnproto libcapnp-dev ffmpeg
```

`ffmpeg` is the camera encoder: the node runs it as a child process to encode camera frames as
H.264, and does not link it. It is needed only when a camera is configured. On a machine that
already has OpenCV, which `cv_bridge` brings, it adds about 100 MB, because the codec libraries
are already installed and what it adds is mostly its filter and device libraries.

Then install colcon and rosdep, if you do not have them, and initialize rosdep once per machine:

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep
sudo rosdep init      # once per machine; skip it if rosdep says it is already initialized
rosdep update
```

Clone the repository into a workspace, install the dependencies rosdep knows, and build:

```bash
mkdir -p ~/c3d_ws/src
git clone --branch v1.1.0 https://github.com/CognitiveVR/c3d-sdk-ros2.git ~/c3d_ws/src/c3d-sdk-ros2
cd ~/c3d_ws
source /opt/ros/jazzy/setup.bash      # or humble
rosdep install --from-paths src --ignore-src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

Clone the whole repository: `cognitive3d_ros` builds the SDK's ROS-free core from the `core/`
directory beside it, and installs the integrator tools from `tools/`. `-DBUILD_TESTING=OFF` skips
building the SDK's own test suites, which you need only to work on the SDK. To build without the
Nav2 package, add `--skip-keys nav2_msgs` to the `rosdep install` line and
`--packages-skip cognitive3d_ros_nav2` to the `colcon build` line.

A missing Cap'n Proto shows up as a CMake error from `find_package(CapnProto)` while
`cognitive3d_ros` configures. A missing `ffmpeg` does not stop the build: it stops a node with a
camera configured at configure, naming the package, and `doctor`'s `camera encoder` check says so
before that.

## Check the install

Source ROS 2 and start the node:

```bash
source /opt/ros/jazzy/setup.bash      # or humble
ros2 run cognitive3d_ros cognitive3d_node
```

It logs one line naming the SDK version, the ROS distro and the RMW, then waits:

```text
[INFO] [...] [cognitive3d]: cognitive3d_ros 1.1.0 (ROS 2 jazzy, rmw_fastrtps_cpp) constructed. Subscribe-only: this node publishes nothing except its own diagnostics.
```

Stop it with Ctrl+C. A node started this way is unconfigured and records nothing. The
[Quickstart](quickstart.md) takes it from here to a session in Session Replay.

## The integrator tools

`cognitive3d_ros` installs `doctor` and six tools beside the node. Run each with
`ros2 run cognitive3d_ros <tool>`. The Python tools need only the distro's `python3`, with no
packages to install.

| tool | what it does | documented in |
| -- | -- | -- |
| `doctor` | Checks your configuration, credentials, network path and live ROS graph without opening a session | [doctor and verification](doctor-and-verification.md) |
| `fetch_application_key.py` | Reads your project's application key with your organization key | [Quickstart](quickstart.md) |
| `verify_gaze_contract.py` | Reads back a recorded session and checks its pose timeline. A diagnostic on an unversioned API | [doctor and verification](doctor-and-verification.md) |
| `verify_dynamics_level.py` | Reads back the robot's Dynamic Object rotations from a session. A diagnostic on an unversioned API | [doctor and verification](doctor-and-verification.md) |
| `export_world_gltf.sh` | Exports a Gazebo world to glTF for upload as a scene | [Scenes](scenes.md#export-a-gazebo-world) |
| `gltf_bbox.py` | Checks a glTF scene or robot mesh before upload | [Scenes](scenes.md#prepare-the-files) |
| `collada_zup_repair.py` | Repairs the up axis of a COLLADA file exported from Gazebo | [Scenes](scenes.md#export-a-gazebo-world) |

Beside the tools, `lib/cognitive3d_ros/` also holds `c3d_api`, the Python package they import. It
is not a command to run.

## Run in a container

The node runs in a container like any other ROS 2 node, and it has to share the robot's DDS graph.
Start the container with:

- `--network=host`, or the robot's own network namespace. DDS discovery does not cross a Docker
  bridge network.
- `--ipc=host`. Fast DDS, the default RMW on Humble and Jazzy, uses shared memory between
  processes it believes share a host. With host networking and a separate IPC namespace, the node
  subscribes to every topic and receives nothing, and no error is logged.
- The robot's ROS distro, RMW and `ROS_DOMAIN_ID`, and any DDS configuration file it uses.
- `C3D_APPLICATION_API_KEY` in the environment.
- A volume for the spool, so data not yet uploaded survives a container restart. Set `spool_dir`
  to a path on that volume; [Sessions](sessions.md) describes the spool.

Start the node from the container's entrypoint, or from a script the entrypoint runs, never with
`docker exec`, and pass `autostart:=true` unless a lifecycle manager in the container configures
and activates it. [Running in a container or under systemd](sessions.md#running-in-a-container-or-under-systemd)
covers stopping the node cleanly and running it under systemd.

```bash
docker run --rm --init --network=host --ipc=host \
  -e C3D_APPLICATION_API_KEY -e ROS_DOMAIN_ID \
  -v cognitive3d-spool:/var/lib/cognitive3d \
  -v "$HOME/cognitive3d/params.yaml:/params.yaml:ro" \
  <your-robot-image> \
  ros2 launch cognitive3d_ros cognitive3d.launch.py params_file:=/params.yaml autostart:=true
```

This example assumes `params.yaml` sets `spool_dir: /var/lib/cognitive3d`.

On a robot whose whole ROS stack is one container that you can reach only with `docker exec`, and
cannot recreate with a volume or another entrypoint, install the packages inside that container and
start the node with `docker exec`. Stop it with SIGINT and give its spool a path that already
persists, as
[A robot stack in a container you cannot recreate](sessions.md#a-robot-stack-in-a-container-you-cannot-recreate)
describes. Packages installed this way live in the container's writable layer, so install them
again whenever the container is replaced.

## Headers are not a supported C++ API

The packages install C++ headers under `include/cognitive3d_ros/` and
`include/cognitive3d_ros_nav2/`. They exist so that the Nav2 package can build against the node.
They are not a supported API and can change in any release, including a patch release.

The supported interfaces are:

- the parameters and environment variables in [Configuration](configuration.md)
- the services and message in `cognitive3d_ros_msgs`
- the launch file, `cognitive3d.launch.py`
- the executables: `cognitive3d_node`, `doctor` and the integrator tools
- the component plugins, `cognitive3d_ros::Cognitive3DNode` and `cognitive3d_ros_nav2::Cognitive3DNode`

## Uninstall

Remove the packages, then the dependencies that were installed only for them:

```bash
source /opt/ros/jazzy/setup.bash      # or humble; sets ROS_DISTRO
sudo apt remove ros-"$ROS_DISTRO"-cognitive3d-ros-msgs
sudo apt autoremove
```

Removing `cognitive3d_ros_msgs` also removes `cognitive3d_ros` and `cognitive3d_ros_nav2`, which
depend on it; apt lists them before it asks you to confirm. `apt autoremove` removes every
automatically installed package that nothing needs any more, not only the SDK's dependencies. Read
its list before you confirm.

For a source build, delete the workspace's `build/`, `install/` and `log/` directories.

Neither removes the spool. It is at `spool_dir` if you set one, otherwise at
`$XDG_STATE_HOME/cognitive3d`, or at `~/.local/state/cognitive3d` for the user that ran the node.
The spool can hold sessions that have not finished uploading. Let the node upload them before you
delete the directory; [Sessions](sessions.md) describes draining the spool.
