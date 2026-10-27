# The Cognitive3D SDK for ROS 2

The Cognitive3D SDK for ROS 2 records a robot's runs as Cognitive3D sessions: its path through a 3D
scene of the site, its sensors, events and action outcomes, and optionally its camera frames and
lidar scans. It runs as one more node on the robot's ROS 2 graph and subscribes to what the robot
already publishes, and you replay each session in Session Replay and analyze it on the Cognitive3D
dashboard.

**Supported platforms:** ROS 2 Humble on Ubuntu 22.04 and ROS 2 Jazzy on Ubuntu 24.04, on amd64 and
arm64.

**Requirements:** a Cognitive3D project and its application key, a scene of the space the robot
works in, and a robot that publishes tf.

## Community support

Join our [Discord](https://discord.gg/x38sNUdDRH) for community support, or email
`support@cognitive3d.com`. Include the full output of `doctor` in every report: it names the SDK
version, the ROS 2 distro and the RMW, and what it found. doctor masks the application key, so its
output is safe to share as it is.

## Install

Each [GitHub Release](https://github.com/CognitiveVR/c3d-sdk-ros2/releases) carries prebuilt
Debian packages for both distros and both architectures. You can also build from source with colcon.
[Install](docs/install.md) covers both, the disk space the packages need, and running in a
container.

| package | what it is |
| -- | -- |
| `cognitive3d_ros` | The node, `doctor`, the launch file, the reference parameter files and the integrator tools |
| `cognitive3d_ros_msgs` | The `StartSession` and `EndSession` services and the `Event` message |
| `cognitive3d_ros_nav2` | Optional. Records Nav2's behavior tree; install it on a robot that runs Nav2 |

## Quickstart

The [Quickstart](docs/quickstart.md) walks through each step in full.

1. Install the packages, as in [Install](docs/install.md).
2. Export your project's application key into the node's environment. The node never reads it
   from a parameter, and the Quickstart shows how to read the key from the platform:

   ```bash
   export C3D_APPLICATION_API_KEY=<your-application-key>
   ```

3. Copy the minimal params file to `~/cognitive3d/params.yaml`, and set your scene, your robot's
   device id (`device_id`), the pose convention your scene was authored in, and your robot's topics:

   ```bash
   mkdir -p ~/cognitive3d
   cp "$(ros2 pkg prefix cognitive3d_ros)/share/cognitive3d_ros/config/params.minimal.yaml" ~/cognitive3d/params.yaml
   ```

4. Check everything without opening a session:

   ```bash
   ros2 run cognitive3d_ros doctor --ros-args --params-file "$HOME/cognitive3d/params.yaml"
   ```

5. Launch the node, drive the robot, and find the session in Session Replay a few minutes later:

   ```bash
   ros2 launch cognitive3d_ros cognitive3d.launch.py params_file:="$HOME/cognitive3d/params.yaml" autostart:=true
   ```

Most integration mistakes produce no error: the platform accepts the session and it is empty,
misplaced or filed under another project. `doctor` finds most of them before you record, and
[Troubleshooting](docs/troubleshooting.md) finds the rest by symptom.

## What a session records

| | from |
| -- | -- |
| The robot's path and heading | tf, from `map` (or `odom`) to the robot's body frame |
| Sensor series | the topics you allowlist: odometry, IMU, joint states, laser scans, plain `Float32` and `Int32MultiArray` sensors |
| Batteries and diagnostics | every `BatteryState` and `DiagnosticArray` topic, found by type |
| Objectives | action servers' goal outcomes, Nav2's and the iRobot Create 3's by default |
| Events | diagnostics and battery changes, localization jumps, Nav2 recoveries, and your own, published on `~/events` |
| Camera frames and lidar scans | topics you name, posed in the scene; off by default |
| The SDK's own health | spool, upload and pose counters, in every session |

## Documentation

| page | what it covers |
| -- | -- |
| [Install](docs/install.md) | Platforms, the prebuilt packages, a source build, containers |
| [Quickstart](docs/quickstart.md) | From an installed SDK to a first session on the dashboard |
| [Scenes](docs/scenes.md) | Getting a scene, exporting a Gazebo world, the robot as a Dynamic Object |
| [Configuration](docs/configuration.md) | Every parameter and environment variable |
| [Sessions](docs/sessions.md) | The lifecycle, the session services, time, the spool, the report line, containers and systemd |
| [Coordinates](docs/coordinates.md) | The pose convention, reference frames, the gaze channel |
| [Recording](docs/recording.md) | The allowlist, sensor series, events, objectives, Nav2, health series |
| [Camera and lidar](docs/camera-and-lidar.md) | Camera frames, laser scans and point clouds, and what they cost |
| [Pose stream](docs/pose.md) | Provisional: the robot's tf tree as a rig, off by default |
| [Remote variables](docs/remote-variables.md) | Remote variables and A/B tests |
| [Doctor and verification](docs/doctor-and-verification.md) | Every `doctor` check, and checking a stored session |
| [Troubleshooting](docs/troubleshooting.md) | What goes wrong, by symptom |
| [Data format](docs/data-format.md) | What a stored session contains, for tools that read it |
| [Changelog](CHANGELOG.md) | What changed in each release, and how to migrate |

## AI-assisted integration

The repository includes an agent skill, [`integrate-cognitive3d-ros2`](.claude/skills/integrate-cognitive3d-ros2/SKILL.md),
that guides an AI coding agent through adding the SDK to a robot that already works: it surveys the
live robot, makes the configuration decisions from what it measured, writes the params file, runs
`doctor`, checks the first session, and reports what it chose and why.

The skill is in this repository only: the Debian packages do not install it. To use it with Claude
Code, copy the skill directory from a checkout of this repository at your release's tag into your
own repository's `.claude/skills/`, which shares it with your team, or into `~/.claude/skills/` for
yourself:

```bash
git clone --depth 1 --branch v1.1.0 https://github.com/CognitiveVR/c3d-sdk-ros2.git /tmp/c3d-sdk-ros2
mkdir -p .claude/skills
cp -r /tmp/c3d-sdk-ros2/.claude/skills/integrate-cognitive3d-ros2 .claude/skills/
```

The skill is plain Markdown with shell commands, so other agents can follow it too: point the agent
at `SKILL.md`, or paste it into the agent's context. [The skills README](.claude/skills/README.md)
has the details.

> [!NOTE]
> AI-assisted integration is experimental. Review the params file the agent writes and the `doctor`
> output it reports, and check the first session in Session Replay yourself.

## Status

Version 1.x follows semantic versioning: a breaking change comes only in a major release, and the
[Changelog](CHANGELOG.md) says how to migrate. Anything the documentation marks as provisional may
also change in a minor release, with a migration note in the Changelog.

## Contributing and security

[CONTRIBUTING.md](CONTRIBUTING.md) covers building, testing and the rules a change follows. To
report a vulnerability, follow [SECURITY.md](SECURITY.md) rather than opening an issue.

## License

Licensed under the Cognitive3D SDK Software License; see [LICENSE](LICENSE). Third-party components
are listed in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
