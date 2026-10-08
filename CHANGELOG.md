# Changelog

Every user-visible change to the Cognitive3D SDK for ROS 2. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the versions follow
[Semantic Versioning](https://semver.org/spec/v2.0.0.html): a breaking change comes only in a
major release, and each **BREAKING** entry says how to migrate. Anything the documentation
marks as provisional may change in a minor release, and its entry here says how to migrate.

## [Unreleased]

## [1.0.0] - 2026-10-08

The first public release of the Cognitive3D SDK for ROS 2. It records a robot's runs as Cognitive3D
sessions: its path through a 3D scene of the site, its sensors, events and action outcomes, and
optionally its camera frames and lidar scans, which you replay in Session Replay and analyze on the
Cognitive3D dashboard. Start with the [Quickstart](docs/quickstart.md).

### Highlights

- Prebuilt Debian packages for ROS 2 Humble (Ubuntu 22.04) and Jazzy (Ubuntu 24.04), on amd64 and
  arm64, with a `SHA256SUMS` file. See [Install](docs/install.md).
- `doctor` checks a configuration before you record, without opening a session, and finds most
  integration mistakes.
- Records the robot's pose from tf, the sensor topics you allowlist, batteries and diagnostics,
  action outcomes as objectives, and your own events on `~/events`.
- Camera frames, laser scans and point clouds, posed in the scene, for Session Replay.
- `cognitive3d_ros_nav2`, an optional package that records Nav2's behavior tree.
- Remote variables and A/B tests.
- An agent skill, `integrate-cognitive3d-ros2`, that guides a coding agent through adding the SDK
  to a robot.

### Known limitations

- Height and tilt read zero on a standard mobile robot: neither `odom` from wheel odometry nor
  `map` from AMCL carries them. See [Planar poses](docs/coordinates.md#planar-poses).
- Two allowlisted odometry, IMU, joint-state or laser-scan topics of the same type record into one
  series. Allowlist one topic per type for these four.
- `topic_config.<key>.ingest`: only `refuse`, and `full_ingest` for a bulk type, have an effect.
- The verifiers are diagnostics on an unversioned API, and the installed C++ headers are not a
  supported API: either can change in any release.

[1.0.0]: https://github.com/CognitiveVR/c3d-sdk-ros2/releases/tag/v1.0.0
