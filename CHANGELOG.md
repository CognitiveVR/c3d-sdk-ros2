# Changelog

Every user-visible change to the Cognitive3D SDK for ROS 2. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the versions follow
[Semantic Versioning](https://semver.org/spec/v2.0.0.html): a breaking change comes only in a
major release, and each **BREAKING** entry says how to migrate. Anything the documentation
marks as provisional may change in a minor release, and its entry here says how to migrate.

## [Unreleased]

## [1.1.0] - 2026-10-09

### Changed

- The pose timeline is sampled at 30 Hz by default (`sample_rate_hz`), up from 10.
- Odometry, joint state, IMU, `std_msgs/msg/Float32` and `std_msgs/msg/Int32MultiArray` topics
  record at up to 30 Hz by default, up from 10 Hz for odometry, 5 Hz for joint states and 1 Hz for
  the rest. Batteries and laser scan summaries stay at 1 Hz.
- Cameras and lidars record at up to 30 frames or scans per second by default (`rate_hz`), up from
  1 and 2.
- Recorded data volume rises with these rates. A source slower than its rate is recorded at its own
  rate. At the defaults the pose timeline is about 2.5 KB per second, one 640 x 480 camera about
  1.7 MB per second (about 6 GB per hour), and a depth point cloud of 20,000 points about 1.8 MB per
  second, up to about 9 MB per second at `max_points`; see
  [What it costs](docs/camera-and-lidar.md#what-it-costs).
- `spool_max_mb` defaults to 1024, up from 512.
- `robot.localisation_jump` compares poses about 0.1 seconds apart rather than consecutive samples,
  so detection does not weaken as `sample_rate_hz` rises: a correction that tf spreads over several
  samples, as it does behind AMCL, is no longer cut into steps too small to detect. Below 10 Hz it
  compares consecutive samples, as before. A localizer slower than 10 Hz can still spread a
  correction across two comparisons, as in 1.0.0.
- `c3d.pose_failures`, `c3d.stale_poses` and `c3d.gaze_frame_failures` count samples, so at 30 Hz
  the same failure counts three times as many as at 10 Hz.
- To keep 1.0.0's rates and spool, set `sample_rate_hz: 10.0`, `camera.<name>.rate_hz: 1.0`,
  `lidar.<name>.rate_hz: 2.0` and `spool_max_mb: 512`, and `topic_config.<key>.rate_hz` on each
  allowlisted topic: `10.0` for odometry, `5.0` for joint states, and `1.0` for IMU, `Float32` and
  `Int32MultiArray` topics.

### Fixed

- Posing a camera frame or lidar scan no longer blocks the node's callback thread. A record whose
  transform has not arrived yet, such as one from a driver that stamps as it publishes, or one whose
  frame tf cannot resolve, used to wait up to 50 ms on that thread, delaying pose sampling with it.
  It is now held for up to 0.2 s and retried while the node does other work; a record that still has
  no pose is dropped and counted as before.

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

[1.1.0]: https://github.com/CognitiveVR/c3d-sdk-ros2/releases/tag/v1.1.0
[1.0.0]: https://github.com/CognitiveVR/c3d-sdk-ros2/releases/tag/v1.0.0
