# Changelog

Every user-visible change to the Cognitive3D SDK for ROS 2. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the versions follow
[Semantic Versioning](https://semver.org/spec/v2.0.0.html): a breaking change comes only in a
major release, and each **BREAKING** entry says how to migrate. Anything the documentation
marks as provisional may change in a minor release, and its entry here says how to migrate.

## [Unreleased]

### Added

- `ffmpeg_path`, the encoder binary, default `ffmpeg`, and per camera `camera.<name>.preset`,
  `camera.<name>.crf` and `camera.<name>.max_bitrate_kbps`, up to 1,000,000.
- Session properties per camera `c3d.ros.camera.<name>.codec`, `.preset`, `.crf`,
  `.max_bitrate_kilobits_per_second` and `.chunk_seconds`, and per `PointCloud2` lidar
  `c3d.ros.lidar.<name>.pointcloud.encoding`, the encoding the clouds are written in.
- Health series per camera, `dropped_backlog`, `dropped_at_stop`, `encoder_restarts`,
  `gap_restarts` and `dropped_no_session`, as `c3d.camera.<name>.<series>`, and
  `c3d.camera_access_units_dropped`.
- A camera's chunk ends before it would pass 8 MiB, under the platform's 10 MiB per object: the
  encoder is restarted and the frames since encoded again from a keyframe, counted per camera in
  `size_restarts`, with `dropped_replay` for any it could not keep. `camera.<name>.replay_max_mb`,
  default 192, bounds the memory that keeps them.
- `doctor`'s `camera encoder` check, which is FATAL when a camera is configured and `ffmpeg` cannot
  run. Its `camera/<name>` check reports an estimated size and rate.

### Changed

- **BREAKING:** Camera frames are stored as 3 second H.264 chunks, encoded by the host's `ffmpeg`,
  not as JPEG frames. `camera.<name>.format` and `camera.<name>.jpeg_quality` are removed, and
  configure now refuses a params file that sets either, naming the key, so a node configured with
  them does not start. The session properties of the same names are removed too. Migrate: delete
  both keys, set `preset` and `crf` if you want other than `ultrafast` and `28`, and install
  `ffmpeg`.
- **BREAKING:** Laser scans are stored as blocks of 3 seconds of uint16 codes, 1 mm up to a
  `range_max` of 65 m and 1 cm beyond, delta coded, byte shuffled and compressed with zstd, and a
  scan with more than 65,535 rays is refused. Migrate: read `ScanBlock` as
  [Data format](docs/data-format.md#laser-scan-blocks) describes.
- **BREAKING:** Point clouds are delta coded per plane and packed into objects that close at 2 MiB or 5
  seconds, a single cloud over 2 MiB being an object of its own, and `lidar.<name>.pointcloud.zstd_level` defaults to 3, up from 1. Migrate: read
  `CloudBlock` as [Data format](docs/data-format.md#point-cloud-objects) describes, and set
  `zstd_level: 1` to keep the old level.
- **BREAKING:** `camera_flush_s`, `camera_flush_bytes`, `lidar_flush_s` and `lidar_flush_bytes` are
  removed: a chunk, a block or an object closes by its own rule. Migrate: delete them.
- **BREAKING:** The camera and lidar payloads carry `formatVersion` `"2.0"`, under the schemas
  `CameraPayload` with `FrameMeta`, and `LidarPayload` with `ScanBlock` and `CloudBlock`. Migrate:
  deploy a gateway that reads them before this release; one that does not cannot read the
  camera and lidar streams.
- A camera on a `CompressedImage` topic is decoded and encoded again as H.264. Its JPEG is no
  longer stored as published, so a compressed topic now costs the CPU of a decode as well as the
  encode.
- `doctor` no longer warns about an encoded camera or lidar record over 900 KiB: a chunk, a block
  or an object closes by its own rule.

## [1.1.0] - 2026-10-13

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
  It is now held for up to 0.2 s of ROS time, simulated time under `use_sim_time`, and retried while
  the node does other work; a record that still has no pose is dropped and counted as before.

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
