#pragma once

#include "c3d/environment.hpp"

#include "c3d/version.hpp"
#include "c3d/wall_clock.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "c3d/types.hpp"

namespace c3d {

struct Config {
  // ---- Identity and destination -------------------------------------------
  std::string application_key;   // sent as `Authorization: APIKEY:DATA <key>`
  std::string scene_id;          // REQUIRED. Empty scene id means 100% data loss.
  std::string scene_version;     // REQUIRED. Empty version means 100% data loss.
  std::string device_id;         // stable per robot
  std::string device_type = "ros2";  // travels as the gaze `hmdtype` field

  // Application identity, sent as session properties.
  //
  // `app_version` is NOT optional. The platform accepts a session whose gaze
  // properties omit `c3d.app.version` with HTTP 200 and the `cvr-request-time`
  // header, and then silently discards it. It never appears in the session list
  // or the dashboard's counts, and no error is surfaced anywhere.
  std::string app_name = "cognitive3d_ros";
  // Derived, not declared. This is the INTEGRATOR's application version and an integrator may set
  // it to anything -- but its default is the SDK's own version, and a separately declared copy of
  // that number is one more to keep in step by hand. It reaches the wire as the mandatory
  // `c3d.app.version`, so a copy left behind by a bump would send two disagreeing versions from
  // one binary.
  std::string app_version = kSdkVersion;

  std::string protocol = "https";
  // The environment NAME this run resolved, and the data host it selected. `core` only ever
  // talks to the gateway, so the name is carried rather than used -- but every consumer needing
  // the api host must resolve it from the same table, never derive one host from the other.
  std::string environment = kDefaultEnvironment;
  std::string gateway = DefaultHosts().data;
  std::string network_version = "0";

  // ---- Spool ---------------------------------------------------------------
  // Required. Every payload is written here before any upload is attempted.
  std::string spool_dir;
  std::uint64_t spool_max_bytes = 1024ull * 1024 * 1024;

  // ---- Batching ------------------------------------------------------------
  std::size_t gaze_batch = 256;
  std::size_t event_batch = 256;
  std::size_t sensor_batch = 512;
  std::size_t dynamic_batch = 512;
  double auto_send_interval_s = 10.0;
  double gaze_interval_s = 0.1;  // advertised in the gaze envelope

  // Bounded in-memory queues. Past these, we drop and say so rather than
  // growing without limit, which is how telemetry agents OOM a robot.
  std::size_t max_records_per_batch = 20000;

  // The binary streams are cut on whichever bound is reached first. They are
  // bounded by BYTES as well as by time because one camera frame is three orders
  // of magnitude larger than one gaze sample: a count-based batch that is right
  // for a 250x250 still is a 40 MB payload on a 4K one, and the platform drops a
  // single frame over 1 MB after the upload carrying it has been accepted.
  double camera_flush_s = 2.0;
  std::uint64_t camera_flush_bytes = 4ull * 1024 * 1024;
  double lidar_flush_s = 2.0;
  std::uint64_t lidar_flush_bytes = 4ull * 1024 * 1024;

  // The pose stream's block interval: a rigged object's samples are encoded in blocks this
  // long. Fixed for the session, 0.1 to 10 s, and sent once as the session property
  // `c3d.stream.pose.blockIntervalMs`. The upload queue's 11 s and 5 MiB are the platform's
  // and not configurable.
  double pose_block_interval_s = 5.0;
  // For test builds: a file the pose upload queue appends one JSON line to per block part as
  // it decides the send (spec §10.4's send log: part, queueStartUs, sentUs, reason and, on a
  // flush part, flush), for c3d-stream-formatter's upload check. "" writes none.
  std::string pose_send_log_path;
  // For tests: the pose stream's monotonic clock, in microseconds. Unset, it is
  // std::chrono::steady_clock. It times the upload queue and the host-clock block close, and
  // nothing else.
  std::function<std::int64_t()> pose_clock_us;

  // ---- Transport -----------------------------------------------------------
  // An upload is judged by whether it is MOVING, not by how long it takes. A part
  // is up to camera_flush_bytes of camera frames, and on a slow uplink sending one
  // legitimately takes longer than any fixed timeout short enough to catch a dead
  // link -- a 1.5 MB part at 100 kB/s is 15 s -- so a fixed timeout fails the same
  // part forever, and the oldest part blocks every stream behind it. So:
  //
  //   * `http_connect_timeout_s` bounds reaching the gateway at all.
  //   * `http_low_speed_limit_bytes_per_s` over `http_low_speed_time_s` is the
  //     stall detector: a request that moves fewer bytes per second than the limit
  //     for that long is abandoned. The window has to cover the stretch where a
  //     healthy upload shows NO progress -- the kernel's send buffer draining over
  //     a slow link after curl has handed it the last byte, then the gateway's
  //     response -- which on a bufferbloated uplink is tens of seconds.
  //   * `http_timeout_s` is the whole-request ceiling, a safety net for a link that
  //     trickles just above the stall limit forever. It has to clear the largest
  //     part at the slowest rate still worth sending: 4 MB in 300 s is ~14 kB/s.
  int http_timeout_s = 300;
  int http_connect_timeout_s = 10;
  int http_low_speed_limit_bytes_per_s = 1024;
  int http_low_speed_time_s = 60;
  // The retryable-failure back-off: starts at retry_min_s, doubles per failure,
  // caps at retry_max_s, and resets on the first success. A Retry-After the server
  // sends can lengthen that one wait, up to an hour, and never shorten it.
  // retry_max_s is also how often a paused uploader tries a refused credential again.
  double retry_min_s = 60.0;
  double retry_max_s = 240.0;

  // Record and spool exactly as normal -- parts are cut on their usual bounds --
  // but attempt no upload. For a run whose uplink cannot carry the data rate: the
  // spool is drained afterwards by a process configured with this off, into the
  // same session ids, because every chunk carries its own destination and body.
  // Settable at runtime through Session::SetUploadsPaused.
  bool uploads_paused = false;

  // How many quick attempts one chunk gets when the credentials are refused (401,
  // 403 or 407), `unauthorized_retry_s` apart, before every upload pauses. 0 and 1
  // both mean "pause at the first refusal". A paused uploader keeps everything
  // spooled and tries again every retry_max_s.
  int unauthorized_attempts = 2;
  double unauthorized_retry_s = 1.0;

  // How long destroying a Session gives the uploader to deliver what is spooled --
  // a session's last parts, its end included -- before abandoning the rest to the
  // next start. End() itself never waits for an upload. Waiting stops early once
  // the spool is empty or nothing in it can go now: paused, backing off, or held.
  double shutdown_drain_s = 2.0;

  // ---- Behaviour -----------------------------------------------------------
  double worker_tick_s = 0.05;

  // Begin() refuses while the wall clock reads earlier than this. See
  // c3d/wall_clock.hpp.
  double wall_clock_floor_epoch_s = kWallClockFloorEpochSeconds;

  // How often a live session refreshes its spool marker, and how long a marker
  // must have gone unrefreshed before recovery may adopt it as an orphan.
  // Recovery writes an `abnormalEnd` and deletes the marker, so adopting a
  // session that is still running tells the platform a running measurement
  // ended -- and leaves it unrecoverable if it really does crash later. So
  // recovery reads the heartbeat before it adopts anything. DiskSpool's lock
  // keeps a second process out of a spool directory, so a live marker is met by
  // another Session on the same ISpool, or by another process only on a
  // filesystem that cannot lock. A process respawned after a crash meets its
  // predecessor's marker still fresh, leaves it alone, and looks again once the
  // threshold has passed.
  //
  // The default threshold is six heartbeats. Validate() refuses a threshold that
  // does not clear the interval PLUS one worker tick, because a heartbeat becomes
  // due on a tick boundary: the real maximum gap between two touches is
  // marker_heartbeat_s + worker_tick_s, and a threshold inside that adopts live
  // sessions by construction.
  //
  // These are also the two values a marker records and a reader honours. A marker
  // written by a process with a long heartbeat is judged by ITS budget, not the
  // reader's -- see SessionMarker::max_silence_s. Both are settable as ROS
  // parameters on the node, because the only reasons to change them are
  // deployment facts the SDK cannot see: a spool on a filesystem that cannot
  // lock, or how soon a respawned process should close the session its
  // predecessor left open.
  double marker_heartbeat_s = 5.0;
  double orphan_stale_after_s = 30.0;

  // Sensor change gating, mirroring Unity's SENSOR_VALUE_CHANGE_THRESHOLD.
  // A sample is kept only if it moved by more than this, or if keepalive elapsed.
  double sensor_change_threshold = 0.001;
  double sensor_keepalive_s = 2.0;

  // Returns an empty string when valid, otherwise the reason it is not.
  std::string Validate() const;
};

}  // namespace c3d
