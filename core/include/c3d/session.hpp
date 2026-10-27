#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "c3d/config.hpp"
#include "c3d/logger.hpp"
#include "c3d/media.hpp"
#include "c3d/pose.hpp"
#include "c3d/pose_manifest.hpp"
#include "c3d/spool.hpp"
#include "c3d/transport.hpp"
#include "c3d/types.hpp"

namespace c3d {

// Unity truncates the participant id at 64 characters and logs an error. We
// match the cap and the loudness.
constexpr std::size_t kParticipantIdMax = 64;

struct Stats {
  std::uint64_t records_recorded = 0;
  std::uint64_t records_dropped = 0;
  std::uint64_t chunks_spooled = 0;
  std::uint64_t chunks_uploaded = 0;
  std::uint64_t chunks_discarded = 0;  // payload rejections: 400, 413, 415, 422
  // Parts that threw while being serialised or spooled. Their records are in
  // records_dropped too; this says that something threw rather than ran out of room.
  std::uint64_t serialization_failures = 0;
  // Parts the spool refused to write: no room under the cap, or the disk failed.
  // Their records are in records_dropped too.
  std::uint64_t spool_write_failures = 0;
  // Exceptions caught on the session's own threads: a consumer-supplied spool that threw
  // from a marker write, a read of the next chunk or a removal, or anything else that cut
  // a pass of either thread short. Each kind is logged once; the thread carries on.
  std::uint64_t thread_exceptions = 0;
  std::uint64_t unauthorized_retries = 0;
  // Every answer that refused the credentials: 401, 403 or 407.
  std::uint64_t auth_rejections = 0;
  std::uint64_t upload_failures = 0;
  std::uint64_t spool_bytes = 0;
  std::size_t spool_count = 0;
  // Chunks waiting behind the rest of the spool after repeated failures of their own.
  std::size_t chunks_deferred = 0;
  // Chunks kept but not sent because they were recorded for a host this process is
  // not configured to send to, or under an application key it does not have.
  std::size_t chunks_held = 0;
  // Chunks the spool evicted to stay under spool_max_bytes. They are never uploaded.
  std::uint64_t chunks_evicted = 0;
  // A retryable failure's back-off is running, and nothing is sent until it ends.
  bool backing_off = false;
  bool uploads_paused = false;
  // Uploads stopped because the credentials were refused. Everything stays spooled,
  // and authentication is tried again every retry_max_s.
  bool uploads_auth_paused = false;
  // The streams whose credentials were refused, in stream order. Their chunks stay
  // spooled, and one of them is asked about again every retry_max_s. Non-empty with
  // uploads_auth_paused false: those streams are refused while every other uploads.
  std::vector<Stream> auth_refused_streams;
  // The streams whose chunks keep failing on their own while the gateway accepts other
  // streams', in stream order. Their chunks stay spooled behind every other stream's, and
  // one is tried again on the stream's own interval, from retry_max_s doubling to an hour,
  // until the gateway accepts it.
  std::vector<Stream> deferred_streams;

  // The binary streams, counted separately from `records_recorded` because they
  // are not comparable to it: one camera frame is worth tens of thousands of gaze
  // samples in bytes, and a single number covering both answers no question about
  // either.
  std::uint64_t camera_frames = 0;
  std::uint64_t camera_bytes = 0;
  std::uint64_t lidar_scans = 0;
  std::uint64_t lidar_bytes = 0;

  // The pose stream. Samples an object's encoder took, and those it refused (a time before
  // the previous sample's, a value outside int32 at its step: said once per object).
  std::uint64_t pose_samples = 0;
  std::uint64_t pose_samples_refused = 0;
  // Blocks encoded and queued, and their bytes as sent.
  std::uint64_t pose_blocks = 0;
  std::uint64_t pose_block_bytes = 0;
  // Parts handed to the spool, block and manifest parts alike, and the manifest entries sent.
  std::uint64_t pose_parts = 0;
  std::uint64_t pose_manifest_sends = 0;
  // Pose parts the gateway refused (400 and the other payload rejections), each discarded and
  // never retried: a bug in what this SDK sent.
  std::uint64_t pose_rejections = 0;
  // Rotations that quantised to all zeros and were repaired, and blocks an encoder dropped.
  std::uint64_t pose_repaired_rotations = 0;
  std::uint64_t pose_encode_failures = 0;
  // The longest any block took from closing to entering the upload queue, encoding included,
  // in microseconds on the steady clock. The platform requires every closed block encoded and
  // queued within 1 s on the target device.
  std::int64_t pose_encode_lag_max_us = 0;
};

// A Cognitive3D session.
//
// Two threads of its own. The worker cuts parts on their bounds, spools them and
// keeps the marker heartbeat; the uploader drains the spool. They are separate so
// that an upload, however slow, can never delay a part being cut: with one thread
// doing both, every frame recorded during an upload piles into the next part, and
// on an uplink slower than the data rate each part comes out larger than the last.
//
// Contract:
//   * Every Record* call is memory-only. It never performs I/O, never issues an
//     HTTP request, and never waits behind one: parts are serialised and written
//     outside the lock Record* takes. A flush from inside Record* would let a
//     sensor sample block a ROS callback thread on a POST.
//   * All state is mutex-guarded, so Record* may be called from any thread.
//   * Data reaches disk before an upload is attempted, so a lost network loses
//     nothing, and a lost robot at most what was batched since the last part was
//     cut: up to auto_send_interval_s of records, and of camera and lidar up to
//     their flush bounds.
class Session {
 public:
  // Fails, loudly, if the config is invalid. In particular an empty scene id or
  // scene version is a hard error here rather than a warning followed by every
  // subsequent payload being silently discarded.
  static std::unique_ptr<Session> Create(const Config& config,
                                         std::shared_ptr<ITransport> transport,
                                         std::shared_ptr<ISpool> spool,
                                         std::shared_ptr<ILogger> logger,
                                         std::string* error_out = nullptr);

  ~Session();

  Session(const Session&) = delete;
  Session& operator=(const Session&) = delete;

  // Scans the spool for sessions that never received an orderly end, and closes
  // each one with a reconstructed length and an abnormal-end reason. Call once
  // at startup, before Begin. Returns how many were recovered.
  int RecoverOrphanedSessions();

  // Opens a session. Refused, with a logged reason, while one is already open or while
  // the wall clock reads earlier than Config::wall_clock_floor_epoch_s -- an unset clock,
  // whose timestamps the platform accepts and never shows.
  bool Begin();
  // Spools the session's end, removes its marker, and returns: it never waits for an
  // upload. What is spooled uploads in the background; the destructor gives what is
  // left Config::shutdown_drain_s, and anything still left waits for the next start.
  //
  // `end_time`, when given, is the epoch second the session ended at on the wire and the
  // length is measured from `session_timestamp()` to it; it is never placed before the start.
  // Omitted, the end is wall clock now and the length is measured on the steady clock.
  void End(const std::string& reason = "normal", std::optional<double> end_time = std::nullopt);
  bool active() const;
  std::string session_id() const;
  double session_timestamp() const;

  // Session properties. These reach the platform only on the gaze stream, which
  // is not obvious and is the reason a properties-only gaze payload is legal.
  void SetProperty(const std::string& key, const PropertyValue& value);
  void SetParticipantId(const std::string& id);
  void SetSessionName(const std::string& name);

  // ---- Recording -----------------------------------------------------------
  // `t` is always epoch wall-clock seconds. Under simulation the ROS clock can
  // start near zero, and the session id embeds an epoch timestamp, so ROS time
  // must not be what lands here.
  void RecordGaze(double t, const Vec3& position, const Quat& rotation, const Vec3& gaze_point);
  void RecordGaze(double t, const Vec3& position, const Quat& rotation);

  void RecordEvent(const std::string& name, double t, const Vec3& point,
                   const Properties& properties = {},
                   const std::string& dynamic_id = "");

  void RecordSensor(const std::string& name, double t, double value);

  void RegisterDynamic(const std::string& id, const std::string& name,
                       const std::string& mesh, double t,
                       const Vec3& position, const Quat& rotation,
                       const Vec3& scale = Vec3{1, 1, 1});
  void RecordDynamic(const std::string& id, double t, const Vec3& position,
                     const Quat& rotation,
                     const std::optional<Vec3>& scale = std::nullopt,
                     const Properties& properties = {});
  void RemoveDynamic(const std::string& id, double t, const Vec3& position,
                     const Quat& rotation);

  // ---- The binary streams --------------------------------------------------
  // Memory-only like every other Record*: the bytes are buffered and a part is
  // cut on the worker thread when `camera_flush_s`/`camera_flush_bytes` (or the
  // lidar twins) is reached. Nothing here encodes, compresses or writes.
  //
  // `time_ms` is epoch WALL-CLOCK milliseconds, not the seconds the other
  // Record* calls take and never ROS time -- the wire field is milliseconds, and
  // a conversion done here rather than at the caller would be a second place for
  // the clock rule to be got wrong. `seq` is the caller's per-sensor counter,
  // because the caller is the one that knows what a sensor is.
  //
  // `fov` is the camera's field of view for THIS frame (see CameraFov), and the
  // default, 0 by 0, records none. A field of view IsKnownCameraFov refuses is
  // written as 0 by 0 as well -- the frame is kept, since losing a picture over
  // its metadata is the worse failure -- and the first such frame per camera in a
  // session is logged as a warning.
  void RecordCameraFrame(const std::string& camera_id, std::uint64_t time_ms,
                         std::uint32_t seq, const std::string& frame_id, const Pose& pose,
                         const std::string& format, std::uint16_t width,
                         std::uint16_t height, const std::string& bytes,
                         const CameraFov& fov = CameraFov{});
  void RecordLaserScan(const std::string& sensor_id, std::uint64_t time_ms, std::uint32_t seq,
                       const std::string& frame_id, const Pose& pose,
                       const LaserScanData& scan);
  // An ALREADY encoded cloud: `c3d::pointcloud::Encode` is a pure function the
  // caller runs on its own thread, so the compression never lands on this one.
  // A cloud whose `ok` is false is refused here rather than sent empty.
  void RecordPointCloud(const std::string& sensor_id, std::uint64_t time_ms, std::uint32_t seq,
                        const std::string& frame_id, const Pose& pose,
                        const EncodedPointCloud& cloud);

  // ---- The pose stream ------------------------------------------------------
  // Rigged objects, each a manifest of joints and samples of every joint, encoded into blocks
  // of Config::pose_block_interval_s (c3d/pose.hpp) and uploaded to /v0/pose by an upload
  // queue of the session's own. The platform's rules are c3d-stream-formatter's
  // docs/pose/spec.md, §10 for everything this class does with a block once it is encoded.
  //
  // Memory-only like every Record*: both calls only hand their argument to the worker
  // thread, which encodes, one object's blocks at a time and in close order, and queues. A
  // block closes when a sample is at or past its interval, when its interval has passed on
  // the steady clock since its first sample was recorded (so a robot clock that lags the host
  // cannot shrink it), before it would pass the format's sample count, on a manifest send and
  // at End.
  //
  // Parts are cut by the queue alone: when the earliest queued block entered it 11 s ago, or
  // first when the next block would take the blocks' bytes past 5 MiB. Flush() and the
  // auto-send interval never cut one. End() sends the queue as it is, then closes every open
  // block and sends those as another part. A manifest goes out at once in a part of its own.
  // Parts are spooled and retried like the camera's.

  // Sends `manifests`, or sends them again: once per object at the start, and again whenever
  // what one says changes (a joint added or gone, a property changed). The entries of one
  // call are made at one moment and share a part, which carries nothing else and goes out at
  // once; entries of different calls never share one. Each send closes its object's open
  // block, which stays under the previous send, and restarts the object's encoding from a
  // keyframe; the send's `firstSeq` is the seq the object's next block will get. A new joint
  // count or new groups make a new encoder, whose blocks carry on the object's seq.
  //
  // Refused whole, with the reason in *error_out and nothing sent, while no session is open,
  // when ManifestProblem names a problem with an entry, for an id listed twice, for a
  // follower whose root is neither sent earlier in this session nor earlier in this call,
  // and for an id that changes between follower and not.
  bool SetPoseManifests(const std::vector<pose::Manifest>& manifests,
                        std::string* error_out = nullptr);
  bool SetPoseManifest(const pose::Manifest& manifest, std::string* error_out = nullptr);

  // Records one sample of object `manifest_id`: every joint of its latest manifest, in that
  // manifest's order, at `sample.t_us` (epoch microseconds, the caller's clock and the
  // caller's conversion; never ROS time). Refused while no session is open, for an id with no
  // manifest or a follower, and for a joint count other than the manifest's. The encoder's own
  // refusals come later, on the worker, and are counted in Stats::pose_samples_refused.
  bool RecordPoseSample(const std::string& manifest_id, const pose::PoseSample& sample,
                        std::string* error_out = nullptr);

  // One pass of the pose worker on the calling thread: encode what was recorded, close blocks
  // whose interval has passed, check the queue's timer, spool any part it sends. Returns once
  // every pose part decided before it is in the spool. Intended for tests, with
  // Config::pose_clock_us; the worker does this every worker_tick_s otherwise.
  void PumpPose();

  // Serialize whatever is batched and hand it to the spool, on the calling thread.
  // Does not wait for upload. The pose stream's parts are its queue's: see above.
  void Flush();

  // One upload pass on the calling thread, up to 64 chunks. Intended for tests; the
  // upload thread does this continuously otherwise. Does nothing while uploads are
  // paused.
  void PumpUploads();

  // Stop or resume uploading, from any thread. Recording, part cutting and
  // spooling carry on either way. A request already in flight when uploads are
  // paused is allowed to finish. Starts from Config::uploads_paused.
  void SetUploadsPaused(bool paused);
  bool uploads_paused() const;

  Stats stats() const;

 private:
  Session(const Config& config, std::shared_ptr<ITransport> transport,
          std::shared_ptr<ISpool> spool, std::shared_ptr<ILogger> logger);

  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace c3d
