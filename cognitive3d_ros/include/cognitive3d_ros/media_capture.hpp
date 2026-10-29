#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "c3d/h264_encoder.hpp"
#include "c3d/media.hpp"
#include "c3d/sensor_blocks.hpp"
#include "cognitive3d_ros/coordinate_transform.hpp"
#include "cognitive3d_ros/media_config.hpp"
#include "cognitive3d_ros/pointcloud_fields.hpp"
#include "cognitive3d_ros/rate_gate.hpp"
#include "cognitive3d_ros/session_runner.hpp"

namespace cognitive3d_ros {

// Everything a capture needs from the node, as callbacks.
//
// Not a pointer to the node, deliberately: a capture is then drivable by a test
// with four lambdas and no ROS graph, which is what makes the rate gate, the
// encoding refusals and the pose-failure path testable at all.
struct MediaEnvironment {
  // The sensor pose in the reference frame in force, in ROS axes, at the message
  // stamp. False when tf cannot answer yet: the record is held for `pose_wait_s`, and
  // dropped if tf still cannot, because a frame with no pose is a picture with no
  // place in the scene.
  std::function<bool(const std::string& frame, const rclcpp::Time& stamp, c3d::Vec3* position,
                     c3d::Quat* rotation)>
      lookup_pose;
  // Epoch SECONDS for a message stamp, by the clock policy's rule. ROS time never
  // reaches the wire, under use_sim_time or otherwise; this is the one conversion point.
  std::function<double(const rclcpp::Time& stamp)> wire_time;
  // When a message arrived, in seconds on the clock the rate gate runs on; see RateGate. Unset is
  // the steady clock.
  std::function<double()> arrival_time;
  // Said once per sensor, not once per message: at 30 Hz a per-message warning
  // is the log.
  std::function<void(const std::string& text)> warn_once;
  // How many publishers a topic has on the graph now. Unset is not known, and
  // nothing that depends on it is said.
  std::function<std::size_t(const std::string& topic)> publishers;
  // How long a record whose pose `lookup_pose` cannot give yet is held for it, in seconds on the
  // `pose_clock`. Held, not waited for: ServiceHeld retries it. 0 drops it at once.
  double pose_wait_s = 0.0;
  // Seconds on the clock the transforms `lookup_pose` waits for arrive on: the tf buffer's, which
  // under use_sim_time is simulated time, so a simulation slower than real time still gives a held
  // record its whole wait. Unset is `arrival_time`.
  std::function<double()> pose_clock;
  // The real clocks, whatever the clock policy: epoch seconds by the system clock, and seconds by
  // the steady clock. The camera encoder stamps frames by the first, so the two moving apart is a
  // wall-clock step it must restart for, and its restart rules and backoff run on the second; see
  // CameraCapture::FeedFrame. Unset is the real clock.
  std::function<double()> wall_time;
  std::function<double()> steady_time;
};

// Records that passed the rate gate before tf could pose them, kept in arrival order. A driver
// that stamps a frame as it publishes it is ahead of the last transform of its chain, so its pose
// is usually there within a tf period; waiting for it would hold the node's one callback thread,
// and the pose timer with it.
class HeldRecords {
 public:
  // True when the record reached the session.
  using Record = std::function<bool(const c3d::Vec3& position, const c3d::Quat& rotation,
                                    const MediaEnvironment&, SessionRunner&)>;

  // Records now when nothing is held ahead of it and tf can pose it, holds it for
  // `environment.pose_wait_s` otherwise. Every record dropped for want of a pose, this one or a
  // held one, is added to `pose_failures`. True when this record reached the session now.
  bool Offer(const std::string& frame, const rclcpp::Time& stamp, Record record,
             const MediaEnvironment& environment, SessionRunner& runner,
             std::uint64_t* pose_failures);
  // Records, in order, each held record tf can now pose, up to the first it cannot; drops those
  // past their deadline. Drops everything, uncounted, when no session is active.
  void Service(const MediaEnvironment& environment, SessionRunner& runner,
               std::uint64_t* pose_failures);
  void Clear() { held_.clear(); }
  std::size_t size() const { return held_.size(); }

  // How long the node holds a record for its pose. Holding costs memory, not the callback
  // thread, so it covers a driver that stamps as it publishes on a slow chain -- up to a tf period
  // behind, 50 ms at 20 Hz, plus transport -- with room to spare.
  static constexpr double kPoseHoldSeconds = 0.2;
  // Past this many, the oldest is dropped: a chain that stopped would otherwise hold a frame from
  // every arrival for the length of the wait. At 30 records a second a sensor holds about 6, 7 with
  // timer jitter.
  static constexpr std::size_t kMaxHeld = 8;

 private:
  struct Held {
    std::string frame;
    rclcpp::Time stamp;
    double deadline = 0.0;
    Record record;
  };
  std::deque<Held> held_;
};

struct CameraStats {
  // Access units that reached the session, and their encoded bytes.
  std::uint64_t frames = 0;
  std::uint64_t bytes = 0;
  std::uint64_t pose_failures = 0;
  std::uint64_t refused = 0;  // encodings we will not convert, and decode failures
  // Posed frames never fed: the encoder's queue was full, no encoder could start, or the frame
  // came while a restart's backoff ran.
  std::uint64_t dropped_backlog = 0;
  // Frames an encoder accepted and could not finish when it was stopped: the sum of its
  // `abandoned`.
  std::uint64_t dropped_at_stop = 0;
  // Every restart counts in exactly one of these, when the replacement starts: encoder_restarts
  // after a failure or a wedge, gap_restarts after a quiet gap on a healthy encoder, which a
  // paused or time-lapse camera makes, size_restarts after a chunk reached kMaxVideoChunkBytes. A
  // restart after a size or frame change, a wall-clock step, a stop or a Reset counts in none.
  std::uint64_t encoder_restarts = 0;
  std::uint64_t gap_restarts = 0;
  std::uint64_t size_restarts = 0;
  // Frames a size restart could not re-encode: not kept for want of replay_max_mb, or the
  // replacement could not take them.
  std::uint64_t dropped_replay = 0;
  // Frames whose time is earlier than the frame before them: kept in the order they came and with
  // the time they were given, never sorted or clamped.
  std::uint64_t backward_stamps = 0;
  // Access units that arrived with no session to take them.
  //
  // This and dropped_at_stop count for the life of the capture, not per session: the stop that
  // ends a session comes after that session's last telemetry tick, so a per-session count of
  // either would never be recorded by any series. Reset keeps both.
  std::uint64_t dropped_no_session = 0;
};

struct LidarStats {
  std::uint64_t scans = 0;
  std::uint64_t bytes = 0;
  std::uint64_t pose_failures = 0;
  std::uint64_t dropped_oversize = 0;
  std::uint64_t dropped_range = 0;
  // Clouds whose layout we cannot read, and LaserScans with more rays than a scan block holds.
  std::uint64_t refused = 0;
  // Scans or clouds whose time is earlier than the one before them, as CameraStats counts frames.
  std::uint64_t backward_stamps = 0;
};

// The header of whichever of the four message types these two streams take.
// False when `type` is not one of them or the bytes do not deserialize.
//
// `doctor` needs the frame id out of one observed message, and doing it by hand
// there would mean a second place that knows which types are handled.
bool MediaMessageHeader(const std::string& type, const rclcpp::SerializedMessage& message,
                        std_msgs::msg::Header* header);

// The axis sign census of one observed message, for doctor's optical-frame check.
// False when `type` is not a PointCloud2, the bytes do not deserialize, or this
// SDK cannot read the layout -- all three of which are reported by other checks,
// so this one simply does not run.
//
// Beside MediaMessageHeader for the same reason: doing the deserialize in doctor
// would be a second place that knows which types these streams take.
bool MediaPointCloudAxisSigns(const std::string& type, const rclcpp::SerializedMessage& message,
                              PointCloudAxisSigns* signs);

// The field of view of the image a CameraInfo describes, by c3d::CameraFovFromPinhole.
// `k`, `width`, `height` and `roi` are all in full-resolution, unbinned pixels, so an
// ROI moves the principal point and replaces the size, and binning, which shrinks the
// published image and all of those alike, changes no angle. Not recorded (0 by 0) for
// an uncalibrated driver's all-zero `k`.
c3d::CameraFov CameraFovFromCameraInfo(const sensor_msgs::msg::CameraInfo& info);

// Builds the encoder a camera feeds. The default runs the host's ffmpeg; a test hands in its own.
using H264EncoderFactory = std::function<std::unique_ptr<c3d::IH264Encoder>(
    const c3d::H264EncoderSettings&, c3d::OnAccessUnit, c3d::OnEncoderFailure)>;
H264EncoderFactory DefaultH264EncoderFactory();

// One frame as the encoder takes it: bgr24, row after row, exactly width * height * 3 bytes, with
// both dimensions even, which 4:2:0 video needs.
struct PreparedFrame {
  std::string bgr24;
  std::uint16_t width = 0;
  std::uint16_t height = 0;
};

// The frames an encoder was fed whose access units are not yet in a chunk, as yuv420p, half of
// bgr24, by token. Bounded in bytes: a frame that would take it past the budget is not kept.
class ReplayStore {
 public:
  explicit ReplayStore(std::uint64_t budget_bytes) : budget_(budget_bytes) {}
  // False, and nothing kept, past the budget.
  bool Put(std::uint64_t token, std::string yuv420p);
  // False when `token` is not kept.
  bool Copy(std::uint64_t token, std::string* yuv420p) const;
  void Release(std::uint64_t token);
  void Clear();
  std::size_t size() const { return frames_.size(); }
  std::uint64_t bytes() const { return bytes_; }

 private:
  std::map<std::uint64_t, std::string> frames_;
  std::uint64_t bytes_ = 0;
  std::uint64_t budget_ = 0;
};

// One configured camera: the rate gate, the frame preparation, the encoder and the per-sensor
// counters.
//
// The encoder runs on threads of its own and hands each frame back as an access unit; this class
// pairs each with the pose and time its frame was fed with, and records it into the session. The
// node's one callback thread feeds, starts and stops the encoder; the encoder's reader thread
// delivers. One encoder instance at a time, and a new one whenever the stream it makes could not
// continue the old: a chunk that reached its byte cap, a size or frame change, a wall-clock step, a
// quiet gap, a failure or a wedge -- see FeedFrame.
class CameraCapture {
 public:
  CameraCapture(CameraConfig config, std::string ffmpeg_path,
                H264EncoderFactory factory = DefaultH264EncoderFactory());
  // Nothing is delivered or warned from here: the runner pointer and the stored warning are
  // cleared first, then the encoder is killed with Finish(0), then the leftover children swept. A
  // capture destroyed without a stop loses its tail rather than touching a runner that may be
  // gone.
  ~CameraCapture();
  CameraCapture(const CameraCapture&) = delete;
  CameraCapture& operator=(const CameraCapture&) = delete;

  const CameraConfig& config() const { return config_; }
  // A copy, taken under the stats lock: the encoder's reader thread advances frames and bytes
  // while the node's telemetry tick reads them.
  CameraStats stats() const;
  // Per session, like every other translator. The encoder is stopped FIRST, while the session it
  // fed can still take its tail; then the seq counter starts again at 0, the rate gate forgets
  // where it was, the held frames, the warnings and the restart state go, and every count but
  // dropped_at_stop and dropped_no_session starts again at 0.
  void Reset();

  // The field of view the next frame carries: the parameters when set, otherwise
  // the latest CameraInfo's, otherwise none (0 by 0). The parameters win because an
  // operator's measured number beats a driver's calibration, which is often absent.
  c3d::CameraFov CurrentFov() const;
  // The source frames are configured to take their field of view from, as
  // recorded in `c3d.ros.camera.<name>.fov_source`: "parameter", "camera_info"
  // or "none". Not whether one has arrived.
  const char* fov_source() const;
  // Whether the CameraInfo topic is worth subscribing: it is configured and the
  // parameters, which would win, are unset.
  bool wants_camera_info() const;
  // A message from `camera_info_topic`. Kept as the latest field of view whether
  // or not a session is open: it describes the camera, not a session, so Reset
  // keeps it. One with no usable intrinsics is said once, naming the parameters.
  // False when the bytes do not deserialize.
  bool HandleCameraInfo(const rclcpp::SerializedMessage& message,
                        const MediaEnvironment& environment);
  void HandleCameraInfo(const sensor_msgs::msg::CameraInfo& info,
                        const MediaEnvironment& environment);

  // True when a frame was fed to the encoder; false also for one held for its pose, which
  // ServiceHeld feeds. Its access unit reaches the session from the encoder's reader thread.
  // `type` is the graph's type name, so a topic that changes publisher mid-run is handled rather
  // than assumed.
  bool Handle(const std::string& type, const rclcpp::SerializedMessage& message,
              const MediaEnvironment& environment, SessionRunner& runner);
  // Feeds the held frames tf can now pose. Called on a timer as well as from Handle.
  void ServiceHeld(const MediaEnvironment& environment, SessionRunner& runner);
  std::size_t held() const { return held_.size(); }

  // One message as the run prepares it -- converted to bgr24, scaled to max_width_px, cropped to
  // even -- with no gate, no pose, no session and no counter touched. `doctor` reports a camera's
  // size with this: the same code path rather than a second idea of it, which would drift out of
  // step with the run silently.
  bool PrepareFrame(const std::string& type, const rclcpp::SerializedMessage& message,
                    PreparedFrame* out, std::string* why) const;

  // Finish(timeout_s), then releases the instance, so there is no encoder afterwards and the next
  // frame starts a new one with a keyframe. The last access units reach the session before this
  // returns. A failure the stop itself raised has already done its work: the backoff and the
  // start reason are set. Clears the runner pointer and the stored warning. Idempotent: with no
  // encoder it does nothing else.
  void StopEncoder(double timeout_s = c3d::kH264FinishTimeoutSeconds);
  bool has_encoder() const;

  // How long after a crash, a kill or a start that failed no encoder is started, on the steady
  // clock: one spawn per 5 s at most, whatever the cause.
  static constexpr double kEncoderRestartMinSeconds = 5.0;
  // How far the wall clock may move against the steady clock under a running encoder before the
  // encoder is restarted. NTP's slewing stays far under it; a step does not.
  static constexpr double kWallClockStepSeconds = 1.0;
  // Size restarts in one session after which the camera's chunks are made smaller.
  static constexpr int kSizeRestartsToEscalate = 3;

 private:
  // Why the next encoder will start, which decides what that start counts as.
  enum class StartReason { kNone, kFailure, kWedge, kGap, kSize, kOther };
  // What an access unit is recorded with: the frame's time, seq and pose in ROS axes, and its
  // chunk context without the parameter sets, which ride on the unit itself.
  struct Pending {
    c3d::SensorEntry entry;
    c3d::VideoChunkContext context;
  };

  bool PrepareImage(const sensor_msgs::msg::Image& image, PreparedFrame* out,
                    std::string* why) const;
  bool PrepareCompressed(const sensor_msgs::msg::CompressedImage& compressed, PreparedFrame* out,
                         std::string* why) const;
  bool Emit(const std_msgs::msg::Header& header, PreparedFrame frame,
            const MediaEnvironment& environment, SessionRunner& runner);
  // The rate gate on the message's arrival, saying once when its stamp is zero.
  bool AcceptHeader(const std_msgs::msg::Header& header, const MediaEnvironment& environment);
  // One posed frame: the restart rules, then the encoder. True when the encoder took it.
  bool FeedFrame(PreparedFrame frame, const c3d::SensorEntry& entry,
                 const c3d::VideoChunkContext& context, const MediaEnvironment& environment,
                 SessionRunner& runner);
  enum class Started { kYes, kFailed, kTooLarge };
  // A new instance for frames of this size and frame, with the chunk length CameraChunkSeconds
  // gives that size, counted by pending_start_reason_. kFailed when it cannot start: the backoff
  // is then set and the reason warned once. kTooLarge, with nothing started and no backoff, when not
  // even the shortest chunk fits: said once a session.
  Started StartEncoder(std::uint16_t width, std::uint16_t height, const std::string& frame_id,
                       double wall_offset, double now, const MediaEnvironment& environment);
  // The size row: kills the instance whose chunk refused a unit, starts another and re-encodes,
  // in order, every frame from the refused one on.
  void RestartForSize(double now, double wall_offset, const MediaEnvironment& environment);
  // StopEncoder, keeping the pending frames and the replay store when `for_replay`.
  void StopEncoder(double timeout_s, bool for_replay);
  void OnAccessUnit(c3d::EncodedAccessUnit unit);
  void OnEncoderFailure(const c3d::EncoderFailure& failure);
  void AddToStats(std::uint64_t CameraStats::*field, std::uint64_t amount = 1);

  CameraConfig config_;
  std::string ffmpeg_path_;
  H264EncoderFactory factory_;
  std::uint32_t seq_ = 0;
  // The time of the last frame given a seq, for backward_stamps.
  std::uint64_t last_time_ms_ = 0;
  RateGate gate_;
  // One per warning: a warning that shared another's flag would be silenced by it.
  bool frame_id_warned_ = false;
  bool refusal_warned_ = false;
  bool unstamped_warned_ = false;
  bool start_warned_ = false;
  bool chunk_warned_ = false;
  bool too_large_warned_ = false;
  bool clock_step_warned_ = false;
  c3d::CameraFov camera_info_fov_;
  bool camera_info_warned_ = false;
  HeldRecords held_;

  mutable std::mutex stats_mu_;
  CameraStats stats_;

  // Everything below is read and written under pending_mu_: the reader thread pops tokens and
  // reads the runner, and on_failure arrives on the reader thread or inside Finish. It is never
  // held across Start, Feed, Finish, a runner call or a warning: the reader takes it to pop a
  // token, so a stop that held it would deadlock. The instance itself is started, fed and
  // stopped by its one owner: the node's callback thread, or the thread StopCameraEncoders gives
  // this capture.
  mutable std::mutex pending_mu_;
  std::unique_ptr<c3d::IH264Encoder> encoder_;  // null after any StopEncoder
  std::uint16_t encoder_width_ = 0;
  std::uint16_t encoder_height_ = 0;
  std::string encoder_frame_id_;
  // The wall clock less the steady clock when the running encoder started.
  double encoder_wall_offset_ = 0.0;
  // The running encoder's chunk length, or the last one's: the quiet-gap and wedge rows wait this
  // long.
  double chunk_s_ = c3d::kVideoChunkSeconds;
  // What c3d.ros.camera.<name>.chunk_seconds and .max_bitrate_kilobits_per_second were last set to
  // in this session.
  double recorded_chunk_s_ = c3d::kVideoChunkSeconds;
  int recorded_bitrate_kbps_ = 0;
  // Every kSizeRestartsToEscalate size restarts in a session, the next encoder's chunk is half
  // the last one's, down to kMinCameraChunkSeconds, and after that the bitrate is capped.
  int size_restarts_since_escalation_ = 0;
  double chunk_cap_s_ = c3d::kVideoChunkSeconds;
  bool escalated_bitrate_cap_ = false;
  bool escalation_warned_ = false;
  std::map<std::uint64_t, Pending> pending_;
  std::uint64_t next_token_ = 0;
  ReplayStore replay_;
  // The first token the session refused, and the units delivered since, held for the size row to
  // re-encode; null when none is waiting.
  std::optional<std::uint64_t> refused_from_;
  std::uint64_t held_units_ = 0;
  // The size row's own stop is under way: its kill is no failure.
  bool size_stop_ = false;
  bool size_warned_ = false;
  // Set by FeedFrame and cleared by StopEncoder; a unit that finds it null is counted in
  // dropped_no_session.
  SessionRunner* runner_ = nullptr;
  std::function<void(const std::string&)> warn_once_;
  // The steady clock on_failure reads its backoff from, so every time here is on one clock.
  std::function<double()> steady_time_;
  bool failed_ = false;  // set by on_failure; cleared by StopEncoder and Reset
  // The last stop's Finish had to kill. Cleared by the replacement's first delivered unit: an
  // encoder that delivers is healthy, and its drain owes nothing to the one before it.
  bool previous_stop_killed_ = false;
  // last_accepted is set when an encoder starts and advanced only by a Feed that took the frame.
  double last_accepted_ = 0.0;
  double last_attempt_ = 0.0;
  bool last_attempt_refused_ = false;
  double backoff_until_ = 0.0;  // 0 when none
  StartReason pending_start_reason_ = StartReason::kNone;
  // The thread a StopEncoder runs Finish on, while it does: on_failure there is the stop's own
  // report, anywhere else an instance that died while running.
  std::thread::id stopping_on_;
  bool failure_warned_ = false;
  bool no_session_warned_ = false;
};

// Stops every camera's encoder at once, each on a thread of its own, then sweeps the leftover
// children once: N cameras cost one kH264FinishTimeoutSeconds plus kH264ReapSeconds at worst, not
// N of them, and a child that died since the last stop is reaped even when no session follows.
void StopCameraEncoders(const std::vector<std::unique_ptr<CameraCapture>>& cameras);

// One configured lidar. LaserScan and PointCloud2 share the rate gate, the pose
// lookup and the counters; only the payload differs.
class LidarCapture {
 public:
  explicit LidarCapture(LidarConfig config);

  const LidarConfig& config() const { return config_; }
  // A copy, as CameraCapture's is, for the same reader.
  LidarStats stats() const { return stats_; }
  void Reset();

  // As CameraCapture::Handle, and held scans the same way.
  bool Handle(const std::string& type, const rclcpp::SerializedMessage& message,
              const MediaEnvironment& environment, SessionRunner& runner);
  void ServiceHeld(const MediaEnvironment& environment, SessionRunner& runner);
  std::size_t held() const { return held_.size(); }

  // What this configuration WOULD send for one message, with no gate, no pose, no session and
  // no counter touched -- the same code path as the run rather than a second estimate, for the
  // reason CameraCapture::PrepareFrame is. A point cloud is encoded through
  // `c3d::pointcloud::Encode` exactly as it is at run time, so the figure reflects the voxel leaf
  // and zstd level the operator configured. A LaserScan is sized as a block of one scan, an upper
  // bound on what it adds to a real block.
  bool MeasureEncodedScan(const std::string& type, const rclcpp::SerializedMessage& message,
                          PoseConvention convention, std::uint64_t* bytes,
                          std::string* why) const;

 private:
  bool ResolveFrame(const std_msgs::msg::Header& header, const MediaEnvironment& environment,
                    std::string* frame_id);
  bool AcceptHeader(const std_msgs::msg::Header& header, const MediaEnvironment& environment);

  LidarConfig config_;
  LidarStats stats_;
  std::uint32_t seq_ = 0;
  std::uint64_t last_time_ms_ = 0;
  RateGate gate_;
  // One per warning, as the camera's are.
  bool frame_id_warned_ = false;
  bool refusal_warned_ = false;
  bool color_warned_ = false;
  bool unstamped_warned_ = false;
  HeldRecords held_;
};

}  // namespace cognitive3d_ros
