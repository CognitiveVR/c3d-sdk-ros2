#include "cognitive3d_ros/media_capture.hpp"

// cv_bridge is the one dependency whose header is spelled differently across the two tier 1
// distros: Humble ships `cv_bridge.h` and nothing else, Jazzy ships both and deprecates the `.h`.
// A plain `.hpp` therefore fails to compile the whole package on Humble, and a plain `.h` warns on
// every Jazzy build.
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include "c3d/scan_block.hpp"
#include "cognitive3d_ros/image_codec.hpp"
#include "cognitive3d_ros/pointcloud_fields.hpp"
#include "cognitive3d_ros/translators.hpp"

namespace cognitive3d_ros {
namespace {

double StampSeconds(const std_msgs::msg::Header& header) {
  return static_cast<double>(header.stamp.sec) +
         static_cast<double>(header.stamp.nanosec) * 1e-9;
}

std::uint64_t ToMilliseconds(double wall_seconds) {
  if (!std::isfinite(wall_seconds) || wall_seconds <= 0.0) return 0;
  return static_cast<std::uint64_t>(std::llround(wall_seconds * 1000.0));
}

double SteadySeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// A number of seconds as a sentence says it: "5", "2.5".
std::string SecondsText(double seconds) {
  std::ostringstream text;
  text << seconds;
  return text.str();
}

double ArrivalSeconds(const MediaEnvironment& environment) {
  return environment.arrival_time ? environment.arrival_time() : SteadySeconds();
}

// The wall clock less the steady clock, both real: it moves only when the wall clock steps.
double WallClockOffset(const MediaEnvironment& environment) {
  using namespace std::chrono;
  const double wall = environment.wall_time
                          ? environment.wall_time()
                          : duration<double>(system_clock::now().time_since_epoch()).count();
  return wall - (environment.steady_time ? environment.steady_time() : SteadySeconds());
}

double PoseClockSeconds(const MediaEnvironment& environment) {
  return environment.pose_clock ? environment.pose_clock() : ArrivalSeconds(environment);
}

std::uint64_t WireMilliseconds(const MediaEnvironment& environment,
                               const std_msgs::msg::Header& header) {
  return ToMilliseconds(environment.wire_time ? environment.wire_time(header.stamp) : 0.0);
}

// The frame the pose is FOR: the configured one, else the message's own. A
// message with neither is refused rather than posed against a guess -- a pose in
// the wrong frame is not a lesser version of the right one.
bool ResolveFrameId(const std::string& configured, const std_msgs::msg::Header& header,
                    std::string* out) {
  *out = configured.empty() ? header.frame_id : configured;
  return !out->empty();
}

// INTER_AREA, which is the right filter for shrinking: the bilinear default
// samples rather than averages, so a downscaled sensor image comes out aliased
// and video of an aliased image is both larger and worse.
cv::Mat ShrinkToWidth(const cv::Mat& source, int max_width_px) {
  if (max_width_px <= 0 || source.cols <= max_width_px) return source;
  const int height = std::max(1, static_cast<int>(std::lround(
                                     static_cast<double>(source.rows) * max_width_px /
                                     static_cast<double>(source.cols))));
  cv::Mat resized;
  cv::resize(source, resized, cv::Size(max_width_px, height), 0, 0, cv::INTER_AREA);
  return resized;
}

// The frame the encoder takes: scaled to `max_width_px`, then cropped to even by dropping the last
// column and row, as one contiguous bgr24 buffer.
bool PrepareDecoded(const cv::Mat& bgr, int max_width_px, PreparedFrame* out, std::string* why) {
  const cv::Mat shrunk = ShrinkToWidth(bgr, max_width_px);
  const int width = shrunk.cols & ~1;
  const int height = shrunk.rows & ~1;
  if (width < 2 || height < 2) {
    *why = "a frame of " + std::to_string(shrunk.cols) + "x" + std::to_string(shrunk.rows) +
           " is too small to encode";
    return false;
  }
  // The wire's width and height are UInt16.
  if (width > 65535 || height > 65535) {
    *why = "a frame of " + std::to_string(shrunk.cols) + "x" + std::to_string(shrunk.rows) +
           " is larger than the 65535 pixels a side the wire carries; set max_width_px";
    return false;
  }
  const std::size_t row_bytes = static_cast<std::size_t>(width) * 3;
  out->bgr24.resize(row_bytes * static_cast<std::size_t>(height));
  for (int row = 0; row < height; ++row) {
    std::memcpy(&out->bgr24[row_bytes * static_cast<std::size_t>(row)], shrunk.ptr(row),
                row_bytes);
  }
  out->width = static_cast<std::uint16_t>(width);
  out->height = static_cast<std::uint16_t>(height);
  return true;
}

std::string Yuv420pOf(const std::string& bgr24, int width, int height) {
  const cv::Mat bgr(height, width, CV_8UC3, const_cast<char*>(bgr24.data()));
  cv::Mat yuv;
  cv::cvtColor(bgr, yuv, cv::COLOR_BGR2YUV_I420);
  return std::string(reinterpret_cast<const char*>(yuv.data), yuv.total() * yuv.elemSize());
}

std::string Bgr24Of(const std::string& yuv420p, int width, int height) {
  const cv::Mat yuv(height * 3 / 2, width, CV_8UC1, const_cast<char*>(yuv420p.data()));
  cv::Mat bgr;
  cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_I420);
  return std::string(reinterpret_cast<const char*>(bgr.data), bgr.total() * bgr.elemSize());
}

constexpr std::uint64_t kMebibyte = 1024ull * 1024ull;

}  // namespace

bool ReplayStore::Put(std::uint64_t token, std::string yuv420p) {
  Release(token);
  if (bytes_ + yuv420p.size() > budget_) return false;
  bytes_ += yuv420p.size();
  frames_[token] = std::move(yuv420p);
  return true;
}

bool ReplayStore::Copy(std::uint64_t token, std::string* yuv420p) const {
  const auto found = frames_.find(token);
  if (found == frames_.end()) return false;
  *yuv420p = found->second;
  return true;
}

void ReplayStore::Release(std::uint64_t token) {
  const auto found = frames_.find(token);
  if (found == frames_.end()) return;
  bytes_ -= found->second.size();
  frames_.erase(found);
}

void ReplayStore::Clear() {
  frames_.clear();
  bytes_ = 0;
}

bool HeldRecords::Offer(const std::string& frame, const rclcpp::Time& stamp, Record record,
                        const MediaEnvironment& environment, SessionRunner& runner,
                        std::uint64_t* pose_failures) {
  Service(environment, runner, pose_failures);
  if (!environment.lookup_pose) {
    ++*pose_failures;
    return false;
  }
  if (held_.empty()) {
    c3d::Vec3 position;
    c3d::Quat rotation;
    if (environment.lookup_pose(frame, stamp, &position, &rotation)) {
      return record(position, rotation, environment, runner);
    }
  }
  if (environment.pose_wait_s <= 0.0) {
    ++*pose_failures;
    return false;
  }
  if (held_.size() >= kMaxHeld) {
    held_.pop_front();
    ++*pose_failures;
  }
  held_.push_back(Held{frame, stamp, PoseClockSeconds(environment) + environment.pose_wait_s,
                       std::move(record)});
  return false;
}

void HeldRecords::Service(const MediaEnvironment& environment, SessionRunner& runner,
                          std::uint64_t* pose_failures) {
  if (held_.empty()) return;
  if (!runner.active() || !environment.lookup_pose) {
    held_.clear();
    return;
  }
  const double now = PoseClockSeconds(environment);
  while (!held_.empty()) {
    Held& front = held_.front();
    c3d::Vec3 position;
    c3d::Quat rotation;
    if (environment.lookup_pose(front.frame, front.stamp, &position, &rotation)) {
      Record record = std::move(front.record);
      held_.pop_front();
      record(position, rotation, environment, runner);
      continue;
    }
    // In order: a record is never written ahead of one that arrived before it.
    if (now < front.deadline) return;
    held_.pop_front();
    ++*pose_failures;
  }
}

bool MediaMessageHeader(const std::string& type, const rclcpp::SerializedMessage& message,
                        std_msgs::msg::Header* header) {
  if (type == "sensor_msgs/msg/Image") {
    sensor_msgs::msg::Image image;
    if (!Deserialize(message, &image)) return false;
    *header = image.header;
    return true;
  }
  if (type == "sensor_msgs/msg/CompressedImage") {
    sensor_msgs::msg::CompressedImage compressed;
    if (!Deserialize(message, &compressed)) return false;
    *header = compressed.header;
    return true;
  }
  if (type == "sensor_msgs/msg/LaserScan") {
    sensor_msgs::msg::LaserScan scan;
    if (!Deserialize(message, &scan)) return false;
    *header = scan.header;
    return true;
  }
  if (type == "sensor_msgs/msg/PointCloud2") {
    sensor_msgs::msg::PointCloud2 cloud;
    if (!Deserialize(message, &cloud)) return false;
    *header = cloud.header;
    return true;
  }
  return false;
}

bool MediaPointCloudAxisSigns(const std::string& type, const rclcpp::SerializedMessage& message,
                              PointCloudAxisSigns* signs) {
  if (type != "sensor_msgs/msg/PointCloud2") return false;
  sensor_msgs::msg::PointCloud2 cloud;
  if (!Deserialize(message, &cloud)) return false;
  const PointCloudLayout layout = ResolvePointCloudLayout(cloud);
  if (!layout.ok) return false;
  *signs = CountPointCloudAxisSigns(cloud, layout);
  return true;
}

H264EncoderFactory DefaultH264EncoderFactory() {
  return [](const c3d::H264EncoderSettings& settings, c3d::OnAccessUnit on_access_unit,
            c3d::OnEncoderFailure on_failure) {
    return c3d::NewFfmpegH264Encoder(settings, std::move(on_access_unit), std::move(on_failure));
  };
}

CameraCapture::CameraCapture(CameraConfig config, std::string ffmpeg_path,
                             H264EncoderFactory factory)
    : config_(std::move(config)),
      ffmpeg_path_(std::move(ffmpeg_path)),
      factory_(std::move(factory)),
      replay_(static_cast<std::uint64_t>(config_.replay_max_mb) * kMebibyte) {}

CameraCapture::~CameraCapture() {
  std::unique_ptr<c3d::IH264Encoder> encoder;
  {
    std::lock_guard<std::mutex> lock(pending_mu_);
    runner_ = nullptr;
    warn_once_ = nullptr;
    arrival_time_ = nullptr;
    encoder = std::move(encoder_);
  }
  if (encoder) encoder->Finish(0.0);
  encoder.reset();
  c3d::SweepLeftoverChildren();
}

CameraStats CameraCapture::stats() const {
  std::lock_guard<std::mutex> lock(stats_mu_);
  return stats_;
}

void CameraCapture::AddToStats(std::uint64_t CameraStats::*field, std::uint64_t amount) {
  std::lock_guard<std::mutex> lock(stats_mu_);
  stats_.*field += amount;
}

bool CameraCapture::has_encoder() const {
  std::lock_guard<std::mutex> lock(pending_mu_);
  return encoder_ != nullptr;
}

void CameraCapture::Reset() {
  StopEncoder();
  {
    std::lock_guard<std::mutex> lock(stats_mu_);
    CameraStats fresh;
    fresh.dropped_at_stop = stats_.dropped_at_stop;
    fresh.dropped_no_session = stats_.dropped_no_session;
    stats_ = fresh;
  }
  seq_ = 0;
  gate_.Reset();
  held_.Clear();
  frame_id_warned_ = false;
  refusal_warned_ = false;
  unstamped_warned_ = false;
  start_warned_ = false;
  chunk_warned_ = false;
  too_large_warned_ = false;
  clock_step_warned_ = false;
  std::lock_guard<std::mutex> lock(pending_mu_);
  failed_ = false;
  previous_stop_killed_ = false;
  last_accepted_ = 0.0;
  last_attempt_ = 0.0;
  last_attempt_refused_ = false;
  backoff_until_ = 0.0;
  pending_start_reason_ = StartReason::kNone;
  failure_warned_ = false;
  no_session_warned_ = false;
  size_warned_ = false;
  recorded_chunk_s_ = c3d::kVideoChunkSeconds;
}

c3d::CameraFov CameraFovFromCameraInfo(const sensor_msgs::msg::CameraInfo& info) {
  // `k` is row-major: fx at 0, cx at 2, fy at 4, cy at 5. An ROI axis of 0 is the
  // full image on that axis, by the message's own definition.
  const bool roi_x = info.roi.width > 0;
  const bool roi_y = info.roi.height > 0;
  const double x0 = roi_x ? static_cast<double>(info.roi.x_offset) : 0.0;
  const double y0 = roi_y ? static_cast<double>(info.roi.y_offset) : 0.0;
  const double width = static_cast<double>(roi_x ? info.roi.width : info.width);
  const double height = static_cast<double>(roi_y ? info.roi.height : info.height);
  return c3d::CameraFovFromPinhole(info.k[0], info.k[4], info.k[2] - x0, info.k[5] - y0, width,
                                   height);
}

c3d::CameraFov CameraCapture::CurrentFov() const {
  if (config_.fov_set()) {
    return c3d::CameraFov{static_cast<float>(config_.hfov_degrees),
                          static_cast<float>(config_.vfov_degrees)};
  }
  if (!config_.camera_info_topic.empty()) return camera_info_fov_;
  return c3d::CameraFov{};
}

const char* CameraCapture::fov_source() const {
  if (config_.fov_set()) return "parameter";
  return config_.camera_info_topic.empty() ? "none" : "camera_info";
}

bool CameraCapture::wants_camera_info() const {
  return !config_.fov_set() && !config_.camera_info_topic.empty();
}

bool CameraCapture::HandleCameraInfo(const rclcpp::SerializedMessage& message,
                                     const MediaEnvironment& environment) {
  sensor_msgs::msg::CameraInfo info;
  if (!Deserialize(message, &info)) return false;
  HandleCameraInfo(info, environment);
  return true;
}

void CameraCapture::HandleCameraInfo(const sensor_msgs::msg::CameraInfo& info,
                                     const MediaEnvironment& environment) {
  camera_info_fov_ = CameraFovFromCameraInfo(info);
  if (c3d::IsKnownCameraFov(camera_info_fov_) || camera_info_warned_ || !wants_camera_info() ||
      !environment.warn_once) {
    return;
  }
  camera_info_warned_ = true;
  environment.warn_once(
      "camera '" + config_.name + "': " + config_.camera_info_topic +
      " carries no usable intrinsics (fx " + std::to_string(info.k[0]) + ", fy " +
      std::to_string(info.k[4]) + ", " + std::to_string(info.width) + "x" +
      std::to_string(info.height) +
      "; an uncalibrated driver publishes zeros), so its frames carry no field of view. Set "
      "camera." + config_.name + ".hfov_degrees and vfov_degrees");
}

bool CameraCapture::AcceptHeader(const std_msgs::msg::Header& header,
                                 const MediaEnvironment& environment) {
  if (StampSeconds(header) <= 0.0 && !unstamped_warned_ && environment.warn_once) {
    unstamped_warned_ = true;
    environment.warn_once("camera '" + config_.name + "' on " + config_.topic +
                          " publishes frames with a zero header stamp: they are placed at their "
                          "arrival and posed at the latest transform");
  }
  return gate_.Accept(ArrivalSeconds(environment), config_.rate_hz);
}

bool CameraCapture::Emit(const std_msgs::msg::Header& header, PreparedFrame frame,
                         const MediaEnvironment& environment, SessionRunner& runner) {
  std::string frame_id;
  if (!ResolveFrameId(config_.frame, header, &frame_id)) {
    AddToStats(&CameraStats::pose_failures);
    if (!frame_id_warned_ && environment.warn_once) {
      frame_id_warned_ = true;
      environment.warn_once("camera '" + config_.name + "' on " + config_.topic +
                            " publishes an empty header.frame_id and no camera." + config_.name +
                            ".frame is set, so no pose can be resolved for its frames");
    }
    return false;
  }

  const std::uint64_t time_ms = WireMilliseconds(environment, header);
  auto record = [this, frame_id, time_ms, frame = std::move(frame)](
                    const c3d::Vec3& position, const c3d::Quat& rotation,
                    const MediaEnvironment& environment, SessionRunner& runner) mutable {
    const bool first_of_session = seq_ == 0;
    const c3d::CameraFov fov = CurrentFov();
    c3d::SensorEntry entry;
    entry.time_ms = time_ms;
    // Every posed frame takes a seq, fed or not, so a frame the encoder dropped is a gap in seq
    // rather than a hole in the chunk.
    entry.seq = seq_++;
    entry.pose = c3d::Pose{position, rotation};
    c3d::VideoChunkContext context;
    context.frame_id = frame_id;
    context.width = frame.width;
    context.height = frame.height;
    context.nominal_fps = static_cast<float>(config_.rate_hz);
    context.hfov_deg = fov.hfov_deg;
    context.vfov_deg = fov.vfov_deg;
    const bool fed = FeedFrame(std::move(frame), entry, context, environment, runner);
    // A CameraInfo topic nothing publishes leaves every frame of the session without a field of
    // view, which the session's fov_source does not show. Not said of a publisher that has not
    // sent yet: its first CameraInfo reaches the frames after it.
    if (first_of_session && wants_camera_info() && !c3d::IsKnownCameraFov(fov) &&
        environment.publishers && environment.warn_once &&
        environment.publishers(config_.camera_info_topic) == 0) {
      environment.warn_once("camera '" + config_.name + "': its frames take their field of view "
                            "from " + config_.camera_info_topic + ", which has no publisher, so "
                            "they carry none. Set camera." + config_.name + ".hfov_degrees and "
                            "vfov_degrees, or point camera." + config_.name +
                            ".camera_info_topic at the driver's CameraInfo");
    }
    return fed;
  };
  std::uint64_t pose_failures = 0;
  const bool fed = held_.Offer(frame_id, header.stamp, std::move(record), environment, runner,
                               &pose_failures);
  if (pose_failures > 0) AddToStats(&CameraStats::pose_failures, pose_failures);
  return fed;
}

void CameraCapture::ServiceHeld(const MediaEnvironment& environment, SessionRunner& runner) {
  std::uint64_t pose_failures = 0;
  held_.Service(environment, runner, &pose_failures);
  if (pose_failures > 0) AddToStats(&CameraStats::pose_failures, pose_failures);
}

// The restart rules, in precedence order. Every one reads the same `now`, the arrival clock read
// once for this frame, and after a stop the frame goes on down the table, so a restart and the feed
// it was for happen on one frame:
//
//   1. under a backoff: the frame is dropped; nothing starts or stops
//   2. the running instance has failed: stopped at once, unclassified -- its on_failure already
//      set the backoff and the reason
//   3. the session refused a unit because it would have taken its chunk past
//      c3d::kMaxVideoChunkBytes: killed at once, with no backoff and nothing counted as dropped,
//      and replaced by an instance that re-encodes, in order and under the same tokens, every
//      frame from the refused one on, so its first is a keyframe that opens the next chunk;
//      reason `size`, said once a session
//   4. a different size or frame: stopped with the full timeout, reason `other`
//   5. the wall clock moved more than kWallClockStepSeconds against the steady clock since the
//      encoder started: stopped with the full timeout, reason `other`, said once a session. The
//      encoder stamps frames by the wall clock and cuts a chunk every 3 s of stamps, so a step
//      forward would make every frame a chunk of its own, and a step back would cut no chunk
//      until the clock had made the step up again
//   6. nothing accepted for a chunk's length while the refusals went on through it: wedged,
//      stopped at once, reason `wedge`, backed off
//   7. nothing accepted for a chunk's length and no refusals through it: a quiet gap, stopped
//      with the full timeout, or with the reap time alone when the last stop had to kill and its
//      replacement has delivered nothing since
//   8. no encoder and no backoff: started, and counted by the reason the last stop left
//
// A stop that had to kill turns the reason into `failure` and sets the backoff from on_failure,
// whatever row asked for the stop, so a kill is never counted as a gap or a size change. Row 3's
// kill is the one exception: it is on purpose, and everything it abandons is re-encoded.
bool CameraCapture::FeedFrame(PreparedFrame frame, const c3d::SensorEntry& entry,
                              const c3d::VideoChunkContext& context,
                              const MediaEnvironment& environment, SessionRunner& runner) {
  const double now = ArrivalSeconds(environment);
  const double wall_offset = WallClockOffset(environment);
  const auto bind = [&]() {
    runner_ = &runner;
    warn_once_ = environment.warn_once;
    arrival_time_ = environment.arrival_time;
  };

  bool stop = false;
  bool refused = false;
  bool wedged = false;
  double wall_step = 0.0;  // non-zero when row 5 stops the encoder
  double stop_timeout = c3d::kH264FinishTimeoutSeconds;
  double chunk_s = c3d::kVideoChunkSeconds;
  {
    std::lock_guard<std::mutex> lock(pending_mu_);
    bind();
    chunk_s = chunk_s_;
    if (now < backoff_until_) {
      // Row 1.
      AddToStats(&CameraStats::dropped_backlog);
      return false;
    }
    if (encoder_) {
      if (failed_) {
        // Row 2.
        stop = true;
        stop_timeout = 0.0;
      } else if (refused_from_) {
        // Row 3.
        refused = true;
      } else if (frame.width != encoder_width_ || frame.height != encoder_height_ ||
                 context.frame_id != encoder_frame_id_) {
        // Row 4.
        stop = true;
        pending_start_reason_ = StartReason::kOther;
      } else if (std::fabs(wall_offset - encoder_wall_offset_) > kWallClockStepSeconds) {
        // Row 5.
        stop = true;
        wall_step = wall_offset - encoder_wall_offset_;
        pending_start_reason_ = StartReason::kOther;
      } else if (now - last_accepted_ >= chunk_s_) {
        stop = true;
        if (last_attempt_refused_ && now - last_attempt_ < chunk_s_) {
          // Row 6.
          wedged = true;
          stop_timeout = 0.0;
          pending_start_reason_ = StartReason::kWedge;
        } else {
          // Row 7.
          stop_timeout =
              previous_stop_killed_ ? c3d::kH264ReapSeconds : c3d::kH264FinishTimeoutSeconds;
          pending_start_reason_ = StartReason::kGap;
        }
      }
    }
  }
  if (refused) {
    RestartForSize(now, wall_offset, environment);
    // The replacement goes on down the table with this frame, which may itself be a size or frame
    // change.
    return FeedFrame(std::move(frame), entry, context, environment, runner);
  }
  if (stop) StopEncoder(stop_timeout);
  if (wall_step != 0.0 && !clock_step_warned_ && environment.warn_once) {
    clock_step_warned_ = true;
    environment.warn_once("camera '" + config_.name + "': the wall clock stepped " +
                          (wall_step > 0.0 ? "forward" : "back") + " by " +
                          SecondsText(std::fabs(wall_step)) +
                          " seconds while its encoder ran. The encoder times its " +
                          SecondsText(chunk_s) +
                          "-second chunks by the wall clock, so it was restarted");
  }

  bool start = false;
  {
    std::lock_guard<std::mutex> lock(pending_mu_);
    bind();
    if (wedged) backoff_until_ = now + kEncoderRestartMinSeconds;
    if (!encoder_) {
      if (now < backoff_until_) {
        // Row 1, for a backoff the stop above just raised.
        AddToStats(&CameraStats::dropped_backlog);
        return false;
      }
      start = true;
    }
  }

  // Row 8.
  if (start) {
    const Started started =
        StartEncoder(frame.width, frame.height, context.frame_id, wall_offset, now, environment);
    if (started == Started::kTooLarge) {
      AddToStats(&CameraStats::refused);
      return false;
    }
    if (started == Started::kFailed) {
      AddToStats(&CameraStats::dropped_backlog);
      return false;
    }
  }

  // Kept, before the feed moves the frame, for a size restart to re-encode.
  std::string replay = Yuv420pOf(frame.bgr24, frame.width, frame.height);
  c3d::IH264Encoder* encoder = nullptr;
  std::uint64_t token = 0;
  {
    std::lock_guard<std::mutex> lock(pending_mu_);
    bind();
    encoder = encoder_.get();
    token = next_token_++;
    pending_[token] = Pending{entry, context};
    replay_.Put(token, std::move(replay));
  }
  // Unlocked: the encoder may deliver this very frame's unit on another thread, or on this one.
  const bool fed = encoder->Feed(std::move(frame.bgr24), token);
  {
    std::lock_guard<std::mutex> lock(pending_mu_);
    last_attempt_ = now;
    last_attempt_refused_ = !fed;
    if (fed) {
      last_accepted_ = now;
    } else {
      pending_.erase(token);
      replay_.Release(token);
    }
  }
  if (!fed) AddToStats(&CameraStats::dropped_backlog);
  return fed;
}

CameraCapture::Started CameraCapture::StartEncoder(std::uint16_t width, std::uint16_t height,
                                                   const std::string& frame_id,
                                                   double wall_offset, double now,
                                                   const MediaEnvironment& environment) {
  const double chunk_s = CameraChunkSeconds(config_, width, height);
  if (chunk_s == 0.0) {
    if (!too_large_warned_ && environment.warn_once) {
      too_large_warned_ = true;
      environment.warn_once("camera '" + config_.name + "' is refused: " +
                            CameraChunkTooLarge(config_, width, height) + ". Its frames are " +
                            "counted in c3d.camera." + config_.name + ".refused");
    }
    return Started::kTooLarge;
  }
  if (chunk_s < c3d::kVideoChunkSeconds && !chunk_warned_ && environment.warn_once) {
    chunk_warned_ = true;
    environment.warn_once("camera '" + config_.name + "': at " + std::to_string(width) + "x" +
                          std::to_string(height) + ", crf " + std::to_string(config_.crf) +
                          " and " + SecondsText(config_.rate_hz) + " Hz a " +
                          SecondsText(c3d::kVideoChunkSeconds) + "-second chunk is estimated " +
                          "past the bytes a chunk is sized to, so its chunks are " +
                          SecondsText(chunk_s) + " seconds. Raise crf, lower max_width_px or "
                          "set max_bitrate_kbps for longer ones");
  }
  StartReason reason = StartReason::kNone;
  {
    std::lock_guard<std::mutex> lock(pending_mu_);
    reason = pending_start_reason_;
  }
  c3d::H264EncoderSettings settings;
  settings.ffmpeg_path = ffmpeg_path_;
  settings.preset = config_.preset;
  settings.crf = config_.crf;
  settings.max_bitrate_kbps = config_.max_bitrate_kbps;
  settings.width = width;
  settings.height = height;
  settings.fps = config_.rate_hz;
  settings.chunk_s = chunk_s;
  std::unique_ptr<c3d::IH264Encoder> encoder =
      factory_(settings, [this](c3d::EncodedAccessUnit unit) { OnAccessUnit(std::move(unit)); },
               [this](const c3d::EncoderFailure& failure) { OnEncoderFailure(failure); });
  std::string error = encoder ? std::string() : std::string("no encoder was built");
  if (!encoder || !encoder->Start(&error)) {
    encoder.reset();
    {
      std::lock_guard<std::mutex> lock(pending_mu_);
      backoff_until_ = now + kEncoderRestartMinSeconds;
    }
    if (!start_warned_ && environment.warn_once) {
      start_warned_ = true;
      environment.warn_once("camera '" + config_.name + "': the encoder could not start (" +
                            error + "); its frames are dropped, and it is tried again every " +
                            SecondsText(kEncoderRestartMinSeconds) +
                            " seconds. Check that ffmpeg_path '" + ffmpeg_path_ +
                            "' still runs");
    }
    return Started::kFailed;
  }
  SessionRunner* runner = nullptr;
  {
    std::lock_guard<std::mutex> lock(pending_mu_);
    if (chunk_s != recorded_chunk_s_) {
      recorded_chunk_s_ = chunk_s;
      runner = runner_;
    }
  }
  if (runner != nullptr && runner->session() != nullptr) {
    runner->session()->SetProperty("c3d.ros.camera." + config_.name + ".chunk_seconds", chunk_s);
  }
  std::lock_guard<std::mutex> lock(pending_mu_);
  encoder_ = std::move(encoder);
  encoder_width_ = width;
  encoder_height_ = height;
  encoder_frame_id_ = frame_id;
  encoder_wall_offset_ = wall_offset;
  chunk_s_ = chunk_s;
  last_accepted_ = now;
  last_attempt_refused_ = false;
  if (reason == StartReason::kFailure || reason == StartReason::kWedge) {
    AddToStats(&CameraStats::encoder_restarts);
  } else if (reason == StartReason::kGap) {
    AddToStats(&CameraStats::gap_restarts);
  } else if (reason == StartReason::kSize) {
    AddToStats(&CameraStats::size_restarts);
  }
  // Unless the new instance has already died, whose on_failure set the reason for the next.
  if (!failed_) pending_start_reason_ = StartReason::kNone;
  return Started::kYes;
}

void CameraCapture::RestartForSize(double now, double wall_offset,
                                   const MediaEnvironment& environment) {
  std::uint16_t width = 0, height = 0;
  std::string frame_id;
  {
    std::lock_guard<std::mutex> lock(pending_mu_);
    size_stop_ = true;
    width = encoder_width_;
    height = encoder_height_;
    frame_id = encoder_frame_id_;
  }
  StopEncoder(0.0, /*for_replay=*/true);
  // Every token still pending: the refused unit's, those delivered after it and held, and those the
  // kill abandoned. Every earlier one was delivered first, the encoder pairing units to tokens in
  // feed order.
  std::vector<std::uint64_t> tokens;
  {
    std::lock_guard<std::mutex> lock(pending_mu_);
    size_stop_ = false;
    refused_from_.reset();
    held_units_ = 0;
    for (const auto& kept : pending_) tokens.push_back(kept.first);
    pending_start_reason_ = StartReason::kSize;
  }
  if (!size_warned_ && environment.warn_once) {
    size_warned_ = true;
    environment.warn_once(
        "camera '" + config_.name + "': a video chunk reached " +
        std::to_string(c3d::kMaxVideoChunkBytes / kMebibyte) +
        " MiB, the most one holds, before its time was up, so its encoder was restarted to begin "
        "the next chunk and re-encode the frames since (c3d.camera." + config_.name +
        ".size_restarts). Each restart costs a keyframe; lower the bytes with a higher crf, "
        "max_width_px or max_bitrate_kbps");
  }
  const auto lose = [this](std::uint64_t token) {
    {
      std::lock_guard<std::mutex> lock(pending_mu_);
      pending_.erase(token);
      replay_.Release(token);
    }
    AddToStats(&CameraStats::dropped_replay);
  };
  if (StartEncoder(width, height, frame_id, wall_offset, now, environment) != Started::kYes) {
    for (const std::uint64_t token : tokens) lose(token);
    return;
  }
  c3d::IH264Encoder* encoder = nullptr;
  {
    std::lock_guard<std::mutex> lock(pending_mu_);
    encoder = encoder_.get();
  }
  for (const std::uint64_t token : tokens) {
    std::string yuv;
    bool kept = false;
    {
      std::lock_guard<std::mutex> lock(pending_mu_);
      kept = replay_.Copy(token, &yuv);
    }
    if (!kept || !encoder->Refeed(Bgr24Of(yuv, width, height), token)) lose(token);
  }
}

void CameraCapture::StopEncoder(double timeout_s) { StopEncoder(timeout_s, false); }

void CameraCapture::StopEncoder(double timeout_s, bool for_replay) {
  std::unique_ptr<c3d::IH264Encoder> encoder;
  {
    std::lock_guard<std::mutex> lock(pending_mu_);
    encoder = std::move(encoder_);
    if (encoder) {
      previous_stop_killed_ = false;
      stopping_on_ = std::this_thread::get_id();
    }
  }
  // Unlocked: the reader keeps delivering, and takes pending_mu_ for each unit, until Finish has
  // joined it.
  if (encoder) encoder->Finish(timeout_s);
  encoder.reset();
  std::uint64_t held = 0;
  {
    std::lock_guard<std::mutex> lock(pending_mu_);
    stopping_on_ = std::thread::id();
    failed_ = false;
    if (!for_replay) {
      // Frames the stop abandoned never come back; their tokens would wait here for good. Units
      // held for a size restart that will not come are lost with them.
      held = held_units_;
      held_units_ = 0;
      refused_from_.reset();
      pending_.clear();
      replay_.Clear();
      runner_ = nullptr;
      warn_once_ = nullptr;
      arrival_time_ = nullptr;
    }
  }
  if (held > 0) AddToStats(&CameraStats::dropped_at_stop, held);
}

void CameraCapture::OnAccessUnit(c3d::EncodedAccessUnit unit) {
  Pending pending;
  SessionRunner* runner = nullptr;
  {
    std::lock_guard<std::mutex> lock(pending_mu_);
    const auto found = pending_.find(unit.token);
    if (found == pending_.end()) return;
    if (refused_from_) {
      // Held, with its frame, for the size row to re-encode.
      ++held_units_;
      return;
    }
    pending = found->second;
    runner = runner_;
    // A unit shows the instance encodes, so the kill before it no longer shortens its quiet-gap
    // stop. A stop that kills reports it after the last unit it delivers.
    previous_stop_killed_ = false;
  }
  const auto settle = [this, &unit] {
    std::lock_guard<std::mutex> lock(pending_mu_);
    pending_.erase(unit.token);
    replay_.Release(unit.token);
  };
  if (runner == nullptr || !runner->active()) {
    settle();
    AddToStats(&CameraStats::dropped_no_session);
    std::function<void(const std::string&)> warn;
    {
      std::lock_guard<std::mutex> lock(pending_mu_);
      if (!no_session_warned_ && warn_once_) {
        no_session_warned_ = true;
        warn = warn_once_;
      }
    }
    if (warn) {
      warn("camera '" + config_.name + "': access units arrived after the session ended: the "
           "encoder was not stopped first, and they are counted in c3d.camera." + config_.name +
           ".dropped_no_session");
    }
    return;
  }
  // The parameter sets from the unit itself, never from the encoder that is current now: a
  // restart on the node's thread may already have replaced the one that made this unit.
  pending.context.codec_params = std::move(unit.codec_params);
  pending.context.encoder_settings = std::move(unit.encoder_settings);
  const std::uint64_t bytes = unit.bytes.size();
  // A copy: the context is moved into the call, in an order the reference would not survive.
  const std::string frame_id = pending.context.frame_id;
  const c3d::VideoUnitFate fate = runner->RecordCameraAccessUnit(
      config_.name, pending.entry.time_ms, pending.entry.seq, frame_id,
      pending.entry.pose.position, pending.entry.pose.rotation, std::move(pending.context),
      std::move(unit.bytes), unit.keyframe);
  if (fate == c3d::VideoUnitFate::kRefused) {
    std::lock_guard<std::mutex> lock(pending_mu_);
    if (!refused_from_) refused_from_ = unit.token;
    ++held_units_;
    return;
  }
  settle();
  std::lock_guard<std::mutex> lock(stats_mu_);
  ++stats_.frames;
  stats_.bytes += bytes;
}

void CameraCapture::OnEncoderFailure(const c3d::EncoderFailure& failure) {
  std::function<double()> clock;
  bool at_stop = false;
  double chunk_s = c3d::kVideoChunkSeconds;
  {
    std::lock_guard<std::mutex> lock(pending_mu_);
    at_stop = stopping_on_ == std::this_thread::get_id();
    // The size row's own kill: everything it abandons is re-encoded.
    if (at_stop && size_stop_) return;
    clock = arrival_time_;
    chunk_s = chunk_s_;
  }
  // Read unlocked: the arrival clock is the node's, which locks itself.
  const double now = clock ? clock() : SteadySeconds();
  std::function<void(const std::string&)> warn;
  bool backed_off = false;
  bool wedge = false;
  {
    std::lock_guard<std::mutex> lock(pending_mu_);
    failed_ = true;
    previous_stop_killed_ = failure.killed;
    // The wedge row kills on purpose, with Finish(0); it set its reason before the stop.
    wedge = at_stop && failure.killed && pending_start_reason_ == StartReason::kWedge;
    // A healthy encoder never needs the kill, so a stop that killed is a failure like a death
    // while running, and both cost the backoff: one spawn per kEncoderRestartMinSeconds, however
    // persistent the cause. A stop that drained, even with a non-zero exit, costs nothing.
    if (!at_stop || failure.killed) {
      backed_off = true;
      backoff_until_ = now + kEncoderRestartMinSeconds;
      pending_start_reason_ = StartReason::kFailure;
    }
    if (!failure_warned_ && warn_once_) {
      failure_warned_ = true;
      warn = warn_once_;
    }
  }
  if (failure.abandoned > 0) AddToStats(&CameraStats::dropped_at_stop, failure.abandoned);
  if (!warn) return;
  std::string cause;
  if (wedge) {
    cause = "it was wedged, taking no frame for " + SecondsText(chunk_s) +
            " seconds while frames kept arriving, and was killed";
  } else if (failure.killed) {
    cause = "it did not finish in time and was killed";
  } else if (failure.exit_code > 0) {
    cause = "it exited with code " + std::to_string(failure.exit_code);
  } else if (failure.exit_code < 0) {
    cause = "it died from a signal";
  } else {
    cause = "it stopped";
  }
  if (failure.abandoned > 0) {
    cause += ", and " + std::to_string(failure.abandoned) +
             " frame(s) it had accepted were lost (c3d.camera." + config_.name +
             ".dropped_at_stop)";
  }
  const std::string why = failure.why.empty() ? std::string() : " (" + failure.why + ")";
  warn("camera '" + config_.name + "': its encoder failed: " + cause + why +
       (backed_off ? ". After a crash or a kill no encoder starts for " +
                         SecondsText(kEncoderRestartMinSeconds) + " seconds."
                   : std::string(".")));
}

bool CameraCapture::PrepareCompressed(const sensor_msgs::msg::CompressedImage& compressed,
                                      PreparedFrame* out, std::string* why) const {
  // Decoded whatever its format: the encoder takes pixels, and a JPEG passed through as it was
  // published would be a second stream format for every reader.
  const cv::Mat decoded = cv::imdecode(
      cv::Mat(1, static_cast<int>(compressed.data.size()), CV_8UC1,
              const_cast<std::uint8_t*>(compressed.data.data())),
      cv::IMREAD_COLOR);
  if (decoded.empty()) {
    *why = "OpenCV cannot decode a CompressedImage with format '" + compressed.format + "'";
    return false;
  }
  return PrepareDecoded(decoded, config_.max_width_px, out, why);
}

bool CameraCapture::PrepareImage(const sensor_msgs::msg::Image& image, PreparedFrame* out,
                                 std::string* why) const {
  if (IsDepthEncoding(image.encoding)) {
    *why = "encoding '" + image.encoding +
           "' is DEPTH, and a depth image encoded as video is a grey rectangle that looks like a "
           "picture. Point this camera at the colour topic.";
    return false;
  }
  if (!AcceptedImageEncoding(image.encoding)) {
    *why = "encoding '" + image.encoding +
           "' is not one this SDK converts; accepted are rgb8, bgr8, rgba8, bgra8 and mono8";
    return false;
  }
  cv::Mat bgr;
  try {
    // toCvCopy rather than toCvShare: `image` is a local deserialized from the
    // generic subscription's buffer, and toCvShare would hand back a Mat pointing
    // into storage that outlives nothing.
    bgr = cv_bridge::toCvCopy(image, "bgr8")->image;
  } catch (const std::exception& e) {
    *why = std::string("cv_bridge could not convert it to bgr8: ") + e.what();
    return false;
  }
  return PrepareDecoded(bgr, config_.max_width_px, out, why);
}

bool CameraCapture::PrepareFrame(const std::string& type,
                                 const rclcpp::SerializedMessage& message, PreparedFrame* out,
                                 std::string* why) const {
  std::string ignored;
  if (why == nullptr) why = &ignored;
  if (type == "sensor_msgs/msg/CompressedImage") {
    sensor_msgs::msg::CompressedImage compressed;
    if (!Deserialize(message, &compressed)) {
      *why = "the message could not be deserialized as a CompressedImage";
      return false;
    }
    return PrepareCompressed(compressed, out, why);
  }
  if (type == "sensor_msgs/msg/Image") {
    sensor_msgs::msg::Image image;
    if (!Deserialize(message, &image)) {
      *why = "the message could not be deserialized as an Image";
      return false;
    }
    return PrepareImage(image, out, why);
  }
  *why = "type " + type + " is not an image";
  return false;
}

bool CameraCapture::Handle(const std::string& type, const rclcpp::SerializedMessage& message,
                           const MediaEnvironment& environment, SessionRunner& runner) {
  if (runner.session() == nullptr) return false;

  std_msgs::msg::Header header;
  PreparedFrame frame;
  std::string why;
  bool prepared = false;

  if (type == "sensor_msgs/msg/CompressedImage") {
    sensor_msgs::msg::CompressedImage compressed;
    if (!Deserialize(message, &compressed)) return false;
    // The gate runs BEFORE the conversion: a dropped frame must cost a deserialize,
    // not a decode.
    if (!AcceptHeader(compressed.header, environment)) return false;
    header = compressed.header;
    prepared = PrepareCompressed(compressed, &frame, &why);
  } else if (type == "sensor_msgs/msg/Image") {
    sensor_msgs::msg::Image image;
    if (!Deserialize(message, &image)) return false;
    if (!AcceptHeader(image.header, environment)) return false;
    header = image.header;
    prepared = PrepareImage(image, &frame, &why);
  } else {
    return false;
  }

  if (!prepared) {
    AddToStats(&CameraStats::refused);
    if (!refusal_warned_ && environment.warn_once) {
      refusal_warned_ = true;
      environment.warn_once("camera '" + config_.name + "' on " + config_.topic +
                            " records nothing: " + why);
    }
    return false;
  }
  return Emit(header, std::move(frame), environment, runner);
}

void StopCameraEncoders(const std::vector<std::unique_ptr<CameraCapture>>& cameras) {
  std::vector<std::thread> stopping;
  stopping.reserve(cameras.size());
  for (const auto& camera : cameras) {
    CameraCapture* capture = camera.get();
    try {
      stopping.emplace_back([capture]() { capture->StopEncoder(); });
    } catch (const std::system_error&) {
      // No thread to be had: this one stops here, in turn, rather than not at all.
      capture->StopEncoder();
    }
  }
  for (auto& thread : stopping) thread.join();
  c3d::SweepLeftoverChildren();
}

LidarCapture::LidarCapture(LidarConfig config) : config_(std::move(config)) {}

void LidarCapture::Reset() {
  stats_ = LidarStats{};
  seq_ = 0;
  gate_.Reset();
  held_.Clear();
  frame_id_warned_ = false;
  refusal_warned_ = false;
  color_warned_ = false;
  unstamped_warned_ = false;
}

bool LidarCapture::AcceptHeader(const std_msgs::msg::Header& header,
                                const MediaEnvironment& environment) {
  if (StampSeconds(header) <= 0.0 && !unstamped_warned_ && environment.warn_once) {
    unstamped_warned_ = true;
    environment.warn_once("lidar '" + config_.name + "' on " + config_.topic +
                          " publishes scans with a zero header stamp: they are placed at their "
                          "arrival and posed at the latest transform");
  }
  return gate_.Accept(ArrivalSeconds(environment), config_.rate_hz);
}

void LidarCapture::ServiceHeld(const MediaEnvironment& environment, SessionRunner& runner) {
  held_.Service(environment, runner, &stats_.pose_failures);
}

bool LidarCapture::ResolveFrame(const std_msgs::msg::Header& header,
                                const MediaEnvironment& environment, std::string* frame_id) {
  if (!ResolveFrameId(config_.frame, header, frame_id)) {
    ++stats_.pose_failures;
    if (!frame_id_warned_ && environment.warn_once) {
      frame_id_warned_ = true;
      environment.warn_once("lidar '" + config_.name + "' on " + config_.topic +
                            " publishes an empty header.frame_id and no lidar." + config_.name +
                            ".frame is set, so no pose can be resolved for its scans");
    }
    return false;
  }
  return true;
}

bool LidarCapture::MeasureEncodedScan(const std::string& type,
                                      const rclcpp::SerializedMessage& message,
                                      PoseConvention convention, std::uint64_t* bytes,
                                      std::string* why) const {
  std::string ignored;
  if (why == nullptr) why = &ignored;
  if (type == "sensor_msgs/msg/LaserScan") {
    sensor_msgs::msg::LaserScan scan;
    if (!Deserialize(message, &scan)) {
      *why = "the message could not be deserialized as a LaserScan";
      return false;
    }
    c3d::LaserScanData data;
    data.angle_min = scan.angle_min;
    data.angle_max = scan.angle_max;
    data.angle_increment = scan.angle_increment;
    data.range_min = scan.range_min;
    data.range_max = scan.range_max;
    data.ranges.assign(scan.ranges.begin(), scan.ranges.end());
    // Handle keeps intensities only when configured, and a block encodes them only when the scan
    // carries one per ray; a configured lidar that publishes none is sized without them, not
    // refused.
    const bool intensities =
        config_.intensities && scan.intensities.size() == scan.ranges.size();
    if (intensities) data.intensities.assign(scan.intensities.begin(), scan.intensities.end());
    const c3d::scanblock::EncodedScanBlock block = c3d::scanblock::Encode({data}, intensities, 1);
    if (!block.ok) {
      *why = "the scan block codec refused the scan: " + block.reason;
      return false;
    }
    if (bytes != nullptr) *bytes = block.ranges.size() + block.intensities.size();
    return true;
  }
  if (type != "sensor_msgs/msg/PointCloud2") {
    *why = "type " + type + " is not a scan";
    return false;
  }
  sensor_msgs::msg::PointCloud2 cloud;
  if (!Deserialize(message, &cloud)) {
    *why = "the message could not be deserialized as a PointCloud2";
    return false;
  }
  const PointCloudLayout layout = ResolvePointCloudLayout(cloud);
  if (!layout.ok) {
    *why = "this SDK cannot read the cloud: " + layout.error;
    return false;
  }
  const c3d::PointCloudInput input =
      ExtractPointCloud(cloud, layout, config_.pointcloud.color != c3d::PointColor::kNone,
                        config_.pointcloud.intensity != c3d::PointIntensity::kNone, convention);
  const c3d::EncodedPointCloud encoded = c3d::pointcloud::Encode(input, config_.pointcloud);
  if (!encoded.ok) {
    *why = "the encoder refused the cloud: " + encoded.reason;
    return false;
  }
  if (bytes != nullptr) *bytes = encoded.data.size();
  return true;
}

bool LidarCapture::Handle(const std::string& type, const rclcpp::SerializedMessage& message,
                          const MediaEnvironment& environment, SessionRunner& runner) {
  if (runner.session() == nullptr) return false;

  if (type == "sensor_msgs/msg/LaserScan") {
    sensor_msgs::msg::LaserScan scan;
    if (!Deserialize(message, &scan)) return false;
    if (scan.ranges.empty()) return false;
    if (!AcceptHeader(scan.header, environment)) return false;
    // Refused here, before it is posed, as well as by the session: a scan the session would drop
    // must not count in this lidar's series as recorded.
    if (scan.ranges.size() > c3d::scanblock::kMaxRaysPerScan) {
      ++stats_.refused;
      if (!refusal_warned_ && environment.warn_once) {
        refusal_warned_ = true;
        environment.warn_once("lidar '" + config_.name + "' on " + config_.topic +
                              " publishes LaserScans of " + std::to_string(scan.ranges.size()) +
                              " rays, and a scan block holds at most " +
                              std::to_string(c3d::scanblock::kMaxRaysPerScan) +
                              ": its scans are refused");
      }
      return false;
    }

    std::string frame_id;
    if (!ResolveFrame(scan.header, environment, &frame_id)) return false;
    const std::uint64_t time_ms = WireMilliseconds(environment, scan.header);

    c3d::LaserScanData data;
    data.angle_min = scan.angle_min;
    data.angle_max = scan.angle_max;
    data.angle_increment = scan.angle_increment;
    data.time_increment = scan.time_increment;
    data.scan_time = scan.scan_time;
    data.range_min = scan.range_min;
    data.range_max = scan.range_max;
    // As published, infinities and NaNs included: a return outside the sensor's
    // own limits is the driver saying "nothing there", and only the sensor knows
    // which spelling it uses.
    data.ranges.assign(scan.ranges.begin(), scan.ranges.end());
    if (config_.intensities) {
      data.intensities.assign(scan.intensities.begin(), scan.intensities.end());
    }

    auto record = [this, frame_id, time_ms, data = std::move(data)](
                      const c3d::Vec3& position, const c3d::Quat& rotation,
                      const MediaEnvironment&, SessionRunner& runner) {
      runner.RecordLaserScan(config_.name, time_ms, seq_++, frame_id, position, rotation, data);
      ++stats_.scans;
      stats_.bytes += (data.ranges.size() + data.intensities.size()) * sizeof(float);
      return true;
    };
    return held_.Offer(frame_id, scan.header.stamp, std::move(record), environment, runner,
                       &stats_.pose_failures);
  }

  if (type != "sensor_msgs/msg/PointCloud2") return false;

  sensor_msgs::msg::PointCloud2 cloud;
  if (!Deserialize(message, &cloud)) return false;
  if (!AcceptHeader(cloud.header, environment)) return false;

  const PointCloudLayout layout = ResolvePointCloudLayout(cloud);
  if (!layout.ok) {
    ++stats_.refused;
    if (!refusal_warned_ && environment.warn_once) {
      refusal_warned_ = true;
      environment.warn_once("lidar '" + config_.name + "' on " + config_.topic +
                            " publishes a PointCloud2 this SDK cannot read: " + layout.error);
    }
    return false;
  }
  // The records then say `none`, while the session property names the colour asked for.
  if (config_.pointcloud.color != c3d::PointColor::kNone &&
      layout.color == PointCloudLayout::ColorKind::kNone && !color_warned_ &&
      environment.warn_once) {
    color_warned_ = true;
    environment.warn_once("lidar '" + config_.name + "' on " + config_.topic +
                          " asks for pointcloud.color " +
                          c3d::PointColorName(config_.pointcloud.color) +
                          ", but its PointCloud2 has no rgb, rgba or r, g, b field: its points "
                          "are recorded without colour");
  }

  // The pose is resolved BEFORE the cloud is encoded. Encoding first and then
  // discovering there is no pose spends the expensive part of the pipeline on a
  // scan that is about to be dropped, and encoding a depth cloud is the most
  // expensive thing this node does.
  std::string frame_id;
  if (!ResolveFrame(cloud.header, environment, &frame_id)) return false;
  const std::uint64_t time_ms = WireMilliseconds(environment, cloud.header);
  const rclcpp::Time stamp = cloud.header.stamp;

  auto record = [this, frame_id, time_ms, cloud = std::move(cloud), layout](
                    const c3d::Vec3& position, const c3d::Quat& rotation,
                    const MediaEnvironment&, SessionRunner& runner) {
    const c3d::PointCloudInput input =
        ExtractPointCloud(cloud, layout, config_.pointcloud.color != c3d::PointColor::kNone,
                          config_.pointcloud.intensity != c3d::PointIntensity::kNone,
                          runner.convention());
    const c3d::EncodedPointCloud encoded = c3d::pointcloud::Encode(input, config_.pointcloud);
    stats_.dropped_range += encoded.dropped_out_of_range;
    if (!encoded.ok) {
      if (encoded.reason == "oversize") {
        ++stats_.dropped_oversize;
      } else if (encoded.reason == "no_points_in_range") {
        // Every point was outside int16's reach, which is the same fact the
        // per-point counter carries and not a separate kind of failure.
      } else {
        ++stats_.refused;
      }
      return false;
    }

    runner.RecordPointCloud(config_.name, time_ms, seq_++, frame_id, position, rotation, encoded);
    ++stats_.scans;
    stats_.bytes += encoded.data.size();
    return true;
  };
  return held_.Offer(frame_id, stamp, std::move(record), environment, runner,
                     &stats_.pose_failures);
}

}  // namespace cognitive3d_ros
