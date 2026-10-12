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
#include <string>

#include "c3d/media.hpp"
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
  // stamp. False when tf cannot answer -- the record is DROPPED, because a frame
  // with no pose is a picture with no place in the scene.
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
  // `arrival_time` clock. Held, not waited for: ServiceHeld retries it. 0 drops it at once.
  double pose_wait_s = 0.0;
};

// Records that passed the rate gate before tf could pose them, kept in arrival order. A driver
// that stamps a frame as it publishes it is ahead of the last transform of its chain, so its pose
// is usually there a few milliseconds later; waiting for it would hold the node's one callback
// thread, and the pose timer with it.
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

  // Past this many, the oldest is dropped: a chain that stopped would otherwise hold a frame from
  // every arrival for the length of the wait.
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
  std::uint64_t frames = 0;
  std::uint64_t bytes = 0;
  std::uint64_t pose_failures = 0;
  std::uint64_t refused = 0;  // encodings we will not encode, and decode failures
};

struct LidarStats {
  std::uint64_t scans = 0;
  std::uint64_t bytes = 0;
  std::uint64_t pose_failures = 0;
  std::uint64_t dropped_oversize = 0;
  std::uint64_t dropped_range = 0;
  std::uint64_t refused = 0;  // clouds whose layout we cannot read
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

// One configured camera: the rate gate, the encoder and the per-sensor counters.
class CameraCapture {
 public:
  explicit CameraCapture(CameraConfig config);

  const CameraConfig& config() const { return config_; }
  const CameraStats& stats() const { return stats_; }
  // Per session, like every other translator: the seq counter starts at 0 for
  // each session and the rate gate forgets where it was.
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

  // True when a frame reached the session; false also for one held for its pose, which
  // ServiceHeld records. `type` is the graph's type name, so a topic that changes publisher
  // mid-run is handled rather than assumed.
  bool Handle(const std::string& type, const rclcpp::SerializedMessage& message,
              const MediaEnvironment& environment, SessionRunner& runner);
  // Records the held frames tf can now pose. Called on a timer as well as from Handle.
  void ServiceHeld(const MediaEnvironment& environment, SessionRunner& runner);
  std::size_t held() const { return held_.size(); }

  // What this configuration WOULD send for one message: the same encode the run
  // performs, with no gate, no pose, no session and no counter touched.
  //
  // `doctor` sizes the stream with this. It is the same code path rather than a
  // second estimate on purpose -- a preflight that works the number out for
  // itself drifts out of step with the run silently, which is the failure class
  // `doctor` exists to end.
  bool MeasureEncodedFrame(const std::string& type, const rclcpp::SerializedMessage& message,
                           std::uint64_t* bytes, std::uint16_t* width, std::uint16_t* height,
                           std::string* why) const;

 private:
  bool EncodeImage(const sensor_msgs::msg::Image& image, std::string* bytes, std::uint16_t* width,
                   std::uint16_t* height, std::string* why) const;
  bool EncodeCompressed(const sensor_msgs::msg::CompressedImage& compressed, std::string* bytes,
                        std::uint16_t* width, std::uint16_t* height, std::string* why) const;
  bool Emit(const std_msgs::msg::Header& header, std::string bytes, std::uint16_t width,
            std::uint16_t height, const MediaEnvironment& environment, SessionRunner& runner);
  // The rate gate on the message's arrival, saying once when its stamp is zero.
  bool AcceptHeader(const std_msgs::msg::Header& header, const MediaEnvironment& environment);

  CameraConfig config_;
  CameraStats stats_;
  std::uint32_t seq_ = 0;
  RateGate gate_;
  // One per warning: a warning that shared another's flag would be silenced by it.
  bool frame_id_warned_ = false;
  bool refusal_warned_ = false;
  bool unstamped_warned_ = false;
  c3d::CameraFov camera_info_fov_;
  bool camera_info_warned_ = false;
  HeldRecords held_;
};

// One configured lidar. LaserScan and PointCloud2 share the rate gate, the pose
// lookup and the counters; only the payload differs.
class LidarCapture {
 public:
  explicit LidarCapture(LidarConfig config);

  const LidarConfig& config() const { return config_; }
  const LidarStats& stats() const { return stats_; }
  void Reset();

  // As CameraCapture::Handle, and held scans the same way.
  bool Handle(const std::string& type, const rclcpp::SerializedMessage& message,
              const MediaEnvironment& environment, SessionRunner& runner);
  void ServiceHeld(const MediaEnvironment& environment, SessionRunner& runner);
  std::size_t held() const { return held_.size(); }

  // The lidar twin of CameraCapture::MeasureEncodedFrame, and the same reason.
  // A point cloud is encoded through `c3d::pointcloud::Encode` here exactly as it
  // is at run time, so the estimate reflects the voxel leaf and zstd level the
  // operator configured rather than a guess about them.
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
  RateGate gate_;
  // One per warning, as the camera's are.
  bool frame_id_warned_ = false;
  bool refusal_warned_ = false;
  bool color_warned_ = false;
  bool unstamped_warned_ = false;
  HeldRecords held_;
};

}  // namespace cognitive3d_ros
