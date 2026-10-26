#include "cognitive3d_ros/media_config.hpp"

#include <cmath>
#include <string>
#include <utility>

namespace cognitive3d_ros {
namespace {

bool Finite(double v) { return std::isfinite(v); }

// 0 (not set), or an angle the wire carries. Judged on the Float32 the wire field
// holds, so a double that rounds to 180 or to 0 is refused here rather than
// silently sent as no field of view.
bool UsableFovParameter(double degrees) {
  if (degrees == 0.0) return true;
  // NaN fails every comparison and an infinity the second, which also keeps the
  // narrowing below inside the range a float can hold.
  if (!(degrees > 0.0 && degrees < 180.0)) return false;
  const float wire = static_cast<float>(degrees);
  return wire > 0.0f && wire < 180.0f;
}

}  // namespace

std::string ValidateCameraConfig(const CameraConfig& camera) {
  const std::string where = "camera." + camera.name + ".";
  if (camera.topic.empty()) {
    return where + "topic is unset: a name in `cameras` with no topic records nothing while the "
                   "run looks configured";
  }
  if (!Finite(camera.rate_hz) || camera.rate_hz <= 0.0) {
    return where + "rate_hz must be a positive number of frames per second";
  }
  if (camera.preset != "ultrafast" && camera.preset != "veryfast") {
    return where + "preset must be 'ultrafast' or 'veryfast', got '" + camera.preset + "'";
  }
  if (camera.crf < kMinCameraCrf || camera.crf > kMaxCameraCrf) {
    return where + "crf must be from " + std::to_string(kMinCameraCrf) + " to " +
           std::to_string(kMaxCameraCrf) + "; lower is higher quality and more bytes, got " +
           std::to_string(camera.crf);
  }
  if (camera.max_bitrate_kbps < 0 || camera.max_bitrate_kbps > kMaxCameraBitrateKbps) {
    return where + "max_bitrate_kbps must be 0 (no cap) or a number of kilobits per second up to " +
           std::to_string(kMaxCameraBitrateKbps) + ", got " +
           std::to_string(camera.max_bitrate_kbps);
  }
  if (camera.max_width_px < 0) {
    return where + "max_width_px must be 0 (never resize) or a positive width";
  }
  for (const auto& [name, value] : {std::pair<const char*, double>{"hfov_degrees",
                                                                   camera.hfov_degrees},
                                    std::pair<const char*, double>{"vfov_degrees",
                                                                   camera.vfov_degrees}}) {
    if (!UsableFovParameter(value)) {
      return where + name + " must be 0 (not set) or a full angle in degrees strictly between 0 "
                            "and 180, got " + std::to_string(value);
    }
  }
  if ((camera.hfov_degrees == 0.0) != (camera.vfov_degrees == 0.0)) {
    // One angle is half a frustum, and a reader would have to invent the other.
    return where + "hfov_degrees and vfov_degrees are set together or not at all; only " +
           (camera.hfov_degrees == 0.0 ? "vfov_degrees" : "hfov_degrees") + " is set";
  }
  return ValidateQosOverride(camera.qos, "camera." + camera.name);
}

const char* const kFfmpegRemedy =
    "Install it with `sudo apt install ffmpeg`, or set ffmpeg_path to a binary that runs and lists "
    "the libx264 encoder";

std::string FfmpegProblem(const std::string& ffmpeg_path, const std::string& why) {
  return "ffmpeg_path '" + ffmpeg_path + "' cannot run as the camera encoder, so no camera can "
         "record: " + why;
}

std::string ValidateLidarConfig(const LidarConfig& lidar) {
  const std::string where = "lidar." + lidar.name + ".";
  if (lidar.topic.empty()) {
    return where + "topic is unset: a name in `lidars` with no topic records nothing while the "
                   "run looks configured";
  }
  if (!Finite(lidar.rate_hz) || lidar.rate_hz <= 0.0) {
    return where + "rate_hz must be a positive number of scans per second";
  }
  const c3d::PointCloudOptions& cloud = lidar.pointcloud;
  if (!Finite(cloud.voxel_leaf_m) || cloud.voxel_leaf_m < 0.0) {
    return where + "pointcloud.voxel_leaf_m must be 0 (keep every point) or a positive length "
                   "in metres";
  }
  // The leaf reaches the wire as a UInt16 of millimetres, so a leaf past 65.535 m
  // would be stored as something else entirely.
  if (cloud.voxel_leaf_m > 65.0) {
    return where + "pointcloud.voxel_leaf_m is larger than the 65 m the wire field can carry";
  }
  if (cloud.max_points == 0) {
    return where + "pointcloud.max_points must be positive; 0 would drop every cloud";
  }
  if (cloud.zstd_level < 0 || cloud.zstd_level > 22) {
    return where + "pointcloud.zstd_level must be between 0 (uncompressed) and 22";
  }
  return "";
}

std::string SiblingCameraInfoTopic(const std::string& image_topic) {
  // The transports image_transport's own plugins and the common third-party ones publish under.
  static const char* const kTransports[] = {"compressed", "compressedDepth", "theora", "zstd",
                                            "ffmpeg"};
  std::string base = image_topic;
  const auto last = base.rfind('/');
  if (last != std::string::npos && last > 0) {
    const std::string leaf = base.substr(last + 1);
    for (const char* transport : kTransports) {
      if (leaf == transport) {
        base.erase(last);
        break;
      }
    }
  }
  const auto slash = base.rfind('/');
  return (slash == std::string::npos ? std::string("/") : base.substr(0, slash + 1)) +
         "camera_info";
}

bool IsCameraType(const std::string& type) {
  return type == "sensor_msgs/msg/Image" || type == "sensor_msgs/msg/CompressedImage";
}

bool IsLidarType(const std::string& type) {
  return type == "sensor_msgs/msg/LaserScan" || type == "sensor_msgs/msg/PointCloud2";
}

}  // namespace cognitive3d_ros
