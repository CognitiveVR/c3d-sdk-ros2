#include "cognitive3d_ros/media_config.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <utility>

#include "c3d/sensor_blocks.hpp"

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

std::string Mebibytes(double bytes) {
  std::ostringstream text;
  text.precision(1);
  text << std::fixed << bytes / (1024.0 * 1024.0);
  return text.str();
}

}  // namespace

double EstimatedCameraBytesPerSecond(int width, int height, int crf, double rate_hz) {
  const double pixels = static_cast<double>(width) * static_cast<double>(height);
  return 5120.0 * pixels / (640.0 * 480.0) * std::pow(2.0, (28.0 - crf) / 4.0) * rate_hz;
}

double CameraBytesPerSecond(const CameraConfig& camera, int width, int height) {
  const double estimate = EstimatedCameraBytesPerSecond(width, height, camera.crf, camera.rate_hz);
  if (camera.max_bitrate_kbps <= 0) return estimate;
  return std::min(estimate, static_cast<double>(camera.max_bitrate_kbps) * 1000.0 / 8.0);
}

double CameraChunkSeconds(const CameraConfig& camera, int width, int height) {
  const double per_second = CameraBytesPerSecond(camera, width, height);
  if (!(per_second > 0.0)) return c3d::kVideoChunkSeconds;
  const double fits = std::floor(kCameraChunkTargetBytes / per_second * 10.0 + 1e-9) / 10.0;
  if (fits < kMinCameraChunkSeconds) return 0.0;
  return std::min(fits, c3d::kVideoChunkSeconds);
}

std::string CameraChunkTooLarge(const CameraConfig& camera, int width, int height) {
  std::ostringstream text;
  text << "at " << width << "x" << height << ", crf " << camera.crf << " and " << camera.rate_hz
       << " Hz";
  if (camera.max_bitrate_kbps > 0) text << " under max_bitrate_kbps " << camera.max_bitrate_kbps;
  text << " its video is estimated at " << Mebibytes(CameraBytesPerSecond(camera, width, height))
       << " MiB a second, so not even a " << kMinCameraChunkSeconds << "-second chunk stays under "
       << "the " << Mebibytes(kCameraChunkTargetBytes) << " MiB a chunk is sized to; the platform "
       << "refuses an object over " << c3d::kMaxObjectBytes / (1024 * 1024) << " MiB. Lower "
       << "max_width_px, raise crf or set max_bitrate_kbps";
  return text.str();
}

bool ReplayNeedsBitrateCap(const CameraConfig& camera, int width, int height) {
  const double frame = static_cast<double>(width) * static_cast<double>(height) * 1.5;
  return kReplayWindowFrames * frame >
         static_cast<double>(camera.replay_max_mb) * 1024.0 * 1024.0;
}

int DefaultCameraBitrateKbps(double chunk_s) {
  return static_cast<int>(kCameraChunkTargetBytes * 8.0 / chunk_s / 1000.0);
}

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
  if (camera.replay_max_mb < 1 || camera.replay_max_mb > kMaxCameraReplayMb) {
    return where + "replay_max_mb must be from 1 to " + std::to_string(kMaxCameraReplayMb) +
           " MiB, got " + std::to_string(camera.replay_max_mb);
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
  // Sized as a 16:9 frame, the widest common shape and so the fewest pixels at that width: what
  // fails here fails for every camera. The node sizes the real frame again at its encoder's start.
  const int width = camera.max_width_px & ~1;
  const int height = (camera.max_width_px * 9 / 16) & ~1;
  if (width >= 2 && height >= 2 && CameraChunkSeconds(camera, width, height) == 0.0) {
    return where + "max_width_px is " + std::to_string(camera.max_width_px) +
           ", and sized as a 16:9 frame " + CameraChunkTooLarge(camera, width, height);
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
