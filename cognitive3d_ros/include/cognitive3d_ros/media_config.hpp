#pragma once

#include <string>
#include <vector>

#include "c3d/media.hpp"
#include "cognitive3d_ros/topic_plan.hpp"

namespace cognitive3d_ros {

// One configured camera. The NAME is what reaches the wire as `cameraId` and what
// the per-sensor `c3d.camera.<name>.*` series are keyed on, so it is a label an
// operator chooses rather than the topic: a robot that renames `/oakd/...` keeps
// its history joined.
struct CameraConfig {
  std::string name;
  std::string topic;
  double rate_hz = 30.0;
  // x264's speed preset: "ultrafast" or "veryfast", the two measured on a robot's
  // CPU. veryfast spends more CPU for fewer bytes.
  std::string preset = "ultrafast";
  // The constant rate factor, kMinCameraCrf to kMaxCameraCrf: lower is higher
  // quality and more bytes.
  int crf = 28;
  // A bitrate cap in kilobits per second, passed as -maxrate with a buffer of
  // twice that. 0 is no cap; at most kMaxCameraBitrateKbps.
  int max_bitrate_kbps = 0;
  // The memory, in MiB, that holds the frames fed to the encoder whose access units are not yet in
  // a chunk, so that a chunk that reaches its byte cap can be ended and those frames re-encoded
  // from a new keyframe. 1 to kMaxCameraReplayMb.
  int replay_max_mb = 192;
  // 0 means never resize. Above 0, a wider frame is scaled down with the aspect
  // ratio kept -- and the width that reaches the wire is the one that was sent,
  // cropped to even, not the one that was configured.
  int max_width_px = 640;
  // Empty means the message's own `header.frame_id`. Set it when the publisher
  // leaves the header blank, which is commoner than it should be.
  std::string frame;
  // The camera's field of view, as two full angles in degrees, sent with every
  // frame. 0 by 0 is "not set". Set, they win over any other source: an operator's
  // measured number beats a driver's calibration, which is often absent or a
  // placeholder. Both or neither, each strictly between 0 and 180.
  double hfov_degrees = 0.0;
  double vfov_degrees = 0.0;
  // The sensor_msgs/CameraInfo topic the field of view is computed from when the
  // parameters above are unset, resolved: the image topic's sibling by default
  // (see SiblingCameraInfoTopic), empty when it is switched off. The latest message
  // is the one used, so a driver that republishes it as it zooms changes the field
  // of view frame by frame.
  std::string camera_info_topic;
  QosOverride qos;

  bool fov_set() const { return hfov_degrees != 0.0 || vfov_degrees != 0.0; }
};

// The crf band the encoder's sizes and picture quality were measured over;
// outside it neither is known.
constexpr int kMinCameraCrf = 24;
constexpr int kMaxCameraCrf = 32;
// 1 Gbit/s: far past anything a robot's camera needs, and short of where twice it overflows an int.
constexpr int kMaxCameraBitrateKbps = 1000000;
constexpr int kMaxCameraReplayMb = 16384;

// The bytes a camera's chunk is sized to. A chunk closes at c3d::kMaxVideoChunkBytes whatever the
// estimate says; the margin is for a scene busier than the footage the estimate was fitted to.
constexpr double kCameraChunkTargetBytes = 6.0 * 1024 * 1024;
constexpr double kMinCameraChunkSeconds = 1.0;

// Bytes per second a camera's video costs, ESTIMATED: 5120 bytes per frame at
// 640x480 and crf 28, in proportion to the pixel count, doubling every 4 crf
// steps lower, times `rate_hz`. Fitted to footage recorded across the 24 to 32
// band, so it says what such a camera usually costs, not what this one will: the
// bytes follow what the scene does. `preset` is not in it: no measurement here
// separates the presets' bytes at one crf.
double EstimatedCameraBytesPerSecond(int width, int height, int crf, double rate_hz);
// That estimate for `camera` at this frame size, capped by its max_bitrate_kbps when set.
double CameraBytesPerSecond(const CameraConfig& camera, int width, int height);
// The chunk length for `camera` at this frame size: the longest, in tenths of a second from
// kMinCameraChunkSeconds to c3d::kVideoChunkSeconds, whose CameraBytesPerSecond fits
// kCameraChunkTargetBytes. 0 when not even kMinCameraChunkSeconds does.
double CameraChunkSeconds(const CameraConfig& camera, int width, int height);
// Why not even kMinCameraChunkSeconds fits at this frame size, naming the parameters that change
// it, for configure, the node and doctor alike.
std::string CameraChunkTooLarge(const CameraConfig& camera, int width, int height);

// The frames a size restart may have to encode again, at most: the encoder's queue, pipe and
// input, x264's lookahead at veryfast, the unit the splitter holds, and those that arrive while
// the restart is found and the replacement spawns.
constexpr int kReplayWindowFrames = 28;
// Whether kReplayWindowFrames frames of this size, as yuv420p, overflow replay_max_mb. Then a size
// restart cannot be relied on to lose nothing, and the encoder caps its bitrate by default.
bool ReplayNeedsBitrateCap(const CameraConfig& camera, int width, int height);
// The default cap then: kCameraChunkTargetBytes over a chunk of `chunk_s`, in kilobits per second.
int DefaultCameraBitrateKbps(double chunk_s);

struct LidarConfig {
  std::string name;
  std::string topic;
  double rate_hz = 30.0;
  std::string frame;
  // LaserScan only. Intensities double a scan's size and most 2D lidars publish
  // a constant, so they are dropped unless asked for.
  bool intensities = false;
  // PointCloud2 only. Ignored for a LaserScan, which is sent as published.
  c3d::PointCloudOptions pointcloud;
};

// Empty when the configuration is usable, otherwise the reason, naming the
// parameter. Configure fails on any of these rather than running with a value
// nobody chose -- the failures they prevent are all silent ones: a rate of 0
// records nothing, an unknown `color` is a 400 per part, and a camera with no
// topic is a stream that never starts while the run looks configured.
std::string ValidateCameraConfig(const CameraConfig& camera);

// Why `ffmpeg_path` cannot encode the configured cameras, given c3d::ProbeFfmpeg's reason, and
// what to do about it. Configure refuses with the two joined by ". "; doctor's `camera encoder`
// check says the same, and reads a refusal back into the two.
std::string FfmpegProblem(const std::string& ffmpeg_path, const std::string& why);
extern const char* const kFfmpegRemedy;

// The CameraInfo topic beside an image topic, by image_transport's convention: a
// transport publishes under the base topic (`/cam/image_raw/compressed` is the
// `compressed` transport of `/cam/image_raw`), and the base's sibling is
// `camera_info` (`/cam/camera_info`). `image_topic` is fully qualified.
std::string SiblingCameraInfoTopic(const std::string& image_topic);
std::string ValidateLidarConfig(const LidarConfig& lidar);

// The message types each stream can take, read from the graph rather than
// configured: a topic's type is a fact, and asking an operator to restate it is
// asking them to get it wrong.
bool IsCameraType(const std::string& type);
bool IsLidarType(const std::string& type);

}  // namespace cognitive3d_ros
