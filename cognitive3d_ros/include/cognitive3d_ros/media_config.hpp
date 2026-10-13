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
  // "jpeg" is the only implemented value. "jpegxl" is reserved on the wire and
  // refused here, because a value the gateway accepts and no browser renders is
  // worse than a configure error.
  std::string format = "jpeg";
  int jpeg_quality = 80;
  // 0 means never resize. Above 0, a wider frame is scaled down with the aspect
  // ratio kept -- and the width that reaches the wire is the one that was sent,
  // not the one that was configured.
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
