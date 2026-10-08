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
#include <vector>

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

double ArrivalSeconds(const MediaEnvironment& environment) {
  return environment.arrival_time ? environment.arrival_time() : SteadySeconds();
}

// The frame the pose is FOR: the configured one, else the message's own. A
// message with neither is refused rather than posed against a guess -- a pose in
// the wrong frame is not a lesser version of the right one.
bool ResolveFrameId(const std::string& configured, const std_msgs::msg::Header& header,
                    std::string* out) {
  *out = configured.empty() ? header.frame_id : configured;
  return !out->empty();
}

std::string EncodeJpeg(const cv::Mat& bgr, int quality) {
  std::vector<int> parameters{cv::IMWRITE_JPEG_QUALITY, quality};
  std::vector<std::uint8_t> buffer;
  if (!cv::imencode(".jpg", bgr, buffer, parameters)) return std::string();
  return std::string(reinterpret_cast<const char*>(buffer.data()), buffer.size());
}

// INTER_AREA, which is the right filter for shrinking: the bilinear default
// samples rather than averages, so a downscaled sensor image comes out aliased
// and a JPEG of an aliased image is both larger and worse.
cv::Mat ShrinkToWidth(const cv::Mat& source, int max_width_px) {
  if (max_width_px <= 0 || source.cols <= max_width_px) return source;
  const int height = std::max(1, static_cast<int>(std::lround(
                                     static_cast<double>(source.rows) * max_width_px /
                                     static_cast<double>(source.cols))));
  cv::Mat resized;
  cv::resize(source, resized, cv::Size(max_width_px, height), 0, 0, cv::INTER_AREA);
  return resized;
}

}  // namespace

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

CameraCapture::CameraCapture(CameraConfig config) : config_(std::move(config)) {}

void CameraCapture::Reset() {
  stats_ = CameraStats{};
  seq_ = 0;
  gate_.Reset();
  frame_id_warned_ = false;
  refusal_warned_ = false;
  unstamped_warned_ = false;
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

bool CameraCapture::Emit(const std_msgs::msg::Header& header, const std::string& bytes,
                         std::uint16_t width, std::uint16_t height,
                         const MediaEnvironment& environment, SessionRunner& runner) {
  std::string frame_id;
  if (!ResolveFrameId(config_.frame, header, &frame_id)) {
    ++stats_.pose_failures;
    if (!frame_id_warned_ && environment.warn_once) {
      frame_id_warned_ = true;
      environment.warn_once("camera '" + config_.name + "' on " + config_.topic +
                            " publishes an empty header.frame_id and no camera." + config_.name +
                            ".frame is set, so no pose can be resolved for its frames");
    }
    return false;
  }

  c3d::Vec3 position;
  c3d::Quat rotation;
  if (!environment.lookup_pose || !environment.lookup_pose(frame_id, header.stamp, &position,
                                                           &rotation)) {
    ++stats_.pose_failures;
    return false;
  }

  const std::uint64_t time_ms =
      ToMilliseconds(environment.wire_time ? environment.wire_time(header.stamp) : 0.0);
  const bool first_of_session = seq_ == 0;
  const c3d::CameraFov fov = CurrentFov();
  runner.RecordCameraFrame(config_.name, time_ms, seq_++, frame_id, position, rotation,
                           config_.format, width, height, bytes, fov);
  ++stats_.frames;
  stats_.bytes += bytes.size();
  // A CameraInfo topic nothing publishes leaves every frame of the session without a field of
  // view, which the session's fov_source does not show. Not said of a publisher that has not sent
  // yet: its first CameraInfo reaches the frames after it.
  if (first_of_session && wants_camera_info() && !c3d::IsKnownCameraFov(fov) &&
      environment.publishers && environment.warn_once &&
      environment.publishers(config_.camera_info_topic) == 0) {
    environment.warn_once("camera '" + config_.name + "': its frames take their field of view "
                          "from " + config_.camera_info_topic + ", which has no publisher, so "
                          "they carry none. Set camera." + config_.name + ".hfov_degrees and "
                          "vfov_degrees, or point camera." + config_.name +
                          ".camera_info_topic at the driver's CameraInfo");
  }
  return true;
}

bool CameraCapture::EncodeCompressed(const sensor_msgs::msg::CompressedImage& compressed,
                                     std::string* bytes, std::uint16_t* width,
                                     std::uint16_t* height, std::string* why) const {
  const std::string raw(reinterpret_cast<const char*>(compressed.data.data()),
                        compressed.data.size());
  std::uint16_t source_width = 0;
  std::uint16_t source_height = 0;
  // Pass-through is the whole point of subscribing to the compressed topic: the
  // publisher already paid for the encode, and re-encoding a JPEG loses quality
  // to no end. It is only correct when nothing else has to change, which is why
  // the width test reads the JPEG itself rather than trusting configuration.
  if (CompressedFormatIsJpeg(compressed.format) &&
      JpegDimensions(raw, &source_width, &source_height) &&
      (config_.max_width_px <= 0 || source_width <= config_.max_width_px)) {
    *bytes = raw;
    *width = source_width;
    *height = source_height;
    return true;
  }

  const cv::Mat decoded = cv::imdecode(
      cv::Mat(1, static_cast<int>(compressed.data.size()), CV_8UC1,
              const_cast<std::uint8_t*>(compressed.data.data())),
      cv::IMREAD_COLOR);
  if (decoded.empty()) {
    *why = "OpenCV cannot decode a CompressedImage with format '" + compressed.format + "'";
    return false;
  }
  const cv::Mat shrunk = ShrinkToWidth(decoded, config_.max_width_px);
  *bytes = EncodeJpeg(shrunk, config_.jpeg_quality);
  if (bytes->empty()) {
    *why = "the JPEG encoder produced nothing";
    return false;
  }
  *width = static_cast<std::uint16_t>(shrunk.cols);
  *height = static_cast<std::uint16_t>(shrunk.rows);
  return true;
}

bool CameraCapture::EncodeImage(const sensor_msgs::msg::Image& image, std::string* bytes,
                                std::uint16_t* width, std::uint16_t* height,
                                std::string* why) const {
  if (IsDepthEncoding(image.encoding)) {
    *why = "encoding '" + image.encoding +
           "' is DEPTH, and a depth image run through a JPEG encoder is a grey rectangle that "
           "looks like a picture. Point this camera at the colour topic.";
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
  const cv::Mat shrunk = ShrinkToWidth(bgr, config_.max_width_px);
  *bytes = EncodeJpeg(shrunk, config_.jpeg_quality);
  if (bytes->empty()) {
    *why = "the JPEG encoder produced nothing";
    return false;
  }
  *width = static_cast<std::uint16_t>(shrunk.cols);
  *height = static_cast<std::uint16_t>(shrunk.rows);
  return true;
}

bool CameraCapture::MeasureEncodedFrame(const std::string& type,
                                        const rclcpp::SerializedMessage& message,
                                        std::uint64_t* bytes, std::uint16_t* width,
                                        std::uint16_t* height, std::string* why) const {
  std::string encoded;
  std::string ignored;
  if (why == nullptr) why = &ignored;
  if (type == "sensor_msgs/msg/CompressedImage") {
    sensor_msgs::msg::CompressedImage compressed;
    if (!Deserialize(message, &compressed)) {
      *why = "the message could not be deserialized as a CompressedImage";
      return false;
    }
    if (!EncodeCompressed(compressed, &encoded, width, height, why)) return false;
  } else if (type == "sensor_msgs/msg/Image") {
    sensor_msgs::msg::Image image;
    if (!Deserialize(message, &image)) {
      *why = "the message could not be deserialized as an Image";
      return false;
    }
    if (!EncodeImage(image, &encoded, width, height, why)) return false;
  } else {
    *why = "type " + type + " is not an image";
    return false;
  }
  if (bytes != nullptr) *bytes = encoded.size();
  return true;
}

bool CameraCapture::Handle(const std::string& type, const rclcpp::SerializedMessage& message,
                           const MediaEnvironment& environment, SessionRunner& runner) {
  if (runner.session() == nullptr) return false;

  std_msgs::msg::Header header;
  std::string bytes;
  std::uint16_t width = 0;
  std::uint16_t height = 0;
  std::string why;
  bool encoded = false;

  if (type == "sensor_msgs/msg/CompressedImage") {
    sensor_msgs::msg::CompressedImage compressed;
    if (!Deserialize(message, &compressed)) return false;
    // The gate runs BEFORE the encode: a dropped frame must cost a deserialize,
    // not a JPEG.
    if (!AcceptHeader(compressed.header, environment)) return false;
    header = compressed.header;
    encoded = EncodeCompressed(compressed, &bytes, &width, &height, &why);
  } else if (type == "sensor_msgs/msg/Image") {
    sensor_msgs::msg::Image image;
    if (!Deserialize(message, &image)) return false;
    if (!AcceptHeader(image.header, environment)) return false;
    header = image.header;
    encoded = EncodeImage(image, &bytes, &width, &height, &why);
  } else {
    return false;
  }

  if (!encoded) {
    ++stats_.refused;
    if (!refusal_warned_ && environment.warn_once) {
      refusal_warned_ = true;
      environment.warn_once("camera '" + config_.name + "' on " + config_.topic +
                            " records nothing: " + why);
    }
    return false;
  }
  return Emit(header, bytes, width, height, environment, runner);
}

LidarCapture::LidarCapture(LidarConfig config) : config_(std::move(config)) {}

void LidarCapture::Reset() {
  stats_ = LidarStats{};
  seq_ = 0;
  gate_.Reset();
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

bool LidarCapture::ResolvePose(const std_msgs::msg::Header& header,
                               const MediaEnvironment& environment, std::string* frame_id,
                               c3d::Vec3* position, c3d::Quat* rotation,
                               std::uint64_t* time_ms) {
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
  if (!environment.lookup_pose ||
      !environment.lookup_pose(*frame_id, header.stamp, position, rotation)) {
    ++stats_.pose_failures;
    return false;
  }
  *time_ms = ToMilliseconds(environment.wire_time ? environment.wire_time(header.stamp) : 0.0);
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
    // A LaserScan goes uncompressed, so its size is exactly its float arrays.
    const std::size_t floats =
        scan.ranges.size() + (config_.intensities ? scan.intensities.size() : 0);
    if (bytes != nullptr) *bytes = floats * sizeof(float);
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

    std::string frame_id;
    c3d::Vec3 position;
    c3d::Quat rotation;
    std::uint64_t time_ms = 0;
    if (!ResolvePose(scan.header, environment, &frame_id, &position, &rotation, &time_ms)) {
      return false;
    }

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

    runner.RecordLaserScan(config_.name, time_ms, seq_++, frame_id, position, rotation, data);
    ++stats_.scans;
    stats_.bytes +=
        (data.ranges.size() + data.intensities.size()) * sizeof(float);
    return true;
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
  // scan that is about to be dropped, and a depth cloud at 1 Hz is the most
  // expensive thing this node does.
  std::string frame_id;
  c3d::Vec3 position;
  c3d::Quat rotation;
  std::uint64_t time_ms = 0;
  if (!ResolvePose(cloud.header, environment, &frame_id, &position, &rotation, &time_ms)) {
    return false;
  }

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
}

}  // namespace cognitive3d_ros
