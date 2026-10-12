#include <gtest/gtest.h>
#include <unistd.h>

#include <opencv2/imgcodecs.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "c3d/spool.hpp"
#include "c3d/transport.hpp"
#include "cognitive3d_ros/doctor.hpp"
#include "cognitive3d_ros/image_codec.hpp"
#include "cognitive3d_ros/media_capture.hpp"
#include "cognitive3d_ros/media_config.hpp"
#include "cognitive3d_ros/pointcloud_fields.hpp"
#include "cognitive3d_ros/rate_gate.hpp"

using namespace cognitive3d_ros;
namespace fs = std::filesystem;
using sensor_msgs::msg::PointField;

namespace {

class NullTransport : public c3d::ITransport {
 public:
  c3d::HttpResponse Post(const std::string&, const std::string&,
                         const std::vector<std::string>&) override {
    c3d::HttpResponse ok;
    ok.status = 200;
    ok.saw_request_time_header = true;
    return ok;
  }
};

template <typename MessageT>
rclcpp::SerializedMessage Serialize(const MessageT& message) {
  static rclcpp::Serialization<MessageT> serializer;
  rclcpp::SerializedMessage out;
  serializer.serialize_message(&message, &out);
  return out;
}

// A real JPEG of a given size, produced by the same encoder the capture uses, so
// the pass-through tests are about real bytes rather than a hand-built header.
std::string MakeJpeg(int width, int height) {
  cv::Mat image(height, width, CV_8UC3, cv::Scalar(20, 120, 200));
  std::vector<std::uint8_t> buffer;
  std::vector<int> parameters{cv::IMWRITE_JPEG_QUALITY, 85};
  EXPECT_TRUE(cv::imencode(".jpg", image, buffer, parameters));
  return std::string(reinterpret_cast<const char*>(buffer.data()), buffer.size());
}

sensor_msgs::msg::Image MakeImage(int width, int height, const std::string& encoding,
                                  const std::string& frame = "camera_link",
                                  double stamp_seconds = 100.0) {
  sensor_msgs::msg::Image image;
  image.header.frame_id = frame;
  image.header.stamp.sec = static_cast<std::int32_t>(stamp_seconds);
  image.header.stamp.nanosec =
      static_cast<std::uint32_t>((stamp_seconds - std::floor(stamp_seconds)) * 1e9);
  image.width = static_cast<std::uint32_t>(width);
  image.height = static_cast<std::uint32_t>(height);
  image.encoding = encoding;
  std::size_t channels = 3;
  if (encoding == "mono8") channels = 1;
  if (encoding == "rgba8" || encoding == "bgra8") channels = 4;
  if (encoding == "32FC1") channels = 4;
  if (encoding == "16UC1" || encoding == "mono16") channels = 2;
  image.step = static_cast<std::uint32_t>(width * channels);
  image.data.assign(static_cast<std::size_t>(width) * height * channels, 64);
  return image;
}

PointField Field(const std::string& name, std::uint32_t offset, std::uint8_t datatype) {
  PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  return field;
}

// One point's worth of a cloud carrying exactly these fields. `point_step` is
// part of the layout -- a field is an offset INTO a step and means nothing
// without it -- so a fields-only fixture would not be resolvable at all.
sensor_msgs::msg::PointCloud2 CloudOf(std::vector<PointField> fields, std::uint32_t point_step) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.fields = std::move(fields);
  cloud.point_step = point_step;
  cloud.height = 1;
  cloud.width = 1;
  cloud.row_step = point_step;
  cloud.is_bigendian = false;
  cloud.data.assign(point_step, 0);
  return cloud;
}

PointCloudLayout LayoutOf(std::vector<PointField> fields, std::uint32_t point_step) {
  return ResolvePointCloudLayout(CloudOf(std::move(fields), point_step));
}

// One point, laid out exactly as the fields say. Written by hand rather than with
// a helper that shares the reader's idea of the layout.
void PutFloat(std::vector<std::uint8_t>* data, std::size_t offset, float value) {
  std::memcpy(data->data() + offset, &value, sizeof(value));
}

void PutUint32(std::vector<std::uint8_t>* data, std::size_t offset, std::uint32_t value) {
  std::memcpy(data->data() + offset, &value, sizeof(value));
}

class MediaTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() /
           ("c3d_media_" + std::to_string(::getpid()) + "_" + std::to_string(counter_++));
    fs::create_directories(dir_);
    c3d::Config cfg;
    cfg.application_key = "K";
    cfg.scene_id = "S";
    cfg.scene_version = "1";
    cfg.device_id = "robot";
    cfg.spool_dir = dir_.string();
    cfg.auto_send_interval_s = 3600.0;
    cfg.worker_tick_s = 0.01;
    runner_ = std::make_unique<SessionRunner>(cfg, PoseConvention::kRep103,
                                             std::make_shared<NullTransport>(),
                                             std::make_shared<c3d::DiskSpool>(
                                                 dir_.string(), 64ull << 20,
                                                 std::make_shared<c3d::NullLogger>()),
                                             std::make_shared<c3d::NullLogger>());
    ASSERT_TRUE(runner_->Begin());

    environment_.lookup_pose = [this](const std::string& frame, const rclcpp::Time& stamp,
                                      c3d::Vec3* position, c3d::Quat* rotation) {
      looked_up_frames_.push_back(frame);
      if (!pose_available_ || stamp.seconds() > pose_until_) return false;
      *position = c3d::Vec3{1.0, 2.0, 3.0};
      *rotation = c3d::Quat{0.0, 0.0, 0.0, 1.0};
      return true;
    };
    environment_.wire_time = [](const rclcpp::Time&) { return 1788383013.5; };
    environment_.warn_once = [this](const std::string& text) { warnings_.push_back(text); };
    // A second between arrivals, so every message these tests hand over passes the gate.
    environment_.arrival_time = [this]() { return next_arrival_ += 1.0; };
  }
  void TearDown() override {
    if (runner_) runner_->End("test");
    runner_.reset();
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }

  std::unique_ptr<SessionRunner> runner_;
  MediaEnvironment environment_;
  std::vector<std::string> looked_up_frames_;
  std::vector<std::string> warnings_;
  bool pose_available_ = true;
  // tf's chain ends here: a stamp after it has no pose yet.
  double pose_until_ = std::numeric_limits<double>::infinity();
  double next_arrival_ = 1000.0;
  fs::path dir_;
  static int counter_;
};
int MediaTest::counter_ = 0;

}  // namespace

// ---------------------------------------------------------------------------
// The JPEG SOF parser, which exists so a pass-through never decodes
// ---------------------------------------------------------------------------

TEST(JpegDimensions, ReadsABaselineFrameHeader) {
  const std::string jpeg = MakeJpeg(320, 240);
  std::uint16_t width = 0, height = 0;
  ASSERT_TRUE(JpegDimensions(jpeg, &width, &height));
  EXPECT_EQ(width, 320);
  EXPECT_EQ(height, 240);
}

TEST(JpegDimensions, ReadsAProgressiveFrameHeader) {
  // SOF2, which is what an encoder asked for a progressive JPEG emits. A parser
  // that matched only SOF0 would report 0x0 for it and the wire would carry that.
  const std::string jpeg =
      std::string("\xff\xd8", 2) +                          // SOI
      std::string("\xff\xc2\x00\x11\x08\x01\x90\x02\x80\x03\x01\x11\x00\x02\x11\x01\x03\x11\x01",
                  19);
  std::uint16_t width = 0, height = 0;
  ASSERT_TRUE(JpegDimensions(jpeg, &width, &height));
  EXPECT_EQ(height, 400);
  EXPECT_EQ(width, 640);
}

TEST(JpegDimensions, SkipsTheSegmentsThatShareTheFrameHeaderRange) {
  // DHT is 0xC4 and sits inside C0-CF. Read as a frame header its table bytes
  // become a plausible width and height, which is worse than failing.
  std::string jpeg = std::string("\xff\xd8", 2);
  jpeg += std::string("\xff\xc4\x00\x06\x00\x01\x02\x03", 8);   // DHT, length 6
  jpeg += std::string("\xff\xc0\x00\x11\x08\x00\x64\x00\xc8\x03\x01\x11\x00\x02\x11\x01\x03\x11\x01",
                      19);                                     // SOF0, 200 x 100
  std::uint16_t width = 0, height = 0;
  ASSERT_TRUE(JpegDimensions(jpeg, &width, &height));
  EXPECT_EQ(height, 100);
  EXPECT_EQ(width, 200);
}

TEST(JpegDimensions, RefusesWhatIsNotAJpegOrIsCutShort) {
  std::uint16_t width = 0, height = 0;
  EXPECT_FALSE(JpegDimensions("", &width, &height));
  EXPECT_FALSE(JpegDimensions("not a jpeg at all", &width, &height));
  EXPECT_FALSE(JpegDimensions(std::string("\x89PNG\r\n\x1a\n", 8), &width, &height));
  // A real JPEG truncated before its frame header: the SOI is there and the
  // answer still is not.
  EXPECT_FALSE(JpegDimensions(MakeJpeg(64, 64).substr(0, 4), &width, &height));
}

// ---------------------------------------------------------------------------
// Encoding acceptance
// ---------------------------------------------------------------------------

TEST(ImageEncodings, ColourAndGreyscaleAreAcceptedAndDepthIsNamedSeparately) {
  for (const char* encoding : {"rgb8", "bgr8", "rgba8", "bgra8", "mono8"}) {
    EXPECT_TRUE(AcceptedImageEncoding(encoding)) << encoding;
    EXPECT_FALSE(IsDepthEncoding(encoding)) << encoding;
  }
  for (const char* encoding : {"32FC1", "16UC1", "mono16"}) {
    EXPECT_TRUE(IsDepthEncoding(encoding)) << encoding;
    EXPECT_FALSE(AcceptedImageEncoding(encoding)) << encoding;
  }
  // Bayer is neither: it is raw sensor data that needs a debayer step nobody here
  // performs, so it is refused as unsupported rather than as depth.
  EXPECT_FALSE(AcceptedImageEncoding("bayer_rggb8"));
  EXPECT_FALSE(IsDepthEncoding("bayer_rggb8"));
}

TEST(ImageEncodings, TheCompressedFormatFieldIsSearchedNotMatched) {
  // All three are published by the ROS image transport for the same JPEG.
  EXPECT_TRUE(CompressedFormatIsJpeg("jpeg"));
  EXPECT_TRUE(CompressedFormatIsJpeg("bgr8; jpeg compressed bgr8"));
  EXPECT_TRUE(CompressedFormatIsJpeg("rgb8; jpeg compressed bgr8"));
  EXPECT_TRUE(CompressedFormatIsJpeg("JPEG"));
  EXPECT_FALSE(CompressedFormatIsJpeg("png"));
  EXPECT_FALSE(CompressedFormatIsJpeg("bgr8; png compressed bgr8"));
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

TEST(MediaConfig, EveryRefusalNamesTheParameter) {
  CameraConfig camera;
  camera.name = "front";
  camera.topic = "/image";
  EXPECT_EQ(ValidateCameraConfig(camera), "");

  CameraConfig no_topic = camera;
  no_topic.topic.clear();
  EXPECT_NE(ValidateCameraConfig(no_topic).find("camera.front.topic"), std::string::npos);

  CameraConfig zero_rate = camera;
  zero_rate.rate_hz = 0.0;
  EXPECT_NE(ValidateCameraConfig(zero_rate).find("camera.front.rate_hz"), std::string::npos);
  zero_rate.rate_hz = -1.0;
  EXPECT_NE(ValidateCameraConfig(zero_rate).find("camera.front.rate_hz"), std::string::npos);
  zero_rate.rate_hz = std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(ValidateCameraConfig(zero_rate).find("camera.front.rate_hz"), std::string::npos);

  // Reserved on the wire, not implemented here: accepting it would produce a
  // stream nothing downstream can open.
  CameraConfig jxl = camera;
  jxl.format = "jpegxl";
  EXPECT_NE(ValidateCameraConfig(jxl).find("camera.front.format"), std::string::npos);
  jxl.format = "JPEG";
  EXPECT_NE(ValidateCameraConfig(jxl).find("camera.front.format"), std::string::npos);

  CameraConfig quality = camera;
  quality.jpeg_quality = 0;
  EXPECT_NE(ValidateCameraConfig(quality).find("jpeg_quality"), std::string::npos);
  quality.jpeg_quality = 101;
  EXPECT_NE(ValidateCameraConfig(quality).find("jpeg_quality"), std::string::npos);

  CameraConfig width = camera;
  width.max_width_px = -1;
  EXPECT_NE(ValidateCameraConfig(width).find("max_width_px"), std::string::npos);
  // 0 is "never resize" and must stay legal.
  width.max_width_px = 0;
  EXPECT_EQ(ValidateCameraConfig(width), "");
}

TEST(MediaConfig, AFieldOfViewIsBothAnglesStrictlyBetween0And180OrNeither) {
  CameraConfig camera;
  camera.name = "front";
  camera.topic = "/image";
  EXPECT_EQ(ValidateCameraConfig(camera), "") << "0 by 0 is unset, and the default";
  camera.hfov_degrees = 61.0;
  camera.vfov_degrees = 45.0;
  EXPECT_EQ(ValidateCameraConfig(camera), "");

  const auto refusal = [&camera](double h, double v) {
    CameraConfig c = camera;
    c.hfov_degrees = h;
    c.vfov_degrees = v;
    return ValidateCameraConfig(c);
  };
  EXPECT_NE(refusal(61.0, 0.0).find("only hfov_degrees"), std::string::npos);
  EXPECT_NE(refusal(0.0, 45.0).find("only vfov_degrees"), std::string::npos);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  // The last two are doubles the Float32 wire field would round to 180 and to 0.
  for (const double bad : {180.0, 181.0, -1.0, nan, inf, -inf, 1e300, 179.999999999, 1e-50}) {
    EXPECT_NE(refusal(bad, 45.0).find("camera.front.hfov_degrees"), std::string::npos) << bad;
    EXPECT_NE(refusal(61.0, bad).find("camera.front.vfov_degrees"), std::string::npos) << bad;
  }
  EXPECT_EQ(refusal(179.99, 0.01), "");
}

TEST(MediaConfig, ALidarsPointCloudOptionsAreBoundedWhereTheWireIs) {
  LidarConfig lidar;
  lidar.name = "depth";
  lidar.topic = "/points";
  EXPECT_EQ(ValidateLidarConfig(lidar), "");

  LidarConfig leaf = lidar;
  leaf.pointcloud.voxel_leaf_m = -0.1;
  EXPECT_NE(ValidateLidarConfig(leaf).find("voxel_leaf_m"), std::string::npos);
  // The leaf reaches the wire as a UInt16 of millimetres.
  leaf.pointcloud.voxel_leaf_m = 100.0;
  EXPECT_NE(ValidateLidarConfig(leaf).find("voxel_leaf_m"), std::string::npos);
  leaf.pointcloud.voxel_leaf_m = 0.0;  // keep every point
  EXPECT_EQ(ValidateLidarConfig(leaf), "");

  LidarConfig cap = lidar;
  cap.pointcloud.max_points = 0;
  EXPECT_NE(ValidateLidarConfig(cap).find("max_points"), std::string::npos);

  LidarConfig level = lidar;
  level.pointcloud.zstd_level = 23;
  EXPECT_NE(ValidateLidarConfig(level).find("zstd_level"), std::string::npos);
  level.pointcloud.zstd_level = -1;
  EXPECT_NE(ValidateLidarConfig(level).find("zstd_level"), std::string::npos);
}

TEST(MediaConfig, OnlyTheTypesEachStreamHandlesAreRecognised) {
  EXPECT_TRUE(IsCameraType("sensor_msgs/msg/Image"));
  EXPECT_TRUE(IsCameraType("sensor_msgs/msg/CompressedImage"));
  EXPECT_FALSE(IsCameraType("sensor_msgs/msg/CameraInfo"));
  EXPECT_FALSE(IsCameraType("sensor_msgs/msg/PointCloud2"));
  EXPECT_TRUE(IsLidarType("sensor_msgs/msg/LaserScan"));
  EXPECT_TRUE(IsLidarType("sensor_msgs/msg/PointCloud2"));
  EXPECT_FALSE(IsLidarType("sensor_msgs/msg/MultiEchoLaserScan"));
  EXPECT_FALSE(IsLidarType("sensor_msgs/msg/Image"));
}

// ---------------------------------------------------------------------------
// The rate gate
// ---------------------------------------------------------------------------

TEST(RateGate, AcceptsOnePerPeriod) {
  RateGate gate;
  const double rate = 2.0;  // one every 0.5 s
  EXPECT_TRUE(gate.Accept(100.0, rate));
  EXPECT_FALSE(gate.Accept(100.2, rate));
  EXPECT_FALSE(gate.Accept(100.49, rate));
  EXPECT_TRUE(gate.Accept(100.5, rate));
  EXPECT_FALSE(gate.Accept(100.9, rate));
  EXPECT_TRUE(gate.Accept(101.0, rate));
}

TEST(RateGate, AGapBanksNoCreditForABurst) {
  RateGate gate;
  EXPECT_TRUE(gate.Accept(100.0, 10.0));
  // Ten seconds of silence, then three messages a millisecond apart. Exactly one
  // passes: a deadline moved on only from the PREVIOUS deadline would let a
  // hundred through, and one clamped to the time itself would let two.
  EXPECT_TRUE(gate.Accept(110.0, 10.0));
  EXPECT_FALSE(gate.Accept(110.001, 10.0));
  EXPECT_FALSE(gate.Accept(110.002, 10.0));
}

TEST(RateGate, ResetForgetsWhereItWas) {
  RateGate gate;
  EXPECT_TRUE(gate.Accept(100.0, 1.0));
  EXPECT_FALSE(gate.Accept(100.5, 1.0));
  gate.Reset();
  EXPECT_TRUE(gate.Accept(100.5, 1.0));
}

// A source at the gate's own rate whose arrivals jitter around the period. Scheduling the next
// accept from the accepted time would reject every message that comes a hair early and deliver
// about half the configured rate.
TEST(RateGate, ASourceAtTheGateRateIsNotHalvedByJitter) {
  RateGate gate;
  int accepted = 0;
  for (int k = 0; k < 200; ++k) {
    const double jitter = (k % 2 == 0 ? 1.0 : -1.0) * 0.004 * ((k * 7) % 5);
    if (gate.Accept(100.0 + 0.1 * k + jitter, 10.0)) ++accepted;
  }
  EXPECT_GE(accepted, 195) << "of 200 messages at the configured rate";
}

// A source three times faster than the gate, jittered, is held to the configured rate.
TEST(RateGate, AFasterSourceIsHeldToTheConfiguredRate) {
  RateGate gate;
  int accepted = 0;
  for (int k = 0; k < 300; ++k) {  // ten seconds at 30 Hz
    const double jitter = (k % 3 == 0 ? -0.003 : 0.002);
    if (gate.Accept(100.0 + k / 30.0 + jitter, 10.0)) ++accepted;
  }
  EXPECT_GE(accepted, 98);
  EXPECT_LE(accepted, 101);
}

// Fed times that run backwards, the gate holds rather than opening: whatever it is given, it
// never admits more than its rate.
TEST(RateGate, ATimeThatRunsBackwardsNeverOpensTheGate) {
  RateGate gate;
  EXPECT_TRUE(gate.Accept(100.0, 2.0));
  EXPECT_TRUE(gate.Accept(100.5, 2.0));
  EXPECT_FALSE(gate.Accept(50.0, 2.0));
  EXPECT_FALSE(gate.Accept(50.6, 2.0));
  EXPECT_FALSE(gate.Accept(100.9, 2.0));
  EXPECT_TRUE(gate.Accept(101.0, 2.0));
}

TEST(RateGate, ARateThatIsNotPositivePassesEverything) {
  RateGate gate;
  for (int k = 0; k < 5; ++k) EXPECT_TRUE(gate.Accept(100.0, 0.0));
}

// ---------------------------------------------------------------------------
// The gate as the captures apply it: on ARRIVAL, whatever the stamps say. Each scenario is one in
// which a gate that reads the stamps delivers far more than its rate.
// ---------------------------------------------------------------------------

class MediaGateTest : public MediaTest {
 protected:
  // Hands each message to `capture` at its own arrival time and returns how many reached the
  // session.
  template <typename Capture, typename MessageT>
  int Deliver(Capture& capture, const std::string& type,
              const std::vector<std::pair<double, MessageT>>& arrivals) {
    int delivered = 0;
    for (const auto& [arrival, message] : arrivals) {
      arrival_ = arrival;
      if (capture.Handle(type, Serialize(message), environment_, *runner_)) ++delivered;
    }
    return delivered;
  }
  CameraCapture Camera(double rate_hz) {
    CameraConfig config;
    config.name = "front";
    config.topic = "/image";
    config.rate_hz = rate_hz;
    return CameraCapture(config);
  }
  void SetUp() override {
    MediaTest::SetUp();
    environment_.arrival_time = [this]() { return arrival_; };
  }
  static constexpr double kSteady0 = 52000.0;  // a steady clock: seconds since boot
  static constexpr double kEpoch0 = 1.79e9;    // a header stamp: epoch seconds
  double arrival_ = 0.0;
};

// A driver that leaves every other stamp at zero. Gated on the stamps, each flip between the
// stamp's time base and the arrival fallback looks like a clock reset and passes: 30 Hz through a
// 1 Hz gate.
TEST_F(MediaGateTest, IntermittentZeroStampsDoNotOpenTheGate) {
  CameraCapture capture = Camera(1.0);
  std::vector<std::pair<double, sensor_msgs::msg::Image>> arrivals;
  for (int k = 0; k < 30 * 30; ++k) {  // thirty seconds at 30 Hz
    const double stamp = k % 2 ? 0.0 : kEpoch0 + k / 30.0;
    arrivals.emplace_back(kSteady0 + k / 30.0, MakeImage(8, 8, "rgb8", "camera_link", stamp));
  }
  const int delivered = Deliver(capture, "sensor_msgs/msg/Image", arrivals);
  EXPECT_GE(delivered, 29);
  EXPECT_LE(delivered, 31) << "a 1 Hz gate over thirty seconds";
}

TEST_F(MediaGateTest, OneZeroStampInTenDoesNotOpenTheGate) {
  CameraCapture capture = Camera(1.0);
  std::vector<std::pair<double, sensor_msgs::msg::Image>> arrivals;
  for (int k = 0; k < 30 * 30; ++k) {
    const double stamp = k % 10 == 0 ? 0.0 : kEpoch0 + k / 30.0;
    arrivals.emplace_back(kSteady0 + k / 30.0, MakeImage(8, 8, "rgb8", "camera_link", stamp));
  }
  const int delivered = Deliver(capture, "sensor_msgs/msg/Image", arrivals);
  EXPECT_GE(delivered, 29);
  EXPECT_LE(delivered, 31);
}

// Two publishers on one topic whose clocks are 0.8 s apart, 10 Hz each, into a 2 Hz gate. Gated on
// the stamps, every step back to the slower clock restarts the gate: 20 Hz delivered.
TEST_F(MediaGateTest, TwoPublishersWithDifferentClocksAreHeldToTheRate) {
  CameraCapture capture = Camera(2.0);
  std::vector<std::pair<double, sensor_msgs::msg::Image>> arrivals;
  for (int k = 0; k < 10 * 30; ++k) {
    const double arrival = kSteady0 + k / 10.0;
    arrivals.emplace_back(arrival, MakeImage(8, 8, "rgb8", "camera_link", kEpoch0 + k / 10.0));
    arrivals.emplace_back(arrival + 0.05,
                          MakeImage(8, 8, "rgb8", "camera_link", kEpoch0 + k / 10.0 - 0.75));
  }
  const int delivered = Deliver(capture, "sensor_msgs/msg/Image", arrivals);
  EXPECT_GE(delivered, 59);
  EXPECT_LE(delivered, 61) << "a 2 Hz gate over thirty seconds";
}

// A driver that never sets the stamp neither freezes after its first frame nor passes every one.
TEST_F(MediaGateTest, AZeroStampedStreamIsGatedOnArrival) {
  CameraCapture capture = Camera(2.0);
  std::vector<std::pair<double, sensor_msgs::msg::Image>> arrivals;
  for (int k = 0; k < 90; ++k) {  // three seconds at 30 Hz
    arrivals.emplace_back(kSteady0 + k / 30.0, MakeImage(8, 8, "rgb8", "camera_link", 0.0));
  }
  const int delivered = Deliver(capture, "sensor_msgs/msg/Image", arrivals);
  EXPECT_GE(delivered, 5);
  EXPECT_LE(delivered, 7);
}

// A simulator reset steps every stamp back; the arrivals keep coming at the source's rate, and so
// do the frames.
TEST_F(MediaGateTest, AStampThatStepsBackwardsChangesNothing) {
  CameraCapture capture = Camera(2.0);
  std::vector<std::pair<double, sensor_msgs::msg::Image>> arrivals;
  for (int k = 0; k < 60; ++k) {  // six seconds at 10 Hz, the clock reset half way
    const double stamp = k < 30 ? 500.0 + k / 10.0 : 1.0 + (k - 30) / 10.0;
    arrivals.emplace_back(kSteady0 + k / 10.0, MakeImage(8, 8, "rgb8", "camera_link", stamp));
  }
  const int delivered = Deliver(capture, "sensor_msgs/msg/Image", arrivals);
  EXPECT_GE(delivered, 11);
  EXPECT_LE(delivered, 13);
}

// The lidar gate is the same gate, on arrival too: front and rear scanners on one topic with
// clocks apart, and a scanner whose driver never sets the stamp.
TEST_F(MediaGateTest, TheLidarGateIsOnArrivalToo) {
  LidarConfig config;
  config.name = "scan";
  config.topic = "/scan";
  config.rate_hz = 2.0;
  const auto scan = [](double stamp) {
    sensor_msgs::msg::LaserScan message;
    message.header.frame_id = "base_scan";
    message.header.stamp.sec = static_cast<std::int32_t>(stamp);
    message.header.stamp.nanosec =
        static_cast<std::uint32_t>((stamp - std::floor(stamp)) * 1e9);
    message.angle_increment = 0.1f;
    message.range_max = 10.0f;
    message.ranges.assign(8, 1.0f);
    return message;
  };
  {
    LidarCapture capture(config);
    std::vector<std::pair<double, sensor_msgs::msg::LaserScan>> arrivals;
    for (int k = 0; k < 10 * 30; ++k) {
      const double arrival = kSteady0 + k / 10.0;
      arrivals.emplace_back(arrival, scan(kEpoch0 + k / 10.0));
      arrivals.emplace_back(arrival + 0.05, scan(kEpoch0 + k / 10.0 - 0.75));
    }
    const int delivered = Deliver(capture, "sensor_msgs/msg/LaserScan", arrivals);
    EXPECT_GE(delivered, 59);
    EXPECT_LE(delivered, 61) << "two publishers into a 2 Hz gate over thirty seconds";
  }
  {
    LidarCapture capture(config);
    std::vector<std::pair<double, sensor_msgs::msg::LaserScan>> arrivals;
    for (int k = 0; k < 60; ++k) arrivals.emplace_back(kSteady0 + k / 10.0, scan(0.0));
    const int delivered = Deliver(capture, "sensor_msgs/msg/LaserScan", arrivals);
    EXPECT_GE(delivered, 11) << "a zero-stamped scanner froze";
    EXPECT_LE(delivered, 13);
  }
}

// ---------------------------------------------------------------------------
// Camera capture
// ---------------------------------------------------------------------------

TEST_F(MediaTest, AnImageIsEncodedResizedAndRecorded) {
  CameraConfig config;
  config.name = "front";
  config.topic = "/image";
  config.rate_hz = 100.0;
  config.max_width_px = 64;
  CameraCapture capture(config);

  const auto message = Serialize(MakeImage(320, 240, "rgb8"));
  ASSERT_TRUE(capture.Handle("sensor_msgs/msg/Image", message, environment_, *runner_));
  EXPECT_EQ(capture.stats().frames, 1u);
  EXPECT_GT(capture.stats().bytes, 0u);
  EXPECT_EQ(capture.stats().refused, 0u);
  ASSERT_EQ(looked_up_frames_.size(), 1u);
  EXPECT_EQ(looked_up_frames_[0], "camera_link");

  // The size on the wire is what was SENT, not what was configured: the aspect
  // ratio is kept, so 320x240 at max_width 64 is 64x48.
  std::uint64_t bytes = 0;
  std::uint16_t width = 0, height = 0;
  std::string why;
  ASSERT_TRUE(capture.MeasureEncodedFrame("sensor_msgs/msg/Image", message, &bytes, &width,
                                          &height, &why))
      << why;
  EXPECT_EQ(width, 64);
  EXPECT_EQ(height, 48);
}

// The field of view of the image a CameraInfo describes, against intrinsics whose angles are known.
// K, width, height and the ROI are full-resolution unbinned pixels: an ROI moves the principal
// point and replaces the size, and binning changes no angle.
namespace {

sensor_msgs::msg::CameraInfo Intrinsics(double fx, double fy, double cx, double cy,
                                        std::uint32_t width, std::uint32_t height) {
  sensor_msgs::msg::CameraInfo info;
  info.width = width;
  info.height = height;
  info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  return info;
}

void ExpectFov(const c3d::CameraFov& fov, double h, double v) {
  EXPECT_NEAR(fov.hfov_deg, h, 1e-4);
  EXPECT_NEAR(fov.vfov_deg, v, 1e-4);
}

}  // namespace

TEST(CameraInfoFov, TheAnglesComeFromKWidthAndHeight) {
  ExpectFov(CameraFovFromCameraInfo(Intrinsics(500, 500, 320, 240, 640, 480)), 65.238486,
            51.282012);
  // fx and fy are each axis's own: swapped, the vertical angle would be the horizontal one's.
  ExpectFov(CameraFovFromCameraInfo(Intrinsics(500, 400, 320, 240, 640, 480)), 65.238486,
            61.927513);
  // An off-centre principal point: not 2 atan(W / 2 fx).
  ExpectFov(CameraFovFromCameraInfo(Intrinsics(500, 500, 400, 240, 640, 480)), 64.300814,
            51.282012);
}

TEST(CameraInfoFov, BinningChangesNoAngle) {
  for (const std::uint32_t binning : {0u, 1u, 2u, 4u}) {
    auto info = Intrinsics(500, 500, 320, 240, 640, 480);
    info.binning_x = binning;
    info.binning_y = binning;
    ExpectFov(CameraFovFromCameraInfo(info), 65.238486, 51.282012);
  }
}

TEST(CameraInfoFov, AnRoiIsTheWindowItNames) {
  // A centred 640 x 480 window of a 1280 x 960 sensor at f = 1000.
  auto info = Intrinsics(1000, 1000, 640, 480, 1280, 960);
  ExpectFov(CameraFovFromCameraInfo(info), 65.238486, 51.282012);
  info.roi.x_offset = 320;
  info.roi.y_offset = 240;
  info.roi.width = 640;
  info.roi.height = 480;
  ExpectFov(CameraFovFromCameraInfo(info), 35.489343, 26.991467);
  // The same window binned by 2 is a 320 x 240 image of the same scene.
  info.binning_x = 2;
  info.binning_y = 2;
  ExpectFov(CameraFovFromCameraInfo(info), 35.489343, 26.991467);
  // A window against the sensor's left edge: the principal point is at its right edge.
  info.roi.x_offset = 0;
  ExpectFov(CameraFovFromCameraInfo(info), 32.619243, 26.991467);
  // An axis whose ROI size is 0 is the full image on that axis.
  info.roi.width = 0;
  info.roi.x_offset = 0;
  ExpectFov(CameraFovFromCameraInfo(info), 65.238486, 26.991467);
}

TEST(CameraInfoFov, AnUncalibratedDriversZeroKIsNoFieldOfView) {
  ExpectFov(CameraFovFromCameraInfo(sensor_msgs::msg::CameraInfo{}), 0.0, 0.0);
  ExpectFov(CameraFovFromCameraInfo(Intrinsics(0, 0, 0, 0, 640, 480)), 0.0, 0.0);
  ExpectFov(CameraFovFromCameraInfo(Intrinsics(-500, 500, 320, 240, 640, 480)), 0.0, 0.0);
  ExpectFov(CameraFovFromCameraInfo(Intrinsics(500, 500, 320, 240, 0, 0)), 0.0, 0.0);
}

TEST(CameraInfoFov, TheSiblingTopicFollowsImageTransport) {
  EXPECT_EQ(SiblingCameraInfoTopic("/image_raw"), "/camera_info");
  EXPECT_EQ(SiblingCameraInfoTopic("/image_raw/compressed"), "/camera_info");
  EXPECT_EQ(SiblingCameraInfoTopic("/oakd/rgb/preview/image_raw"), "/oakd/rgb/preview/camera_info");
  EXPECT_EQ(SiblingCameraInfoTopic("/cam/image_raw/compressed"), "/cam/camera_info");
  EXPECT_EQ(SiblingCameraInfoTopic("/cam/image_raw/compressedDepth"), "/cam/camera_info");
  EXPECT_EQ(SiblingCameraInfoTopic("/cam/image_raw/theora"), "/cam/camera_info");
  EXPECT_EQ(SiblingCameraInfoTopic("/cam/image_raw/zstd"), "/cam/camera_info");
  EXPECT_EQ(SiblingCameraInfoTopic("/cam/image_raw/ffmpeg"), "/cam/camera_info");
  // Only a transport's name is stripped, and only the last segment.
  EXPECT_EQ(SiblingCameraInfoTopic("/compressed/image"), "/compressed/camera_info");
  EXPECT_EQ(SiblingCameraInfoTopic("/cam/rgb/image_rect_color"), "/cam/rgb/camera_info");
}

// The parameters win; without them the latest CameraInfo is the field of view, so a driver that
// republishes it as it zooms changes it frame to frame; without either, none.
TEST(CameraFovSource, ParametersThenTheLatestCameraInfoThenNone) {
  std::vector<std::string> warnings;
  MediaEnvironment environment;
  environment.warn_once = [&warnings](const std::string& text) { warnings.push_back(text); };

  CameraConfig config;
  config.name = "front";
  config.topic = "/cam/image_raw";
  config.camera_info_topic = "/cam/camera_info";
  CameraCapture capture(config);
  EXPECT_STREQ(capture.fov_source(), "camera_info");
  EXPECT_TRUE(capture.wants_camera_info());
  ExpectFov(capture.CurrentFov(), 0.0, 0.0);  // nothing yet

  capture.HandleCameraInfo(Intrinsics(500, 500, 320, 240, 640, 480), environment);
  ExpectFov(capture.CurrentFov(), 65.238486, 51.282012);
  // Zoomed in: twice the focal length.
  capture.HandleCameraInfo(Intrinsics(1000, 1000, 320, 240, 640, 480), environment);
  ExpectFov(capture.CurrentFov(), 35.489343, 26.991467);
  // The camera's, not the session's: a new session keeps it.
  capture.Reset();
  ExpectFov(capture.CurrentFov(), 35.489343, 26.991467);
  EXPECT_TRUE(warnings.empty());

  // A calibration that disappears is no field of view, said once, naming the parameters.
  capture.HandleCameraInfo(Intrinsics(0, 0, 0, 0, 640, 480), environment);
  ExpectFov(capture.CurrentFov(), 0.0, 0.0);
  capture.HandleCameraInfo(Intrinsics(0, 0, 0, 0, 640, 480), environment);
  ASSERT_EQ(warnings.size(), 1u);
  EXPECT_NE(warnings[0].find("camera.front.hfov_degrees"), std::string::npos) << warnings[0];
  EXPECT_NE(warnings[0].find("/cam/camera_info"), std::string::npos) << warnings[0];

  // Serialized, as the subscription hands it over.
  EXPECT_TRUE(capture.HandleCameraInfo(Serialize(Intrinsics(500, 500, 320, 240, 640, 480)),
                                       environment));
  ExpectFov(capture.CurrentFov(), 65.238486, 51.282012);

  // The parameters win over a usable CameraInfo, and the topic is then not wanted.
  config.hfov_degrees = 61.0;
  config.vfov_degrees = 45.0;
  CameraCapture measured(config);
  EXPECT_STREQ(measured.fov_source(), "parameter");
  EXPECT_FALSE(measured.wants_camera_info());
  measured.HandleCameraInfo(Intrinsics(500, 500, 320, 240, 640, 480), environment);
  ExpectFov(measured.CurrentFov(), 61.0, 45.0);
  measured.HandleCameraInfo(Intrinsics(0, 0, 0, 0, 640, 480), environment);
  EXPECT_EQ(warnings.size(), 1u) << "nothing to say while the parameters are in force";

  // Switched off, a CameraInfo that arrives anyway is not the field of view.
  config.hfov_degrees = 0.0;
  config.vfov_degrees = 0.0;
  config.camera_info_topic.clear();
  CameraCapture off(config);
  EXPECT_STREQ(off.fov_source(), "none");
  EXPECT_FALSE(off.wants_camera_info());
  off.HandleCameraInfo(Intrinsics(500, 500, 320, 240, 640, 480), environment);
  ExpectFov(off.CurrentFov(), 0.0, 0.0);
}

// A camera whose frames take their field of view from a CameraInfo topic nothing publishes carries
// none on any frame, and the session's fov_source still says camera_info: the first frame of each
// session says so, naming the topic and the two parameters. A publisher that has not sent yet is a
// slow driver, not a missing one, and a camera with a field of view to carry has nothing to say.
TEST_F(MediaTest, ACameraInfoTopicWithNoPublisherIsSaidAtTheFirstFrameOfEachSession) {
  std::map<std::string, std::size_t> publishers;
  environment_.publishers = [&publishers](const std::string& topic) { return publishers[topic]; };
  CameraConfig config;
  config.name = "front";
  config.topic = "/cam/image_raw";
  config.camera_info_topic = "/cam/camera_info";
  CameraCapture capture(config);
  const auto frame = [this](CameraCapture& camera) {
    return camera.Handle("sensor_msgs/msg/Image", Serialize(MakeImage(32, 32, "rgb8")),
                         environment_, *runner_);
  };
  ASSERT_TRUE(frame(capture));
  ASSERT_TRUE(frame(capture));
  ASSERT_EQ(warnings_.size(), 1u);
  for (const std::string said : {"camera 'front'", "/cam/camera_info, which has no publisher",
                                 "camera.front.hfov_degrees and vfov_degrees"}) {
    EXPECT_NE(warnings_[0].find(said), std::string::npos) << said << ": " << warnings_[0];
  }
  capture.Reset();
  ASSERT_TRUE(frame(capture));
  EXPECT_EQ(warnings_.size(), 2u) << "each session is told";

  publishers["/cam/camera_info"] = 1;
  capture.Reset();
  ASSERT_TRUE(frame(capture));
  EXPECT_EQ(warnings_.size(), 2u) << "a publisher that has not sent yet";

  // A CameraInfo already received is carried, whatever publishes now.
  publishers["/cam/camera_info"] = 0;
  capture.HandleCameraInfo(Intrinsics(500, 500, 320, 240, 640, 480), environment_);
  capture.Reset();
  ASSERT_TRUE(frame(capture));
  EXPECT_EQ(warnings_.size(), 2u) << "a field of view to carry";

  config.hfov_degrees = 61.0;
  config.vfov_degrees = 45.0;
  CameraCapture measured(config);
  ASSERT_TRUE(frame(measured));
  EXPECT_EQ(warnings_.size(), 2u) << "the parameters are the field of view";
}

// Every frame carries the parameters' field of view when they are set, and none otherwise. What
// reaches the wire is the capture tier's to show; this is what Emit hands the session.
TEST(CameraFovSource, TheParametersAreTheFieldOfViewWhenSet) {
  CameraConfig config;
  config.name = "front";
  config.topic = "/image";
  CameraCapture unset(config);
  EXPECT_EQ(unset.CurrentFov().hfov_deg, 0.0f);
  EXPECT_EQ(unset.CurrentFov().vfov_deg, 0.0f);
  EXPECT_STREQ(unset.fov_source(), "none");

  config.hfov_degrees = 61.0;
  config.vfov_degrees = 45.5;
  CameraCapture set(config);
  EXPECT_EQ(set.CurrentFov().hfov_deg, 61.0f);
  EXPECT_EQ(set.CurrentFov().vfov_deg, 45.5f);
  EXPECT_STREQ(set.fov_source(), "parameter");
}

TEST_F(MediaTest, AConfiguredFrameWinsOverTheMessageHeader) {
  CameraConfig config;
  config.name = "front";
  config.topic = "/image";
  config.rate_hz = 100.0;
  config.frame = "front_optical_frame";
  CameraCapture capture(config);

  ASSERT_TRUE(capture.Handle("sensor_msgs/msg/Image",
                             Serialize(MakeImage(32, 32, "rgb8", "wrong_frame")), environment_,
                             *runner_));
  ASSERT_EQ(looked_up_frames_.size(), 1u);
  EXPECT_EQ(looked_up_frames_[0], "front_optical_frame");
}

TEST_F(MediaTest, ADepthImageIsRefusedOncePerTopicAndNamedAsDepth) {
  CameraConfig config;
  config.name = "front";
  config.topic = "/depth";
  config.rate_hz = 100.0;
  CameraCapture capture(config);

  for (int i = 0; i < 5; ++i) {
    EXPECT_FALSE(capture.Handle("sensor_msgs/msg/Image",
                                Serialize(MakeImage(32, 32, "32FC1", "camera_link", 100.0 + i)),
                                environment_, *runner_));
  }
  EXPECT_EQ(capture.stats().frames, 0u);
  EXPECT_EQ(capture.stats().refused, 5u);
  // Counted five times, said once: at 30 Hz a per-message warning IS the log.
  ASSERT_EQ(warnings_.size(), 1u);
  EXPECT_NE(warnings_[0].find("DEPTH"), std::string::npos) << warnings_[0];
}

TEST_F(MediaTest, AJpegThatNeedsNothingDoneToItIsPassedThroughByteForByte) {
  CameraConfig config;
  config.name = "front";
  config.topic = "/image/compressed";
  config.rate_hz = 100.0;
  config.max_width_px = 640;
  CameraCapture capture(config);

  sensor_msgs::msg::CompressedImage compressed;
  compressed.header.frame_id = "camera_link";
  compressed.header.stamp.sec = 100;
  compressed.format = "rgb8; jpeg compressed bgr8";
  const std::string original = MakeJpeg(320, 240);
  compressed.data.assign(original.begin(), original.end());

  const auto message = Serialize(compressed);
  std::uint64_t bytes = 0;
  std::uint16_t width = 0, height = 0;
  std::string why;
  ASSERT_TRUE(capture.MeasureEncodedFrame("sensor_msgs/msg/CompressedImage", message, &bytes,
                                          &width, &height, &why))
      << why;
  EXPECT_EQ(bytes, original.size()) << "a pass-through must not re-encode";
  EXPECT_EQ(width, 320);
  EXPECT_EQ(height, 240);

  ASSERT_TRUE(capture.Handle("sensor_msgs/msg/CompressedImage", message, environment_, *runner_));
  EXPECT_EQ(capture.stats().bytes, original.size());
}

TEST_F(MediaTest, AJpegWiderThanTheCapIsDecodedAndReEncoded) {
  CameraConfig config;
  config.name = "front";
  config.topic = "/image/compressed";
  config.rate_hz = 100.0;
  config.max_width_px = 100;
  CameraCapture capture(config);

  sensor_msgs::msg::CompressedImage compressed;
  compressed.header.frame_id = "camera_link";
  compressed.header.stamp.sec = 100;
  compressed.format = "jpeg";
  const std::string original = MakeJpeg(320, 240);
  compressed.data.assign(original.begin(), original.end());

  std::uint64_t bytes = 0;
  std::uint16_t width = 0, height = 0;
  std::string why;
  ASSERT_TRUE(capture.MeasureEncodedFrame("sensor_msgs/msg/CompressedImage",
                                          Serialize(compressed), &bytes, &width, &height, &why))
      << why;
  EXPECT_EQ(width, 100);
  EXPECT_EQ(height, 75);
  EXPECT_NE(bytes, original.size());
}

TEST_F(MediaTest, AFrameWithNoPoseIsDroppedAndCounted) {
  pose_available_ = false;
  CameraConfig config;
  config.name = "front";
  config.topic = "/image";
  config.rate_hz = 100.0;
  CameraCapture capture(config);

  EXPECT_FALSE(capture.Handle("sensor_msgs/msg/Image", Serialize(MakeImage(32, 32, "rgb8")),
                              environment_, *runner_));
  EXPECT_EQ(capture.stats().frames, 0u);
  EXPECT_EQ(capture.stats().pose_failures, 1u);
}

// A frame stamped ahead of its tf chain is held, not waited for, and recorded once the chain
// catches up: the node's one callback thread never stops for it.
class MediaHoldTest : public MediaTest {
 protected:
  void SetUp() override {
    MediaTest::SetUp();
    environment_.pose_wait_s = 0.05;
    environment_.arrival_time = [this]() { return now_; };
  }

  static LidarConfig Scanner() {
    LidarConfig config;
    config.name = "scan";
    config.topic = "/scan";
    config.rate_hz = 1000.0;
    return config;
  }

  rclcpp::SerializedMessage ScanAt(double stamp_s) {
    sensor_msgs::msg::LaserScan scan;
    scan.header.frame_id = "base_scan";
    scan.header.stamp.sec = static_cast<std::int32_t>(stamp_s);
    scan.header.stamp.nanosec =
        static_cast<std::uint32_t>(std::lround((stamp_s - std::floor(stamp_s)) * 1e9));
    scan.range_max = 10.0f;
    scan.ranges.assign(8, 1.0f);
    now_ += 0.002;
    return Serialize(scan);
  }

  double now_ = 2000.0;
};

TEST_F(MediaHoldTest, AFrameAheadOfItsChainIsHeldAndRecordedWhenTheChainCatchesUp) {
  pose_until_ = 100.0;
  CameraConfig config;
  config.name = "front";
  config.topic = "/image";
  config.rate_hz = 1000.0;
  CameraCapture capture(config);

  const auto image = Serialize(MakeImage(32, 32, "rgb8", "camera_link", 100.03));
  EXPECT_FALSE(capture.Handle("sensor_msgs/msg/Image", image, environment_, *runner_));
  EXPECT_EQ(capture.held(), 1u);
  EXPECT_EQ(capture.stats().frames, 0u);
  EXPECT_EQ(capture.stats().pose_failures, 0u) << "a frame still inside its wait is not a failure";

  now_ += 0.02;
  pose_until_ = 100.05;
  capture.ServiceHeld(environment_, *runner_);
  EXPECT_EQ(capture.held(), 0u);
  EXPECT_EQ(capture.stats().frames, 1u);
  EXPECT_EQ(capture.stats().pose_failures, 0u);
}

TEST_F(MediaHoldTest, AHeldScanWhosePoseNeverArrivesIsDroppedAtItsDeadline) {
  pose_available_ = false;
  LidarCapture capture(Scanner());
  EXPECT_FALSE(capture.Handle("sensor_msgs/msg/LaserScan", ScanAt(100.0), environment_, *runner_));
  ASSERT_EQ(capture.held(), 1u);

  now_ += 0.04;
  capture.ServiceHeld(environment_, *runner_);
  EXPECT_EQ(capture.held(), 1u) << "dropped before its wait was out";
  EXPECT_EQ(capture.stats().pose_failures, 0u);

  now_ += 0.02;
  capture.ServiceHeld(environment_, *runner_);
  EXPECT_EQ(capture.held(), 0u);
  EXPECT_EQ(capture.stats().pose_failures, 1u);
  EXPECT_EQ(capture.stats().scans, 0u);
}

// Records reach the session in arrival order, so `seq` follows the sensor: one whose pose is
// already there waits behind one that arrived before it.
TEST_F(MediaHoldTest, ARecordIsNeverWrittenAheadOfOneHeldBeforeIt) {
  pose_until_ = 100.0;
  LidarCapture capture(Scanner());
  EXPECT_FALSE(capture.Handle("sensor_msgs/msg/LaserScan", ScanAt(100.03), environment_, *runner_));
  EXPECT_FALSE(capture.Handle("sensor_msgs/msg/LaserScan", ScanAt(99.99), environment_, *runner_));
  EXPECT_EQ(capture.held(), 2u);
  EXPECT_EQ(capture.stats().scans, 0u);

  pose_until_ = 101.0;
  capture.ServiceHeld(environment_, *runner_);
  EXPECT_EQ(capture.held(), 0u);
  EXPECT_EQ(capture.stats().scans, 2u);
}

TEST_F(MediaHoldTest, AStoppedChainHoldsAtMostTheCapAndCountsWhatItPushesOut) {
  pose_available_ = false;
  LidarCapture capture(Scanner());
  const std::size_t offered = HeldRecords::kMaxHeld + 3;
  for (std::size_t i = 0; i < offered; ++i) {
    capture.Handle("sensor_msgs/msg/LaserScan", ScanAt(100.0 + 0.001 * i), environment_, *runner_);
  }
  EXPECT_EQ(capture.held(), HeldRecords::kMaxHeld);
  EXPECT_EQ(capture.stats().pose_failures, 3u);
}

TEST_F(MediaHoldTest, NothingHeldOutlivesItsSession) {
  pose_available_ = false;
  LidarCapture capture(Scanner());
  capture.Handle("sensor_msgs/msg/LaserScan", ScanAt(100.0), environment_, *runner_);
  ASSERT_EQ(capture.held(), 1u);
  capture.Reset();
  EXPECT_EQ(capture.held(), 0u) << "a scan from the last session would open the next one";

  capture.Handle("sensor_msgs/msg/LaserScan", ScanAt(100.1), environment_, *runner_);
  ASSERT_EQ(capture.held(), 1u);
  runner_->End("test");
  pose_available_ = true;
  capture.ServiceHeld(environment_, *runner_);
  EXPECT_EQ(capture.held(), 0u);
  EXPECT_EQ(capture.stats().scans, 0u) << "recorded into a session that had ended";
  EXPECT_EQ(capture.stats().pose_failures, 0u);
}

TEST_F(MediaTest, AnEmptyFrameIdWithNoConfiguredFrameIsRefusedWithAnExplanation) {
  CameraConfig config;
  config.name = "front";
  config.topic = "/image";
  config.rate_hz = 100.0;
  CameraCapture capture(config);

  EXPECT_FALSE(capture.Handle("sensor_msgs/msg/Image", Serialize(MakeImage(32, 32, "rgb8", "")),
                              environment_, *runner_));
  EXPECT_EQ(capture.stats().pose_failures, 1u);
  ASSERT_EQ(warnings_.size(), 1u);
  EXPECT_NE(warnings_[0].find("camera.front.frame"), std::string::npos) << warnings_[0];
}

// ---------------------------------------------------------------------------
// The pose every binary record carries
// ---------------------------------------------------------------------------

namespace {

constexpr double kPi = 3.14159265358979323846;

// Written out here rather than borrowed, so the expectation owes nothing to the
// code under test. A test that reaches for the SDK's own conversion to say what
// the SDK's conversion should produce proves only that it is self-consistent.
c3d::Quat Multiply(const c3d::Quat& a, const c3d::Quat& b) {
  return c3d::Quat{a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                   a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                   a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                   a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

c3d::Vec3 Rotate(const c3d::Quat& q, const c3d::Vec3& v) {
  const c3d::Quat point{v.x, v.y, v.z, 0.0};
  const c3d::Quat conjugate{-q.x, -q.y, -q.z, q.w};
  const c3d::Quat out = Multiply(Multiply(q, point), conjugate);
  return c3d::Vec3{out.x, out.y, out.z};
}

// Yaw 40 degrees then pitch 25, about ROS Z then ROS Y. Deliberately not a pure
// yaw and not axis-aligned with anything: under a pure yaw several wrong rules
// agree, which is how a rotation rule ships unverified.
constexpr double kYaw = 40.0 * kPi / 180.0;
constexpr double kPitch = 25.0 * kPi / 180.0;

c3d::Quat SensorRotation() {
  const c3d::Quat about_z{0.0, 0.0, std::sin(kYaw / 2.0), std::cos(kYaw / 2.0)};
  const c3d::Quat about_y{0.0, std::sin(kPitch / 2.0), 0.0, std::cos(kPitch / 2.0)};
  return Multiply(about_z, about_y);
}

const c3d::Vec3 kSensorOrigin{1.3, -2.7, 0.9};
const c3d::Vec3 kSensorPoint{2.0, 0.5, -1.1};

// The same transform as a MATRIX, applied to the point: the world position of
// the sensor point in ROS axes, derived without a quaternion anywhere.
c3d::Vec3 RosWorldPoint() {
  const double cy = std::cos(kYaw), sy = std::sin(kYaw);
  const double cp = std::cos(kPitch), sp = std::sin(kPitch);
  const double m[3][3] = {{cy * cp, -sy, cy * sp}, {sy * cp, cy, sy * sp}, {-sp, 0.0, cp}};
  const double p[3] = {kSensorPoint.x, kSensorPoint.y, kSensorPoint.z};
  return c3d::Vec3{m[0][0] * p[0] + m[0][1] * p[1] + m[0][2] * p[2] + kSensorOrigin.x,
                   m[1][0] * p[0] + m[1][1] * p[1] + m[1][2] * p[2] + kSensorOrigin.y,
                   m[2][0] * p[0] + m[2][1] * p[1] + m[2][2] * p[2] + kSensorOrigin.z};
}

// What a reader of the stored bytes does: rotate the cloud's own point by the
// record's quaternion and add the record's position.
c3d::Vec3 Reconstruct(const c3d::Pose& pose, PoseConvention convention) {
  const c3d::Vec3 rotated = Rotate(pose.rotation, ConvertPosition(kSensorPoint, convention));
  return c3d::Vec3{rotated.x + pose.position.x, rotated.y + pose.position.y,
                   rotated.z + pose.position.z};
}

}  // namespace

TEST(ScenePoseRule, ASensorPointReconstructsUnderEveryConvention) {
  const c3d::Vec3 ros_world = RosWorldPoint();
  for (const PoseConvention convention :
       {PoseConvention::kRep103, PoseConvention::kUnity, PoseConvention::kGltfAuthored}) {
    const c3d::Pose pose = ScenePose(kSensorOrigin, SensorRotation(), convention);
    const c3d::Vec3 got = Reconstruct(pose, convention);
    const c3d::Vec3 want = ConvertPosition(ros_world, convention);
    const std::string which = PoseConventionName(convention);
    EXPECT_NEAR(got.x, want.x, 1e-4) << which;
    EXPECT_NEAR(got.y, want.y, 1e-4) << which;
    EXPECT_NEAR(got.z, want.z, 1e-4) << which;
  }
}

TEST(ScenePoseRule, TheGazeRotationIsTheRuleThisIsNot) {
  // The two rules are the SAME map under rep103 and unity, so those two can
  // never tell them apart and this whole rule is decided by gltf_authored. The
  // numbers are here rather than only the inequality because "not equal" passes
  // for a rule that is wrong in some third way too.
  const c3d::Quat ros = SensorRotation();
  const c3d::Vec3 want = ConvertPosition(RosWorldPoint(), PoseConvention::kGltfAuthored);
  EXPECT_NEAR(want.x, -2.011031, 1e-5);
  EXPECT_NEAR(want.y, -0.942175, 1e-5);
  EXPECT_NEAR(want.z, 1.450670, 1e-5);

  c3d::Pose gaze;
  gaze.position = ConvertPosition(kSensorOrigin, PoseConvention::kGltfAuthored);
  gaze.rotation = ConvertGazeRotation(ros, PoseConvention::kGltfAuthored);
  const c3d::Vec3 misplaced = Reconstruct(gaze, PoseConvention::kGltfAuthored);
  EXPECT_NEAR(misplaced.x, 0.688830, 1e-5);
  EXPECT_NEAR(misplaced.y, 0.114371, 1e-5);
  EXPECT_NEAR(misplaced.z, 1.758012, 1e-5);

  // Under unity the two rules agree, which is exactly why a test written only
  // against the default convention proves nothing about the choice.
  EXPECT_EQ(ConvertGazeRotation(ros, PoseConvention::kUnity).x,
            ConvertDynamicRotation(ros, PoseConvention::kUnity).x);
}

// ---------------------------------------------------------------------------
// The pose a LaserScan carries, which is not the pose the other two carry
// ---------------------------------------------------------------------------

namespace {

// Yaw 40, pitch 25 and ROLL 15 degrees, composed Rz Ry Rx. The roll is what makes
// this rule testable at all: a sensor without one has a level scan plane, and a
// level plane stays level under rules that are wrong about everything else.
constexpr double kRoll = 15.0 * kPi / 180.0;

c3d::Quat ScanSensorRotation() {
  const c3d::Quat about_z{0.0, 0.0, std::sin(kYaw / 2.0), std::cos(kYaw / 2.0)};
  const c3d::Quat about_y{0.0, std::sin(kPitch / 2.0), 0.0, std::cos(kPitch / 2.0)};
  const c3d::Quat about_x{std::sin(kRoll / 2.0), 0.0, 0.0, std::cos(kRoll / 2.0)};
  return Multiply(about_z, Multiply(about_y, about_x));
}

// Where one ray's endpoint lands in ROS axes, from the MATRIX Rz(yaw) Ry(pitch)
// Rx(roll) written out here, so the expectation owes nothing to the quaternion
// code under test. The ray itself lies in the sensor's own XY plane, which is the
// fact the whole rule rests on.
c3d::Vec3 RosRayEndpoint(double range, double angle) {
  const double cy = std::cos(kYaw), sy = std::sin(kYaw);
  const double cp = std::cos(kPitch), sp = std::sin(kPitch);
  const double cr = std::cos(kRoll), sr = std::sin(kRoll);
  const double m[3][3] = {{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
                          {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
                          {-sp, cp * sr, cp * cr}};
  const double p[3] = {range * std::cos(angle), range * std::sin(angle), 0.0};
  return c3d::Vec3{m[0][0] * p[0] + m[0][1] * p[1] + m[0][2] * p[2] + kSensorOrigin.x,
                   m[1][0] * p[0] + m[1][1] * p[1] + m[1][2] * p[2] + kSensorOrigin.y,
                   m[2][0] * p[0] + m[2][1] * p[1] + m[2][2] * p[2] + kSensorOrigin.z};
}

struct Ray {
  double range;
  double angle;
};

// Theta 0 is the sensor's own +X, where a wrong rule can still agree with the
// right one; +/-120 degrees is where they part company.
const Ray kRays[] = {{2.5, 0.0},
                     {1.75, 120.0 * kPi / 180.0},
                     {3.1, -120.0 * kPi / 180.0},
                     {0.8, 37.0 * kPi / 180.0}};

// What a reader of the stored bytes does, holding an angle, a range and the
// record's pose -- and no idea which convention the session used.
c3d::Vec3 ReconstructRay(const c3d::Pose& pose, const Ray& ray) {
  const c3d::Vec3 naive{ray.range * std::cos(ray.angle), ray.range * std::sin(ray.angle), 0.0};
  const c3d::Vec3 rotated = Rotate(pose.rotation, naive);
  return c3d::Vec3{rotated.x + pose.position.x, rotated.y + pose.position.y,
                   rotated.z + pose.position.z};
}

}  // namespace

TEST(ScanPlaneRule, EveryRayReconstructsFromTheRecordAloneUnderEveryConvention) {
  for (const PoseConvention convention :
       {PoseConvention::kRep103, PoseConvention::kUnity, PoseConvention::kGltfAuthored}) {
    const std::string which = PoseConventionName(convention);
    const c3d::Pose pose = ScanPlanePose(kSensorOrigin, ScanSensorRotation(), convention);
    const c3d::Quat& q = pose.rotation;
    // A proper rotation or the reader's own rotate() is meaningless. The
    // construction builds a matrix and reads a quaternion back out of it, which is
    // exactly where a normalisation is dropped.
    EXPECT_NEAR(std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w), 1.0, 1e-12) << which;

    for (const Ray& ray : kRays) {
      const c3d::Vec3 got = ReconstructRay(pose, ray);
      const c3d::Vec3 want = ConvertPosition(RosRayEndpoint(ray.range, ray.angle), convention);
      const std::string where = which + " at " + std::to_string(ray.angle) + " rad";
      EXPECT_NEAR(got.x, want.x, 1e-6) << where;
      EXPECT_NEAR(got.y, want.y, 1e-6) << where;
      EXPECT_NEAR(got.z, want.z, 1e-6) << where;
    }
  }
}

TEST(ScanPlaneRule, ItIsNotTheRotationTheOtherRecordsCarry) {
  const c3d::Quat ros = ScanSensorRotation();

  // Under rep103 the position map is the identity, so there is no mirror to absorb
  // and the scan plane's rotation is the sensor's own, component for component.
  const c3d::Pose rep103 = ScanPlanePose(kSensorOrigin, ros, PoseConvention::kRep103);
  EXPECT_EQ(rep103.rotation.x, ros.x);
  EXPECT_EQ(rep103.rotation.y, ros.y);
  EXPECT_EQ(rep103.rotation.z, ros.z);
  EXPECT_EQ(rep103.rotation.w, ros.w);

  // Under the other two they are different rotations. The numbers are here rather
  // than only the inequality because "not equal" passes for a rule that is wrong
  // in some third way too.
  const c3d::Pose unity_scan = ScanPlanePose(kSensorOrigin, ros, PoseConvention::kUnity);
  EXPECT_NEAR(unity_scan.rotation.x, -0.406800, 1e-5);
  EXPECT_NEAR(unity_scan.rotation.y, 0.757663, 1e-5);
  EXPECT_NEAR(unity_scan.rotation.z, -0.466078, 1e-5);
  EXPECT_NEAR(unity_scan.rotation.w, -0.207923, 1e-5);
  const c3d::Pose unity_scene = ScenePose(kSensorOrigin, ros, PoseConvention::kUnity);
  EXPECT_NEAR(unity_scene.rotation.x, -0.245231, 1e-5);
  EXPECT_NEAR(unity_scene.rotation.y, 0.304509, 1e-5);
  EXPECT_NEAR(unity_scene.rotation.z, 0.046354, 1e-5);
  EXPECT_NEAR(unity_scene.rotation.w, -0.919232, 1e-5);

  const c3d::Pose gltf_scan = ScanPlanePose(kSensorOrigin, ros, PoseConvention::kGltfAuthored);
  EXPECT_NEAR(gltf_scan.rotation.x, 0.041916, 1e-5);
  EXPECT_NEAR(gltf_scan.rotation.y, 0.682772, 1e-5);
  EXPECT_NEAR(gltf_scan.rotation.z, -0.617218, 1e-5);
  EXPECT_NEAR(gltf_scan.rotation.w, 0.388725, 1e-5);
  const c3d::Pose gltf_scene = ScenePose(kSensorOrigin, ros, PoseConvention::kGltfAuthored);
  EXPECT_NEAR(gltf_scene.rotation.x, 0.046354, 1e-5);
  EXPECT_NEAR(gltf_scene.rotation.y, -0.304509, 1e-5);
  EXPECT_NEAR(gltf_scene.rotation.z, 0.245231, 1e-5);
  EXPECT_NEAR(gltf_scene.rotation.w, 0.919232, 1e-5);

  // Only the rotation differs: both poses place the sensor at the same point, so a
  // consumer comparing the two sees one number change and has to know why.
  EXPECT_EQ(gltf_scan.position.x, gltf_scene.position.x);
  EXPECT_EQ(gltf_scan.position.y, gltf_scene.position.y);
  EXPECT_EQ(gltf_scan.position.z, gltf_scene.position.z);
}

TEST(ScanPlaneRule, TheOtherRecordsRotationPutsARayMetresAway) {
  // The defect this rule closes, in numbers: a reader applying the reconstruction
  // above to the rotation a camera frame carries draws the fan somewhere else
  // entirely. It is metres, not a rounding difference, and nothing in the record
  // says so -- so a viewer renders it and believes it.
  const c3d::Quat ros = ScanSensorRotation();
  const Ray straight_ahead = kRays[0];

  c3d::Pose wrong = ScenePose(kSensorOrigin, ros, PoseConvention::kGltfAuthored);
  const c3d::Vec3 misplaced = ReconstructRay(wrong, straight_ahead);
  EXPECT_NEAR(misplaced.x, 0.435680, 1e-5);
  EXPECT_NEAR(misplaced.y, 1.956546, 1e-5);
  EXPECT_NEAR(misplaced.z, 4.156409, 1e-5);

  const c3d::Vec3 want =
      ConvertPosition(RosRayEndpoint(straight_ahead.range, straight_ahead.angle),
                      PoseConvention::kGltfAuthored);
  EXPECT_NEAR(want.x, -3.035680, 1e-5);
  EXPECT_NEAR(want.y, -0.156546, 1e-5);
  EXPECT_NEAR(want.z, 1.243591, 1e-5);
}

// ---------------------------------------------------------------------------
// PointCloud2 field layouts
// ---------------------------------------------------------------------------

TEST(PointCloudLayoutTest, XyzAreRequiredAndMustBeFloat32) {
  EXPECT_FALSE(LayoutOf({Field("x", 0, PointField::FLOAT32),
                         Field("y", 4, PointField::FLOAT32)},
                        12)
                   .ok);
  const auto not_float = LayoutOf({Field("x", 0, PointField::FLOAT64),
                                   Field("y", 8, PointField::FLOAT64),
                                   Field("z", 16, PointField::FLOAT64)},
                                  24);
  EXPECT_FALSE(not_float.ok);
  EXPECT_NE(not_float.error.find("FLOAT32"), std::string::npos);

  const auto plain = LayoutOf({Field("x", 0, PointField::FLOAT32),
                               Field("y", 4, PointField::FLOAT32),
                               Field("z", 8, PointField::FLOAT32)},
                              12);
  ASSERT_TRUE(plain.ok);
  EXPECT_EQ(plain.color, PointCloudLayout::ColorKind::kNone);
  EXPECT_EQ(plain.intensity, PointCloudLayout::IntensityKind::kNone);
}

TEST(PointCloudLayoutTest, EveryColourAndIntensityVariantIsRecognised) {
  const std::vector<PointField> base{Field("x", 0, PointField::FLOAT32),
                                     Field("y", 4, PointField::FLOAT32),
                                     Field("z", 8, PointField::FLOAT32)};

  auto with = [&base](std::vector<PointField> extra) {
    std::vector<PointField> fields = base;
    for (auto& field : extra) fields.push_back(field);
    return LayoutOf(fields, 24);
  };

  // The PCL convention: a uint32 bit-punned into a float32.
  EXPECT_EQ(with({Field("rgb", 16, PointField::FLOAT32)}).color,
            PointCloudLayout::ColorKind::kPackedFloat32);
  EXPECT_EQ(with({Field("rgb", 16, PointField::UINT32)}).color,
            PointCloudLayout::ColorKind::kPackedUint32);
  EXPECT_EQ(with({Field("rgba", 16, PointField::UINT32)}).color,
            PointCloudLayout::ColorKind::kPackedUint32);
  const auto three = with({Field("r", 12, PointField::UINT8), Field("g", 13, PointField::UINT8),
                           Field("b", 14, PointField::UINT8)});
  EXPECT_EQ(three.color, PointCloudLayout::ColorKind::kThreeBytes);
  EXPECT_EQ(three.color_offset, 12u);
  EXPECT_EQ(three.g_offset, 13u);
  EXPECT_EQ(three.b_offset, 14u);

  EXPECT_EQ(with({Field("intensity", 12, PointField::FLOAT32)}).intensity,
            PointCloudLayout::IntensityKind::kFloat32);
  EXPECT_EQ(with({Field("intensity", 12, PointField::UINT8)}).intensity,
            PointCloudLayout::IntensityKind::kUint8);
  EXPECT_EQ(with({Field("intensity", 12, PointField::UINT16)}).intensity,
            PointCloudLayout::IntensityKind::kUint16);
  // A type nobody publishes intensity as is ignored rather than read as bytes.
  EXPECT_EQ(with({Field("intensity", 12, PointField::FLOAT64)}).intensity,
            PointCloudLayout::IntensityKind::kNone);
}

TEST(PointCloudExtract, PointsTakeTheSameAxisMapAsThePosePosition) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.fields = {Field("x", 0, PointField::FLOAT32), Field("y", 4, PointField::FLOAT32),
                  Field("z", 8, PointField::FLOAT32), Field("rgb", 12, PointField::FLOAT32)};
  cloud.point_step = 16;
  cloud.height = 1;
  cloud.width = 1;
  cloud.row_step = 16;
  cloud.is_bigendian = false;
  cloud.data.assign(16, 0);
  PutFloat(&cloud.data, 0, 1.0f);
  PutFloat(&cloud.data, 4, 2.0f);
  PutFloat(&cloud.data, 8, 3.0f);
  PutUint32(&cloud.data, 12, 0x00FF8000u);

  const PointCloudLayout layout = ResolvePointCloudLayout(cloud);
  ASSERT_TRUE(layout.ok);

  // rep103 changes nothing, which is what makes it the control.
  const auto rep103 = ExtractPointCloud(cloud, layout, true, false, PoseConvention::kRep103);
  ASSERT_EQ(rep103.xyz.size(), 3u);
  EXPECT_FLOAT_EQ(rep103.xyz[0], 1.0f);
  EXPECT_FLOAT_EQ(rep103.xyz[1], 2.0f);
  EXPECT_FLOAT_EQ(rep103.xyz[2], 3.0f);
  ASSERT_EQ(rep103.rgb.size(), 3u);
  EXPECT_EQ(rep103.rgb[0], 0xFF);
  EXPECT_EQ(rep103.rgb[1], 0x80);
  EXPECT_EQ(rep103.rgb[2], 0x00);

  // unity: (x, y, z) -> (-y, z, x)
  const auto unity = ExtractPointCloud(cloud, layout, false, false, PoseConvention::kUnity);
  EXPECT_FLOAT_EQ(unity.xyz[0], -2.0f);
  EXPECT_FLOAT_EQ(unity.xyz[1], 3.0f);
  EXPECT_FLOAT_EQ(unity.xyz[2], 1.0f);

  // gltf_authored: (x, y, z) -> (-x, z, -y)
  const auto gltf =
      ExtractPointCloud(cloud, layout, false, false, PoseConvention::kGltfAuthored);
  EXPECT_FLOAT_EQ(gltf.xyz[0], -1.0f);
  EXPECT_FLOAT_EQ(gltf.xyz[1], 3.0f);
  EXPECT_FLOAT_EQ(gltf.xyz[2], -2.0f);
}

TEST(PointCloudExtract, AnIntegerIntensityFieldIsMarkedAsIntegral) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.fields = {Field("x", 0, PointField::FLOAT32), Field("y", 4, PointField::FLOAT32),
                  Field("z", 8, PointField::FLOAT32), Field("intensity", 12, PointField::UINT8)};
  cloud.point_step = 16;
  cloud.height = 1;
  cloud.width = 1;
  cloud.is_bigendian = false;
  cloud.data.assign(16, 0);
  cloud.data[12] = 200;

  const PointCloudLayout layout = ResolvePointCloudLayout(cloud);
  ASSERT_TRUE(layout.ok);
  const auto input = ExtractPointCloud(cloud, layout, false, true, PoseConvention::kRep103);
  ASSERT_EQ(input.intensity.size(), 1u);
  EXPECT_FLOAT_EQ(input.intensity[0], 200.0f);
  // The flag, not the values, is what stops the encoder scaling a 0..1-looking
  // integer field by 255.
  EXPECT_TRUE(input.intensity_is_integral);

  cloud.fields[3] = Field("intensity", 12, PointField::FLOAT32);
  PutFloat(&cloud.data, 12, 0.5f);
  const auto as_float = ExtractPointCloud(cloud, ResolvePointCloudLayout(cloud), false,
                                          true, PoseConvention::kRep103);
  EXPECT_FALSE(as_float.intensity_is_integral);
}

TEST(PointCloudLayoutTest, ABigEndianCloudAndAZeroPointStepAreLayoutErrors) {
  const std::vector<PointField> xyz{Field("x", 0, PointField::FLOAT32),
                                    Field("y", 4, PointField::FLOAT32),
                                    Field("z", 8, PointField::FLOAT32)};

  sensor_msgs::msg::PointCloud2 big_endian = CloudOf(xyz, 12);
  big_endian.is_bigendian = true;
  const auto refused = ResolvePointCloudLayout(big_endian);
  EXPECT_FALSE(refused.ok);
  EXPECT_NE(refused.error.find("big-endian"), std::string::npos) << refused.error;

  // Both are layout errors rather than checks inside ExtractPointCloud, where the
  // only thing the caller would see is a cloud with no points -- which is also
  // what an empty cloud looks like. As layout errors they carry a reason to the
  // warning.
  sensor_msgs::msg::PointCloud2 no_step = CloudOf(xyz, 12);
  no_step.point_step = 0;
  const auto stepless = ResolvePointCloudLayout(no_step);
  EXPECT_FALSE(stepless.ok);
  EXPECT_NE(stepless.error.find("point_step"), std::string::npos) << stepless.error;
}

TEST(PointCloudLayoutTest, AFieldThatDoesNotFitInThePointStepIsRefusedByName) {
  // `z` needs bytes 12..15 of a 12-byte point, so it is read out of the NEXT
  // point -- a plausible number, never an obviously wrong one.
  const auto past_end = LayoutOf({Field("x", 0, PointField::FLOAT32),
                                  Field("y", 4, PointField::FLOAT32),
                                  Field("z", 12, PointField::FLOAT32)},
                                 12);
  EXPECT_FALSE(past_end.ok);
  EXPECT_NE(past_end.error.find("'z'"), std::string::npos) << past_end.error;
  EXPECT_NE(past_end.error.find("point_step"), std::string::npos) << past_end.error;

  // An optional field is no different: the cloud's self-description is wrong,
  // and reading positions out of it while ignoring the colour would be trusting
  // half of a description that has already been shown to be untrustworthy.
  const auto colour_past_end = LayoutOf({Field("x", 0, PointField::FLOAT32),
                                         Field("y", 4, PointField::FLOAT32),
                                         Field("z", 8, PointField::FLOAT32),
                                         Field("rgb", 14, PointField::UINT32)},
                                        16);
  EXPECT_FALSE(colour_past_end.ok);
  EXPECT_NE(colour_past_end.error.find("'rgb'"), std::string::npos) << colour_past_end.error;

  const auto intensity_past_end = LayoutOf({Field("x", 0, PointField::FLOAT32),
                                            Field("y", 4, PointField::FLOAT32),
                                            Field("z", 8, PointField::FLOAT32),
                                            Field("intensity", 15, PointField::UINT16)},
                                           16);
  EXPECT_FALSE(intensity_past_end.ok);
  EXPECT_NE(intensity_past_end.error.find("'intensity'"), std::string::npos)
      << intensity_past_end.error;

  // The boundary case is the one an off-by-one gets wrong: the last byte of the
  // point is a legal place for a field to end.
  EXPECT_TRUE(LayoutOf({Field("x", 0, PointField::FLOAT32), Field("y", 4, PointField::FLOAT32),
                        Field("z", 8, PointField::FLOAT32)},
                       12)
                  .ok);
}

TEST(PointCloudExtract, PaddedRowsAreWalkedByRowStepRatherThanAsOneFlatArray) {
  // Three points per row, two rows, and four bytes of padding at the end of each
  // row -- what an organised cloud aligned to a hardware stride looks like. Read
  // flat, the second row starts inside the padding and every one of its points
  // comes back shifted.
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.fields = {Field("x", 0, PointField::FLOAT32), Field("y", 4, PointField::FLOAT32),
                  Field("z", 8, PointField::FLOAT32)};
  cloud.point_step = 12;
  cloud.width = 3;
  cloud.height = 2;
  cloud.row_step = 3 * 12 + 4;
  cloud.is_bigendian = false;
  cloud.data.assign(cloud.row_step * 2, 0xEE);
  for (std::size_t row = 0; row < 2; ++row) {
    for (std::size_t col = 0; col < 3; ++col) {
      const std::size_t at = row * cloud.row_step + col * 12;
      PutFloat(&cloud.data, at, static_cast<float>(row * 3 + col));
      PutFloat(&cloud.data, at + 4, 0.0f);
      PutFloat(&cloud.data, at + 8, 0.0f);
    }
  }

  const auto input =
      ExtractPointCloud(cloud, ResolvePointCloudLayout(cloud), false, false,
                        PoseConvention::kRep103);
  ASSERT_EQ(input.xyz.size(), 6u * 3u);
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_FLOAT_EQ(input.xyz[i * 3], static_cast<float>(i)) << "point " << i;
  }
}

TEST(PointCloudExtract, ABufferShorterThanTheCloudItDescribesStopsAtTheBytesItHas) {
  // `width * height` says 4 points and the buffer holds two and a half. The
  // missing bytes are not zeros to be read as points at the origin -- they are
  // not there at all.
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.fields = {Field("x", 0, PointField::FLOAT32), Field("y", 4, PointField::FLOAT32),
                  Field("z", 8, PointField::FLOAT32)};
  cloud.point_step = 12;
  cloud.width = 4;
  cloud.height = 1;
  cloud.row_step = 4 * 12;
  cloud.is_bigendian = false;
  cloud.data.assign(30, 0);
  PutFloat(&cloud.data, 0, 1.0f);
  PutFloat(&cloud.data, 12, 2.0f);

  const auto input =
      ExtractPointCloud(cloud, ResolvePointCloudLayout(cloud), false, false,
                        PoseConvention::kRep103);
  ASSERT_EQ(input.xyz.size(), 2u * 3u);
  EXPECT_FLOAT_EQ(input.xyz[0], 1.0f);
  EXPECT_FLOAT_EQ(input.xyz[3], 2.0f);
}

TEST(PointCloudExtract, ABufferLongerThanTheCloudItDescribesContributesNothingExtra) {
  // The mirror: two points declared, room for four. A reader that divides the
  // buffer by the point step sends two points of whatever the allocation held.
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.fields = {Field("x", 0, PointField::FLOAT32), Field("y", 4, PointField::FLOAT32),
                  Field("z", 8, PointField::FLOAT32)};
  cloud.point_step = 12;
  cloud.width = 2;
  cloud.height = 1;
  cloud.row_step = 2 * 12;
  cloud.is_bigendian = false;
  cloud.data.assign(48, 0x7F);
  PutFloat(&cloud.data, 0, 1.0f);
  PutFloat(&cloud.data, 12, 2.0f);

  const auto input =
      ExtractPointCloud(cloud, ResolvePointCloudLayout(cloud), false, false,
                        PoseConvention::kRep103);
  EXPECT_EQ(input.xyz.size(), 2u * 3u);
}

// ---------------------------------------------------------------------------
// Lidar capture
// ---------------------------------------------------------------------------

TEST_F(MediaTest, LaserScanIntensitiesAreDroppedUnlessAskedFor) {
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "base_scan";
  scan.header.stamp.sec = 100;
  scan.angle_min = -3.14f;
  scan.angle_max = 3.14f;
  scan.angle_increment = 0.01f;
  scan.range_min = 0.1f;
  scan.range_max = 12.0f;
  scan.ranges.assign(640, 1.5f);
  scan.intensities.assign(640, 100.0f);
  const auto message = Serialize(scan);

  LidarConfig config;
  config.name = "scan";
  config.topic = "/scan";
  config.rate_hz = 100.0;
  LidarCapture without(config);
  ASSERT_TRUE(without.Handle("sensor_msgs/msg/LaserScan", message, environment_, *runner_));
  EXPECT_EQ(without.stats().bytes, 640u * sizeof(float));

  config.intensities = true;
  LidarCapture with(config);
  ASSERT_TRUE(with.Handle("sensor_msgs/msg/LaserScan", message, environment_, *runner_));
  EXPECT_EQ(with.stats().bytes, 2u * 640u * sizeof(float));
}

TEST_F(MediaTest, AnOversizeCloudIsDroppedAndCountedAsOversize) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.fields = {Field("x", 0, PointField::FLOAT32), Field("y", 4, PointField::FLOAT32),
                  Field("z", 8, PointField::FLOAT32)};
  cloud.header.frame_id = "camera_link";
  cloud.header.stamp.sec = 100;
  cloud.point_step = 12;
  cloud.height = 1;
  cloud.width = 100;
  cloud.is_bigendian = false;
  cloud.data.assign(12 * 100, 0);
  for (int i = 0; i < 100; ++i) PutFloat(&cloud.data, i * 12, static_cast<float>(i));

  LidarConfig config;
  config.name = "depth";
  config.topic = "/points";
  config.rate_hz = 100.0;
  config.pointcloud.voxel_leaf_m = 0.0;
  config.pointcloud.max_points = 10;
  LidarCapture capture(config);

  EXPECT_FALSE(capture.Handle("sensor_msgs/msg/PointCloud2", Serialize(cloud), environment_,
                              *runner_));
  EXPECT_EQ(capture.stats().scans, 0u);
  EXPECT_EQ(capture.stats().dropped_oversize, 1u);
  EXPECT_EQ(capture.stats().refused, 0u);
}

TEST_F(MediaTest, ACloudThisSdkCannotReadIsCountedAsRefusedAndWarnedAboutOnce) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.fields = {Field("x", 0, PointField::FLOAT32), Field("y", 4, PointField::FLOAT32),
                  Field("z", 8, PointField::FLOAT32)};
  cloud.header.frame_id = "camera_link";
  cloud.header.stamp.sec = 100;
  cloud.point_step = 12;
  cloud.height = 1;
  cloud.width = 2;
  cloud.is_bigendian = true;
  cloud.row_step = 24;
  cloud.data.assign(24, 0);

  LidarConfig config;
  config.name = "depth";
  config.topic = "/points";
  config.rate_hz = 100.0;
  LidarCapture capture(config);

  EXPECT_FALSE(capture.Handle("sensor_msgs/msg/PointCloud2", Serialize(cloud), environment_,
                              *runner_));
  EXPECT_EQ(capture.stats().scans, 0u);
  // Counted, and counted as a REFUSAL rather than as an oversize or a range
  // drop: the difference is the operator's next move.
  EXPECT_EQ(capture.stats().refused, 1u);
  EXPECT_EQ(capture.stats().dropped_oversize, 0u);
  ASSERT_EQ(warnings_.size(), 1u);
  EXPECT_NE(warnings_[0].find("big-endian"), std::string::npos) << warnings_[0];
  EXPECT_NE(warnings_[0].find("depth"), std::string::npos) << warnings_[0];
}

TEST_F(MediaTest, PointsBeyondInt16AreCountedAsRangeDrops) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.fields = {Field("x", 0, PointField::FLOAT32), Field("y", 4, PointField::FLOAT32),
                  Field("z", 8, PointField::FLOAT32)};
  cloud.header.frame_id = "camera_link";
  cloud.header.stamp.sec = 100;
  cloud.point_step = 12;
  cloud.height = 1;
  cloud.width = 2;
  cloud.is_bigendian = false;
  cloud.data.assign(24, 0);
  PutFloat(&cloud.data, 0, 1.0f);   // in range
  PutFloat(&cloud.data, 12, 90.0f);  // 90 m, past int16 millimetres

  LidarConfig config;
  config.name = "depth";
  config.topic = "/points";
  config.rate_hz = 100.0;
  config.pointcloud.voxel_leaf_m = 0.0;
  LidarCapture capture(config);

  ASSERT_TRUE(capture.Handle("sensor_msgs/msg/PointCloud2", Serialize(cloud), environment_,
                             *runner_));
  EXPECT_EQ(capture.stats().scans, 1u);
  EXPECT_EQ(capture.stats().dropped_range, 1u);
}

namespace {

std::size_t WarningsMentioning(const std::vector<std::string>& warnings, const std::string& text) {
  return static_cast<std::size_t>(
      std::count_if(warnings.begin(), warnings.end(), [&text](const std::string& warning) {
        return warning.find(text) != std::string::npos;
      }));
}

sensor_msgs::msg::PointCloud2 XyzCloud(const std::string& frame, bool big_endian = false) {
  sensor_msgs::msg::PointCloud2 cloud = CloudOf(
      {Field("x", 0, PointField::FLOAT32), Field("y", 4, PointField::FLOAT32),
       Field("z", 8, PointField::FLOAT32)},
      12);
  cloud.header.frame_id = frame;
  cloud.header.stamp.sec = 100;
  cloud.is_bigendian = big_endian;
  PutFloat(&cloud.data, 0, 1.0f);
  return cloud;
}

}  // namespace

// A camera says two different things once each: that its frames carry no frame id, and that it
// cannot encode them. Each is said whichever comes first, and neither is said twice.
TEST_F(MediaTest, ACamerasTwoWarningsAreEachSaidOnceAndNeitherHidesTheOther) {
  CameraConfig config;
  config.name = "front";
  config.topic = "/image";
  config.rate_hz = 100.0;
  const auto unframed = Serialize(MakeImage(32, 32, "rgb8", ""));
  const auto depth = Serialize(MakeImage(32, 32, "32FC1", "camera_link"));

  for (const bool frame_first : {true, false}) {
    warnings_.clear();
    CameraCapture capture(config);
    for (int i = 0; i < 3; ++i) {
      capture.Handle("sensor_msgs/msg/Image", frame_first ? unframed : depth, environment_,
                     *runner_);
      capture.Handle("sensor_msgs/msg/Image", frame_first ? depth : unframed, environment_,
                     *runner_);
    }
    EXPECT_EQ(warnings_.size(), 2u) << (frame_first ? "frame id first" : "refusal first");
    EXPECT_EQ(WarningsMentioning(warnings_, "camera.front.frame"), 1u);
    EXPECT_EQ(WarningsMentioning(warnings_, "DEPTH"), 1u);
  }
}

TEST_F(MediaTest, ALidarsWarningsAreEachSaidOnceAndNeitherHidesTheOther) {
  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp.sec = 100;
  scan.angle_increment = 0.01f;
  scan.range_max = 12.0f;
  scan.ranges.assign(16, 1.5f);
  const auto unframed = Serialize(scan);
  const auto unreadable = Serialize(XyzCloud("camera_link", /*big_endian=*/true));

  LidarConfig config;
  config.name = "depth";
  config.topic = "/points";
  config.rate_hz = 100.0;
  for (const bool frame_first : {true, false}) {
    warnings_.clear();
    LidarCapture capture(config);
    for (int i = 0; i < 3; ++i) {
      if (frame_first) {
        capture.Handle("sensor_msgs/msg/LaserScan", unframed, environment_, *runner_);
        capture.Handle("sensor_msgs/msg/PointCloud2", unreadable, environment_, *runner_);
      } else {
        capture.Handle("sensor_msgs/msg/PointCloud2", unreadable, environment_, *runner_);
        capture.Handle("sensor_msgs/msg/LaserScan", unframed, environment_, *runner_);
      }
    }
    EXPECT_EQ(warnings_.size(), 2u) << (frame_first ? "frame id first" : "refusal first");
    EXPECT_EQ(WarningsMentioning(warnings_, "lidar.depth.frame"), 1u);
    EXPECT_EQ(WarningsMentioning(warnings_, "big-endian"), 1u);
  }
}

// The records of a cloud with no colour field say `none`, whatever pointcloud.color asked for, and
// the session property still names the request. Said once per session, so the two can be told
// apart while the run is live.
TEST_F(MediaTest, AColourTheCloudDoesNotCarryIsWarnedAboutOncePerSession) {
  LidarConfig config;
  config.name = "depth";
  config.topic = "/points";
  config.rate_hz = 100.0;
  config.pointcloud.voxel_leaf_m = 0.0;
  config.pointcloud.color = c3d::PointColor::kRgb565;
  LidarCapture capture(config);
  const auto colourless = Serialize(XyzCloud("camera_link"));

  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(capture.Handle("sensor_msgs/msg/PointCloud2", colourless, environment_, *runner_));
  }
  ASSERT_EQ(warnings_.size(), 1u);
  EXPECT_NE(warnings_[0].find("pointcloud.color rgb565"), std::string::npos) << warnings_[0];
  EXPECT_NE(warnings_[0].find("without colour"), std::string::npos) << warnings_[0];

  capture.Reset();
  EXPECT_TRUE(capture.Handle("sensor_msgs/msg/PointCloud2", colourless, environment_, *runner_));
  EXPECT_EQ(warnings_.size(), 2u) << "a new session is told again";

  // Neither a cloud that carries colour nor a lidar that asks for none has anything to say.
  sensor_msgs::msg::PointCloud2 coloured = CloudOf(
      {Field("x", 0, PointField::FLOAT32), Field("y", 4, PointField::FLOAT32),
       Field("z", 8, PointField::FLOAT32), Field("rgb", 12, PointField::FLOAT32)},
      16);
  coloured.header.frame_id = "camera_link";
  coloured.header.stamp.sec = 100;
  LidarCapture with_colour(config);
  EXPECT_TRUE(
      with_colour.Handle("sensor_msgs/msg/PointCloud2", Serialize(coloured), environment_, *runner_));
  config.pointcloud.color = c3d::PointColor::kNone;
  LidarCapture without_request(config);
  EXPECT_TRUE(
      without_request.Handle("sensor_msgs/msg/PointCloud2", colourless, environment_, *runner_));
  EXPECT_EQ(warnings_.size(), 2u);
}

// ---------------------------------------------------------------------------
// doctor
// ---------------------------------------------------------------------------

// A camera with no field of view source records fine and has no frustum in Session Replay, and
// nothing else says so. Every way of having none is an advisory naming the two parameters; the
// parameters, or a CameraInfo with usable intrinsics, pass and say the angles.
TEST(DoctorCameraFov, EveryWayOfHavingNoSourceIsAnAdvisoryNamingTheParameters) {
  doctor::CameraFovFacts facts;
  facts.name = "front";
  const auto advisory = [](const doctor::Check& check, const std::string& detail) {
    EXPECT_EQ(check.severity, doctor::Severity::kAdvisory) << check.detail;
    EXPECT_NE(check.detail.find(detail), std::string::npos) << check.detail;
    EXPECT_NE(check.remedy.find("camera.front.hfov_degrees and vfov_degrees"), std::string::npos)
        << check.remedy;
  };
  EXPECT_EQ(doctor::CheckCameraFov(facts).name, "camera/front field of view");
  advisory(doctor::CheckCameraFov(facts), "camera_info_topic is empty");

  facts.camera_info_topic = "/cam/camera_info";
  advisory(doctor::CheckCameraFov(facts), "/cam/camera_info is not on the graph");

  facts.topic_present = true;
  facts.type = "sensor_msgs/msg/Image";
  advisory(doctor::CheckCameraFov(facts), "not sensor_msgs/msg/CameraInfo");

  facts.type = "sensor_msgs/msg/CameraInfo";
  advisory(doctor::CheckCameraFov(facts), "published nothing");

  facts.sample_seen = true;
  advisory(doctor::CheckCameraFov(facts), "no usable intrinsics (fx 0)");
  facts.sample_hfov_degrees = 65.2f;
  advisory(doctor::CheckCameraFov(facts), "no usable intrinsics");

  facts.sample_vfov_degrees = 51.3f;
  facts.sample_fx = 500.0;
  const doctor::Check from_info = doctor::CheckCameraFov(facts);
  EXPECT_EQ(from_info.severity, doctor::Severity::kOk) << from_info.detail;
  EXPECT_NE(from_info.detail.find("65.2 x 51.3 degrees, from /cam/camera_info"), std::string::npos)
      << from_info.detail;

  // The parameters win whatever the topic says.
  doctor::CameraFovFacts measured;
  measured.name = "front";
  measured.parameters_set = true;
  measured.hfov_degrees = 61.0;
  measured.vfov_degrees = 45.0;
  const doctor::Check from_parameters = doctor::CheckCameraFov(measured);
  EXPECT_EQ(from_parameters.severity, doctor::Severity::kOk);
  EXPECT_NE(from_parameters.detail.find("61.0 x 45.0 degrees, from camera.front.hfov_degrees"),
            std::string::npos)
      << from_parameters.detail;
}

TEST(DoctorMedia, AbsentTopicIsAdvisoryAndAWrongTypeIsFatal) {
  doctor::MediaSensorFacts facts;
  facts.kind = "camera";
  facts.name = "front";
  facts.topic = "/oakd/rgb/preview/image_raw";
  facts.rate_hz = 1.0;

  const doctor::Check absent = doctor::CheckMediaSensor(facts);
  EXPECT_EQ(absent.severity, doctor::Severity::kAdvisory);
  EXPECT_FALSE(absent.remedy.empty());

  facts.topic_present = true;
  facts.type = "sensor_msgs/msg/CameraInfo";
  const doctor::Check wrong_type = doctor::CheckMediaSensor(facts);
  EXPECT_EQ(wrong_type.severity, doctor::Severity::kFatal);
  EXPECT_NE(wrong_type.detail.find("sensor_msgs/msg/CameraInfo"), std::string::npos);
}

TEST(DoctorMedia, AFrameThatDoesNotResolveIsFatalBecauseEveryRecordWouldBeDropped) {
  doctor::MediaSensorFacts facts;
  facts.kind = "lidar";
  facts.name = "depth";
  facts.topic = "/points";
  facts.rate_hz = 1.0;
  facts.topic_present = true;
  facts.type = "sensor_msgs/msg/PointCloud2";
  facts.type_handled = true;
  facts.frame = "oakd_rgb_camera_optical_fram";
  facts.frame_resolves = false;
  const doctor::Check check = doctor::CheckMediaSensor(facts);
  EXPECT_EQ(check.severity, doctor::Severity::kFatal);
  EXPECT_NE(check.detail.find("oakd_rgb_camera_optical_fram"), std::string::npos);
  // view_frames' default 5 seconds can miss a frame on a slow simulation.
  EXPECT_NE(check.remedy.find("`ros2 run tf2_tools view_frames --wait-time 15`"),
            std::string::npos)
      << check.remedy;
}

TEST(DoctorMedia, ASilentTopicIsAdvisoryAndNamesTheQosPossibility) {
  doctor::MediaSensorFacts facts;
  facts.kind = "camera";
  facts.name = "front";
  facts.topic = "/image";
  facts.rate_hz = 1.0;
  facts.topic_present = true;
  facts.type = "sensor_msgs/msg/Image";
  facts.type_handled = true;
  facts.qos = "reliable/volatile/keep_last(10)";
  facts.frame = "camera_link";
  facts.frame_resolves = true;
  const doctor::Check check = doctor::CheckMediaSensor(facts);
  EXPECT_EQ(check.severity, doctor::Severity::kAdvisory);
  EXPECT_NE(check.remedy.find("QoS"), std::string::npos);
}

TEST(DoctorMedia, AMeasuredSampleIsReportedAsASizeAndARate) {
  doctor::MediaSensorFacts facts;
  facts.kind = "camera";
  facts.name = "front";
  facts.topic = "/image";
  facts.rate_hz = 2.0;
  facts.topic_present = true;
  facts.type = "sensor_msgs/msg/Image";
  facts.type_handled = true;
  facts.qos = "best_effort/volatile/keep_last(1)";
  facts.frame = "camera_link";
  facts.frame_resolves = true;
  facts.sample_seen = true;
  facts.encoded_bytes = 20 * 1024;
  const doctor::Check check = doctor::CheckMediaSensor(facts);
  EXPECT_EQ(check.severity, doctor::Severity::kOk);
  EXPECT_NE(check.detail.find("20.0 KB"), std::string::npos) << check.detail;
  EXPECT_NE(check.detail.find("40.0 KB/s"), std::string::npos) << check.detail;
}

// The bandwidth check adds sensors up over a second. This is a different limit
// with a different failure: the platform stores each frame or scan as ONE
// record and drops a record over 1 MiB after the upload has already answered
// 200, so a slow sensor can sail under the bandwidth budget while every one of
// its records is dropped that way.
TEST(DoctorMedia, AnOversizedSingleRecordWarnsWhileTheBandwidthIsFine) {
  doctor::MediaSensorFacts lidar;
  lidar.kind = "lidar";
  lidar.name = "depth";
  lidar.topic = "/points";
  lidar.rate_hz = 1.0;
  lidar.topic_present = true;
  lidar.type = "sensor_msgs/msg/PointCloud2";
  lidar.type_handled = true;
  lidar.qos = "best_effort/volatile/keep_last(1)";
  lidar.frame = "base_scan";
  lidar.frame_resolves = true;
  lidar.sample_seen = true;
  lidar.encoded_bytes = 1200 * 1024;

  const doctor::Check check = doctor::CheckMediaSensor(lidar);
  EXPECT_EQ(check.severity, doctor::Severity::kAdvisory);
  EXPECT_NE(check.remedy.find("depth"), std::string::npos) << check.remedy;
  EXPECT_NE(check.remedy.find("1.2 MB"), std::string::npos) << check.remedy;
  EXPECT_NE(check.remedy.find("max_points"), std::string::npos) << check.remedy;
  EXPECT_EQ(doctor::CheckMediaBandwidth({lidar}).severity, doctor::Severity::kOk)
      << "1.2 MB/s is inside the bandwidth budget, so nothing else would have said this";

  // The knobs named are the ones the sensor actually has.
  doctor::MediaSensorFacts camera = lidar;
  camera.kind = "camera";
  camera.name = "front";
  camera.type = "sensor_msgs/msg/Image";
  const doctor::Check camera_check = doctor::CheckMediaSensor(camera);
  EXPECT_EQ(camera_check.severity, doctor::Severity::kAdvisory);
  EXPECT_NE(camera_check.remedy.find("max_width_px"), std::string::npos) << camera_check.remedy;
  EXPECT_NE(camera_check.remedy.find("jpeg_quality"), std::string::npos) << camera_check.remedy;
}

namespace {

// A plain xyz float32 cloud, the layout every depth driver publishes.
sensor_msgs::msg::PointCloud2 XyzCloud(const std::vector<std::array<float, 3>>& points) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.fields = {Field("x", 0, PointField::FLOAT32), Field("y", 4, PointField::FLOAT32),
                  Field("z", 8, PointField::FLOAT32)};
  cloud.point_step = 12;
  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.is_bigendian = false;
  cloud.data.assign(points.size() * 12, 0);
  for (std::size_t i = 0; i < points.size(); ++i) {
    PutFloat(&cloud.data, i * 12 + 0, points[i][0]);
    PutFloat(&cloud.data, i * 12 + 4, points[i][1]);
    PutFloat(&cloud.data, i * 12 + 8, points[i][2]);
  }
  return cloud;
}

// The SAME scene -- a wall of returns across a field of view -- written in the two
// frames a depth sensor has. Optical: depth on +Z, x horizontal and therefore
// mixed. Body: depth on +X, z vertical and mixed about the optical axis. The
// horizontal and vertical spreads straddle zero on purpose; a fixture whose points
// all sat on one side would make either frame look like the other.
std::vector<std::array<float, 3>> DepthWall(bool optical, int count) {
  std::vector<std::array<float, 3>> points;
  points.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    const float horizontal = -0.6f + 1.2f * static_cast<float>(i % 20) / 19.0f;
    const float vertical = -0.5f + 1.0f * static_cast<float>((i / 20) % 20) / 19.0f;
    const float depth = 1.5f + 0.01f * static_cast<float>(i % 7);
    points.push_back(optical ? std::array<float, 3>{horizontal, vertical, depth}
                             : std::array<float, 3>{depth, -horizontal, -vertical});
  }
  return points;
}

doctor::MediaSensorFacts CloudFacts(const sensor_msgs::msg::PointCloud2& cloud,
                                    const std::string& frame) {
  doctor::MediaSensorFacts facts;
  facts.kind = "lidar";
  facts.name = "depth";
  facts.topic = "/oakd/rgb/preview/depth/points";
  facts.type = "sensor_msgs/msg/PointCloud2";
  facts.frame = frame;
  const PointCloudAxisSigns signs =
      CountPointCloudAxisSigns(cloud, ResolvePointCloudLayout(cloud));
  facts.finite_points = signs.finite;
  facts.x_positive_points = signs.x_positive;
  facts.z_positive_points = signs.z_positive;
  return facts;
}

}  // namespace

TEST(PointCloudAxisCensus, CountsOnlyTheFinitePoints) {
  std::vector<std::array<float, 3>> points = DepthWall(true, 400);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  // What a depth camera returns where it saw nothing, which on an organised cloud
  // is most of the sky. Counted, they would move both fractions towards whatever
  // the empty part of the scene happened to be.
  points.push_back({nan, nan, nan});
  points.push_back({1.0f, 2.0f, inf});
  points.push_back({-inf, 0.0f, 3.0f});

  const sensor_msgs::msg::PointCloud2 cloud = XyzCloud(points);
  const PointCloudAxisSigns signs =
      CountPointCloudAxisSigns(cloud, ResolvePointCloudLayout(cloud));
  EXPECT_EQ(signs.finite, 400u);
  EXPECT_EQ(signs.z_positive, 400u) << "every finite point of an optical cloud is in front of it";
  EXPECT_GT(signs.x_positive, 150u);
  EXPECT_LT(signs.x_positive, 250u) << "x is the horizontal axis and straddles zero";
}

TEST(DoctorOpticalCloud, AnOpticalFrameCloudWhoseGeometryAgreesPasses) {
  const sensor_msgs::msg::PointCloud2 cloud = XyzCloud(DepthWall(true, 400));
  const doctor::Check check =
      doctor::CheckOpticalCloud(CloudFacts(cloud, "oakd_rgb_camera_optical_frame"));
  EXPECT_EQ(check.severity, doctor::Severity::kOk);
  EXPECT_EQ(check.name.find("NOT CHECKED"), std::string::npos) << check.name;
  EXPECT_NE(check.detail.find("100% have z > 0"), std::string::npos) << check.detail;
  EXPECT_TRUE(check.remedy.empty());
}

TEST(DoctorOpticalCloud, ABodyFrameCloudLabelledOpticalWarnsAndNamesTheOverride) {
  // What Gazebo's rgbd_camera publishes: the optical frame in the header and the
  // points still in the frame the sensor is bolted to. Both clouds are the same
  // scene, so nothing but the axes separates them.
  const sensor_msgs::msg::PointCloud2 cloud = XyzCloud(DepthWall(false, 400));
  const doctor::Check check =
      doctor::CheckOpticalCloud(CloudFacts(cloud, "oakd_rgb_camera_optical_frame"));
  EXPECT_EQ(check.severity, doctor::Severity::kAdvisory);
  EXPECT_NE(check.detail.find("/oakd/rgb/preview/depth/points"), std::string::npos) << check.detail;
  EXPECT_NE(check.detail.find("oakd_rgb_camera_optical_frame"), std::string::npos) << check.detail;
  EXPECT_NE(check.detail.find("100% have x > 0"), std::string::npos) << check.detail;
  EXPECT_NE(check.detail.find("% have z > 0"), std::string::npos) << check.detail;
  EXPECT_NE(check.remedy.find("lidar.depth.frame"), std::string::npos) << check.remedy;
  EXPECT_NE(check.remedy.find("oakd_rgb_camera_frame"), std::string::npos) << check.remedy;
  EXPECT_NE(check.remedy.find("rgbd_camera"), std::string::npos) << check.remedy;
  EXPECT_NE(check.remedy.find("`ros2 run tf2_tools view_frames --wait-time 15`"),
            std::string::npos)
      << check.remedy;
}

TEST(DoctorOpticalCloud, TheAttachedLinkGuessNamesAFrameWorthLookingFor) {
  // The bare `<x>_optical` spelling of the pair, whose partner is `<x>_link`. A
  // guess that drops the suffix and appends nothing names `oakd_rgb_camera` -- a
  // frame that is in no tf tree, in a remedy whose whole job is to give someone
  // something to check against theirs.
  const sensor_msgs::msg::PointCloud2 cloud = XyzCloud(DepthWall(false, 400));
  const doctor::Check check =
      doctor::CheckOpticalCloud(CloudFacts(cloud, "oakd_rgb_camera_optical"));
  EXPECT_EQ(check.severity, doctor::Severity::kAdvisory);
  EXPECT_NE(check.remedy.find("oakd_rgb_camera_link"), std::string::npos) << check.remedy;
}

TEST(DoctorOpticalCloud, DeclinesWhereItCannotDecide) {
  // A frame that is not optical: the points are expected in it and there is no
  // mismatch of this shape to look for. RENAMED rather than passed, because a
  // check that did not run must not read as one that did.
  const sensor_msgs::msg::PointCloud2 body = XyzCloud(DepthWall(false, 400));
  const doctor::Check not_optical = doctor::CheckOpticalCloud(CloudFacts(body, "base_scan"));
  EXPECT_NE(not_optical.name.find("(NOT CHECKED)"), std::string::npos) << not_optical.name;
  EXPECT_EQ(not_optical.severity, doctor::Severity::kOk);

  // Too few points for a fraction to mean anything, on a cloud that WOULD have
  // warned with more of them.
  const sensor_msgs::msg::PointCloud2 few = XyzCloud(DepthWall(false, 40));
  const doctor::Check thin =
      doctor::CheckOpticalCloud(CloudFacts(few, "oakd_rgb_camera_optical_frame"));
  EXPECT_NE(thin.name.find("(NOT CHECKED)"), std::string::npos) << thin.name;
  EXPECT_EQ(thin.severity, doctor::Severity::kOk);
  EXPECT_NE(thin.detail.find("40 finite points"), std::string::npos) << thin.detail;

  // Neither signature: a cloud centred on its own origin is in some third frame,
  // and naming a cause for it would be a guess printed as a finding.
  std::vector<std::array<float, 3>> scattered;
  for (int i = 0; i < 400; ++i) {
    const float a = -1.0f + 2.0f * static_cast<float>(i % 20) / 19.0f;
    const float b = -1.0f + 2.0f * static_cast<float>((i / 20) % 20) / 19.0f;
    scattered.push_back({a, 0.5f, b});
  }
  const doctor::Check undecided =
      doctor::CheckOpticalCloud(CloudFacts(XyzCloud(scattered), "camera_optical"));
  EXPECT_NE(undecided.name.find("(NOT CHECKED)"), std::string::npos) << undecided.name;
  EXPECT_EQ(undecided.severity, doctor::Severity::kOk);
  EXPECT_NE(undecided.detail.find("neither"), std::string::npos) << undecided.detail;
}

TEST(DoctorMedia, BandwidthWarnsAboveTheBudgetAndDeclinesWithNothingMeasured) {
  doctor::MediaSensorFacts camera;
  camera.kind = "camera";
  camera.name = "front";
  camera.rate_hz = 5.0;
  camera.sample_seen = true;
  camera.encoded_bytes = 100 * 1024;  // 500 KB/s

  doctor::MediaSensorFacts lidar;
  lidar.kind = "lidar";
  lidar.name = "depth";
  lidar.rate_hz = 1.0;
  lidar.sample_seen = true;
  lidar.encoded_bytes = 50 * 1024;

  const doctor::Check under = doctor::CheckMediaBandwidth({camera, lidar});
  EXPECT_EQ(under.severity, doctor::Severity::kOk);

  camera.rate_hz = 30.0;  // 3 MB/s
  const doctor::Check over = doctor::CheckMediaBandwidth({camera, lidar});
  EXPECT_EQ(over.severity, doctor::Severity::kAdvisory);
  EXPECT_NE(over.remedy.find("rate_hz"), std::string::npos);

  // A check that could not be asked is RENAMED, so it cannot be skim-read as one
  // that passed.
  const doctor::Check none = doctor::CheckMediaBandwidth({});
  EXPECT_NE(none.name.find("(NOT CHECKED)"), std::string::npos);
  EXPECT_EQ(none.severity, doctor::Severity::kOk);
}
