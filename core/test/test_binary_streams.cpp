#include <gtest/gtest.h>
#include <unistd.h>

#include <capnp/serialize-packed.h>
#include <kj/io.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "c3d/media.hpp"
#include "c3d/session.hpp"
#include "c3d/version.hpp"
#include "camera_payload.capnp.h"
#include "fake_transport.hpp"
#include "lidar_payload.capnp.h"

using namespace c3d;
namespace fs = std::filesystem;

namespace {

// Reads a packed body back through the GENERATED reader, generated from the same
// schemas the gateway reads these bodies with. A hand-rolled parser here would
// prove that this file agrees with itself.
class PackedBody {
 public:
  explicit PackedBody(const std::string& body)
      : bytes_(reinterpret_cast<const kj::byte*>(body.data()), body.size()),
        stream_(bytes_),
        reader_(stream_) {}

  cvr::capnp::CameraPayload::Reader camera() {
    return reader_.getRoot<cvr::capnp::CameraPayload>();
  }
  cvr::capnp::LidarPayload::Reader lidar() {
    return reader_.getRoot<cvr::capnp::LidarPayload>();
  }

 private:
  kj::ArrayPtr<const kj::byte> bytes_;
  kj::ArrayInputStream stream_;
  capnp::PackedMessageReader reader_;
};

std::string EveryByteValue() {
  std::string all;
  all.reserve(256);
  for (int i = 0; i < 256; ++i) all.push_back(static_cast<char>(i));
  return all;
}

Pose SomePose() {
  Pose pose;
  pose.position = Vec3{1.5, -2.25, 3.125};
  pose.rotation = Quat{0.1, 0.2, 0.3, 0.9273618};
  return pose;
}

LaserScanData SomeScan() {
  LaserScanData scan;
  scan.angle_min = -3.14f;
  scan.angle_max = 3.14f;
  scan.angle_increment = 0.01f;
  scan.time_increment = 0.0001f;
  scan.scan_time = 0.1f;
  scan.range_min = 0.1f;
  scan.range_max = 12.0f;
  scan.ranges = {1.0f, 2.0f, std::numeric_limits<float>::infinity()};
  return scan;
}

EncodedPointCloud SomeCloud() {
  PointCloudInput input;
  input.xyz = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
  PointCloudOptions options;
  options.voxel_leaf_m = 0.0;
  options.zstd_level = 1;
  return pointcloud::Encode(input, options);
}

class CountingLogger : public ILogger {
 public:
  void Log(LogLevel level, const std::string& message) override {
    std::lock_guard<std::mutex> lock(mu_);
    if (level == LogLevel::kWarn) warnings_.push_back(message);
  }
  int Warnings(const std::string& fragment) const {
    std::lock_guard<std::mutex> lock(mu_);
    int n = 0;
    for (const auto& m : warnings_) n += m.find(fragment) != std::string::npos ? 1 : 0;
    return n;
  }

 private:
  mutable std::mutex mu_;
  std::vector<std::string> warnings_;
};

class BinaryStreamTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() /
           ("c3d_bin_" + std::to_string(::getpid()) + "_" + std::to_string(counter_++));
    fs::create_directories(dir_);
    cfg_.application_key = "APPKEY";
    cfg_.scene_id = "SCENEGUID";
    cfg_.scene_version = "7";
    cfg_.device_id = "robot01";
    cfg_.app_version = "9.9.9-app";
    cfg_.spool_dir = dir_.string();
    cfg_.gateway = "data.example.invalid";
    cfg_.auto_send_interval_s = 3600.0;  // only the binary bounds may cut a payload
    cfg_.worker_tick_s = 0.01;
    // Deliberately unreachable by default: a test that wants a bound to fire
    // lowers exactly the one it is testing. The byte bounds sit at the largest
    // value Validate() accepts, because anything higher does not configure.
    cfg_.camera_flush_s = 3600.0;
    cfg_.camera_flush_bytes = 32ull * 1024 * 1024;
    cfg_.lidar_flush_s = 3600.0;
    cfg_.lidar_flush_bytes = 32ull * 1024 * 1024;
    transport_ = std::make_shared<FakeTransport>();
    spool_ = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                         std::make_shared<NullLogger>());
  }
  void TearDown() override {
    session_.reset();
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }
  void Build() {
    std::string err;
    session_ = Session::Create(cfg_, transport_, spool_, logger_, &err);
    ASSERT_NE(session_, nullptr) << err;
    ASSERT_TRUE(session_->Begin());
  }
  // Waits for the WORKER to post to ONE stream, so a test can tell "a bound fired"
  // from "Flush was called". Per stream and not by total count, because Begin()
  // flushes the session-start event before any of these tests record anything --
  // so a total-count wait is already satisfied when it is first asked.
  bool WaitForStream(const std::string& suffix, std::size_t at_least, double seconds = 3.0) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < deadline) {
      if (RequestsTo(suffix).size() >= at_least) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return RequestsTo(suffix).size() >= at_least;
  }
  std::vector<FakeTransport::Request> RequestsTo(const std::string& suffix) const {
    std::vector<FakeTransport::Request> out;
    for (const auto& r : transport_->requests()) {
      if (r.url.size() >= suffix.size() &&
          r.url.compare(r.url.size() - suffix.size(), suffix.size(), suffix) == 0) {
        out.push_back(r);
      }
    }
    return out;
  }

  Config cfg_;
  std::shared_ptr<FakeTransport> transport_;
  std::shared_ptr<DiskSpool> spool_;
  std::shared_ptr<CountingLogger> logger_ = std::make_shared<CountingLogger>();
  std::unique_ptr<Session> session_;
  fs::path dir_;
  static int counter_;
};
int BinaryStreamTest::counter_ = 0;

}  // namespace

TEST_F(BinaryStreamTest, CameraPayloadRoundTripsThroughTheGeneratedReader) {
  Build();
  const std::string jpeg = EveryByteValue();
  session_->RecordCameraFrame("front", 1788383013123ull, 0, "oakd_rgb_camera_optical_frame",
                              SomePose(), "jpeg", 640, 480, jpeg);
  session_->Flush();
  session_->PumpUploads();

  const auto requests = RequestsTo("/v0/camera");
  ASSERT_EQ(requests.size(), 1u);
  PackedBody body(requests[0].body);
  const auto payload = body.camera();

  EXPECT_EQ(payload.getUserId().cStr(), std::string("robot01"));
  EXPECT_EQ(payload.getSessionId().cStr(), session_->session_id());
  EXPECT_EQ(payload.getSceneId().cStr(), std::string("SCENEGUID"));
  EXPECT_EQ(payload.getSceneVersion(), 7u);
  EXPECT_EQ(payload.getFormatVersion().cStr(), std::string("1.0"));
  EXPECT_EQ(payload.getSdkVersion().cStr(), std::string(kSdkVersion));
  EXPECT_EQ(payload.getPart(), 1u);
  // Milliseconds, and the same instant the session id names.
  EXPECT_EQ(payload.getSessionStartMs(),
            static_cast<std::uint64_t>(session_->session_timestamp() * 1000.0));

  ASSERT_EQ(payload.getFrames().size(), 1u);
  const auto frame = payload.getFrames()[0];
  EXPECT_EQ(frame.getCameraId().cStr(), std::string("front"));
  EXPECT_EQ(frame.getTime(), 1788383013123ull);
  EXPECT_EQ(frame.getSeq(), 0u);
  EXPECT_EQ(frame.getFrameId().cStr(), std::string("oakd_rgb_camera_optical_frame"));
  EXPECT_EQ(frame.getFormat().cStr(), std::string("jpeg"));
  EXPECT_EQ(frame.getWidth(), 640);
  EXPECT_EQ(frame.getHeight(), 480);
  ASSERT_EQ(frame.getData().size(), 256u);
  // Every byte value 0-255 survives, which is the property a JSON body could not
  // have carried at all.
  for (int i = 0; i < 256; ++i) EXPECT_EQ(frame.getData()[i], static_cast<kj::byte>(i));

  const auto pose = frame.getPose();
  EXPECT_FLOAT_EQ(pose.getPx(), 1.5f);
  EXPECT_FLOAT_EQ(pose.getPy(), -2.25f);
  EXPECT_FLOAT_EQ(pose.getPz(), 3.125f);
  EXPECT_FLOAT_EQ(pose.getQx(), 0.1f);
  EXPECT_FLOAT_EQ(pose.getQy(), 0.2f);
  EXPECT_FLOAT_EQ(pose.getQz(), 0.3f);
  EXPECT_FLOAT_EQ(pose.getQw(), 0.9273618f);
}

// The field of view a caller passes is the one on the wire, per frame: two frames of
// one camera in one part can differ, which is what a zoom lens needs. A frame
// recorded without one reads back as 0 by 0, the schema's "not recorded".
TEST_F(BinaryStreamTest, EachFrameCarriesItsOwnFieldOfView) {
  Build();
  session_->RecordCameraFrame("front", 1ull, 0, "f", SomePose(), "jpeg", 2, 2, "ab",
                              CameraFov{61.0f, 45.0f});
  session_->RecordCameraFrame("front", 2ull, 1, "f", SomePose(), "jpeg", 2, 2, "ab",
                              CameraFov{30.5f, 17.25f});
  session_->RecordCameraFrame("front", 3ull, 2, "f", SomePose(), "jpeg", 2, 2, "ab");
  session_->RecordCameraFrame("rear", 4ull, 0, "r", SomePose(), "jpeg", 2, 2, "ab",
                              CameraFov{0.01f, 179.99f});
  session_->Flush();
  session_->PumpUploads();

  const auto requests = RequestsTo("/v0/camera");
  ASSERT_EQ(requests.size(), 1u);
  PackedBody body(requests[0].body);
  const auto frames = body.camera().getFrames();
  ASSERT_EQ(frames.size(), 4u);
  EXPECT_EQ(frames[0].getHfovDegrees(), 61.0f);
  EXPECT_EQ(frames[0].getVfovDegrees(), 45.0f);
  EXPECT_EQ(frames[1].getHfovDegrees(), 30.5f);
  EXPECT_EQ(frames[1].getVfovDegrees(), 17.25f);
  EXPECT_EQ(frames[2].getHfovDegrees(), 0.0f);
  EXPECT_EQ(frames[2].getVfovDegrees(), 0.0f);
  // Just inside both ends of the open interval.
  EXPECT_EQ(frames[3].getHfovDegrees(), 0.01f);
  EXPECT_EQ(frames[3].getVfovDegrees(), 179.99f);
  EXPECT_EQ(logger_->Warnings("field of view"), 0) << "a usable or absent one is no fault";
}

// A field of view that is not two usable angles reaches the wire as 0 by 0 and never as
// NaN, half of one, or an angle a reader would draw. The frame itself is kept, and each
// camera says so once per session rather than once per frame.
TEST_F(BinaryStreamTest, AnUnusableFieldOfViewIsSentAsNoneAndTheFrameIsKept) {
  Build();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const std::vector<CameraFov> unusable = {
      {61.0f, 0.0f}, {0.0f, 45.0f}, {nan, 45.0f}, {61.0f, nan}, {inf, 45.0f},
      {61.0f, -inf}, {-61.0f, 45.0f}, {61.0f, -45.0f}, {180.0f, 45.0f}, {61.0f, 180.0f},
      {270.0f, 45.0f}};
  std::uint32_t seq = 0;
  for (const CameraFov& fov : unusable) {
    session_->RecordCameraFrame("front", 1ull + seq, seq, "f", SomePose(), "jpeg", 2, 2, "ab",
                                fov);
    ++seq;
  }
  session_->Flush();
  session_->PumpUploads();

  const auto requests = RequestsTo("/v0/camera");
  ASSERT_EQ(requests.size(), 1u);
  PackedBody body(requests[0].body);
  const auto frames = body.camera().getFrames();
  ASSERT_EQ(frames.size(), unusable.size()) << "a frame was lost over its field of view";
  for (unsigned int i = 0; i < frames.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_EQ(frames[i].getHfovDegrees(), 0.0f);
    EXPECT_EQ(frames[i].getVfovDegrees(), 0.0f);
  }
  EXPECT_EQ(logger_->Warnings("camera 'front'"), 1);

  // Per camera, and again in the next session.
  session_->RecordCameraFrame("rear", 1ull, 0, "r", SomePose(), "jpeg", 2, 2, "ab",
                              CameraFov{nan, nan});
  EXPECT_EQ(logger_->Warnings("camera 'rear'"), 1);
  session_->End();
  ASSERT_TRUE(session_->Begin());
  session_->RecordCameraFrame("front", 1ull, 0, "f", SomePose(), "jpeg", 2, 2, "ab",
                              CameraFov{61.0f, 0.0f});
  EXPECT_EQ(logger_->Warnings("camera 'front'"), 2);
}

TEST(CameraFov, KnownIsBothAnglesStrictlyBetweenZeroAnd180) {
  EXPECT_TRUE(IsKnownCameraFov(CameraFov{61.0f, 45.0f}));
  EXPECT_TRUE(IsKnownCameraFov(CameraFov{179.5f, 0.5f}));
  EXPECT_FALSE(IsKnownCameraFov(CameraFov{}));
  EXPECT_FALSE(IsKnownCameraFov(CameraFov{61.0f, 0.0f}));
  EXPECT_FALSE(IsKnownCameraFov(CameraFov{0.0f, 45.0f}));
  EXPECT_FALSE(IsKnownCameraFov(CameraFov{180.0f, 45.0f}));
  EXPECT_FALSE(IsKnownCameraFov(CameraFov{61.0f, 180.0f}));
  EXPECT_FALSE(IsKnownCameraFov(CameraFov{-1.0f, 45.0f}));
  EXPECT_FALSE(IsKnownCameraFov(CameraFov{std::numeric_limits<float>::quiet_NaN(), 45.0f}));
  EXPECT_FALSE(IsKnownCameraFov(CameraFov{61.0f, std::numeric_limits<float>::infinity()}));
}

// Against intrinsics whose angles are known independently: a centred 640 x 480 camera at
// f = 500, a 90 degree one at f = W / 2, focal lengths that differ by axis, and a principal point
// off centre, where the field of view is not 2 atan(W / 2f).
TEST(CameraFov, FromPinholeIntrinsics) {
  const auto near = [](const CameraFov& fov, double h, double v) {
    EXPECT_NEAR(fov.hfov_deg, h, 1e-4);
    EXPECT_NEAR(fov.vfov_deg, v, 1e-4);
  };
  near(CameraFovFromPinhole(500, 500, 320, 240, 640, 480), 65.238486, 51.282012);
  near(CameraFovFromPinhole(320, 320, 320, 240, 640, 480), 90.0, 73.739795);
  near(CameraFovFromPinhole(500, 400, 320, 240, 640, 480), 65.238486, 61.927513);
  near(CameraFovFromPinhole(500, 500, 400, 240, 640, 480), 64.300814, 51.282012);
  near(CameraFovFromPinhole(500, 400, 320, 300, 640, 480),
       65.238486, 61.097643);  // atan(300/400) + atan(180/400)
}

// An uncalibrated driver publishes K as zeros. That, and anything else that is not a camera,
// is no field of view rather than a number.
TEST(CameraFov, FromPinholeIsNoneUnlessTheIntrinsicsDescribeACamera) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const std::vector<std::vector<double>> unusable = {
      {0, 0, 0, 0, 640, 480},       {0, 500, 320, 240, 640, 480},  {500, 0, 320, 240, 640, 480},
      {-500, 500, 320, 240, 640, 480}, {500, -500, 320, 240, 640, 480},
      {500, 500, 320, 240, 0, 480},  {500, 500, 320, 240, 640, 0},
      {500, 500, 320, 240, -640, 480}, {nan, 500, 320, 240, 640, 480},
      {500, 500, nan, 240, 640, 480}, {500, 500, 320, 240, inf, 480},
      {500, 500, 320, inf, 640, 480},
      // A focal length so short the angle is 180 degrees as a Float32: not a pinhole camera's.
      {1e-9, 500, 320, 240, 640, 480},
      // Every sign flipped: the arithmetic alone gives a plausible 65 x 51, which is why the
      // focal lengths and the size are checked before it rather than its result after.
      {-500, -500, -320, -240, -640, -480}};
  for (const auto& k : unusable) {
    const CameraFov fov = CameraFovFromPinhole(k[0], k[1], k[2], k[3], k[4], k[5]);
    EXPECT_EQ(fov.hfov_deg, 0.0f) << k[0] << " " << k[1] << " " << k[2] << " " << k[4];
    EXPECT_EQ(fov.vfov_deg, 0.0f) << k[0] << " " << k[1] << " " << k[3] << " " << k[5];
  }
}

TEST_F(BinaryStreamTest, AllSevenPoseComponentsAreWrittenIncludingAnIdentityQw) {
  Build();
  Pose identity;
  identity.position = Vec3{0.0, 0.0, 0.0};
  identity.rotation = Quat{0.0, 0.0, 0.0, 1.0};
  session_->RecordCameraFrame("front", 1ull, 0, "f", identity, "jpeg", 2, 2, "ab");
  session_->Flush();
  session_->PumpUploads();

  const auto requests = RequestsTo("/v0/camera");
  ASSERT_EQ(requests.size(), 1u);
  PackedBody body(requests[0].body);
  // `qw` DEFAULTS to 1 in the schema, so this passes whether or not the writer
  // set it -- which is exactly why nothing downstream may rely on the default.
  // What this pins is that a pose we sent reads back as the rotation we measured,
  // and the zero-quaternion case the default exists to prevent never occurs.
  const auto pose = body.camera().getFrames()[0].getPose();
  EXPECT_FLOAT_EQ(pose.getQw(), 1.0f);
  EXPECT_FALSE(pose.getQx() == 0.0f && pose.getQy() == 0.0f && pose.getQz() == 0.0f &&
               pose.getQw() == 0.0f);
}

TEST_F(BinaryStreamTest, LidarCarriesExactlyOneUnionArmPerScan) {
  Build();
  session_->RecordLaserScan("scan", 1000ull, 0, "base_scan", SomePose(), SomeScan());
  const EncodedPointCloud cloud = SomeCloud();
  ASSERT_TRUE(cloud.ok) << cloud.reason;
  session_->RecordPointCloud("depth", 1001ull, 0, "oakd_rgb_camera_optical_frame", SomePose(),
                             cloud);
  session_->Flush();
  session_->PumpUploads();

  const auto requests = RequestsTo("/v0/lidar");
  ASSERT_EQ(requests.size(), 1u);
  PackedBody body(requests[0].body);
  const auto payload = body.lidar();
  ASSERT_EQ(payload.getScans().size(), 2u);

  const auto laser_scan = payload.getScans()[0];
  EXPECT_EQ(laser_scan.getSensorId().cStr(), std::string("scan"));
  ASSERT_EQ(laser_scan.which(), cvr::capnp::Scan::LASERSCAN);
  const auto laser = laser_scan.getLaserscan();
  EXPECT_FLOAT_EQ(laser.getAngleMin(), -3.14f);
  EXPECT_FLOAT_EQ(laser.getRangeMax(), 12.0f);
  ASSERT_EQ(laser.getRanges().size(), 3u);
  EXPECT_FLOAT_EQ(laser.getRanges()[0], 1.0f);
  // An infinity is the sensor saying "nothing there" and is preserved, not
  // rewritten into a distance.
  EXPECT_TRUE(std::isinf(laser.getRanges()[2]));
  // Intensities were not asked for, so the list is present and empty rather than
  // filled with zeros.
  EXPECT_EQ(laser.getIntensities().size(), 0u);

  const auto cloud_scan = payload.getScans()[1];
  EXPECT_EQ(cloud_scan.getSensorId().cStr(), std::string("depth"));
  ASSERT_EQ(cloud_scan.which(), cvr::capnp::Scan::POINTCLOUD);
  const auto encoded = cloud_scan.getPointcloud();
  EXPECT_EQ(encoded.getPointCount(), 2u);
  EXPECT_EQ(encoded.getPosition().cStr(), std::string("int16_mm"));
  EXPECT_EQ(encoded.getColor().cStr(), std::string("none"));
  EXPECT_EQ(encoded.getIntensity().cStr(), std::string("none"));
  EXPECT_EQ(encoded.getVoxelLeafMm(), 0);
  EXPECT_EQ(encoded.getZstdLevel(), 1);
  EXPECT_EQ(encoded.getData().size(), cloud.data.size());
}

TEST_F(BinaryStreamTest, PartCountsFromOneAndIsIndependentPerStream) {
  cfg_.camera_flush_bytes = 1024;
  cfg_.lidar_flush_bytes = 1024;
  Build();
  for (int i = 0; i < 3; ++i) {
    session_->RecordCameraFrame("front", 100ull + i, static_cast<std::uint32_t>(i), "f",
                                SomePose(), "jpeg", 2, 2, std::string(2048, 'x'));
    session_->RecordLaserScan("scan", 200ull + i, static_cast<std::uint32_t>(i), "s", SomePose(),
                              SomeScan());
    session_->Flush();
  }
  session_->PumpUploads();

  const auto cameras = RequestsTo("/v0/camera");
  const auto lidars = RequestsTo("/v0/lidar");
  ASSERT_EQ(cameras.size(), 3u);
  ASSERT_EQ(lidars.size(), 3u);
  for (std::size_t i = 0; i < 3; ++i) {
    PackedBody camera(cameras[i].body);
    EXPECT_EQ(camera.camera().getPart(), static_cast<std::uint32_t>(i + 1));
    PackedBody lidar(lidars[i].body);
    EXPECT_EQ(lidar.lidar().getPart(), static_cast<std::uint32_t>(i + 1));
  }
}

TEST_F(BinaryStreamTest, TheByteBoundCutsAPayloadWithoutAnyoneCallingFlush) {
  cfg_.camera_flush_bytes = 4096;
  Build();
  // Under the bound: the worker must leave it buffered.
  session_->RecordCameraFrame("front", 1ull, 0, "f", SomePose(), "jpeg", 2, 2,
                              std::string(1000, 'a'));
  EXPECT_FALSE(WaitForStream("/v0/camera", 1, 0.3));

  // Over it: the next worker tick cuts, with no Flush() anywhere in this test.
  session_->RecordCameraFrame("front", 2ull, 1, "f", SomePose(), "jpeg", 2, 2,
                              std::string(4000, 'b'));
  ASSERT_TRUE(WaitForStream("/v0/camera", 1));
  const auto requests = RequestsTo("/v0/camera");
  ASSERT_EQ(requests.size(), 1u);
  PackedBody body(requests[0].body);
  // Both frames, because the bound is on the part and not on the frame.
  EXPECT_EQ(body.camera().getFrames().size(), 2u);
}

TEST_F(BinaryStreamTest, TheTimeBoundCutsAPayloadThatNeverReachesTheByteBound) {
  cfg_.lidar_flush_s = 0.1;
  Build();
  session_->RecordLaserScan("scan", 1ull, 0, "s", SomePose(), SomeScan());
  ASSERT_TRUE(WaitForStream("/v0/lidar", 1));
  EXPECT_EQ(RequestsTo("/v0/lidar").size(), 1u);
}

TEST_F(BinaryStreamTest, EachStreamGetsItsOwnUrlAndContentType) {
  Build();
  session_->RecordCameraFrame("front", 1ull, 0, "f", SomePose(), "jpeg", 2, 2, "ab");
  session_->RecordLaserScan("scan", 1ull, 0, "s", SomePose(), SomeScan());
  session_->RecordGaze(1.0, Vec3{}, Quat{});
  session_->Flush();
  session_->PumpUploads();

  auto content_type = [](const FakeTransport::Request& request) {
    for (const auto& header : request.headers) {
      if (header.rfind("Content-Type: ", 0) == 0) return header.substr(14);
    }
    return std::string("<none>");
  };

  bool saw_camera = false, saw_lidar = false, saw_gaze = false;
  for (const auto& request : transport_->requests()) {
    if (request.url == "http://data.example.invalid/v0/camera" ||
        request.url == "https://data.example.invalid/v0/camera") {
      saw_camera = true;
      EXPECT_EQ(content_type(request), "application/octet-stream");
    } else if (request.url.find("/v0/lidar") != std::string::npos) {
      saw_lidar = true;
      // Session-keyed: no scene id in the path and no version query.
      EXPECT_EQ(request.url.find("SCENEGUID"), std::string::npos);
      EXPECT_EQ(request.url.find("?version="), std::string::npos);
      EXPECT_EQ(content_type(request), "application/octet-stream");
    } else if (request.url.find("/v0/gaze/") != std::string::npos) {
      saw_gaze = true;
      // Unchanged: the JSON streams keep the scene-keyed url and the JSON type.
      EXPECT_NE(request.url.find("SCENEGUID?version=7"), std::string::npos);
      EXPECT_EQ(content_type(request), "application/json");
    }
  }
  EXPECT_TRUE(saw_camera);
  EXPECT_TRUE(saw_lidar);
  EXPECT_TRUE(saw_gaze);
}

TEST_F(BinaryStreamTest, EndCutsWhateverIsStillBuffered) {
  Build();
  session_->RecordCameraFrame("front", 1ull, 0, "f", SomePose(), "jpeg", 2, 2, "ab");
  session_->RecordLaserScan("scan", 1ull, 0, "s", SomePose(), SomeScan());
  session_->End("test");
  session_->PumpUploads();  // End spools what is buffered and never waits for an upload
  EXPECT_EQ(RequestsTo("/v0/camera").size(), 1u);
  EXPECT_EQ(RequestsTo("/v0/lidar").size(), 1u);
}

TEST_F(BinaryStreamTest, ARefusedCloudIsNeverSentAsAnEmptyScan) {
  Build();
  EncodedPointCloud refused;
  refused.ok = false;
  refused.reason = "oversize";
  session_->RecordPointCloud("depth", 1ull, 0, "f", SomePose(), refused);
  session_->Flush();
  session_->PumpUploads();
  EXPECT_TRUE(RequestsTo("/v0/lidar").empty());
  EXPECT_GE(session_->stats().records_dropped, 1u);
}

TEST_F(BinaryStreamTest, BinaryRecordsAreCountedApartFromTheJsonOnes) {
  Build();
  session_->RecordCameraFrame("front", 1ull, 0, "f", SomePose(), "jpeg", 2, 2,
                              std::string(500, 'x'));
  session_->RecordLaserScan("scan", 1ull, 0, "s", SomePose(), SomeScan());
  const Stats stats = session_->stats();
  EXPECT_EQ(stats.camera_frames, 1u);
  EXPECT_EQ(stats.camera_bytes, 500u);
  EXPECT_EQ(stats.lidar_scans, 1u);
  // Neither reached `records_recorded`, which counts the JSON streams' records.
  EXPECT_EQ(stats.records_recorded, 0u);
}

TEST(BinaryStreamConfig, ASceneVersionThatIsNotANumberIsRefused) {
  Config cfg;
  cfg.application_key = "K";
  cfg.scene_id = "S";
  cfg.device_id = "D";
  cfg.spool_dir = "/tmp";
  cfg.scene_version = "2";
  EXPECT_EQ(cfg.Validate(), "");
  // The binary payloads carry this as a UInt32 the gateway resolves to a version
  // id, so a word here is not a cosmetic problem.
  cfg.scene_version = "v2";
  EXPECT_NE(cfg.Validate().find("scene_version"), std::string::npos);
  cfg.scene_version = " 2";
  EXPECT_NE(cfg.Validate().find("scene_version"), std::string::npos);
  cfg.scene_version = "99999999999";
  EXPECT_NE(cfg.Validate().find("scene_version"), std::string::npos);
}

TEST(BinaryStreamConfig, ASceneVersionOfZeroIsRefused) {
  Config cfg;
  cfg.application_key = "K";
  cfg.scene_id = "S";
  cfg.device_id = "D";
  cfg.spool_dir = "/tmp";
  cfg.scene_version = "1";
  ASSERT_EQ(cfg.Validate(), "");
  // Digits, in range, and still not a version: scene versions are numbered from
  // 1, so there is nothing for the gateway to resolve a 0 to and it answers 400
  // to every part of the run.
  cfg.scene_version = "0";
  EXPECT_NE(cfg.Validate().find("scene_version"), std::string::npos);
  cfg.scene_version = "000";
  EXPECT_NE(cfg.Validate().find("scene_version"), std::string::npos);
}

TEST(BinaryStreamConfig, AFlushBoundAboveTheGatewayBodyCapIsRefused) {
  Config cfg;
  cfg.application_key = "K";
  cfg.scene_id = "S";
  cfg.device_id = "D";
  cfg.spool_dir = "/tmp";
  cfg.scene_version = "2";
  cfg.camera_flush_bytes = 32ull * 1024 * 1024;
  cfg.lidar_flush_bytes = 32ull * 1024 * 1024;
  EXPECT_EQ(cfg.Validate(), "") << "32 MiB is the bound itself, not past it";

  // Past it, a part is a 413. That is not retryable, so the part and every frame
  // in it is discarded -- the opposite of what a bigger buffer was meant to buy.
  cfg.camera_flush_bytes = 32ull * 1024 * 1024 + 1;
  EXPECT_NE(cfg.Validate(), "") << "a camera bound above the cap was accepted";
  EXPECT_NE(cfg.Validate().find("32 MiB"), std::string::npos);

  cfg.camera_flush_bytes = 4ull * 1024 * 1024;
  cfg.lidar_flush_bytes = 64ull * 1024 * 1024;
  EXPECT_NE(cfg.Validate(), "") << "a lidar bound above the cap was accepted";
}
