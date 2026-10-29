#include <gtest/gtest.h>
#include <unistd.h>
#include <zstd.h>

#include <capnp/schema.h>
#include <capnp/serialize-packed.h>
#include <kj/io.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "c3d/media.hpp"
#include "c3d/scan_block.hpp"
#include "c3d/sensor_blocks.hpp"
#include "c3d/session.hpp"
#include "c3d/version.hpp"
#include "camera_payload.capnp.h"
#include "capnp_payloads.hpp"
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

std::string BytesOf(capnp::Data::Reader data) {
  return std::string(reinterpret_cast<const char*>(data.begin()), data.size());
}

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

// A pose that differs per index, so an entry written from its neighbour's record shows.
Pose PoseNumber(int i) {
  Pose pose;
  pose.position = Vec3{0.5 + i, -1.0 * i, 2.0 * i};
  pose.rotation = Quat{0.1 * i, 0.2, 0.3, 0.9};
  return pose;
}

void ExpectPose(cvr::capnp::Pose::Reader got, const Pose& want) {
  EXPECT_FLOAT_EQ(got.getPx(), static_cast<float>(want.position.x));
  EXPECT_FLOAT_EQ(got.getPy(), static_cast<float>(want.position.y));
  EXPECT_FLOAT_EQ(got.getPz(), static_cast<float>(want.position.z));
  EXPECT_FLOAT_EQ(got.getQx(), static_cast<float>(want.rotation.x));
  EXPECT_FLOAT_EQ(got.getQy(), static_cast<float>(want.rotation.y));
  EXPECT_FLOAT_EQ(got.getQz(), static_cast<float>(want.rotation.z));
  EXPECT_FLOAT_EQ(got.getQw(), static_cast<float>(want.rotation.w));
}

SensorEntry EntryAt(std::uint64_t time_ms, std::uint32_t seq, const Pose& pose = SomePose()) {
  SensorEntry entry;
  entry.time_ms = time_ms;
  entry.seq = seq;
  entry.pose = pose;
  return entry;
}

VideoChunkContext SomeContext() {
  VideoChunkContext context;
  context.frame_id = "oakd_rgb_camera_optical_frame";
  context.width = 640;
  context.height = 360;
  context.nominal_fps = 30.0f;
  context.codec_params = std::string("\x01\x64\x00\x1e\xff\xe1", 6);
  context.encoder_settings = "x264 ultrafast crf=28 threads=1 gop=3s";
  return context;
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

// The session never looks inside a cloud's block, so a block of a known size stands in for one.
EncodedPointCloud CloudOfBytes(std::size_t bytes, char fill) {
  EncodedPointCloud cloud;
  cloud.ok = true;
  cloud.point_count = 10;
  cloud.voxel_leaf_mm = 50;
  cloud.zstd_level = 3;
  cloud.encoding = PointCloudEncoding::kPlanarDeltaShuffleZstd;
  cloud.data.assign(bytes, fill);
  return cloud;
}

// A scan block's range codes, undone by hand from the wire: zstd, then the inverse of the
// per-scan delta and the byte shuffle.
std::vector<std::uint16_t> RangeCodes(cvr::capnp::ScanBlock::Reader block) {
  const auto ranges = block.getRanges();
  const std::size_t raw_bytes =
      static_cast<std::size_t>(block.getEntries().size()) * block.getRays() * 2;
  std::string raw(raw_bytes, '\0');
  const std::size_t n = ZSTD_decompress(raw.data(), raw.size(), ranges.begin(), ranges.size());
  EXPECT_FALSE(ZSTD_isError(n));
  EXPECT_EQ(n, raw_bytes);
  return scanblock::InverseDeltaShuffle(raw, block.getRays());
}

scanblock::RangeUnit UnitOf(cvr::capnp::ScanBlock::Reader block) {
  return block.getRangeUnit() == cvr::capnp::RangeUnit::CENTIMETRE
             ? scanblock::RangeUnit::kCentimetre
             : scanblock::RangeUnit::kMillimetre;
}

bool HasField(capnp::StructSchema schema, const std::string& name) {
  for (const auto field : schema.getFields()) {
    if (name == field.getProto().getName().cStr()) return true;
  }
  return false;
}

BinaryEnvelope SomeEnvelope() {
  BinaryEnvelope envelope;
  envelope.user_id = "robot01";
  envelope.session_id = "1788383013_robot01";
  envelope.scene_id = "SCENEGUID";
  envelope.scene_version = 7;
  envelope.format_version = "2.0";
  envelope.session_start_ms = 1788383013000ull;
  envelope.sdk_version = "9.8.7";
  return envelope;
}

template <typename Reader>
void ExpectEnvelope(Reader payload, const BinaryEnvelope& envelope) {
  EXPECT_EQ(payload.getUserId().cStr(), envelope.user_id);
  EXPECT_EQ(payload.getSessionId().cStr(), envelope.session_id);
  EXPECT_EQ(payload.getSceneId().cStr(), envelope.scene_id);
  EXPECT_EQ(payload.getSceneVersion(), envelope.scene_version);
  EXPECT_EQ(payload.getFormatVersion().cStr(), envelope.format_version);
  EXPECT_EQ(payload.getSessionStartMs(), envelope.session_start_ms);
  EXPECT_EQ(payload.getSdkVersion().cStr(), envelope.sdk_version);
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
    cfg_.auto_send_interval_s = 3600.0;  // only the binary closes may cut a payload
    cfg_.worker_tick_s = 0.01;
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
  // Waits for the WORKER to post to ONE stream, so a test can tell "a close fired"
  // from "Flush was called". Per stream and not by total count, because Begin()
  // flushes the session-start event before any of these tests record anything --
  // so a total-count wait is already satisfied when it is first asked.
  bool WaitForStream(const std::string& suffix, std::size_t at_least, double seconds = 3.0) {
    return WaitForStreamUntil(suffix, at_least,
                              std::chrono::steady_clock::now() +
                                  std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                      std::chrono::duration<double>(seconds)));
  }
  bool WaitForStreamUntil(const std::string& suffix, std::size_t at_least,
                          std::chrono::steady_clock::time_point deadline) {
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
  std::size_t RequestsContaining(const std::string& fragment) const {
    std::size_t n = 0;
    for (const auto& r : transport_->requests()) n += r.url.find(fragment) != std::string::npos;
    return n;
  }
  // Every body sent for one sensor, by the part's seq. Keyed rather than listed because the
  // uploader's order is the spool's, which is no promise about seq.
  std::map<std::uint32_t, std::string> LidarBodies(const std::string& sensor_id) const {
    std::map<std::uint32_t, std::string> out;
    for (const auto& r : RequestsTo("/v0/lidar")) {
      PackedBody body(r.body);
      if (sensor_id == body.lidar().getSensorId().cStr()) out[body.lidar().getSeq()] = r.body;
    }
    return out;
  }
  std::map<std::uint32_t, std::string> CameraBodies(const std::string& camera_id) const {
    std::map<std::uint32_t, std::string> out;
    for (const auto& r : RequestsTo("/v0/camera")) {
      PackedBody body(r.body);
      if (camera_id == body.camera().getCameraId().cStr()) out[body.camera().getSeq()] = r.body;
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

// ---------------------------------------------------------------------------
// The writers, called directly
// ---------------------------------------------------------------------------

TEST(CapnpWriters, ACameraChunkRoundTripsEveryField) {
  CameraChunkRecord chunk;
  chunk.camera_id = "front";
  chunk.chunk_seq = 4;
  chunk.start_ms = 1788383013100ull;
  chunk.end_ms = 1788383013166ull;
  chunk.context.frame_id = "oakd_rgb_camera_optical_frame";
  chunk.context.width = 640;
  chunk.context.height = 360;
  chunk.context.nominal_fps = 29.97f;
  chunk.context.hfov_deg = 69.5f;
  chunk.context.vfov_deg = 42.25f;
  chunk.context.codec = "h264";
  chunk.context.codec_params = std::string("\x01\x64\x00\x1e\xff\xe1\x00\x00", 8);
  chunk.context.encoder_settings = "x264 ultrafast crf=28 threads=1 gop=3s";
  const std::vector<std::string> units = {EveryByteValue(), std::string(7, '\0'), "xyz"};
  for (std::size_t i = 0; i < units.size(); ++i) {
    FrameMeta meta;
    meta.time_ms = chunk.start_ms + 33 * i;
    meta.seq = 120 + static_cast<std::uint32_t>(i);
    meta.pose = PoseNumber(static_cast<int>(i));
    meta.byte_offset = static_cast<std::uint32_t>(chunk.data.size());
    meta.byte_length = static_cast<std::uint32_t>(units[i].size());
    meta.keyframe = i == 0;
    chunk.frames.push_back(meta);
    chunk.data += units[i];
  }

  const BinaryEnvelope envelope = SomeEnvelope();
  const std::string serialized = SerializeCameraChunk(envelope, chunk);
  PackedBody body(serialized);
  const auto payload = body.camera();
  ExpectEnvelope(payload, envelope);
  EXPECT_EQ(payload.getCameraId().cStr(), std::string("front"));
  EXPECT_EQ(payload.getSeq(), 4u);
  EXPECT_EQ(payload.getStartMs(), 1788383013100ull);
  EXPECT_EQ(payload.getEndMs(), 1788383013166ull);
  EXPECT_EQ(payload.getCodec().cStr(), std::string("h264"));
  EXPECT_EQ(BytesOf(payload.getCodecParams()), chunk.context.codec_params);
  EXPECT_EQ(payload.getWidth(), 640);
  EXPECT_EQ(payload.getHeight(), 360);
  EXPECT_FLOAT_EQ(payload.getNominalFps(), 29.97f);
  EXPECT_EQ(payload.getFrameId().cStr(), std::string("oakd_rgb_camera_optical_frame"));
  EXPECT_EQ(payload.getHfovDegrees(), 69.5f);
  EXPECT_EQ(payload.getVfovDegrees(), 42.25f);
  EXPECT_EQ(payload.getEncoderSettings().cStr(), chunk.context.encoder_settings);

  const auto frames = payload.getFrames();
  ASSERT_EQ(frames.size(), 3u);
  const std::string data = BytesOf(payload.getData());
  EXPECT_EQ(data, chunk.data);
  for (unsigned int i = 0; i < frames.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_EQ(frames[i].getTime(), chunk.start_ms + 33 * i);
    EXPECT_EQ(frames[i].getSeq(), 120 + i);
    ExpectPose(frames[i].getPose(), PoseNumber(static_cast<int>(i)));
    EXPECT_EQ(frames[i].getKeyframe(), i == 0);
    // Each access unit is found again from its own offset and length, and from nothing else.
    EXPECT_EQ(data.substr(frames[i].getByteOffset(), frames[i].getByteLength()), units[i]);
  }
}

TEST(CapnpWriters, AScanBlockCarriesTheGeometryOnceAndTheEntriesPerScan) {
  LaserScanData first;
  first.angle_min = -2.0f;
  first.angle_max = 2.0f;
  first.angle_increment = 0.5f;
  first.range_min = 0.2f;
  first.range_max = 80.0f;  // beyond 65 m, so the unit is a centimetre
  first.time_increment = 0.0002f;
  first.scan_time = 0.1f;
  first.ranges = {0.5f, 1.0f, 1.5f, 2.0f, 70.0f, 3.0f, 3.5f, 4.0f, 4.5f};
  first.intensities = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f};
  // A second scan whose timing is its own. Its geometry differs too, which the assembler never
  // lets into one block, so a header written from anything but the first scan shows.
  LaserScanData second = first;
  second.angle_min = -1.0f;
  second.angle_max = 1.0f;
  second.angle_increment = 0.25f;
  second.range_min = 0.3f;
  second.range_max = 90.0f;
  second.time_increment = 0.00025f;
  second.scan_time = 0.125f;
  second.ranges = {9.0f, 8.0f, 7.0f, 6.0f, 5.0f, 4.0f, 3.0f, 2.0f, 1.0f};

  ScanBlockRecord block;
  block.sensor_id = "front_lidar";
  block.frame_id = "base_scan";
  block.block_seq = 2;
  block.start_ms = 1000;
  block.end_ms = 1100;
  block.intensities = true;
  block.entries = {EntryAt(1000, 10, PoseNumber(1)), EntryAt(1100, 11, PoseNumber(2))};
  block.scans = {first, second};
  const scanblock::EncodedScanBlock encoded = scanblock::Encode(block.scans, true, 1);
  ASSERT_TRUE(encoded.ok) << encoded.reason;
  ASSERT_EQ(encoded.unit, scanblock::RangeUnit::kCentimetre);
  ASSERT_FALSE(encoded.intensities.empty());

  const BinaryEnvelope envelope = SomeEnvelope();
  const std::string serialized = SerializeScanBlock(envelope, block, encoded);
  PackedBody body(serialized);
  const auto payload = body.lidar();
  ExpectEnvelope(payload, envelope);
  EXPECT_EQ(payload.getSensorId().cStr(), std::string("front_lidar"));
  EXPECT_EQ(payload.getSeq(), 2u);
  EXPECT_EQ(payload.getStartMs(), 1000u);
  EXPECT_EQ(payload.getEndMs(), 1100u);
  EXPECT_EQ(payload.getFrameId().cStr(), std::string("base_scan"));
  ASSERT_EQ(payload.which(), cvr::capnp::LidarPayload::SCANS);

  const auto scans = payload.getScans();
  EXPECT_FLOAT_EQ(scans.getAngleMin(), -2.0f);
  EXPECT_FLOAT_EQ(scans.getAngleMax(), 2.0f);
  EXPECT_FLOAT_EQ(scans.getAngleIncrement(), 0.5f);
  EXPECT_FLOAT_EQ(scans.getRangeMin(), 0.2f);
  EXPECT_FLOAT_EQ(scans.getRangeMax(), 80.0f);
  EXPECT_EQ(scans.getRays(), 9);
  // CENTIMETRE is not the schema's default, so this reads back only if it was written.
  EXPECT_EQ(scans.getRangeUnit(), cvr::capnp::RangeUnit::CENTIMETRE);
  EXPECT_EQ(scans.getEncoding(), cvr::capnp::ScanEncoding::DELTA_SHUFFLE_ZSTD);

  const auto entries = scans.getEntries();
  ASSERT_EQ(entries.size(), 2u);
  EXPECT_EQ(entries[0].getTime(), 1000u);
  EXPECT_EQ(entries[0].getSeq(), 10u);
  ExpectPose(entries[0].getPose(), PoseNumber(1));
  EXPECT_FLOAT_EQ(entries[0].getTimeIncrement(), 0.0002f);
  EXPECT_FLOAT_EQ(entries[0].getScanTime(), 0.1f);
  EXPECT_EQ(entries[1].getTime(), 1100u);
  EXPECT_EQ(entries[1].getSeq(), 11u);
  ExpectPose(entries[1].getPose(), PoseNumber(2));
  EXPECT_FLOAT_EQ(entries[1].getTimeIncrement(), 0.00025f);
  EXPECT_FLOAT_EQ(entries[1].getScanTime(), 0.125f);

  EXPECT_EQ(BytesOf(scans.getRanges()), encoded.ranges);
  EXPECT_EQ(BytesOf(scans.getIntensities()), encoded.intensities);
}

TEST(CapnpWriters, ACloudObjectOffsetsTileItsData) {
  CloudObjectRecord object;
  object.sensor_id = "depth";
  object.frame_id = "oakd_rgb_camera_optical_frame";
  object.object_seq = 3;
  object.start_ms = 2000;
  object.end_ms = 2200;
  object.position = PointPosition::kInt16Mm;
  object.color = PointColor::kRgb565;
  object.intensity = PointIntensity::kUint8;
  object.voxel_leaf_mm = 50;
  object.encoding = PointCloudEncoding::kPlanarDeltaShuffleZstd;
  object.zstd_level = 3;
  object.entries = {EntryAt(2000, 30, PoseNumber(1)), EntryAt(2100, 31, PoseNumber(2)),
                    EntryAt(2200, 32, PoseNumber(3))};
  object.point_counts = {7, 300, 2};
  object.blocks = {std::string(5, 'a'), EveryByteValue(), std::string(17, '\0')};
  for (const std::string& b : object.blocks) object.bytes += b.size();

  const BinaryEnvelope envelope = SomeEnvelope();
  const std::string serialized = SerializeCloudObject(envelope, object);
  PackedBody body(serialized);
  const auto payload = body.lidar();
  ExpectEnvelope(payload, envelope);
  EXPECT_EQ(payload.getSensorId().cStr(), std::string("depth"));
  EXPECT_EQ(payload.getSeq(), 3u);
  EXPECT_EQ(payload.getStartMs(), 2000u);
  EXPECT_EQ(payload.getEndMs(), 2200u);
  EXPECT_EQ(payload.getFrameId().cStr(), std::string("oakd_rgb_camera_optical_frame"));
  ASSERT_EQ(payload.which(), cvr::capnp::LidarPayload::CLOUDS);

  const auto clouds = payload.getClouds();
  EXPECT_EQ(clouds.getPosition().cStr(), std::string("int16_mm"));
  EXPECT_EQ(clouds.getColor().cStr(), std::string("rgb565"));
  EXPECT_EQ(clouds.getIntensity().cStr(), std::string("uint8"));
  EXPECT_EQ(clouds.getVoxelLeafMm(), 50);
  EXPECT_EQ(clouds.getEncoding(), cvr::capnp::CloudEncoding::PLANAR_DELTA_SHUFFLE_ZSTD);
  EXPECT_EQ(clouds.getZstdLevel(), 3);

  const std::string data = BytesOf(clouds.getData());
  EXPECT_EQ(data, object.blocks[0] + object.blocks[1] + object.blocks[2]);
  const auto entries = clouds.getEntries();
  ASSERT_EQ(entries.size(), 3u);
  const std::vector<std::uint32_t> offsets = {0, 5, 261};
  std::uint64_t covered = 0;
  for (unsigned int i = 0; i < entries.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_EQ(entries[i].getTime(), 2000u + 100 * i);
    EXPECT_EQ(entries[i].getSeq(), 30 + i);
    ExpectPose(entries[i].getPose(), PoseNumber(static_cast<int>(i) + 1));
    EXPECT_EQ(entries[i].getPointCount(), object.point_counts[i]);
    EXPECT_EQ(entries[i].getByteOffset(), offsets[i]);
    EXPECT_EQ(entries[i].getByteLength(), object.blocks[i].size());
    EXPECT_EQ(data.substr(entries[i].getByteOffset(), entries[i].getByteLength()),
              object.blocks[i]);
    covered += entries[i].getByteLength();
  }
  // The entries tile `data` exactly: no gap, no overlap, nothing past the end.
  EXPECT_EQ(covered, data.size());
}

TEST(CapnpWriters, TheHeaderIsFormatVersion2WithNoPartField) {
  BinaryEnvelope envelope = SomeEnvelope();
  ASSERT_EQ(envelope.format_version, "2.0");

  CameraChunkRecord chunk;
  chunk.camera_id = "front";
  ScanBlockRecord block;
  block.sensor_id = "scan";
  block.entries = {EntryAt(1, 0)};
  block.scans = {SomeScan()};
  const scanblock::EncodedScanBlock encoded = scanblock::Encode(block.scans, false, 1);
  ASSERT_TRUE(encoded.ok) << encoded.reason;
  CloudObjectRecord object;
  object.sensor_id = "depth";

  const std::string camera = SerializeCameraChunk(envelope, chunk);
  const std::string scans = SerializeScanBlock(envelope, block, encoded);
  const std::string clouds = SerializeCloudObject(envelope, object);
  {
    PackedBody body(camera);
    ExpectEnvelope(body.camera(), envelope);
  }
  {
    PackedBody body(scans);
    ExpectEnvelope(body.lidar(), envelope);
  }
  {
    PackedBody body(clouds);
    ExpectEnvelope(body.lidar(), envelope);
  }
  // The header is the seven fields above and nothing else: a v2 payload is a whole object of
  // its own, with no part number.
  EXPECT_FALSE(HasField(capnp::Schema::from<cvr::capnp::CameraPayload>(), "part"));
  EXPECT_FALSE(HasField(capnp::Schema::from<cvr::capnp::LidarPayload>(), "part"));
  EXPECT_TRUE(HasField(capnp::Schema::from<cvr::capnp::LidarPayload>(), "formatVersion"))
      << "the field lookup itself is broken";
}

// ---------------------------------------------------------------------------
// The session: what is grouped into a part, and when a part is closed
// ---------------------------------------------------------------------------

TEST_F(BinaryStreamTest, CameraPayloadRoundTripsThroughTheGeneratedReader) {
  Build();
  const std::string unit = EveryByteValue();
  session_->RecordCameraAccessUnit("front", EntryAt(1788383013123ull, 0), SomeContext(), unit,
                                   true);
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
  EXPECT_EQ(payload.getSdkVersion().cStr(), std::string(kSdkVersion));
  // Milliseconds, and the same instant the session id names.
  EXPECT_EQ(payload.getSessionStartMs(),
            static_cast<std::uint64_t>(session_->session_timestamp() * 1000.0));

  EXPECT_EQ(payload.getCameraId().cStr(), std::string("front"));
  EXPECT_EQ(payload.getSeq(), 0u);
  EXPECT_EQ(payload.getFrameId().cStr(), std::string("oakd_rgb_camera_optical_frame"));
  EXPECT_EQ(payload.getWidth(), 640);
  EXPECT_EQ(payload.getHeight(), 360);
  ASSERT_EQ(payload.getFrames().size(), 1u);
  const auto frame = payload.getFrames()[0];
  EXPECT_EQ(frame.getTime(), 1788383013123ull);
  EXPECT_EQ(frame.getSeq(), 0u);
  EXPECT_TRUE(frame.getKeyframe());
  EXPECT_EQ(frame.getByteOffset(), 0u);
  EXPECT_EQ(frame.getByteLength(), 256u);
  ASSERT_EQ(payload.getData().size(), 256u);
  // Every byte value 0-255 survives, which is the property a JSON body could not
  // have carried at all.
  for (int i = 0; i < 256; ++i) EXPECT_EQ(payload.getData()[i], static_cast<kj::byte>(i));
  ExpectPose(frame.getPose(), SomePose());
}

// The field of view a caller passes is the one on the wire, per chunk: the context of the
// chunk's first frame. Two chunks of one camera can differ, which is what a zoom lens needs. A
// chunk recorded without one reads back as 0 by 0, the schema's "not recorded".
TEST_F(BinaryStreamTest, EachChunkCarriesItsOwnFieldOfView) {
  Build();
  VideoChunkContext context = SomeContext();
  context.hfov_deg = 61.0f;
  context.vfov_deg = 45.0f;
  session_->RecordCameraAccessUnit("front", EntryAt(1, 0), context, "ab", true);
  context.hfov_deg = 30.5f;
  context.vfov_deg = 17.25f;
  session_->RecordCameraAccessUnit("front", EntryAt(2, 1), context, "ab", true);
  session_->RecordCameraAccessUnit("front", EntryAt(3, 2), SomeContext(), "ab", true);
  context.hfov_deg = 0.01f;
  context.vfov_deg = 179.99f;
  session_->RecordCameraAccessUnit("rear", EntryAt(4, 0), context, "ab", true);
  session_->Flush();
  session_->PumpUploads();

  auto front = CameraBodies("front");
  auto rear = CameraBodies("rear");
  ASSERT_EQ(front.size(), 3u);
  ASSERT_EQ(rear.size(), 1u);
  const auto fov = [](const std::string& serialized) {
    PackedBody body(serialized);
    return std::make_pair(body.camera().getHfovDegrees(), body.camera().getVfovDegrees());
  };
  EXPECT_EQ(fov(front[0]), std::make_pair(61.0f, 45.0f));
  EXPECT_EQ(fov(front[1]), std::make_pair(30.5f, 17.25f));
  EXPECT_EQ(fov(front[2]), std::make_pair(0.0f, 0.0f));
  // Just inside both ends of the open interval.
  EXPECT_EQ(fov(rear[0]), std::make_pair(0.01f, 179.99f));
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
  const auto with_fov = [](const CameraFov& fov) {
    VideoChunkContext context = SomeContext();
    context.hfov_deg = fov.hfov_deg;
    context.vfov_deg = fov.vfov_deg;
    return context;
  };
  std::uint32_t seq = 0;
  // A keyframe each, so that each field of view is a chunk's own.
  for (const CameraFov& fov : unusable) {
    session_->RecordCameraAccessUnit("front", EntryAt(1ull + seq, seq), with_fov(fov), "ab", true);
    ++seq;
  }
  session_->Flush();
  session_->PumpUploads();

  auto chunks = CameraBodies("front");
  ASSERT_EQ(chunks.size(), unusable.size()) << "a frame was lost over its field of view";
  for (auto& kv : chunks) {
    SCOPED_TRACE(kv.first);
    PackedBody body(kv.second);
    EXPECT_EQ(body.camera().getHfovDegrees(), 0.0f);
    EXPECT_EQ(body.camera().getVfovDegrees(), 0.0f);
    EXPECT_EQ(body.camera().getFrames().size(), 1u);
  }
  EXPECT_EQ(logger_->Warnings("camera 'front'"), 1);

  // Per camera, and again in the next session.
  session_->RecordCameraAccessUnit("rear", EntryAt(1ull, 0), with_fov(CameraFov{nan, nan}), "ab",
                                   true);
  EXPECT_EQ(logger_->Warnings("camera 'rear'"), 1);
  session_->End();
  ASSERT_TRUE(session_->Begin());
  session_->RecordCameraAccessUnit("front", EntryAt(1ull, 0), with_fov(CameraFov{61.0f, 0.0f}),
                                   "ab", true);
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
  session_->RecordLaserScan("scan", 1ull, 0, "s", identity, SomeScan());
  session_->Flush();
  session_->PumpUploads();

  const auto requests = RequestsTo("/v0/lidar");
  ASSERT_EQ(requests.size(), 1u);
  PackedBody body(requests[0].body);
  // `qw` DEFAULTS to 1 in the schema, so this passes whether or not the writer
  // set it -- which is exactly why nothing downstream may rely on the default.
  // What this pins is that a pose we sent reads back as the rotation we measured,
  // and the zero-quaternion case the default exists to prevent never occurs.
  const auto pose = body.lidar().getScans().getEntries()[0].getPose();
  EXPECT_FLOAT_EQ(pose.getQw(), 1.0f);
  EXPECT_FALSE(pose.getQx() == 0.0f && pose.getQy() == 0.0f && pose.getQz() == 0.0f &&
               pose.getQw() == 0.0f);
}

TEST_F(BinaryStreamTest, LidarCarriesExactlyOneUnionArmPerPart) {
  Build();
  session_->RecordLaserScan("scan", 1000ull, 0, "base_scan", SomePose(), SomeScan());
  const EncodedPointCloud cloud = SomeCloud();
  ASSERT_TRUE(cloud.ok) << cloud.reason;
  session_->RecordPointCloud("depth", 1001ull, 0, "oakd_rgb_camera_optical_frame", SomePose(),
                             cloud);
  session_->Flush();
  session_->PumpUploads();

  auto scans = LidarBodies("scan");
  auto clouds = LidarBodies("depth");
  ASSERT_EQ(scans.size(), 1u);
  ASSERT_EQ(clouds.size(), 1u);

  PackedBody scan_body(scans[0]);
  ASSERT_EQ(scan_body.lidar().which(), cvr::capnp::LidarPayload::SCANS);
  const auto laser = scan_body.lidar().getScans();
  EXPECT_FLOAT_EQ(laser.getAngleMin(), -3.14f);
  EXPECT_FLOAT_EQ(laser.getRangeMax(), 12.0f);
  ASSERT_EQ(laser.getRays(), 3);
  const std::vector<std::uint16_t> codes = RangeCodes(laser);
  ASSERT_EQ(codes.size(), 3u);
  EXPECT_NEAR(scanblock::DequantiseRange(codes[0], UnitOf(laser)), 1.0f, 0.0005f);
  // An infinity is the sensor saying "nothing there" and is preserved, not
  // rewritten into a distance.
  EXPECT_TRUE(std::isinf(scanblock::DequantiseRange(codes[2], UnitOf(laser))));
  // Intensities were not asked for, so the field is empty rather than filled with zeros.
  EXPECT_EQ(laser.getIntensities().size(), 0u);

  PackedBody cloud_body(clouds[0]);
  ASSERT_EQ(cloud_body.lidar().which(), cvr::capnp::LidarPayload::CLOUDS);
  const auto encoded = cloud_body.lidar().getClouds();
  ASSERT_EQ(encoded.getEntries().size(), 1u);
  EXPECT_EQ(encoded.getEntries()[0].getPointCount(), 2u);
  EXPECT_EQ(encoded.getPosition().cStr(), std::string("int16_mm"));
  EXPECT_EQ(encoded.getColor().cStr(), std::string("none"));
  EXPECT_EQ(encoded.getIntensity().cStr(), std::string("none"));
  EXPECT_EQ(encoded.getVoxelLeafMm(), 0);
  EXPECT_EQ(encoded.getZstdLevel(), 1);
  EXPECT_EQ(BytesOf(encoded.getData()), cloud.data);
}

TEST_F(BinaryStreamTest, ScansSpanningThreeSecondsBecomeTwoBlocks) {
  Build();
  std::vector<LaserScanData> recorded;
  for (std::uint32_t i = 0; i < 8; ++i) {
    LaserScanData scan = SomeScan();
    scan.ranges = {0.5f + 0.0371f * i, 1.25f + 0.0007f * i, 11.9994f - 0.001f * i};
    session_->RecordLaserScan("scan", 500ull * i, i, "base_scan", PoseNumber(i), scan);
    recorded.push_back(scan);
  }
  session_->Flush();
  session_->PumpUploads();

  auto blocks = LidarBodies("scan");
  ASSERT_EQ(RequestsTo("/v0/lidar").size(), 2u);
  ASSERT_EQ(blocks.size(), 2u);
  std::uint32_t scan_index = 0;
  for (std::uint32_t seq = 0; seq < 2; ++seq) {
    SCOPED_TRACE(seq);
    PackedBody body(blocks[seq]);
    const auto payload = body.lidar();
    const auto block = payload.getScans();
    const std::size_t n = seq == 0 ? 6 : 2;
    ASSERT_EQ(block.getEntries().size(), n);
    EXPECT_EQ(payload.getStartMs(), seq == 0 ? 0u : 3000u);
    EXPECT_EQ(payload.getEndMs(), seq == 0 ? 2500u : 3500u);
    const std::vector<std::uint16_t> codes = RangeCodes(block);
    ASSERT_EQ(codes.size(), n * 3);
    for (std::size_t e = 0; e < n; ++e, ++scan_index) {
      const auto entry = block.getEntries()[static_cast<unsigned int>(e)];
      EXPECT_EQ(entry.getTime(), 500u * scan_index);
      EXPECT_EQ(entry.getSeq(), scan_index);
      ExpectPose(entry.getPose(), PoseNumber(static_cast<int>(scan_index)));
      for (std::size_t r = 0; r < 3; ++r) {
        EXPECT_NEAR(scanblock::DequantiseRange(codes[e * 3 + r], UnitOf(block)),
                    recorded[scan_index].ranges[r], 0.0005f)
            << "scan " << scan_index << " ray " << r;
      }
    }
  }
}

TEST_F(BinaryStreamTest, TwoLidarsNeverShareABlock) {
  Build();
  for (std::uint32_t i = 0; i < 6; ++i) {
    session_->RecordLaserScan(i % 2 == 0 ? "a" : "b", 100ull * i, i / 2, "base_scan", SomePose(),
                              SomeScan());
  }
  session_->Flush();
  session_->PumpUploads();

  ASSERT_EQ(RequestsTo("/v0/lidar").size(), 2u);
  for (const std::string sensor : {"a", "b"}) {
    SCOPED_TRACE(sensor);
    auto blocks = LidarBodies(sensor);
    ASSERT_EQ(blocks.size(), 1u);
    PackedBody body(blocks[0]);
    const auto entries = body.lidar().getScans().getEntries();
    ASSERT_EQ(entries.size(), 3u);
    const std::uint64_t first = sensor == "a" ? 0 : 100;
    for (unsigned int e = 0; e < entries.size(); ++e) {
      EXPECT_EQ(entries[e].getTime(), first + 200 * e);
      EXPECT_EQ(entries[e].getSeq(), e);
    }
  }
}

TEST_F(BinaryStreamTest, CloudObjectsCloseAtTheByteCap) {
  Build();
  for (std::uint32_t i = 0; i < 25; ++i) {
    session_->RecordPointCloud("depth", 10ull * i, i, "f", SomePose(),
                               CloudOfBytes(100000, static_cast<char>('a' + i)));
  }
  // The 21st cloud would take the object past 2 MiB, so the first 20 closed without a Flush.
  EXPECT_EQ(session_->stats().lidar_blocks, 1u);
  session_->Flush();
  session_->PumpUploads();

  auto objects = LidarBodies("depth");
  ASSERT_EQ(objects.size(), 2u);
  PackedBody first(objects[0]);
  const auto full = first.lidar().getClouds();
  ASSERT_EQ(full.getEntries().size(), 20u);
  EXPECT_EQ(full.getData().size(), 20u * 100000);
  EXPECT_EQ(full.getEntries()[19].getSeq(), 19u);
  PackedBody second(objects[1]);
  const auto rest = second.lidar().getClouds();
  ASSERT_EQ(rest.getEntries().size(), 5u);
  EXPECT_EQ(rest.getEntries()[0].getSeq(), 20u);
}

TEST_F(BinaryStreamTest, AccessUnitsBecomeAChunkPerKeyframe) {
  Build();
  VideoChunkContext context = SomeContext();
  context.hfov_deg = 69.0f;
  context.vfov_deg = 42.0f;
  std::vector<std::string> units;
  for (std::uint32_t i = 0; i < 12; ++i) {
    units.push_back(std::string(3 + i, static_cast<char>('a' + i)));
    session_->RecordCameraAccessUnit("front", EntryAt(1000 + 33 * i, i, PoseNumber(i)), context,
                                     units.back(), i == 0 || i == 6);
  }
  EXPECT_EQ(session_->stats().camera_chunks, 1u) << "the second keyframe closes the first chunk";
  session_->End();
  session_->PumpUploads();

  auto chunks = CameraBodies("front");
  ASSERT_EQ(RequestsTo("/v0/camera").size(), 2u);
  ASSERT_EQ(chunks.size(), 2u);
  for (std::uint32_t c = 0; c < 2; ++c) {
    SCOPED_TRACE(c);
    PackedBody body(chunks[c]);
    const auto payload = body.camera();
    EXPECT_EQ(payload.getStartMs(), 1000u + 33 * 6 * c);
    EXPECT_EQ(payload.getEndMs(), 1000u + 33 * (6 * c + 5));
    EXPECT_EQ(payload.getCodec().cStr(), std::string("h264"));
    EXPECT_EQ(BytesOf(payload.getCodecParams()), context.codec_params);
    EXPECT_EQ(payload.getWidth(), 640);
    EXPECT_EQ(payload.getHeight(), 360);
    EXPECT_FLOAT_EQ(payload.getNominalFps(), 30.0f);
    EXPECT_EQ(payload.getFrameId().cStr(), context.frame_id);
    EXPECT_EQ(payload.getHfovDegrees(), 69.0f);
    EXPECT_EQ(payload.getVfovDegrees(), 42.0f);
    EXPECT_EQ(payload.getEncoderSettings().cStr(), context.encoder_settings);
    const auto frames = payload.getFrames();
    ASSERT_EQ(frames.size(), 6u);
    const std::string data = BytesOf(payload.getData());
    std::uint32_t offset = 0;
    for (std::uint32_t f = 0; f < 6; ++f) {
      const std::uint32_t i = 6 * c + f;
      EXPECT_EQ(frames[f].getTime(), 1000u + 33 * i);
      EXPECT_EQ(frames[f].getSeq(), i);
      ExpectPose(frames[f].getPose(), PoseNumber(static_cast<int>(i)));
      EXPECT_EQ(frames[f].getKeyframe(), f == 0) << "frame " << i;
      EXPECT_EQ(frames[f].getByteOffset(), offset);
      EXPECT_EQ(data.substr(frames[f].getByteOffset(), frames[f].getByteLength()), units[i]);
      offset += frames[f].getByteLength();
    }
    EXPECT_EQ(offset, data.size());
  }
}

TEST_F(BinaryStreamTest, AForcedCloseDropsTheFramesUntilTheNextKeyframe) {
  Build();
  const VideoChunkContext context = SomeContext();
  session_->RecordCameraAccessUnit("front", EntryAt(100, 0), context, "i0", true);
  session_->RecordCameraAccessUnit("front", EntryAt(133, 1), context, "p1", false);
  session_->RecordCameraAccessUnit("front", EntryAt(166, 2), context, "p2", false);
  session_->Flush();
  // The GOP these belong to was cut short, so nothing can decode them.
  session_->RecordCameraAccessUnit("front", EntryAt(200, 3), context, "p3", false);
  session_->RecordCameraAccessUnit("front", EntryAt(233, 4), context, "p4", false);
  EXPECT_EQ(session_->stats().camera_access_units_dropped, 2u);
  session_->RecordCameraAccessUnit("front", EntryAt(266, 5), context, "i5", true);
  session_->RecordCameraAccessUnit("front", EntryAt(300, 6), context, "p6", false);
  session_->End();
  session_->PumpUploads();

  EXPECT_EQ(session_->stats().camera_access_units_dropped, 2u);
  auto chunks = CameraBodies("front");
  ASSERT_EQ(chunks.size(), 2u);
  PackedBody first(chunks[0]);
  ASSERT_EQ(first.camera().getFrames().size(), 3u);
  EXPECT_EQ(BytesOf(first.camera().getData()), "i0p1p2");
  PackedBody second(chunks[1]);
  const auto frames = second.camera().getFrames();
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_TRUE(frames[0].getKeyframe());
  EXPECT_EQ(frames[0].getSeq(), 5u);
  EXPECT_EQ(BytesOf(second.camera().getData()), "i5p6");
}

// A unit that would pass the chunk's cap is refused, not dropped: the session says so, so that the
// caller re-encodes it from a keyframe, and counts it apart from the frames it kept.
TEST_F(BinaryStreamTest, AUnitPastTheChunkCapIsRefusedAndCountedApart) {
  Build();
  const VideoChunkContext context = SomeContext();
  EXPECT_EQ(session_->RecordCameraAccessUnit("front", EntryAt(100, 0), context,
                                             std::string(kMaxVideoChunkBytes, 'i'), true),
            VideoUnitFate::kAppended);
  EXPECT_EQ(session_->RecordCameraAccessUnit("front", EntryAt(133, 1), context, "p1", false),
            VideoUnitFate::kRefused);
  EXPECT_EQ(session_->RecordCameraAccessUnit("front", EntryAt(166, 2), context, "p2", false),
            VideoUnitFate::kRefused);
  Stats stats = session_->stats();
  EXPECT_EQ(stats.camera_access_units_refused, 2u);
  EXPECT_EQ(stats.camera_access_units_dropped, 0u);
  EXPECT_EQ(stats.camera_frames, 1u);
  EXPECT_EQ(stats.camera_chunks, 1u) << "the full chunk was not closed";
  EXPECT_EQ(session_->RecordCameraAccessUnit("front", EntryAt(133, 1), context, "i1", true),
            VideoUnitFate::kAppended);
  session_->End();
  EXPECT_EQ(session_->RecordCameraAccessUnit("front", EntryAt(200, 3), context, "i3", true),
            VideoUnitFate::kDropped)
      << "a unit with no session reads as kept";
  session_->PumpUploads();
  auto chunks = CameraBodies("front");
  ASSERT_EQ(chunks.size(), 2u);
  PackedBody second(chunks[1]);
  EXPECT_EQ(BytesOf(second.camera().getData()), "i1");
}

namespace {
// A consumer's logger may call back into the Session it logs for on any line, including the one
// Begin logs once the session is live and before Begin spools its first batch.
class RecordsAKeyframeOnBeginLogger : public ILogger {
 public:
  void Log(LogLevel, const std::string& message) override {
    static const std::string kBegun = " begun";
    Session* session = session_.load();
    if (session == nullptr || message.size() < kBegun.size() ||
        message.compare(message.size() - kBegun.size(), kBegun.size(), kBegun) != 0) {
      return;
    }
    session->RecordCameraAccessUnit("front", EntryAt(100, 0), SomeContext(), "i0", true);
  }
  void CallBackInto(Session* session) { session_.store(session); }

 private:
  std::atomic<Session*> session_{nullptr};
};
}  // namespace

// Begin spools its first batch once the session is live, so a frame may already have opened a chunk
// by then. That batch closes no chunk: one it closed would cost every frame until the next keyframe.
TEST_F(BinaryStreamTest, BeginsFirstBatchLeavesAnOpenChunkOpen) {
  auto logger = std::make_shared<RecordsAKeyframeOnBeginLogger>();
  std::string err;
  session_ = Session::Create(cfg_, transport_, spool_, logger, &err);
  ASSERT_NE(session_, nullptr) << err;
  logger->CallBackInto(session_.get());
  ASSERT_TRUE(session_->Begin());
  logger->CallBackInto(nullptr);
  ASSERT_EQ(session_->stats().camera_frames, 1u) << "the logger recorded no frame during Begin";
  EXPECT_EQ(session_->stats().camera_chunks, 0u) << "Begin closed the chunk the frame opened";
  session_->RecordCameraAccessUnit("front", EntryAt(133, 1), SomeContext(), "p1", false);
  EXPECT_EQ(session_->stats().camera_access_units_dropped, 0u);
  session_->End();
  session_->PumpUploads();
  auto chunks = CameraBodies("front");
  ASSERT_EQ(chunks.size(), 1u);
  PackedBody body(chunks[0]);
  EXPECT_EQ(BytesOf(body.camera().getData()), "i0p1");
}

TEST_F(BinaryStreamTest, TheAutoSendTimerDoesNotCutAVideoChunk) {
  cfg_.auto_send_interval_s = 0.2;
  cfg_.worker_tick_s = 0.05;
  Build();
  session_->PumpUploads();  // the session start, so that it is not mistaken for the timer's part
  const std::size_t gaze_parts = RequestsContaining("/v0/gaze/");
  const VideoChunkContext context = SomeContext();
  const auto start = std::chrono::steady_clock::now();
  session_->RecordCameraAccessUnit("front", EntryAt(1000, 0), context, "i0", true);
  // A gaze sample only the timer cuts, recorded once the chunk is open: its part arriving proves
  // the timer fired, with `force`, while the chunk was open.
  session_->RecordGaze(1.0, Vec3{}, Quat{});
  for (std::uint32_t i = 1; i <= 20; ++i) {
    std::this_thread::sleep_until(start + std::chrono::milliseconds(30 * i));
    session_->RecordCameraAccessUnit("front", EntryAt(1000 + 30 * i, i), context, "p", false);
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (RequestsContaining("/v0/gaze/") <= gaze_parts &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_GT(RequestsContaining("/v0/gaze/"), gaze_parts) << "the auto-send timer never fired";
  session_->PumpUploads();

  EXPECT_TRUE(RequestsTo("/v0/camera").empty()) << "the timer cut a video chunk";
  EXPECT_EQ(session_->stats().camera_chunks, 0u);
  EXPECT_EQ(session_->stats().camera_access_units_dropped, 0u);

  session_->End();
  session_->PumpUploads();
  auto chunks = CameraBodies("front");
  ASSERT_EQ(chunks.size(), 1u);
  PackedBody body(chunks[0]);
  EXPECT_EQ(body.camera().getFrames().size(), 21u);
}

// The idle close is a real-time one, and core has no injectable steady clock: the Config field is
// the seam. The window in which no close may come is half the idle time, measured from a moment
// taken before the record, so a late start on this thread cannot stretch it past the idle time,
// and a close that ignored the idle time -- a worker tick and an uploader tick, 0.1 s at most --
// lands inside it.
TEST_F(BinaryStreamTest, AnIdleScanBlockIsClosedByTheWorker) {
  cfg_.scan_block_idle_s = 0.4;
  cfg_.worker_tick_s = 0.05;
  Build();
  const auto recorded = std::chrono::steady_clock::now();
  session_->RecordLaserScan("scan", 1000ull, 0, "base_scan", SomePose(), SomeScan());
  EXPECT_FALSE(WaitForStreamUntil("/v0/lidar", 1, recorded + std::chrono::milliseconds(200)))
      << "closed before it had been idle";
  ASSERT_TRUE(WaitForStream("/v0/lidar", 1, 5.0)) << "the worker never closed the idle block";
  EXPECT_EQ(RequestsTo("/v0/lidar").size(), 1u);
  EXPECT_EQ(session_->stats().lidar_blocks, 1u);
}

TEST_F(BinaryStreamTest, AnIdleCloudObjectIsClosedByTheWorker) {
  cfg_.cloud_object_idle_s = 0.4;
  cfg_.worker_tick_s = 0.05;
  Build();
  const EncodedPointCloud cloud = SomeCloud();
  ASSERT_TRUE(cloud.ok) << cloud.reason;
  const auto recorded = std::chrono::steady_clock::now();
  session_->RecordPointCloud("depth", 1000ull, 0, "f", SomePose(), cloud);
  EXPECT_FALSE(WaitForStreamUntil("/v0/lidar", 1, recorded + std::chrono::milliseconds(200)))
      << "closed before it had been idle";
  ASSERT_TRUE(WaitForStream("/v0/lidar", 1, 5.0)) << "the worker never closed the idle object";
  EXPECT_EQ(RequestsTo("/v0/lidar").size(), 1u);
  EXPECT_EQ(session_->stats().lidar_blocks, 1u);
}

TEST_F(BinaryStreamTest, EndCutsWhateverIsStillBuffered) {
  Build();
  const std::string id = session_->session_id();
  session_->RecordLaserScan("scan", 1ull, 0, "s", SomePose(), SomeScan());
  session_->RecordPointCloud("depth", 1ull, 0, "f", SomePose(), SomeCloud());
  session_->RecordCameraAccessUnit("front", EntryAt(1, 0), SomeContext(), "ab", true);
  session_->End("test");
  session_->PumpUploads();  // End spools what is buffered and never waits for an upload
  const auto cameras = RequestsTo("/v0/camera");
  const auto lidars = RequestsTo("/v0/lidar");
  ASSERT_EQ(cameras.size(), 1u);
  ASSERT_EQ(lidars.size(), 2u);
  {
    PackedBody camera(cameras[0].body);
    EXPECT_EQ(camera.camera().getSessionId().cStr(), id);
  }
  for (const auto& r : lidars) {
    PackedBody lidar(r.body);
    EXPECT_EQ(lidar.lidar().getSessionId().cStr(), id);
  }

  // Nothing was left behind for the next session to send.
  ASSERT_TRUE(session_->Begin());
  session_->Flush();
  session_->PumpUploads();
  EXPECT_EQ(RequestsTo("/v0/camera").size(), 1u);
  EXPECT_EQ(RequestsTo("/v0/lidar").size(), 2u);
}

TEST_F(BinaryStreamTest, ASecondSessionStartsEverySequenceAtZero) {
  Build();
  const auto one_of_each = [&](std::uint64_t t) {
    session_->RecordLaserScan("scan", t, 0, "s", SomePose(), SomeScan());
    session_->RecordPointCloud("depth", t, 0, "f", SomePose(), SomeCloud());
    session_->RecordCameraAccessUnit("front", EntryAt(t, 0), SomeContext(), "ab", true);
  };
  one_of_each(1000);
  session_->End();
  ASSERT_TRUE(session_->Begin());
  const std::string second = session_->session_id();
  one_of_each(2000);
  session_->End();
  session_->PumpUploads();

  int seen = 0;
  for (const auto& r : RequestsTo("/v0/camera")) {
    PackedBody body(r.body);
    if (second != body.camera().getSessionId().cStr()) continue;
    EXPECT_EQ(body.camera().getSeq(), 0u);
    ++seen;
  }
  for (const auto& r : RequestsTo("/v0/lidar")) {
    PackedBody body(r.body);
    if (second != body.lidar().getSessionId().cStr()) continue;
    EXPECT_EQ(body.lidar().getSeq(), 0u) << body.lidar().getSensorId().cStr();
    ++seen;
  }
  EXPECT_EQ(seen, 3);
}

TEST_F(BinaryStreamTest, AScanWithMoreRaysThanTheWireCanCountIsDroppedAndCounted) {
  Build();
  const std::uint64_t dropped = session_->stats().records_dropped;
  LaserScanData too_many = SomeScan();
  too_many.ranges.assign(65536, 1.0f);
  session_->RecordLaserScan("scan", 1000ull, 0, "s", SomePose(), too_many);
  EXPECT_EQ(session_->stats().records_dropped, dropped + 1);
  EXPECT_EQ(session_->stats().lidar_scans, 0u);
  session_->Flush();
  session_->PumpUploads();
  EXPECT_TRUE(RequestsTo("/v0/lidar").empty());

  LaserScanData most = SomeScan();
  most.ranges.assign(65535, 1.0f);
  session_->RecordLaserScan("scan", 1100ull, 1, "s", SomePose(), most);
  session_->Flush();
  session_->PumpUploads();
  const auto requests = RequestsTo("/v0/lidar");
  ASSERT_EQ(requests.size(), 1u);
  PackedBody body(requests[0].body);
  EXPECT_EQ(body.lidar().getScans().getRays(), 65535);
  EXPECT_EQ(session_->stats().records_dropped, dropped + 1);
}

TEST_F(BinaryStreamTest, EveryBinaryHeaderIsFormatVersion2) {
  Build();
  session_->RecordLaserScan("scan", 1ull, 0, "s", SomePose(), SomeScan());
  session_->RecordPointCloud("depth", 1ull, 0, "f", SomePose(), SomeCloud());
  session_->RecordCameraAccessUnit("front", EntryAt(1, 0), SomeContext(), "ab", true);
  session_->Flush();
  session_->PumpUploads();

  const auto cameras = RequestsTo("/v0/camera");
  const auto lidars = RequestsTo("/v0/lidar");
  ASSERT_EQ(cameras.size(), 1u);
  ASSERT_EQ(lidars.size(), 2u);
  PackedBody camera(cameras[0].body);
  EXPECT_EQ(camera.camera().getFormatVersion().cStr(), std::string("2.0"));
  for (const auto& r : lidars) {
    PackedBody lidar(r.body);
    EXPECT_EQ(lidar.lidar().getFormatVersion().cStr(), std::string("2.0"));
  }
}

TEST_F(BinaryStreamTest, EachStreamGetsItsOwnUrlAndContentType) {
  Build();
  session_->RecordCameraAccessUnit("front", EntryAt(1, 0), SomeContext(), "ab", true);
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
  session_->RecordCameraAccessUnit("front", EntryAt(1, 0), SomeContext(), std::string(500, 'x'),
                                   true);
  session_->RecordLaserScan("scan", 1ull, 0, "s", SomePose(), SomeScan());
  Stats stats = session_->stats();
  EXPECT_EQ(stats.camera_frames, 1u);
  EXPECT_EQ(stats.camera_bytes, 500u);
  EXPECT_EQ(stats.lidar_scans, 1u);
  // Neither reached `records_recorded`, which counts the JSON streams' records.
  EXPECT_EQ(stats.records_recorded, 0u);
  // A part is counted when it closes, not when a record arrives.
  EXPECT_EQ(stats.camera_chunks, 0u);
  EXPECT_EQ(stats.lidar_blocks, 0u);
  session_->Flush();
  stats = session_->stats();
  EXPECT_EQ(stats.camera_chunks, 1u);
  EXPECT_EQ(stats.lidar_blocks, 1u);
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
