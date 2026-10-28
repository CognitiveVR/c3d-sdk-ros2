// The pose stream through a Session: manifests, samples, the worker's encoding, the upload
// queue, End's flush and the spool, read back from what was posted through the generated
// Cap'n Proto reader and the tests' own block decoder.
//
// The pose clock is the test's (Config::pose_clock_us) and PumpPose runs the worker's pass on
// the test thread, so a whole session of queue timing runs in milliseconds. Two tests run on
// the real clock: the encode lag, and nothing else depends on the worker's own ticks.
//
// PoseEndToEnd writes what it posted to $C3D_POSE_CAPTURE when that names a directory, for
// c3d-stream-formatter's upload check (tools/README.md).

#include <gtest/gtest.h>
#include <unistd.h>

#include <capnp/serialize-packed.h>
#include <kj/io.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "c3d/pose.hpp"
#include "c3d/pose_manifest.hpp"
#include "c3d/session.hpp"
#include "c3d/version.hpp"
#include "fake_transport.hpp"
#include "pose_payload.capnp.h"
#include "pose_test_decoder.hpp"

using namespace c3d;
namespace fs = std::filesystem;

namespace {

constexpr std::int64_t kS = 1'000'000;
// Sample times: epoch microseconds in 2026, the pose clock's reading added on.
constexpr std::int64_t kEpochUs = 1'790'000'000'000'000;

struct Joint {
  std::uint32_t j = 0;
  std::string mesh_name;
  std::map<std::string, std::string> props;  // every value as text, its type prefixed
};
struct Entry {
  std::string id, name, mesh, file_type, mapping, root;
  std::uint32_t first_seq = 0;
  std::vector<Joint> joints;
  std::map<std::string, std::string> props;
};
struct Blk {
  std::string id;
  std::uint32_t seq = 0;
  std::int64_t start_us = 0;
  std::uint32_t n = 0;
  bool reset_before = false;
  std::string data;
};
struct Part {
  std::uint32_t part = 0;
  std::string session_id, user_id, scene_id, format_version, sdk_version;
  std::uint32_t scene_version = 0;
  std::uint64_t session_start_ms = 0;
  std::int64_t start_us = 0, end_us = 0;
  std::vector<Entry> manifests;
  std::vector<Blk> blocks;
  std::string body;
};

std::map<std::string, std::string> Props(
    capnp::List<cvr::capnp::Property>::Reader list) {
  std::map<std::string, std::string> out;
  for (auto p : list) {
    std::string v;
    switch (p.which()) {
      case cvr::capnp::Property::STR: v = "str:" + std::string(p.getStr().cStr()); break;
      case cvr::capnp::Property::NUM: v = "num:" + std::to_string(p.getNum()); break;
      case cvr::capnp::Property::BOOL: v = std::string("bool:") + (p.getBool() ? "1" : "0"); break;
    }
    out[p.getKey().cStr()] = v;
  }
  return out;
}

// Through the GENERATED reader, from the schema the gateway reads with.
Part Read(const std::string& body) {
  kj::ArrayPtr<const kj::byte> bytes(reinterpret_cast<const kj::byte*>(body.data()), body.size());
  kj::ArrayInputStream stream(bytes);
  capnp::PackedMessageReader reader(stream);
  auto p = reader.getRoot<cvr::capnp::PosePayload>();
  Part out;
  out.body = body;
  out.part = p.getPart();
  out.session_id = p.getSessionId().cStr();
  out.user_id = p.getUserId().cStr();
  out.scene_id = p.getSceneId().cStr();
  out.scene_version = p.getSceneVersion();
  out.format_version = p.getFormatVersion().cStr();
  out.sdk_version = p.getSdkVersion().cStr();
  out.session_start_ms = p.getSessionStartMs();
  out.start_us = p.getStartUs();
  out.end_us = p.getEndUs();
  for (auto m : p.getManifests()) {
    Entry e;
    e.id = m.getManifestId().cStr();
    e.name = m.getName().cStr();
    e.mesh = m.getMesh().cStr();
    e.file_type = m.getFileType().cStr();
    e.mapping = m.getRiggingMappingType().cStr();
    e.root = m.getRoot().cStr();
    e.first_seq = m.getFirstSeq();
    e.props = Props(m.getProperties());
    for (auto j : m.getJoints()) e.joints.push_back(Joint{j.getJ(), j.getMeshName().cStr(), Props(j.getProperties())});
    out.manifests.push_back(std::move(e));
  }
  for (auto b : p.getBlocks()) {
    Blk k;
    k.id = b.getManifestId().cStr();
    k.seq = b.getSeq();
    k.start_us = b.getStartUs();
    k.n = b.getSampleCount();
    k.reset_before = b.getResetBefore();
    k.data.assign(reinterpret_cast<const char*>(b.getData().begin()), b.getData().size());
    out.blocks.push_back(std::move(k));
  }
  return out;
}

pose::Manifest Rig(const std::string& id, int joints,
                   std::uint8_t groups = pose::kPosition | pose::kRotation) {
  pose::Manifest m;
  m.manifest_id = id;
  m.name = id + " name";
  m.rigging_mapping_type = "meshName";
  m.properties = {{"c3d.profile", std::string("ros2/1")}, {"ros.poseConvention", std::string("unity")}};
  for (int j = 0; j < joints; ++j) {
    pose::ManifestJoint joint;
    joint.mesh_name = j == 0 ? "base_link" : "link_" + std::to_string(j);
    joint.groups = groups;
    joint.properties = {{"ros.parent", std::string(j == 0 ? "map" : "base_link")}};
    m.joints.push_back(joint);
  }
  return m;
}

// Smooth motion, so blocks compress as a robot's do. With `noise`, positions jump by up to
// that many metres, rotations are random and scales wander, so blocks barely compress.
pose::PoseSample Sample(std::int64_t t_us, int joints, double noise = 0.0,
                        std::uint32_t* seed = nullptr) {
  pose::PoseSample s;
  s.t_us = t_us;
  const double t = static_cast<double>(t_us % (1000 * kS)) / 1e6;
  auto next = [seed]() {
    *seed ^= *seed << 13;
    *seed ^= *seed >> 17;
    *seed ^= *seed << 5;
    return static_cast<double>(*seed) / 4294967296.0 - 0.5;
  };
  for (int j = 0; j < joints; ++j) {
    pose::JointPose p;
    p.position = Vec3{std::sin(t + j) * 2.0, std::cos(t * 0.5 + j), 0.1 * j};
    if (noise > 0.0) {
      p.position.x += noise * next();
      p.position.y += noise * next();
      p.position.z += noise * next();
    }
    const double a = 0.3 * t + 0.1 * j;
    p.rotation = Quat{0.0, 0.0, std::sin(a / 2), std::cos(a / 2)};
    p.scale = Vec3{1.0, 1.0, 1.0};
    if (noise > 0.0) {
      Quat q{next(), next(), next(), next()};
      const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) + 1e-9;
      p.rotation = Quat{q.x / n, q.y / n, q.z / n, q.w / n};
      p.scale = Vec3{1.0 + next(), 1.0 + next(), 1.0 + next()};
    }
    s.joints.push_back(p);
  }
  return s;
}

class CountingLogger : public ILogger {
 public:
  void Log(LogLevel level, const std::string& message) override {
    std::lock_guard<std::mutex> lock(mu_);
    lines_.emplace_back(level, message);
  }
  int Count(LogLevel level, const std::string& fragment) const {
    std::lock_guard<std::mutex> lock(mu_);
    int n = 0;
    for (const auto& l : lines_) n += l.first == level && l.second.find(fragment) != std::string::npos;
    return n;
  }

 private:
  mutable std::mutex mu_;
  std::vector<std::pair<LogLevel, std::string>> lines_;
};

class PoseSessionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() /
           ("c3d_pose_" + std::to_string(::getpid()) + "_" + std::to_string(counter_++));
    fs::create_directories(dir_ / "spool");
    cfg_.application_key = "APPKEY";
    cfg_.scene_id = "SCENEGUID";
    cfg_.scene_version = "7";
    cfg_.device_id = "robot01";
    cfg_.app_version = "9.9.9-app";
    cfg_.spool_dir = (dir_ / "spool").string();
    cfg_.gateway = "data.example.invalid";
    // Neither the auto-send timer nor the worker's own ticks decide anything here: the pose
    // clock is the test's, and PumpPose runs the worker's pass.
    cfg_.auto_send_interval_s = 3600.0;
    cfg_.worker_tick_s = 1.0;
    cfg_.pose_block_interval_s = 1.0;
    cfg_.pose_send_log_path = (dir_ / "send_log.jsonl").string();
    cfg_.pose_clock_us = [clock = clock_] { return clock->load(); };
    cfg_.shutdown_drain_s = 0.0;
    transport_ = std::make_shared<FakeTransport>();
  }
  void TearDown() override {
    session_.reset();
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }
  void Build(bool begin = true) {
    spool_ = std::make_shared<DiskSpool>(cfg_.spool_dir, 256ull * 1024 * 1024,
                                         std::make_shared<NullLogger>());
    std::string err;
    session_ = Session::Create(cfg_, transport_, spool_, logger_, &err);
    ASSERT_NE(session_, nullptr) << err;
    if (begin) ASSERT_TRUE(session_->Begin());
  }
  std::int64_t Now() const { return clock_->load(); }
  void At(std::int64_t t) { clock_->store(t); }
  // Records a sample of `id` now, its time the pose clock's on the epoch.
  void Record(const std::string& id, int joints, std::int64_t lag_us = 0) {
    std::string err;
    ASSERT_TRUE(session_->RecordPoseSample(id, Sample(kEpochUs + Now() - lag_us, joints), &err))
        << err;
  }
  // `id` at `hz` until the clock reads `until`, a worker pass after every sample.
  void Run(const std::string& id, int joints, double hz, std::int64_t until, std::int64_t lag = 0) {
    const auto period = static_cast<std::int64_t>(1e6 / hz);
    while (Now() + period <= until) {
      At(Now() + period);
      Record(id, joints, lag);
      session_->PumpPose();
    }
  }
  void Upload() {
    for (int i = 0; i < 20 && spool_->Count() > 0; ++i) session_->PumpUploads();
  }
  // Every /v0/pose body posted, read back and in part order; a resend appears once.
  std::vector<Part> Parts() {
    Upload();
    std::map<std::uint32_t, Part> by_part;
    for (const auto& r : transport_->requests()) {
      if (r.url.size() < 8 || r.url.compare(r.url.size() - 8, 8, "/v0/pose") != 0) continue;
      Part p = Read(r.body);
      by_part.emplace(p.part, std::move(p));
    }
    std::vector<Part> out;
    for (auto& kv : by_part) out.push_back(std::move(kv.second));
    return out;
  }
  std::vector<std::string> SendLog() const {
    std::ifstream f(cfg_.pose_send_log_path);
    std::vector<std::string> lines;
    for (std::string line; std::getline(f, line);) lines.push_back(line);
    return lines;
  }
  std::vector<FakeTransport::Request> GazePosts() const {
    std::vector<FakeTransport::Request> out;
    for (const auto& r : transport_->requests()) {
      if (r.url.find("/v0/gaze/") != std::string::npos) out.push_back(r);
    }
    return out;
  }

  Config cfg_;
  std::shared_ptr<std::atomic<std::int64_t>> clock_ =
      std::make_shared<std::atomic<std::int64_t>>(1000 * kS);
  std::shared_ptr<FakeTransport> transport_;
  std::shared_ptr<DiskSpool> spool_;
  std::shared_ptr<CountingLogger> logger_ = std::make_shared<CountingLogger>();
  std::unique_ptr<Session> session_;
  fs::path dir_;
  static int counter_;
};
int PoseSessionTest::counter_ = 0;

int BlockParts(const std::vector<Part>& parts) {
  int n = 0;
  for (const auto& p : parts) n += p.blocks.empty() ? 0 : 1;
  return n;
}

}  // namespace

// Rule 6 and §9: a manifest goes out at once, in a part with no blocks and no envelope
// times; `j` is each joint's index, the properties keep their types, an empty mesh is no
// model, and the header names the session.
TEST_F(PoseSessionTest, AManifestGoesOutAtOnceInAPartOfItsOwn) {
  Build();
  pose::Manifest m = Rig("robot", 3);
  m.properties.emplace_back("ros.meshYawOffsetDeg", 90.5);
  m.properties.emplace_back("hand.scale", 2);
  m.properties.emplace_back("ros.urdf", true);
  m.properties.emplace_back("empty", std::string());
  ASSERT_TRUE(session_->SetPoseManifest(m));
  session_->PumpPose();
  const auto parts = Parts();
  ASSERT_EQ(parts.size(), 1u);
  const Part& p = parts[0];
  EXPECT_EQ(p.part, 1u);
  EXPECT_EQ(p.session_id, session_->session_id());
  EXPECT_EQ(p.user_id, "robot01");
  EXPECT_EQ(p.scene_id, "SCENEGUID");
  EXPECT_EQ(p.scene_version, 7u);
  EXPECT_EQ(p.format_version, "1.0");
  EXPECT_EQ(p.sdk_version, kSdkVersion);
  EXPECT_EQ(p.session_start_ms, static_cast<std::uint64_t>(session_->session_timestamp() * 1000.0));
  EXPECT_TRUE(p.blocks.empty());
  EXPECT_EQ(p.start_us, 0);
  EXPECT_EQ(p.end_us, 0);
  ASSERT_EQ(p.manifests.size(), 1u);
  const Entry& e = p.manifests[0];
  EXPECT_EQ(e.id, "robot");
  EXPECT_EQ(e.name, "robot name");
  EXPECT_EQ(e.mesh, "");
  EXPECT_EQ(e.file_type, "");
  EXPECT_EQ(e.mapping, "meshName");
  EXPECT_EQ(e.root, "");
  EXPECT_EQ(e.first_seq, 0u);
  EXPECT_EQ(e.props.at("c3d.profile"), "str:ros2/1");
  EXPECT_EQ(e.props.at("ros.meshYawOffsetDeg"), "num:" + std::to_string(90.5));
  EXPECT_EQ(e.props.at("hand.scale"), "num:" + std::to_string(2.0));
  EXPECT_EQ(e.props.at("ros.urdf"), "bool:1");
  EXPECT_EQ(e.props.at("empty"), "str:") << "an explicit empty string is a value";
  ASSERT_EQ(e.joints.size(), 3u);
  for (std::uint32_t j = 0; j < 3; ++j) EXPECT_EQ(e.joints[j].j, j);
  EXPECT_EQ(e.joints[0].mesh_name, "base_link");
  EXPECT_EQ(e.joints[2].mesh_name, "link_2");
  EXPECT_EQ(e.joints[1].props.at("ros.parent"), "str:base_link");
  EXPECT_EQ(session_->stats().pose_manifest_sends, 1u);
  EXPECT_EQ(session_->stats().pose_parts, 1u);
}

// One call is one part, whatever it lists; two calls are two parts even with no block
// between them, so two sends of one object never share a part.
TEST_F(PoseSessionTest, EntriesOfOneCallShareAPartAndCallsNeverDo) {
  Build();
  ASSERT_TRUE(session_->SetPoseManifests({Rig("a", 2), Rig("b", 3)}));
  ASSERT_TRUE(session_->SetPoseManifest(Rig("a", 2)));
  session_->PumpPose();
  const auto parts = Parts();
  ASSERT_EQ(parts.size(), 2u);
  ASSERT_EQ(parts[0].manifests.size(), 2u);
  ASSERT_EQ(parts[1].manifests.size(), 1u);
  EXPECT_EQ(parts[1].manifests[0].id, "a");
  EXPECT_EQ(parts[1].manifests[0].first_seq, 0u) << "no block between the two sends";
}

// Rule 3: "nowhere else". Neither the JSON streams' auto-send timer nor Flush() cuts a pose
// part; only the queue's own 11 s does.
TEST_F(PoseSessionTest, TheAutoSendTimerAndFlushNeverCutAPosePart) {
  cfg_.auto_send_interval_s = 0.02;
  cfg_.worker_tick_s = 0.005;
  Build();
  ASSERT_TRUE(session_->SetPoseManifest(Rig("r", 4)));
  Run("r", 4, 10.0, Now() + 5 * kS);
  EXPECT_GE(session_->stats().pose_blocks, 3u);
  for (int i = 0; i < 5; ++i) {
    session_->Flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
  EXPECT_EQ(BlockParts(Parts()), 0) << "a block part went out before the queue's 11 s";
}

// Rule 3: the queue goes out at the first check 11 s or more after its first block ENTERED
// it, and the next queue starts with the next block.
TEST_F(PoseSessionTest, TheQueueGoesOutElevenSecondsAfterItsFirstBlockArrived) {
  Build();
  ASSERT_TRUE(session_->SetPoseManifest(Rig("r", 4)));
  const std::int64_t t0 = Now();
  // 10 Hz: the block of samples 1..10 closes when sample 11 arrives, at t0 + 1.1 s.
  Run("r", 4, 10.0, t0 + 1100 * 1000);
  ASSERT_EQ(session_->stats().pose_blocks, 1u);
  const std::int64_t first_entry = Now();
  Run("r", 4, 10.0, first_entry + 11 * kS - 100 * 1000);
  EXPECT_EQ(BlockParts(Parts()), 0);
  Run("r", 4, 10.0, first_entry + 11 * kS);
  const auto parts = Parts();
  ASSERT_EQ(BlockParts(parts), 1);
  const Part& p = parts.back();
  ASSERT_EQ(p.blocks.size(), 11u) << "blocks entering at 1.1 s .. 11.1 s, every one before the send";
  for (std::uint32_t i = 0; i < p.blocks.size(); ++i) EXPECT_EQ(p.blocks[i].seq, i);
  const auto log = SendLog();
  ASSERT_EQ(log.size(), 1u);
  EXPECT_EQ(log[0], "{\"part\": 2, \"queueStartUs\": " + std::to_string(first_entry) +
                        ", \"sentUs\": " + std::to_string(first_entry + 11 * kS) +
                        ", \"reason\": \"timer\"}");
}

// Rule 3: the entry time is the clock read when the block is added, never when it could have
// closed. A block whose host-clock close came due at 1.1 s but was queued at 6 s starts a
// queue that is due at 17 s, not 12.1 s.
TEST_F(PoseSessionTest, TheEntryTimeIsWhenABlockIsQueuedNotWhenItCouldClose) {
  Build();
  ASSERT_TRUE(session_->SetPoseManifest(Rig("r", 2)));
  const std::int64_t t0 = Now();
  Run("r", 2, 10.0, t0 + 100 * 1000);  // one sample at 0.1 s: the block is due at 1.1 s
  At(t0 + 6 * kS);
  session_->PumpPose();
  ASSERT_EQ(session_->stats().pose_blocks, 1u);
  At(t0 + 17 * kS - 1);
  session_->PumpPose();
  EXPECT_EQ(BlockParts(Parts()), 0);
  At(t0 + 17 * kS);
  session_->PumpPose();
  EXPECT_EQ(BlockParts(Parts()), 1);
  const auto log = SendLog();
  ASSERT_EQ(log.size(), 1u);
  EXPECT_NE(log[0].find("\"queueStartUs\": " + std::to_string(t0 + 6 * kS)), std::string::npos)
      << log[0];
  EXPECT_GE(session_->stats().pose_encode_lag_max_us, 4900 * 1000) << "queued 4.9 s after due";
}

// Rule 7: a block part's times are its earliest block start and its latest SAMPLE, inclusive,
// and each block's envelope copies its header.
TEST_F(PoseSessionTest, ABlockPartCarriesItsBlocksTimesAndHeaders) {
  Build();
  ASSERT_TRUE(session_->SetPoseManifests({Rig("a", 2), Rig("b", 3)}));
  const std::int64_t t0 = Now();
  for (int i = 1; i <= 25; ++i) {
    At(t0 + i * 100 * 1000);
    Record("a", 2);
    if (i % 2) Record("b", 3);
    session_->PumpPose();
  }
  At(t0 + 4 * kS);  // the open blocks close on the host clock, into the same queue
  session_->PumpPose();
  At(t0 + 20 * kS);
  session_->PumpPose();
  const auto parts = Parts();
  ASSERT_EQ(BlockParts(parts), 1);
  const Part& p = parts.back();
  std::int64_t start = INT64_MAX, end = 0;
  std::map<std::string, pose_test::Decoder> decoders;
  for (const Blk& b : p.blocks) {
    auto it = decoders.emplace(b.id, pose_test::Decoder(b.id == "a" ? 2 : 3)).first;
    const auto d = it->second.Decode(b.data);
    ASSERT_EQ(d.error, "") << b.id << " " << b.seq;
    EXPECT_EQ(d.start_us, b.start_us);
    EXPECT_EQ(static_cast<std::uint32_t>(d.n), b.n);
    start = std::min(start, d.start_us);
    end = std::max(end, d.t_us.back());
  }
  EXPECT_EQ(p.start_us, start);
  EXPECT_EQ(p.end_us, end);
  EXPECT_EQ(p.end_us, kEpochUs + t0 + 25 * 100 * 1000) << "the last sample, not a window end";
}

// Rule 1: a block closes once its interval has passed on the HOST clock since its first sample
// was taken, with no sample needed.
TEST_F(PoseSessionTest, ABlockClosesOnTheHostClockWhenItsSamplesStop) {
  Build();
  ASSERT_TRUE(session_->SetPoseManifest(Rig("r", 4)));
  const std::int64_t t0 = Now();
  Run("r", 4, 10.0, t0 + 300 * 1000);
  At(t0 + 100 * 1000 + kS - 1);
  session_->PumpPose();
  EXPECT_EQ(session_->stats().pose_blocks, 0u);
  At(t0 + 100 * 1000 + kS);
  session_->PumpPose();
  EXPECT_EQ(session_->stats().pose_blocks, 1u);
}

// Rule 1: elapsed time is the host's, never host now - first stamp. Samples stamped 5 s
// behind the host still make 1 s blocks of 10 samples, not one-sample blocks.
TEST_F(PoseSessionTest, ALaggingSampleClockDoesNotShrinkBlocks) {
  Build();
  ASSERT_TRUE(session_->SetPoseManifest(Rig("r", 2)));
  Run("r", 2, 10.0, Now() + 3 * kS, 5 * kS);
  At(Now() + kS);  // the last block closes on the host clock
  session_->PumpPose();
  At(Now() + 12 * kS);
  session_->PumpPose();
  const auto parts = Parts();
  ASSERT_EQ(BlockParts(parts), 1);
  ASSERT_EQ(parts.back().blocks.size(), 3u);
  for (const Blk& b : parts.back().blocks) EXPECT_EQ(b.n, 10u);
}

// Rule 1 and §A 10b: a re-send closes the open block under the previous send; firstSeq is the
// seq the object's next block gets, which alone carries resetBefore and is a keyframe that
// writes every group.
TEST_F(PoseSessionTest, AResendClosesTheOpenBlockAndTheNextBlockStartsOver) {
  Build();
  ASSERT_TRUE(session_->SetPoseManifest(Rig("r", 3)));
  Run("r", 3, 10.0, Now() + 2500 * 1000);  // blocks 0 and 1 closed, the third open
  ASSERT_TRUE(session_->SetPoseManifest(Rig("r", 3)));
  Run("r", 3, 10.0, Now() + 1500 * 1000);
  At(Now() + 20 * kS);
  session_->PumpPose();
  const auto parts = Parts();
  std::vector<Entry> sends;
  std::vector<Blk> blocks;
  for (const Part& p : parts) {
    ASSERT_TRUE(p.manifests.empty() || p.blocks.empty()) << "part " << p.part;
    for (const auto& m : p.manifests) sends.push_back(m);
    for (const auto& b : p.blocks) blocks.push_back(b);
  }
  ASSERT_EQ(sends.size(), 2u);
  EXPECT_EQ(sends[0].first_seq, 0u);
  EXPECT_EQ(sends[1].first_seq, 3u) << "seq 2 was open at the re-send and closed under the first";
  ASSERT_GE(blocks.size(), 4u);
  pose_test::Decoder decoder(3);
  for (std::uint32_t i = 0; i < blocks.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_EQ(blocks[i].seq, i);
    EXPECT_EQ(blocks[i].reset_before, i == 0 || i == 3);
    const auto d = decoder.Decode(blocks[i].data);
    ASSERT_EQ(d.error, "");
    EXPECT_EQ(d.keyframe, i == 0 || i == 3);
  }
  EXPECT_LT(blocks[2].n, 10u) << "the re-send closed it early";
}

// A new joint set is a new encoder, and the object's seq carries on through it.
TEST_F(PoseSessionTest, ANewJointSetTakesANewEncoderAndTheSeqCarriesOn) {
  Build();
  ASSERT_TRUE(session_->SetPoseManifest(Rig("r", 3)));
  Run("r", 3, 10.0, Now() + 1500 * 1000);
  ASSERT_TRUE(session_->SetPoseManifest(Rig("r", 4)));
  std::string err;
  EXPECT_FALSE(session_->RecordPoseSample("r", Sample(kEpochUs + Now(), 3), &err));
  EXPECT_NE(err.find("manifest has 4"), std::string::npos) << err;
  Run("r", 4, 10.0, Now() + 1500 * 1000);
  ASSERT_TRUE(session_->SetPoseManifest(Rig("r", 4, pose::kPosition | pose::kRotation | pose::kScale)));
  Run("r", 4, 10.0, Now() + 500 * 1000);
  session_->End();
  std::vector<Entry> sends;
  std::vector<Blk> blocks;
  for (const Part& p : Parts()) {
    for (const auto& m : p.manifests) sends.push_back(m);
    for (const auto& b : p.blocks) blocks.push_back(b);
  }
  std::sort(blocks.begin(), blocks.end(), [](const Blk& a, const Blk& b) { return a.seq < b.seq; });
  ASSERT_EQ(sends.size(), 3u);
  EXPECT_EQ(sends[1].first_seq, 2u);
  EXPECT_EQ(sends[1].joints.size(), 4u);
  ASSERT_GE(blocks.size(), 5u);
  std::set<std::uint32_t> resets;
  for (std::uint32_t i = 0; i < blocks.size(); ++i) {
    EXPECT_EQ(blocks[i].seq, i) << "the seq never restarts";
    if (blocks[i].reset_before) resets.insert(i);
  }
  EXPECT_EQ(resets, (std::set<std::uint32_t>{0, sends[1].first_seq, sends[2].first_seq}));
  // Each run decodes with its own joint count.
  pose_test::Decoder three(3), four(4);
  for (const Blk& b : blocks) {
    const auto d = (b.seq < sends[1].first_seq ? three : four).Decode(b.data);
    EXPECT_EQ(d.error, "") << b.seq;
  }
}

// A follower has no encoder; its firstSeq counts in its root's seq and its send leaves the
// root's open block alone.
TEST_F(PoseSessionTest, AFollowersFirstSeqCountsInItsRoot) {
  Build();
  ASSERT_TRUE(session_->SetPoseManifest(Rig("body", 2)));
  Run("body", 2, 10.0, Now() + 2500 * 1000);  // seq 0, 1 closed, 2 open
  pose::Manifest skin;
  skin.manifest_id = "skin";
  skin.name = "skin";
  skin.mesh = "skin_mesh";
  skin.file_type = "gltf";
  skin.root = "body";
  ASSERT_TRUE(session_->SetPoseManifest(skin));
  session_->PumpPose();
  std::string err;
  EXPECT_FALSE(session_->RecordPoseSample("skin", Sample(kEpochUs, 0), &err));
  const auto parts = Parts();
  ASSERT_EQ(parts.back().manifests.size(), 1u);
  EXPECT_EQ(parts.back().manifests[0].first_seq, 2u);
  EXPECT_TRUE(parts.back().manifests[0].joints.empty());
  EXPECT_EQ(session_->stats().pose_blocks, 2u) << "a follower's send closes nothing";
}

// Rule 5: End sends the queue as it is first, then everything recorded and every open block
// as the flush's own part; both are flush:end parts of flush 1, and the session's repaired
// rotation count rides its last gaze part.
TEST_F(PoseSessionTest, EndSendsTheQueueAsItIsThenEveryOpenBlock) {
  Build();
  ASSERT_TRUE(session_->SetPoseManifests({Rig("a", 2), Rig("b", 2)}));
  const std::int64_t t0 = Now();
  Run("a", 2, 10.0, t0 + 2500 * 1000);  // a: two blocks queued, one open
  At(Now() + 100 * 1000);
  Record("b", 2);  // b: in the inbox, never through a worker pass
  const std::int64_t end_at = Now();
  session_->End();
  const auto parts = Parts();
  ASSERT_EQ(BlockParts(parts), 2);
  const Part& queued = parts[parts.size() - 2];
  const Part& closed = parts.back();
  ASSERT_EQ(queued.blocks.size(), 2u);
  EXPECT_EQ(queued.blocks[0].id, "a");
  ASSERT_EQ(closed.blocks.size(), 2u);
  std::set<std::string> ids;
  for (const Blk& b : closed.blocks) ids.insert(b.id + ":" + std::to_string(b.seq));
  EXPECT_EQ(ids, (std::set<std::string>{"a:2", "b:0"}));
  const auto log = SendLog();
  ASSERT_EQ(log.size(), 2u);
  const std::string tail = ", \"sentUs\": " + std::to_string(end_at) +
                           ", \"reason\": \"flush:end\", \"flush\": 1}";
  EXPECT_NE(log[0].find("\"part\": " + std::to_string(queued.part)), std::string::npos) << log[0];
  EXPECT_NE(log[0].find(tail), std::string::npos) << log[0];
  EXPECT_NE(log[1].find("\"part\": " + std::to_string(closed.part)), std::string::npos) << log[1];
  EXPECT_NE(log[1].find(tail), std::string::npos) << log[1];
  EXPECT_LT(queued.part, closed.part);
  bool counted = false;
  for (const auto& g : GazePosts()) {
    counted |= g.body.find("\"c3d.stream.pose.repairedRotations\":0") != std::string::npos;
  }
  EXPECT_TRUE(counted);
}

// R9: the interval travels once, as integer milliseconds, on the gaze stream, and only in a
// session that uses the pose stream.
TEST_F(PoseSessionTest, TheBlockIntervalTravelsAsASessionProperty) {
  cfg_.pose_block_interval_s = 2.5;
  Build();
  session_->Flush();
  session_->PumpUploads();
  for (const auto& g : GazePosts()) {
    EXPECT_EQ(g.body.find("c3d.stream.pose"), std::string::npos) << "no pose in this session yet";
  }
  ASSERT_TRUE(session_->SetPoseManifest(Rig("r", 1)));
  ASSERT_TRUE(session_->SetPoseManifest(Rig("r", 1)));
  session_->Flush();
  session_->PumpUploads();
  int found = 0;
  for (const auto& g : GazePosts()) {
    std::size_t at = 0;
    while ((at = g.body.find("\"c3d.stream.pose.blockIntervalMs\":2500", at)) != std::string::npos) {
      ++found;
      ++at;
    }
  }
  EXPECT_EQ(found, 1);
}

// A rotation of all zeros is repaired, counted for the session property, and said once per
// object.
TEST_F(PoseSessionTest, AZeroRotationIsRepairedCountedAndSaidOnce) {
  Build();
  ASSERT_TRUE(session_->SetPoseManifest(Rig("r", 2)));
  for (int i = 0; i < 3; ++i) {
    At(Now() + 100 * 1000);
    pose::PoseSample s = Sample(kEpochUs + Now(), 2);
    s.joints[1].rotation = Quat{0, 0, 0, 0};
    ASSERT_TRUE(session_->RecordPoseSample("r", s));
    session_->PumpPose();
  }
  EXPECT_EQ(session_->stats().pose_repaired_rotations, 3u);
  EXPECT_EQ(logger_->Count(LogLevel::kWarn, "rotation of all zeros"), 1);
  session_->End();
  Upload();
  bool counted = false;
  for (const auto& g : GazePosts()) {
    counted |= g.body.find("\"c3d.stream.pose.repairedRotations\":3") != std::string::npos;
  }
  EXPECT_TRUE(counted);
}

// What the encoder refuses is counted and said once per object; the samples after it encode.
TEST_F(PoseSessionTest, ASampleTheEncoderRefusesIsCountedAndSaidOnce) {
  Build();
  ASSERT_TRUE(session_->SetPoseManifest(Rig("r", 2)));
  Record("r", 2);
  session_->PumpPose();
  for (int i = 0; i < 2; ++i) {
    pose::PoseSample back = Sample(kEpochUs + Now() - 10 * kS, 2);  // before the previous one
    ASSERT_TRUE(session_->RecordPoseSample("r", back));
  }
  session_->PumpPose();
  At(Now() + 100 * 1000);
  Record("r", 2);
  session_->PumpPose();
  const Stats s = session_->stats();
  EXPECT_EQ(s.pose_samples, 2u);
  EXPECT_EQ(s.pose_samples_refused, 2u);
  EXPECT_EQ(logger_->Count(LogLevel::kWarn, "a sample was refused"), 1);
}

TEST_F(PoseSessionTest, SamplesAreRefusedAtTheDoor) {
  Build(false);
  std::string err;
  EXPECT_FALSE(session_->RecordPoseSample("r", Sample(kEpochUs, 2), &err));
  EXPECT_EQ(err, "no session is open");
  EXPECT_FALSE(session_->SetPoseManifest(Rig("r", 2), &err));
  EXPECT_EQ(err, "no session is open");
  ASSERT_TRUE(session_->Begin());
  EXPECT_FALSE(session_->RecordPoseSample("r", Sample(kEpochUs, 2), &err));
  EXPECT_NE(err.find("no manifest"), std::string::npos) << err;
  ASSERT_TRUE(session_->SetPoseManifest(Rig("r", 2)));
  EXPECT_FALSE(session_->RecordPoseSample("r", Sample(kEpochUs, 3), &err));
  EXPECT_NE(err.find("3 joints"), std::string::npos) << err;
  EXPECT_TRUE(session_->RecordPoseSample("r", Sample(kEpochUs, 2), &err)) << err;
  // A new session starts over: its objects are sent again before they record.
  session_->End();
  ASSERT_TRUE(session_->Begin());
  EXPECT_FALSE(session_->RecordPoseSample("r", Sample(kEpochUs, 2), &err));
}

// What the gateway would refuse is refused here, whole, before any block depends on it.
TEST_F(PoseSessionTest, AManifestTheGatewayWouldRefuseIsRefusedWhole) {
  Build();
  const auto refused = [&](std::vector<pose::Manifest> ms, const std::string& fragment) {
    std::string err;
    EXPECT_FALSE(session_->SetPoseManifests(ms, &err)) << fragment;
    EXPECT_NE(err.find(fragment), std::string::npos) << err;
  };
  pose::Manifest m = Rig("r", 2);
  m.mesh = "robot";
  refused({m}, "manifests[0].file_type: empty while mesh is set");
  m = Rig("r", 2);
  m.file_type = "gltf";
  refused({m}, "manifests[0].mesh: empty while file_type is set");
  m = Rig("r", 2);
  m.name = "";
  refused({m}, "manifests[0].name: must not be empty");
  m = Rig("", 2);
  refused({m}, "manifests[0].manifest_id: must not be empty");
  m = Rig("r", 2);
  m.rigging_mapping_type = "boneName";
  refused({m}, "rigging_mapping_type: unsupported value 'boneName'");
  m = Rig("r", 0);
  refused({m}, "joints: must not be empty unless root is set");
  m = Rig("r", 2);
  m.joints[1].properties.emplace_back(std::string(65, 'k'), 1.0);
  refused({m}, "manifests[0].joints[1].properties[1].key: 65 bytes");
  m = Rig("r", 2);
  m.properties.emplace_back("c3d.profile", std::string("again"));
  refused({m}, "properties[2].key: 'c3d.profile' is listed twice");
  m = Rig("r", 2);
  m.properties.emplace_back("big", 1e39);
  refused({m}, "properties[2].num");
  m = Rig("r", 2);
  m.properties.emplace_back("nan", std::nan(""));
  refused({m}, "properties[2].num");
  m = Rig("r", 2);
  m.properties.emplace_back("long", std::string(1025, 'x'));
  refused({m}, "properties[2].str: 1025 bytes");
  m = Rig("r", 2);
  m.properties.emplace_back("bad", std::string("\xff"));
  refused({m}, "properties[2].str: not valid UTF-8");
  m = Rig("r", 2);
  for (int i = 0; i < 63; ++i) m.properties.emplace_back("k" + std::to_string(i), 1.0);
  refused({m}, "properties: 65 properties, at most 64");
  m = Rig("r", 2);
  m.joints[0].groups = 0;
  refused({m}, "joints[0].groups");
  refused({Rig("r", 2), Rig("r", 3)}, "manifests[1].manifest_id: 'r' is listed twice");
  pose::Manifest skin;
  skin.manifest_id = "skin";
  skin.name = "skin";
  skin.root = "skin";
  refused({skin}, "root: names the entry itself");
  skin.root = "body";
  refused({skin}, "root: 'body' has not been sent in this session");
  skin.joints = Rig("x", 1).joints;
  refused({Rig("body", 1), skin}, "joints: must be empty, the entry is a follower");
  skin.joints.clear();
  refused({Rig("x", 1), Rig("skin", 1), skin}, "listed twice");
  EXPECT_TRUE(session_->SetPoseManifests({Rig("body", 1), skin})) << "a root earlier in the call";
  refused({Rig("skin", 1)}, "an id is one or the other");
  session_->PumpPose();
  EXPECT_EQ(session_->stats().pose_manifest_sends, 2u) << "only the accepted call was sent";
  // Exactly at the caps is accepted.
  m = Rig("cap", 1);
  m.properties.clear();
  m.properties.emplace_back(std::string(64, 'k'), std::string(1024, 'v'));
  for (int i = 1; i < 64; ++i) m.properties.emplace_back("k" + std::to_string(i), 3.4028235e38);
  std::string err;
  EXPECT_TRUE(session_->SetPoseManifest(m, &err)) << err;
}

// Rule 8 and §A 15: a refused part is discarded, counted, never retried, and said with the
// gateway's own words, which name the field.
TEST_F(PoseSessionTest, ARefusedPartIsDiscardedCountedAndNotRetried) {
  Build();
  session_->PumpUploads();  // Begin's parts, before the scripted answer
  HttpResponse bad;
  bad.status = 400;
  bad.body = "manifests[0].mesh: empty while file_type is set; set both or neither";
  transport_->Script(bad);
  ASSERT_TRUE(session_->SetPoseManifest(Rig("r", 1)));
  session_->PumpPose();
  // Only the pose part is spooled: the session's opening parts went out at Begin.
  for (int i = 0; i < 5; ++i) session_->PumpUploads();
  int posts = 0;
  for (const auto& r : transport_->requests()) posts += r.url.find("/v0/pose") != std::string::npos;
  EXPECT_EQ(posts, 1);
  EXPECT_EQ(session_->stats().pose_rejections, 1u);
  EXPECT_EQ(logger_->Count(LogLevel::kError, "gateway said: manifests[0].mesh"), 1);
}

// Rule 8: a 500 is retried with the same body.
TEST_F(PoseSessionTest, AServerErrorIsRetriedWithTheSameBody) {
  cfg_.retry_min_s = 0.01;
  cfg_.retry_max_s = 0.02;
  Build();
  session_->PumpUploads();  // Begin's parts, before the scripted answer
  HttpResponse oops;
  oops.status = 500;
  transport_->Script(oops);
  ASSERT_TRUE(session_->SetPoseManifest(Rig("r", 1)));
  session_->PumpPose();
  for (int i = 0; i < 50 && spool_->Count() > 0; ++i) {
    session_->PumpUploads();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::vector<std::string> bodies;
  for (const auto& r : transport_->requests()) {
    if (r.url.find("/v0/pose") != std::string::npos) bodies.push_back(r.body);
  }
  ASSERT_EQ(bodies.size(), 2u);
  EXPECT_EQ(bodies[0], bodies[1]);
  EXPECT_EQ(session_->stats().pose_rejections, 0u);
}

// A manifest part is spooled as one, so a full spool evicts it with the events rather than with
// the blocks that cannot be read without it; a block part is spooled at the pose stream's tier.
TEST_F(PoseSessionTest, AManifestPartIsSpooledAtTheEventsTier) {
  cfg_.uploads_paused = true;
  Build();
  ASSERT_TRUE(session_->SetPoseManifest(Rig("r", 2)));
  Run("r", 2, 10.0, Now() + 2500 * 1000);
  session_->End();
  int manifests = 0;
  int blocks = 0;
  std::set<std::string> seen;
  Chunk chunk;
  while (spool_->PeekOldest(&chunk, seen)) {
    seen.insert(chunk.id);
    if (chunk.stream != Stream::kPose) continue;
    const Part p = Read(chunk.body);
    if (p.blocks.empty()) {
      ++manifests;
      EXPECT_TRUE(chunk.carries_pose_manifest) << chunk.id;
    } else {
      ++blocks;
      EXPECT_FALSE(chunk.carries_pose_manifest) << chunk.id;
    }
  }
  EXPECT_EQ(manifests, 1);
  EXPECT_GT(blocks, 0);
}

// Parts are spooled like the camera's: a process that stops before they upload leaves them
// for the next, which sends them as they were.
TEST_F(PoseSessionTest, PartsSurviveARestart) {
  cfg_.uploads_paused = true;
  Build();
  ASSERT_TRUE(session_->SetPoseManifest(Rig("r", 2)));
  Run("r", 2, 10.0, Now() + 2500 * 1000);
  session_->End();
  session_.reset();
  spool_.reset();  // its lock on the directory
  int posted = 0;
  for (const auto& r : transport_->requests()) posted += r.url.find("/v0/pose") != std::string::npos;
  ASSERT_EQ(posted, 0);
  cfg_.uploads_paused = false;
  Build(false);
  const auto parts = Parts();
  ASSERT_EQ(parts.size(), 3u) << "the manifest, the queue as it was, the open block";
  for (std::uint32_t i = 0; i < parts.size(); ++i) EXPECT_EQ(parts[i].part, i + 1);
  EXPECT_EQ(parts[1].blocks.size() + parts[2].blocks.size(), 3u);
}

// Rule 1: every closed block encoded and queued within 1 s. On the real clock and the
// worker's own ticks: a 25-joint rig at 10 Hz, as the ROS tier records one.
TEST(PoseEncodeLag, ATwentyFiveJointRigAtTenHertzIsQueuedWellWithinOneSecond) {
  const fs::path dir = fs::temp_directory_path() / ("c3d_pose_lag_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  Config cfg;
  cfg.application_key = "APPKEY";
  cfg.scene_id = "SCENEGUID";
  cfg.scene_version = "7";
  cfg.device_id = "robot01";
  cfg.spool_dir = dir.string();
  cfg.gateway = "data.example.invalid";
  cfg.uploads_paused = true;
  cfg.pose_block_interval_s = 0.5;  // seven closes in the run instead of none
  auto spool = std::make_shared<DiskSpool>(dir.string(), 64ull << 20, std::make_shared<NullLogger>());
  {
    auto session = Session::Create(cfg, std::make_shared<FakeTransport>(), spool,
                                   std::make_shared<NullLogger>());
    ASSERT_NE(session, nullptr);
    ASSERT_TRUE(session->Begin());
    ASSERT_TRUE(session->SetPoseManifest(Rig("r", 25)));
    const auto start = std::chrono::system_clock::now();
    for (int i = 0; i < 40; ++i) {
      const std::int64_t t = std::chrono::duration_cast<std::chrono::microseconds>(
                                 std::chrono::system_clock::now().time_since_epoch())
                                 .count();
      ASSERT_TRUE(session->RecordPoseSample("r", Sample(t, 25)));
      std::this_thread::sleep_until(start + std::chrono::milliseconds(100 * (i + 1)));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(700));
    const Stats s = session->stats();
    EXPECT_GE(s.pose_blocks, 7u);
    EXPECT_GT(s.pose_encode_lag_max_us, 0);
    EXPECT_LT(s.pose_encode_lag_max_us, 1'000'000);
    std::cout << "[ lag      ] 25 joints at 10 Hz, " << s.pose_blocks
              << " blocks: longest close-to-queued " << s.pose_encode_lag_max_us << " us"
              << std::endl;
    ::testing::Test::RecordProperty("pose_encode_lag_max_us",
                                    static_cast<int>(s.pose_encode_lag_max_us));
  }
  std::error_code ec;
  fs::remove_all(dir, ec);
}

// Two objects through a whole session: manifests, a follower, re-sends, a new joint set,
// timer sends, cap sends and End. Every invariant of spec §10 is checked here on the parts
// as posted; with C3D_POSE_CAPTURE set they are also written out, with the send log, for the
// formatter's upload check, which judges them independently.
TEST_F(PoseSessionTest, PoseEndToEnd) {
  Build();
  // "arm" is the largest rig the format allows a full block of, its values noisy enough that
  // its blocks barely compress, so the 5 MiB cap fires between timer sends.
  const int kArm = 240;
  const std::uint8_t prs = pose::kPosition | pose::kRotation | pose::kScale;
  ASSERT_TRUE(session_->SetPoseManifests({Rig("robot", 25), Rig("arm", kArm, prs)}));
  pose::Manifest skin;
  skin.manifest_id = "robot_skin";
  skin.name = "robot skin";
  skin.mesh = "robot_skin";
  skin.file_type = "gltf";
  skin.root = "robot";
  ASSERT_TRUE(session_->SetPoseManifest(skin));
  std::uint32_t seed = 2463534242u;
  int robot_joints = 25;
  const std::int64_t t0 = Now();
  const std::int64_t period = 1'000'000 / 120;
  for (int step = 1; Now() < t0 + 50 * kS; ++step) {
    At(t0 + step * period);
    // The arm pauses from 20 s to 35 s: a host-clock close, and the robot's small blocks go out
    // on the timer alone.
    if (Now() < t0 + 20 * kS || (Now() > t0 + 35 * kS && Now() < t0 + 45 * kS)) {
      ASSERT_TRUE(session_->RecordPoseSample("arm", Sample(kEpochUs + Now(), kArm, 200.0, &seed)));
    }
    if (step % 12 == 0) Record("robot", robot_joints);
    if (step == 12 * 120) ASSERT_TRUE(session_->SetPoseManifest(Rig("robot", 25)));
    if (step == 20 * 120) {
      robot_joints = 26;
      ASSERT_TRUE(session_->SetPoseManifest(Rig("robot", 26)));
    }
    session_->PumpPose();
  }
  session_->End();
  const auto parts = Parts();
  const auto log = SendLog();

  // Part numbers 1.. with none missing; manifests and blocks never share a part; no part
  // over 5 MiB, no block over 983,616 bytes; one send-log line per block part.
  std::map<std::string, std::vector<std::pair<std::uint32_t, std::uint32_t>>> sends;  // id -> (firstSeq, part)
  std::map<std::string, std::map<std::uint32_t, Blk>> blocks;
  std::map<std::uint32_t, std::string> reason;
  for (const std::string& line : log) {
    const auto p = line.find("\"part\": ");
    const auto r = line.find("\"reason\": \"");
    ASSERT_NE(p, std::string::npos);
    ASSERT_NE(r, std::string::npos);
    reason[static_cast<std::uint32_t>(std::stoul(line.substr(p + 8)))] =
        line.substr(r + 11, line.find('"', r + 11) - r - 11);
  }
  std::set<std::string> reasons;
  for (std::uint32_t i = 0; i < parts.size(); ++i) {
    const Part& p = parts[i];
    SCOPED_TRACE("part " + std::to_string(p.part));
    ASSERT_EQ(p.part, i + 1);
    EXPECT_EQ(p.session_id, parts[0].session_id);
    EXPECT_TRUE(p.manifests.empty() != p.blocks.empty());
    std::set<std::string> ids;
    for (const Entry& e : p.manifests) {
      EXPECT_TRUE(ids.insert(e.id).second) << "two sends of one object in a part";
      sends[e.id].emplace_back(e.first_seq, p.part);
    }
    std::uint64_t bytes = 0;
    std::int64_t start = INT64_MAX;
    for (const Blk& b : p.blocks) {
      EXPECT_LE(b.data.size(), 983'616u);
      bytes += b.data.size();
      start = std::min(start, b.start_us);
      EXPECT_TRUE(blocks[b.id].emplace(b.seq, b).second) << b.id << " seq " << b.seq << " twice";
    }
    EXPECT_LE(bytes, 5'242'880u);
    if (!p.blocks.empty()) {
      EXPECT_EQ(p.start_us, start);
      ASSERT_TRUE(reason.count(p.part)) << "no send-log line";
      reasons.insert(reason[p.part]);
    } else {
      EXPECT_EQ(p.start_us, 0);
      EXPECT_EQ(p.end_us, 0);
      EXPECT_FALSE(reason.count(p.part)) << "a manifest part writes no send-log line";
    }
  }
  EXPECT_EQ(reasons, (std::set<std::string>{"timer", "cap", "flush:end"}));
  EXPECT_FALSE(blocks.count("robot_skin"));
  // Per object: seq 0.. with no gap, resetBefore exactly at a send's firstSeq.
  for (const auto& [id, by_seq] : blocks) {
    SCOPED_TRACE(id);
    std::set<std::uint32_t> firsts;
    for (const auto& s : sends[id]) firsts.insert(s.first);
    std::uint32_t expect = 0;
    for (const auto& [seq, b] : by_seq) {
      EXPECT_EQ(seq, expect++);
      EXPECT_EQ(b.reset_before, firsts.count(seq) == 1) << seq;
    }
  }
  EXPECT_EQ(sends["robot"].size(), 3u);
  EXPECT_EQ(sends["robot_skin"].size(), 1u);
  const Stats s = session_->stats();
  EXPECT_EQ(s.pose_parts, parts.size());
  EXPECT_EQ(s.pose_samples_refused, 0u);
  EXPECT_EQ(s.pose_encode_failures, 0u);
  std::uint64_t total_blocks = 0, total_bytes = 0;
  for (const auto& [id, by_seq] : blocks) {
    total_blocks += by_seq.size();
    for (const auto& kv : by_seq) total_bytes += kv.second.data.size();
  }
  EXPECT_EQ(s.pose_blocks, total_blocks);
  EXPECT_EQ(s.pose_block_bytes, total_bytes);

  const char* capture = std::getenv("C3D_POSE_CAPTURE");
  if (capture && *capture) {
    const fs::path out(capture);
    fs::create_directories(out);
    for (const Part& p : parts) {
      std::ofstream f(out / ("part_" + std::to_string(p.part) + ".bin"), std::ios::binary);
      f << p.body;
    }
    // The checker reads a JSON list; the SDK appends lines.
    std::ofstream f(out / "send_log.json");
    f << "[\n";
    for (std::size_t i = 0; i < log.size(); ++i) f << log[i] << (i + 1 < log.size() ? ",\n" : "\n");
    f << "]\n";
    std::cout << "[ capture  ] " << parts.size() << " parts and the send log in " << capture
              << std::endl;
  }
  std::cout << "[ e2e      ] " << parts.size() << " parts, " << BlockParts(parts)
            << " block parts, " << total_blocks << " blocks, " << total_bytes << " bytes"
            << std::endl;
}
