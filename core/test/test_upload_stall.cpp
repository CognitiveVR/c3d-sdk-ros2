// A camera feed pushed through a real CurlTransport into an uplink slower than the
// feed. 7.5 Hz of ~98 KB JPEGs is ~735 KB/s, and if uploading delays cutting, a link
// below that makes every camera part bigger than the one before it, until a part can
// no longer be sent inside the request timeout and the spool grows without bound.
// The bounds a part is cut on have to hold whatever the uploader is doing.

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <mutex>
#include <random>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "c3d/session.hpp"
#include "c3d/spool.hpp"
#include "c3d/transport.hpp"
#include "throttled_http_server.hpp"

using namespace c3d;
using c3d::testing_support::ThrottledHttpServer;
namespace fs = std::filesystem;

namespace {

// Chunks and markers in memory. The worker writes each part it cuts before it cuts the next, so a
// disk write that stalls for half a second makes the part behind it half a second larger whatever
// the uploader is doing; on this spool a part's size is the cutting schedule's alone.
class MemorySpool : public ISpool {
 public:
  bool Write(const Chunk& c) override { return WriteNamed(c, nullptr); }
  bool WriteNamed(const Chunk& c, std::string* id) override {
    std::lock_guard<std::mutex> lock(mu_);
    char name[32];
    std::snprintf(name, sizeof(name), "%012llu", ++seq_);
    Chunk stored = c;
    stored.id = name;
    if (id) *id = stored.id;
    bytes_ += stored.body.size();
    chunks_.emplace(stored.id, std::move(stored));
    return true;
  }
  using ISpool::PeekOldest;
  bool PeekOldest(Chunk* out, const std::set<std::string>& skip) override {
    std::lock_guard<std::mutex> lock(mu_);
    for (const auto& kv : chunks_) {
      if (skip.count(kv.first) != 0) continue;
      *out = kv.second;
      return true;
    }
    return false;
  }
  void Remove(const std::string& id) override {
    std::lock_guard<std::mutex> lock(mu_);
    const auto it = chunks_.find(id);
    if (it == chunks_.end()) return;
    bytes_ -= it->second.body.size();
    chunks_.erase(it);
  }
  std::uint64_t TotalBytes() const override {
    std::lock_guard<std::mutex> lock(mu_);
    return bytes_;
  }
  std::size_t Count() const override {
    std::lock_guard<std::mutex> lock(mu_);
    return chunks_.size();
  }
  void WriteMarker(const SessionMarker& m) override {
    std::lock_guard<std::mutex> lock(mu_);
    markers_[m.session_id] = m;
  }
  std::vector<SessionMarker> OpenMarkers() const override {
    std::lock_guard<std::mutex> lock(mu_);
    std::vector<SessionMarker> open;
    for (const auto& kv : markers_) open.push_back(kv.second);
    return open;
  }
  void CloseMarker(const std::string& id) override {
    std::lock_guard<std::mutex> lock(mu_);
    markers_.erase(id);
  }
  bool Contains(const std::string& id) const override {
    std::lock_guard<std::mutex> lock(mu_);
    return chunks_.count(id) != 0;
  }

 private:
  mutable std::mutex mu_;
  unsigned long long seq_ = 0;
  std::uint64_t bytes_ = 0;
  std::map<std::string, Chunk> chunks_;
  std::map<std::string, SessionMarker> markers_;
};

// Records the size of every part the session spools, which is where a part's size
// is decided -- the uplink only decides how long it takes to leave.
class RecordingSpool : public ISpool {
 public:
  explicit RecordingSpool(std::shared_ptr<ISpool> inner) : inner_(std::move(inner)) {}

  bool Write(const Chunk& c) override {
    const bool ok = inner_->Write(c);
    if (ok && c.stream == Stream::kCamera) {
      std::lock_guard<std::mutex> lock(mu_);
      camera_part_bytes_.push_back(c.body.size());
    }
    return ok;
  }
  using ISpool::PeekOldest;
  bool PeekOldest(Chunk* out, const std::set<std::string>& skip) override {
    return inner_->PeekOldest(out, skip);
  }
  void Remove(const std::string& id) override { inner_->Remove(id); }
  std::uint64_t TotalBytes() const override { return inner_->TotalBytes(); }
  std::size_t Count() const override { return inner_->Count(); }
  void WriteMarker(const SessionMarker& m) override { inner_->WriteMarker(m); }
  std::vector<SessionMarker> OpenMarkers() const override { return inner_->OpenMarkers(); }
  void CloseMarker(const std::string& id) override { inner_->CloseMarker(id); }

  std::vector<std::size_t> camera_part_bytes() const {
    std::lock_guard<std::mutex> lock(mu_);
    return camera_part_bytes_;
  }

 private:
  std::shared_ptr<ISpool> inner_;
  mutable std::mutex mu_;
  std::vector<std::size_t> camera_part_bytes_;
};

struct Scale {
  double camera_hz;
  std::size_t frame_bytes;
  double flush_s;
  double uplink_bytes_per_s;
  double run_s;
};

// The suite runs a compressed version of a robot's numbers: the same feed and a
// uplink at a third of it, with 0.5 s parts so growth shows inside a few seconds.
// C3D_STALL_REPRO_ROBOT=1 runs the full-scale 2 s parts for a minute.
Scale ChosenScale() {
  const char* robot = std::getenv("C3D_STALL_REPRO_ROBOT");
  if (robot != nullptr && std::string(robot) == "1") return Scale{7.5, 98 * 1024, 2.0, 400e3, 60.0};
  return Scale{7.5, 98 * 1024, 0.5, 250e3, 8.0};
}

std::string Incompressible(std::size_t n, unsigned seed) {
  std::mt19937 rng(seed);
  std::string s(n, '\0');
  for (auto& c : s) c = static_cast<char>(rng() & 0xFF);
  return s;
}

struct Outcome {
  std::vector<std::size_t> parts;
  std::size_t largest = 0;
  Stats stats;
};

// Feeds `scale`'s camera into a session whose uplink is `scale`'s, for its run
// time, and reports every camera part the session cut.
Outcome FeedThroughASlowUplink(const Scale& scale, double flush_s, std::uint64_t flush_bytes,
                               const std::string& label) {
  Outcome outcome;
  const fs::path dir = fs::temp_directory_path() /
                       ("c3d_stall_" + std::to_string(::getpid()));
  fs::create_directories(dir);

  ThrottledHttpServer::Options server_options;
  server_options.bytes_per_s = scale.uplink_bytes_per_s;
  ThrottledHttpServer server(server_options);

  Config cfg;
  cfg.application_key = "APPKEY";
  cfg.scene_id = "SCENEGUID";
  cfg.scene_version = "2";
  cfg.device_id = "robot01";
  cfg.spool_dir = dir.string();
  cfg.protocol = "http";
  cfg.gateway = server.host();
  cfg.auto_send_interval_s = 3600.0;
  cfg.camera_flush_s = flush_s;
  cfg.camera_flush_bytes = flush_bytes;
  cfg.retry_min_s = 0.2;
  cfg.retry_max_s = 0.2;

  auto spool = std::make_shared<RecordingSpool>(std::make_shared<MemorySpool>());
  auto transport = std::make_shared<CurlTransport>(UploadTimeouts(cfg));
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
  if (!session || !session->Begin()) {
    ADD_FAILURE() << "the session did not start";
    return outcome;
  }

  const std::string frame = Incompressible(scale.frame_bytes, 7);
  const auto period = std::chrono::duration<double>(1.0 / scale.camera_hz);
  const auto started = std::chrono::steady_clock::now();
  auto next = started;
  std::uint32_t seq = 0;
  while (std::chrono::steady_clock::now() - started < std::chrono::duration<double>(scale.run_s)) {
    const auto wall_ms = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
    session->RecordCameraFrame("front", wall_ms, seq++, "camera", Pose{}, "jpeg", 640, 480,
                               frame);
    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    std::this_thread::sleep_until(next);
  }

  outcome.parts = spool->camera_part_bytes();
  outcome.stats = session->stats();
  for (std::size_t bytes : outcome.parts) outcome.largest = std::max(outcome.largest, bytes);

  const double feed_bytes_per_s = scale.camera_hz * static_cast<double>(scale.frame_bytes);
  std::cout << "[stall repro: " << label << "] feed "
            << static_cast<int>(feed_bytes_per_s / 1000) << " kB/s into a "
            << static_cast<int>(scale.uplink_bytes_per_s / 1000) << " kB/s uplink, "
            << scale.run_s << " s\n[stall repro: " << label
            << "] camera part sizes (kB), in the order they were cut:";
  for (std::size_t bytes : outcome.parts) std::cout << ' ' << bytes / 1000;
  std::cout << "\n[stall repro: " << label << "] parts=" << outcome.parts.size()
            << " largest=" << outcome.largest / 1000
            << " kB uploaded=" << outcome.stats.chunks_uploaded
            << " spooled=" << outcome.stats.spool_count
            << " upload_failures=" << outcome.stats.upload_failures << "\n";

  server.SetBytesPerSecond(0.0);  // let the closing drain run at loopback speed
  session.reset();
  std::error_code ec;
  fs::remove_all(dir, ec);
  return outcome;
}

}  // namespace

TEST(UploadStall, CameraPartsStayWithinTheirTimeBoundWhileAnUploadIsInFlight) {
  const Scale scale = ChosenScale();
  const Outcome outcome =
      FeedThroughASlowUplink(scale, scale.flush_s, 32ull * 1024 * 1024, "time bound");

  // A part holds at most flush_s of frames, plus the worker tick it waits to be
  // cut on and the one frame that crosses the bound. Anything beyond that is
  // frames the part collected because nobody was cutting.
  const double feed_bytes_per_s = scale.camera_hz * static_cast<double>(scale.frame_bytes);
  const double bound =
      (scale.flush_s + Config{}.worker_tick_s + 1.0 / scale.camera_hz) * feed_bytes_per_s +
      4096.0;
  // With a part per flush_s the run cuts about run_s / flush_s of them. One thread
  // cutting and uploading would cut one per upload instead: far fewer, far larger.
  EXPECT_GE(outcome.parts.size(), static_cast<std::size_t>(0.8 * scale.run_s / scale.flush_s))
      << "parts were cut on the uploader's schedule, not the camera_flush_s one";
  EXPECT_LE(static_cast<double>(outcome.largest), bound)
      << "a camera part outgrew camera_flush_s while an upload was in flight";
  EXPECT_EQ(outcome.stats.upload_failures, 0u)
      << "a slow-but-moving upload failed; the transport gave up on a link that works";
}

TEST(UploadStall, CameraPartsStayWithinTheirByteBoundWhileAnUploadIsInFlight) {
  Scale scale = ChosenScale();
  scale.run_s = std::min(scale.run_s, 6.0);
  const std::uint64_t flush_bytes = 300 * 1000;
  const Outcome outcome = FeedThroughASlowUplink(scale, 3600.0, flush_bytes, "byte bound");

  // The part is cut on the first tick at or past the bound, so it may carry the
  // frame that crossed it and whatever else arrived within that tick.
  const double feed_bytes_per_s = scale.camera_hz * static_cast<double>(scale.frame_bytes);
  const double bound = static_cast<double>(flush_bytes) + static_cast<double>(scale.frame_bytes) +
                       Config{}.worker_tick_s * feed_bytes_per_s + 4096.0;
  ASSERT_GE(outcome.parts.size(), 3u);
  EXPECT_LE(static_cast<double>(outcome.largest), bound)
      << "a camera part outgrew camera_flush_bytes while an upload was in flight";
}

// Stall detection legitimately lets one upload run for a minute and the ceiling
// for five, so a session shutting down must not wait one out: the chunk is still
// in the spool for the next process.
TEST(UploadStall, ShuttingDownDoesNotWaitOutAnUploadInFlight) {
  const fs::path dir = fs::temp_directory_path() /
                       ("c3d_stall_shutdown_" + std::to_string(::getpid()));
  fs::remove_all(dir);
  fs::create_directories(dir);
  ThrottledHttpServer::Options server_options;
  server_options.stall_after_bytes = 1;
  ThrottledHttpServer server(server_options);

  Config cfg;
  cfg.application_key = "APPKEY";
  cfg.scene_id = "SCENEGUID";
  cfg.scene_version = "2";
  cfg.device_id = "robot01";
  cfg.spool_dir = dir.string();
  cfg.protocol = "http";
  cfg.gateway = server.host();
  cfg.http_low_speed_time_s = 100;
  cfg.http_timeout_s = 200;

  auto make_spool = [&] {
    return std::make_shared<DiskSpool>(dir.string(), 64ull * 1024 * 1024,
                                       std::make_shared<NullLogger>());
  };
  {
    Config paused = cfg;
    paused.uploads_paused = true;
    auto session = Session::Create(paused, std::make_shared<CurlTransport>(UploadTimeouts(paused)),
                                   make_spool(), std::make_shared<NullLogger>());
    ASSERT_TRUE(session->Begin());
    session->RecordEvent("work", 1000.0, Vec3{});
    session->End();
  }
  auto spool = make_spool();
  const std::size_t spooled = spool->Count();
  ASSERT_GT(spooled, 0u);

  auto session = Session::Create(cfg, std::make_shared<CurlTransport>(UploadTimeouts(cfg)), spool,
                                 std::make_shared<NullLogger>());
  std::this_thread::sleep_for(std::chrono::milliseconds(500));  // the uploader is now stuck
  const auto start = std::chrono::steady_clock::now();
  session.reset();
  const double took =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

  EXPECT_LT(took, 5.0) << "the session waited " << took << " s for a stalled upload to finish";
  spool.reset();
  EXPECT_EQ(make_spool()->Count(), spooled) << "the interrupted chunk must stay spooled";
  EXPECT_EQ(server.completed(), 0u);
  std::error_code ec;
  fs::remove_all(dir, ec);
}
