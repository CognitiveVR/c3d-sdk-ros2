// A camera feed pushed through a real CurlTransport into an uplink slower than the
// feed. 7.5 Hz of ~98 KB frames is ~735 KB/s, and if uploading delays cutting, a link
// below that makes every camera part bigger than the one before it, until a part can
// no longer be sent inside the request timeout and the spool grows without bound.
// The bounds a part is cut on have to hold whatever the uploader is doing.

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
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
// is decided -- the uplink only decides how long it takes to leave -- and when.
class RecordingSpool : public ISpool {
 public:
  RecordingSpool(std::shared_ptr<ISpool> inner, Stream stream)
      : inner_(std::move(inner)), stream_(stream) {}

  bool Write(const Chunk& c) override {
    const bool ok = inner_->Write(c);
    if (ok && c.stream == stream_) {
      std::lock_guard<std::mutex> lock(mu_);
      part_bytes_.push_back(c.body.size());
      part_times_.push_back(std::chrono::steady_clock::now());
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

  std::vector<std::size_t> part_bytes() const {
    std::lock_guard<std::mutex> lock(mu_);
    return part_bytes_;
  }
  std::vector<std::chrono::steady_clock::time_point> part_times() const {
    std::lock_guard<std::mutex> lock(mu_);
    return part_times_;
  }

 private:
  std::shared_ptr<ISpool> inner_;
  const Stream stream_;
  mutable std::mutex mu_;
  std::vector<std::size_t> part_bytes_;
  std::vector<std::chrono::steady_clock::time_point> part_times_;
};

struct Scale {
  double camera_hz;
  std::size_t frame_bytes;
  double chunk_s;
  double uplink_bytes_per_s;
  double run_s;
};

// The suite runs a compressed version of a robot's numbers: the same feed and a
// uplink at a third of it, with 0.5 s chunks so growth shows inside a few seconds.
// C3D_STALL_REPRO_ROBOT=1 runs the full-scale 2 s chunks for a minute.
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

// The camera's encoder makes a keyframe every this many frames, so a chunk is never shorter
// than chunk_s.
std::uint32_t FramesPerChunk(const Scale& scale) {
  return static_cast<std::uint32_t>(std::ceil(scale.chunk_s * scale.camera_hz));
}

struct Outcome {
  std::vector<std::size_t> parts;
  std::size_t largest = 0;
  // The longest any part waited between being closed and being spooled, counting a part
  // still unspooled when the run ended as waiting until then.
  double longest_wait_s = 0.0;
  Stats stats;
};

std::uint64_t PartsClosed(const Stats& stats, Stream stream) {
  return stream == Stream::kCamera ? stats.camera_chunks : stats.lidar_blocks;
}

// Feeds `scale`'s sensor into a session whose uplink is `scale`'s, for its run time, and
// reports every part of `stream` the session spooled: a camera's access units of frame_bytes
// with a keyframe every FramesPerChunk, or a depth sensor's clouds of frame_bytes.
Outcome FeedThroughASlowUplink(const Scale& scale, Stream stream, const std::string& label) {
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
  cfg.retry_min_s = 0.2;
  cfg.retry_max_s = 0.2;

  auto spool = std::make_shared<RecordingSpool>(std::make_shared<MemorySpool>(), stream);
  auto transport = std::make_shared<CurlTransport>(UploadTimeouts(cfg));
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
  if (!session || !session->Begin()) {
    ADD_FAILURE() << "the session did not start";
    return outcome;
  }

  const std::string frame = Incompressible(scale.frame_bytes, 7);
  VideoChunkContext context;
  context.frame_id = "camera";
  context.width = 640;
  context.height = 480;
  context.nominal_fps = static_cast<float>(scale.camera_hz);
  EncodedPointCloud cloud;
  cloud.ok = true;
  cloud.point_count = 1000;
  cloud.data = frame;
  const std::uint32_t frames_per_chunk = FramesPerChunk(scale);
  const auto period = std::chrono::duration<double>(1.0 / scale.camera_hz);
  const auto started = std::chrono::steady_clock::now();
  auto next = started;
  std::uint32_t seq = 0;
  // When each part closed: inside the Record* call that closed it.
  std::vector<std::chrono::steady_clock::time_point> closed_at;
  while (std::chrono::steady_clock::now() - started < std::chrono::duration<double>(scale.run_s)) {
    const auto wall_ms = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
    const std::uint64_t closed_before = PartsClosed(session->stats(), stream);
    if (stream == Stream::kCamera) {
      SensorEntry entry;
      entry.time_ms = wall_ms;
      entry.seq = seq;
      session->RecordCameraAccessUnit("front", entry, context, frame,
                                      seq % frames_per_chunk == 0);
    } else {
      session->RecordPointCloud("depth", wall_ms, seq, "depth", Pose{}, cloud);
    }
    const auto now = std::chrono::steady_clock::now();
    for (std::uint64_t n = PartsClosed(session->stats(), stream); n > closed_before; --n) {
      closed_at.push_back(now);
    }
    ++seq;
    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    std::this_thread::sleep_until(next);
  }

  const auto ended = std::chrono::steady_clock::now();
  outcome.parts = spool->part_bytes();
  outcome.stats = session->stats();
  for (std::size_t bytes : outcome.parts) outcome.largest = std::max(outcome.largest, bytes);
  // Parts are spooled in the order they closed, so the i-th spooled is the i-th closed.
  const auto spooled_at = spool->part_times();
  for (std::size_t i = 0; i < closed_at.size(); ++i) {
    const auto until = i < spooled_at.size() ? spooled_at[i] : ended;
    outcome.longest_wait_s = std::max(
        outcome.longest_wait_s, std::chrono::duration<double>(until - closed_at[i]).count());
  }

  const double feed_bytes_per_s = scale.camera_hz * static_cast<double>(scale.frame_bytes);
  std::cout << "[stall repro: " << label << "] feed "
            << static_cast<int>(feed_bytes_per_s / 1000) << " kB/s into a "
            << static_cast<int>(scale.uplink_bytes_per_s / 1000) << " kB/s uplink, "
            << scale.run_s << " s\n[stall repro: " << label << "] " << StreamName(stream)
            << " part sizes (kB), in the order they were cut:";
  for (std::size_t bytes : outcome.parts) std::cout << ' ' << bytes / 1000;
  std::cout << "\n[stall repro: " << label << "] parts=" << outcome.parts.size()
            << " largest=" << outcome.largest / 1000
            << " kB uploaded=" << outcome.stats.chunks_uploaded
            << " spooled=" << outcome.stats.spool_count
            << " upload_failures=" << outcome.stats.upload_failures
            << " longest_wait_s=" << outcome.longest_wait_s << "\n";

  server.SetBytesPerSecond(0.0);  // let the closing drain run at loopback speed
  session.reset();
  std::error_code ec;
  fs::remove_all(dir, ec);
  return outcome;
}

}  // namespace

TEST(UploadStall, CameraPartsStayWithinTheirTimeBoundWhileAnUploadIsInFlight) {
  const Scale scale = ChosenScale();
  const Outcome outcome = FeedThroughASlowUplink(scale, Stream::kCamera, "time bound");

  // A part holds one chunk: the frames from one keyframe to the next. Anything
  // beyond that is frames the part collected because nobody was cutting.
  const double bound =
      FramesPerChunk(scale) * static_cast<double>(scale.frame_bytes) + 4096.0;
  // With a part per chunk the run spools about run_s / chunk of them.
  const double chunk_s = FramesPerChunk(scale) / scale.camera_hz;
  EXPECT_GE(outcome.parts.size(), static_cast<std::size_t>(0.8 * scale.run_s / chunk_s))
      << "parts were not closed on their keyframes";
  EXPECT_LE(static_cast<double>(outcome.largest), bound)
      << "a camera part outgrew its keyframe interval while an upload was in flight";
  // A chunk closes on the recording thread whatever the worker is doing, so its size
  // cannot show a worker stuck behind an upload; how long the chunk then waits for the
  // spool does. The worker spools it on its next tick, and one thread spooling and
  // uploading would leave it waiting for a whole upload, seconds on this uplink.
  EXPECT_LT(outcome.longest_wait_s, 1.0)
      << "a closed chunk waited " << outcome.longest_wait_s << " s to reach the spool";
  EXPECT_EQ(outcome.stats.upload_failures, 0u)
      << "a slow-but-moving upload failed; the transport gave up on a link that works";
}

TEST(UploadStall, CloudObjectsStayWithinTheirByteBoundWhileAnUploadIsInFlight) {
  Scale scale = ChosenScale();
  scale.run_s = std::min(scale.run_s, 6.0);
  // Clouds large enough for several objects to close within the run.
  scale.frame_bytes = 300 * 1000;
  const Outcome outcome = FeedThroughASlowUplink(scale, Stream::kLidar, "byte bound");

  // An object closes before the cloud that would take it past the cap joins it.
  const double bound = static_cast<double>(kCloudObjectMaxBytes) + 4096.0;
  ASSERT_GE(outcome.parts.size(), 3u);
  EXPECT_LE(static_cast<double>(outcome.largest), bound)
      << "a cloud object outgrew kCloudObjectMaxBytes while an upload was in flight";
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
