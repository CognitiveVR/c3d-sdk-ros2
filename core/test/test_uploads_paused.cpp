// uploads_paused: record and spool as normal, send nothing, and let a later
// process with uploads on drain the spool into the original session ids. That is
// the whole long-session procedure for a robot whose uplink cannot carry its data
// rate, so each step of it is pinned here: nothing leaves while paused, parts are
// still cut on schedule, and a process that never begins a session drains it all.
//
// Also here: the abnormalEnd recovery writes for a paused run that lost power,
// which must take an events part number the dead session never used.

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <unistd.h>

#include <capnp/serialize-packed.h>
#include <kj/io.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <thread>

#include "c3d/session.hpp"
#include "c3d/spool.hpp"
#include "camera_payload.capnp.h"
#include "fake_transport.hpp"
#include "process_death.hpp"

using namespace c3d;
using c3d::testing_support::RunAndDieWithoutEnding;
using json = nlohmann::json;
namespace fs = std::filesystem;

namespace {

Config MakeConfig(const std::string& dir) {
  Config cfg;
  cfg.application_key = "APPKEY";
  cfg.scene_id = "SCENEGUID";
  cfg.scene_version = "2";
  cfg.device_id = "robot01";
  cfg.spool_dir = dir;
  cfg.auto_send_interval_s = 3600.0;
  cfg.worker_tick_s = 0.01;
  cfg.retry_min_s = 0.02;
  cfg.retry_max_s = 0.05;
  cfg.marker_heartbeat_s = 0.01;
  cfg.orphan_stale_after_s = 0.05;
  return cfg;
}

double NowSeconds() {
  return std::chrono::duration<double>(
             std::chrono::system_clock::now().time_since_epoch()).count();
}

std::shared_ptr<DiskSpool> OpenSpool(const fs::path& dir) {
  return std::make_shared<DiskSpool>(dir.string(), 64ull * 1024 * 1024,
                                     std::make_shared<NullLogger>());
}

bool WaitFor(const std::function<bool()>& done, double seconds = 3.0) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (std::chrono::steady_clock::now() < deadline) {
    if (done()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return done();
}

bool EndsWith(const std::string& s, const std::string& suffix) {
  return s.size() >= suffix.size() &&
         s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// The session a request is filed under, read the way the gateway reads it: from
// the JSON envelope, or from the packed Cap'n Proto header of a camera part.
std::string SessionOf(const FakeTransport::Request& r) {
  if (EndsWith(r.url, "/v0/camera")) {
    kj::ArrayPtr<const kj::byte> bytes(reinterpret_cast<const kj::byte*>(r.body.data()),
                                       r.body.size());
    kj::ArrayInputStream stream(bytes);
    capnp::PackedMessageReader reader(stream);
    return reader.getRoot<cvr::capnp::CameraPayload>().getSessionId().cStr();
  }
  return json::parse(r.body).value("sessionid", "");
}

// Every events payload for one session, by part number.
std::multimap<std::uint64_t, json> EventsParts(const FakeTransport& transport,
                                               const std::string& session_id) {
  std::multimap<std::uint64_t, json> parts;
  for (const auto& r : transport.requests()) {
    if (r.url.find("/events/") == std::string::npos) continue;
    const json body = json::parse(r.body);
    if (body.value("sessionid", "") != session_id) continue;
    parts.emplace(body["part"].get<std::uint64_t>(), body);
  }
  return parts;
}

class UploadsPausedTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() /
           ("c3d_paused_" + std::to_string(::getpid()) + "_" + std::to_string(counter_++));
    fs::create_directories(dir_);
  }
  void TearDown() override {
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }
  fs::path dir_;
  static int counter_;
};
int UploadsPausedTest::counter_ = 0;

}  // namespace

TEST_F(UploadsPausedTest, NothingIsSentWhilePausedAndALaterConfigureOnlyProcessDrainsIt) {
  std::string recorded_id;
  std::size_t spooled = 0;
  {
    auto transport = std::make_shared<FakeTransport>();
    auto spool = OpenSpool(dir_);
    auto cfg = MakeConfig(dir_.string());
    cfg.uploads_paused = true;
    cfg.camera_flush_s = 0.05;
    auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
    ASSERT_NE(session, nullptr);
    EXPECT_TRUE(session->uploads_paused());
    ASSERT_TRUE(session->Begin());
    recorded_id = session->session_id();

    // Camera frames across several time bounds: parts must still be cut on
    // schedule with nothing uploading, or a paused run is one enormous part. Nothing
    // here flushes, so only the worker's time bound can cut each one.
    const auto camera_chunks = [&] {
      std::size_t n = 0;
      for (const auto& e : fs::directory_iterator(dir_ / "chunks")) {
        if (EndsWith(e.path().stem().string(), "-camera")) ++n;
      }
      return n;
    };
    for (int i = 0; i < 3; ++i) {
      session->RecordGaze(1000.0 + i, Vec3{}, Quat{});
      session->RecordCameraFrame("front", 1000000ull + i, i, "cam", Pose{}, "jpeg", 2, 2,
                                 std::string(512, 'x'));
      ASSERT_TRUE(WaitFor([&] { return camera_chunks() > static_cast<std::size_t>(i); }))
          << "camera part " << i << " was not cut on camera_flush_s while paused";
    }
    session->RecordEvent("work", 1010.0, Vec3{});
    session->PumpUploads();  // an explicit drain must not send either
    session->End("orderly");

    EXPECT_EQ(transport->count(), 0u) << "a paused session uploaded something";
    EXPECT_TRUE(session->stats().uploads_paused);
    spooled = spool->Count();
    std::size_t camera_parts = 0;
    for (const auto& e : fs::directory_iterator(dir_ / "chunks")) {
      if (EndsWith(e.path().stem().string(), "-camera")) ++camera_parts;
    }
    EXPECT_GE(camera_parts, 3u) << "camera parts were not cut on camera_flush_s while paused";
  }
  ASSERT_GT(spooled, 3u);

  // The drain: a fresh process, uploads on, that recovers and never begins a
  // session -- what a node configured and never activated does.
  auto transport = std::make_shared<FakeTransport>();
  auto spool = OpenSpool(dir_);
  auto session = Session::Create(MakeConfig(dir_.string()), transport, spool,
                                 std::make_shared<NullLogger>());
  EXPECT_EQ(session->RecoverOrphanedSessions(), 0) << "the recorded session ended orderly";
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; }))
      << spool->Count() << " chunk(s) never left without a Begin";

  EXPECT_EQ(transport->count(), spooled);
  std::size_t camera_parts = 0;
  for (const auto& r : transport->requests()) {
    EXPECT_EQ(SessionOf(r), recorded_id)
        << "a drained chunk is not filed under the session that recorded it: " << r.url;
    if (EndsWith(r.url, "/v0/camera")) ++camera_parts;
  }
  EXPECT_GE(camera_parts, 3u);
  EXPECT_FALSE(EventsParts(*transport, recorded_id).empty());
  EXPECT_TRUE(session->session_id().empty()) << "draining must not open a session";
}

TEST_F(UploadsPausedTest, PausingAtRuntimeStopsUploadsAndResumingDrainsTheBacklog) {
  auto transport = std::make_shared<FakeTransport>();
  auto spool = OpenSpool(dir_);
  auto session = Session::Create(MakeConfig(dir_.string()), transport, spool,
                                 std::make_shared<NullLogger>());
  ASSERT_TRUE(session->Begin());
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; }));
  const std::size_t before = transport->count();
  ASSERT_GT(before, 0u) << "the session start never uploaded; nothing to compare against";

  session->SetUploadsPaused(true);
  for (int i = 0; i < 5; ++i) {
    session->RecordEvent("while_paused", 1000.0 + i, Vec3{});
    session->Flush();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  EXPECT_EQ(transport->count(), before) << "the upload thread ignored the pause";
  EXPECT_EQ(spool->Count(), 5u);

  session->SetUploadsPaused(false);
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; })) << "resuming did not drain";
  EXPECT_EQ(transport->count(), before + 5);
  session->End();
}

TEST_F(UploadsPausedTest, APausedRunThatLostPowerIsClosedAndDrainedByTheNextConfigure) {
  // A robot losing power, not a slow one: the child process dies with its session
  // open, and nothing of it outlives the death.
  const std::string dead_id = RunAndDieWithoutEnding([&] {
    auto transport = std::make_shared<FakeTransport>();
    auto spool = OpenSpool(dir_);
    auto cfg = MakeConfig(dir_.string());
    cfg.uploads_paused = true;
    cfg.marker_heartbeat_s = 3600.0;
    cfg.orphan_stale_after_s = 7200.0;
    auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
    if (!session || !session->Begin()) return std::string();
    for (int i = 0; i < 3; ++i) {
      session->RecordEvent("work", 1000.0 + i, Vec3{});
      session->Flush();
    }
    const std::string id =
        transport->count() == 0 ? session->session_id() : std::string("uploaded-while-paused");
    session.release();  // the child exits next; nothing may end the session first
    return id;
  });
  ASSERT_FALSE(dead_id.empty()) << "the child never began its session";
  ASSERT_NE(dead_id, "uploaded-while-paused");
  // Age the marker past the budget it declared, keeping everything else it wrote.
  {
    auto spool = OpenSpool(dir_);
    const auto markers = spool->OpenMarkers();
    ASSERT_EQ(markers.size(), 1u);
    SessionMarker aged = markers.front();
    aged.last_seen = NowSeconds() - 30000.0;
    spool->WriteMarker(aged);
  }

  auto transport = std::make_shared<FakeTransport>();
  auto spool = OpenSpool(dir_);
  auto session = Session::Create(MakeConfig(dir_.string()), transport, spool,
                                 std::make_shared<NullLogger>());
  EXPECT_EQ(session->RecoverOrphanedSessions(), 1);
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; }));

  const auto parts = EventsParts(*transport, dead_id);
  // sessionStart, three flushes, and the recovered end.
  ASSERT_EQ(parts.size(), 5u);
  std::set<std::uint64_t> distinct;
  for (const auto& kv : parts) distinct.insert(kv.first);
  EXPECT_EQ(distinct.size(), parts.size())
      << "an events part number was sent twice for one session; the gateway keys events on "
         "part and index, so the second overwrites the first";
  const json& last = parts.rbegin()->second;
  ASSERT_EQ(last["data"].size(), 1u);
  EXPECT_EQ(last["data"][0]["name"], "c3d.sessionEnd");
  EXPECT_EQ(last["data"][0]["properties"]["Reason"], "abnormalEnd");
  EXPECT_EQ(parts.rbegin()->first, 5u) << "the recovered end takes the next part, not a new base";
}

TEST_F(UploadsPausedTest, AMarkerThatDoesNotNameTheNextPartNeverGetsPartOne) {
  // A marker that does not record the next events part. Part 1 is always that
  // session's sessionStart, so falling back to the part base would overwrite it.
  fs::create_directories(dir_ / "sessions");
  json j;
  j["session_id"] = "1790000000_robot01";
  j["scene_id"] = "SCENEGUID";
  j["scene_version"] = "2";
  j["device_id"] = "robot01";
  j["started_at"] = 1790000000.0;
  j["last_seen"] = 1790000100.0;
  std::ofstream(dir_ / "sessions" / "1790000000_robot01.c3dsession") << j.dump();

  auto transport = std::make_shared<FakeTransport>();
  auto spool = OpenSpool(dir_);
  auto session = Session::Create(MakeConfig(dir_.string()), transport, spool,
                                 std::make_shared<NullLogger>());
  EXPECT_EQ(session->RecoverOrphanedSessions(), 1);
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; }));
  const auto parts = EventsParts(*transport, "1790000000_robot01");
  ASSERT_EQ(parts.size(), 1u);
  EXPECT_GT(parts.begin()->first, 1000u)
      << "a recovered end with no declared next part must take one no session reaches";
}
