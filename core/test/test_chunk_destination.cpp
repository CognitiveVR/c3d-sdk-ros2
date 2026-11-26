// Where a spooled chunk is sent. The URL is rebuilt from the configuration of the process doing
// the upload, and a chunk recorded for another host is kept and never sent: the application key
// goes only where this process was configured to send it, and a file in the spool directory --
// left by a run against another environment, a capture stub, or another local user -- is not
// configuration.

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "c3d/session.hpp"
#include "c3d/spool.hpp"
#include "fake_transport.hpp"
#include "process_death.hpp"
#include "serialize.hpp"

using namespace c3d;
using json = nlohmann::json;
namespace fs = std::filesystem;

namespace {

const char* kKey = "APPKEY-0123456789";

Config MakeConfig(const std::string& dir) {
  Config cfg;
  cfg.application_key = kKey;
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

class CapturingLogger : public ILogger {
 public:
  void Log(LogLevel, const std::string& message) override {
    std::lock_guard<std::mutex> lock(mu_);
    messages_.push_back(message);
  }
  int Count(const std::string& fragment) const {
    std::lock_guard<std::mutex> lock(mu_);
    int n = 0;
    for (const auto& m : messages_) n += m.find(fragment) != std::string::npos ? 1 : 0;
    return n;
  }

 private:
  mutable std::mutex mu_;
  std::vector<std::string> messages_;
};

bool WaitFor(const std::function<bool()>& done, double seconds = 3.0) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (std::chrono::steady_clock::now() < deadline) {
    if (done()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return done();
}

bool CarriesTheKey(const FakeTransport::Request& r) {
  for (const auto& h : r.headers) {
    if (h.find(kKey) != std::string::npos) return true;
  }
  return false;
}

class ChunkDestinationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() /
           ("c3d_dest_" + std::to_string(::getpid()) + "_" + std::to_string(counter_++));
    fs::create_directories(dir_);
  }
  void TearDown() override {
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }
  // Writes a chunk as a spool would have, recorded for `url`.
  void Plant(const std::string& url, const std::string& body, Stream stream = Stream::kEvents,
             const std::string& scene_id = "", const std::string& scene_version = "") {
    DiskSpool planter(dir_.string(), 64ull << 20, std::make_shared<NullLogger>());
    Chunk planted;
    planted.url = url;
    planted.scene_id = scene_id;
    planted.scene_version = scene_version;
    planted.body = body;
    planted.session_id = "1_planted";
    planted.stream = stream;
    ASSERT_TRUE(planter.Write(planted));
  }
  fs::path dir_;
  static int counter_;
};
int ChunkDestinationTest::counter_ = 0;

}  // namespace

TEST_F(ChunkDestinationTest, AChunkRecordedForAnotherHostIsKeptAndNeverSentTheKey) {
  Plant("https://collector.attacker.example/v0/events/SCENEGUID?version=2",
        R"({"planted":true})", Stream::kEvents, "SCENEGUID", "2");
  auto transport = std::make_shared<FakeTransport>();
  auto log = std::make_shared<CapturingLogger>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull << 20,
                                           std::make_shared<NullLogger>());
  auto session = Session::Create(MakeConfig(dir_.string()), transport, spool, log);
  ASSERT_TRUE(session->Begin());
  session->RecordEvent("behind_it", 1000.0, Vec3{});
  session->Flush();

  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 1; }))
      << "the session's own chunks are stuck behind the held one";
  for (const auto& r : transport->requests()) {
    EXPECT_EQ(r.url.find("attacker.example"), std::string::npos) << r.url;
    EXPECT_EQ(r.body.find("planted"), std::string::npos)
        << "the planted chunk was sent, with the key, to " << r.url;
    EXPECT_TRUE(CarriesTheKey(r));
    EXPECT_EQ(r.url.rfind("https://data.cognitive3d.com/", 0), 0u) << r.url;
  }
  EXPECT_EQ(session->stats().chunks_held, 1u);
  EXPECT_EQ(session->stats().chunks_discarded, 0u) << "held means kept";
  EXPECT_EQ(log->Count("collector.attacker.example"), 1)
      << "said once, naming the host it was recorded for";
  session->End();
}

TEST_F(ChunkDestinationTest, EveryChunkForAHeldHostIsSaidOnceNotOncePerPass) {
  for (int i = 0; i < 3; ++i) {
    Plant("http://127.0.0.1:8787/v0/events/SCENEGUID?version=2",
          "{\"stub\":" + std::to_string(i) + "}", Stream::kEvents, "SCENEGUID", "2");
  }
  auto transport = std::make_shared<FakeTransport>();
  auto log = std::make_shared<CapturingLogger>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull << 20,
                                           std::make_shared<NullLogger>());
  auto session = Session::Create(MakeConfig(dir_.string()), transport, spool, log);
  ASSERT_TRUE(WaitFor([&] { return session->stats().chunks_held == 3; }));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(transport->count(), 0u);
  EXPECT_EQ(spool->Count(), 3u);
  EXPECT_EQ(log->Count("127.0.0.1:8787"), 1);
}

// The host matches, so the chunk goes -- but to a URL built from this process's configuration,
// whatever scheme or form the recorded one had. The scene it was recorded under is kept, because
// that is a fact about the data, not about where to send it.
TEST_F(ChunkDestinationTest, TheUploadUrlIsBuiltFromTheCurrentConfiguration) {
  Plant("http://DATA.cognitive3d.com/v7/events/OTHERSCENE?version=9&extra=1", R"({"a":1})",
        Stream::kEvents, "OTHERSCENE", "9");
  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull << 20,
                                           std::make_shared<NullLogger>());
  auto session = Session::Create(MakeConfig(dir_.string()), transport, spool,
                                 std::make_shared<NullLogger>());
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; }));
  ASSERT_EQ(transport->count(), 1u);
  EXPECT_EQ(transport->requests()[0].url,
            "https://data.cognitive3d.com/v0/events/OTHERSCENE?version=9");
}

TEST_F(ChunkDestinationTest, ABinaryChunkGoesToItsSessionKeyedRoute) {
  Plant("https://data.cognitive3d.com/v0/camera", std::string("\x00\xFF\x01", 3), Stream::kCamera);
  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull << 20,
                                           std::make_shared<NullLogger>());
  auto session = Session::Create(MakeConfig(dir_.string()), transport, spool,
                                 std::make_shared<NullLogger>());
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; }));
  ASSERT_EQ(transport->count(), 1u);
  EXPECT_EQ(transport->requests()[0].url, "https://data.cognitive3d.com/v0/camera");
}

// A recorded destination that cannot be read, or a JSON chunk with no usable scene version, is
// not guessed at: it is held like a foreign host's.
TEST_F(ChunkDestinationTest, AChunkThatCannotBeRebuiltIsHeldNotGuessed) {
  Plant("not a url", R"({"a":1})", Stream::kEvents, "SCENEGUID", "2");
  Plant("https://data.cognitive3d.com@evil.example/v0/events/S?version=2", R"({"b":1})",
        Stream::kEvents, "S", "2");
  Plant("https://data.cognitive3d.com/v0/events/S?version=2#@evil", R"({"c":1})");
  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull << 20,
                                           std::make_shared<NullLogger>());
  auto session = Session::Create(MakeConfig(dir_.string()), transport, spool,
                                 std::make_shared<NullLogger>());
  ASSERT_TRUE(WaitFor([&] { return session->stats().chunks_held == 3; }))
      << session->stats().chunks_held;
  EXPECT_EQ(transport->count(), 0u);
}

// A session recorded against one stack crashes, and the robot is then configured for another.
// Its data is held for its own host, and so is the abnormalEnd recovery writes for it: sent
// here, it would open a stray session on this stack under this key, and the original stack's
// session would stay open even after its data drained. A process configured for that host
// closes it there.
TEST_F(ChunkDestinationTest, ACrashedSessionIsClosedOnTheHostItWasRecordedFor) {
  const std::string other = "data.staging.example.test";
  const std::string dead = c3d::testing_support::RunAndDieWithoutEnding([&] {
    Config cfg = MakeConfig(dir_.string());
    cfg.gateway = other;
    cfg.uploads_paused = true;
    cfg.marker_heartbeat_s = 3600.0;
    cfg.orphan_stale_after_s = 7200.0;
    auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull << 20,
                                             std::make_shared<NullLogger>());
    auto session = Session::Create(cfg, std::make_shared<FakeTransport>(), spool,
                                   std::make_shared<NullLogger>());
    if (!session || !session->Begin()) return std::string();
    session->RecordEvent("work", 1000.0, Vec3{});
    session->Flush();
    const std::string id = session->session_id();
    session.release();
    return id;
  });
  ASSERT_FALSE(dead.empty());
  // Silent for far longer than the budget the marker declares.
  for (const auto& entry : fs::directory_iterator(dir_ / "sessions")) {
    json marker;
    std::ifstream(entry.path()) >> marker;
    EXPECT_EQ(marker.value("gateway", ""), other) << "the marker did not record its host";
    marker["last_seen"] = 1000.0;
    marker["max_silence_s"] = 1.0;
    std::ofstream(entry.path()) << marker.dump();
  }

  {
    auto transport = std::make_shared<FakeTransport>();
    auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull << 20,
                                             std::make_shared<NullLogger>());
    auto log = std::make_shared<CapturingLogger>();
    auto session = Session::Create(MakeConfig(dir_.string()), transport, spool, log);
    EXPECT_EQ(session->RecoverOrphanedSessions(), 1);
    EXPECT_TRUE(spool->OpenMarkers().empty());
    // Its start, its properties, its work and its end.
    ASSERT_TRUE(WaitFor([&] { return session->stats().chunks_held == 4; }))
        << session->stats().chunks_held << " held";
    for (const auto& r : transport->requests()) {
      EXPECT_EQ(r.body.find(dead), std::string::npos)
          << "a request for the crashed session went to " << r.url;
    }
    EXPECT_EQ(log->Count("recorded against " + other), 1);
  }

  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull << 20,
                                           std::make_shared<NullLogger>());
  Config there = MakeConfig(dir_.string());
  there.gateway = other;
  auto session = Session::Create(there, transport, spool, std::make_shared<NullLogger>());
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; }));
  bool closed_there = false;
  for (const auto& r : transport->requests()) {
    if (r.body.find("abnormalEnd") == std::string::npos) continue;
    closed_there = r.url.rfind("https://" + other + "/", 0) == 0;
  }
  EXPECT_TRUE(closed_there) << "the crashed session was never closed on its own host";
}

// Every file under the spool directory, as text.
std::vector<std::string> SpoolFiles(const fs::path& dir) {
  std::vector<std::string> out;
  std::error_code ec;
  for (const auto& e : fs::recursive_directory_iterator(dir, ec)) {
    if (!e.is_regular_file()) continue;
    std::ifstream in(e.path(), std::ios::binary);
    out.emplace_back((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  }
  return out;
}

bool CarriesKey(const FakeTransport::Request& r, const std::string& key) {
  for (const auto& h : r.headers) {
    if (h.find(key) != std::string::npos) return true;
  }
  return false;
}

const char* kKeyOfProjectA = "KEY-OF-PROJECT-A-0123456789";
const char* kKeyOfProjectB = "KEY-OF-PROJECT-B-9876543210";

// Records one session under `key`, for `scene`: its start and properties, an event, a camera
// frame, its end and its closing properties. Uploads paused, so all of it stays spooled.
std::string RecordABacklog(const fs::path& dir, const std::string& key, const std::string& scene) {
  Config cfg = MakeConfig(dir.string());
  cfg.application_key = key;
  cfg.scene_id = scene;
  cfg.uploads_paused = true;
  auto spool = std::make_shared<DiskSpool>(dir.string(), 64ull << 20,
                                           std::make_shared<NullLogger>());
  auto session = Session::Create(cfg, std::make_shared<FakeTransport>(), spool,
                                 std::make_shared<NullLogger>());
  if (!session || !session->Begin()) return {};
  const std::string id = session->session_id();
  session->RecordEvent("work", 1000.0, Vec3{});
  session->RecordCameraAccessUnit("front", SensorEntry{1000000ull, 0, Pose{}}, VideoChunkContext{},
                                  "ab", true);
  session->End();
  return id;
}

// A process that crashed recording under `key`, for `scene`: its start and properties, an event
// and a camera frame are spooled, and its marker is left open and long silent. On a device of
// its own, so its session id never meets RecordABacklog's within one second.
std::string CrashWhileRecording(const fs::path& dir, const std::string& key,
                                const std::string& scene) {
  const std::string dead = c3d::testing_support::RunAndDieWithoutEnding([&] {
    Config cfg = MakeConfig(dir.string());
    cfg.application_key = key;
    cfg.scene_id = scene;
    cfg.device_id = "robot03";
    cfg.uploads_paused = true;
    cfg.marker_heartbeat_s = 3600.0;
    cfg.orphan_stale_after_s = 7200.0;
    auto spool = std::make_shared<DiskSpool>(dir.string(), 64ull << 20,
                                             std::make_shared<NullLogger>());
    auto session = Session::Create(cfg, std::make_shared<FakeTransport>(), spool,
                                   std::make_shared<NullLogger>());
    if (!session || !session->Begin()) return std::string();
    session->RecordEvent("work", 1000.0, Vec3{});
    session->RecordCameraAccessUnit("front", SensorEntry{1000000ull, 0, Pose{}},
                                    VideoChunkContext{}, "ab", true);
    session->Flush();
    const std::string id = session->session_id();
    session.release();
    return id;
  });
  std::error_code ec;
  for (const auto& entry : fs::directory_iterator(dir / "sessions", ec)) {
    json marker;
    std::ifstream(entry.path()) >> marker;
    if (marker.value("key_fingerprint", "").empty()) return {};  // the marker did not record it
    marker["last_seen"] = 1000.0;
    marker["max_silence_s"] = 1.0;
    std::ofstream(entry.path()) << marker.dump();
  }
  return dead;
}

// A robot moved to another project, with a backlog still spooled for the first: another key AND
// another scene. Posted under the new key, the first project's scene id would land in a new,
// empty scene of the second project, and the first would never get its data. The backlog is
// held, and a process with the key it was recorded under sends it.
TEST_F(ChunkDestinationTest, ABacklogMovedToAnotherProjectIsHeldUntilAProcessWithItsKey) {
  const std::string recorded = RecordABacklog(dir_, kKeyOfProjectA, "SCENEGUID");
  ASSERT_FALSE(recorded.empty());
  for (const auto& text : SpoolFiles(dir_)) {
    EXPECT_EQ(text.find(kKeyOfProjectA), std::string::npos) << "the spool holds the key itself";
  }

  {
    Config b = MakeConfig(dir_.string());
    b.application_key = kKeyOfProjectB;
    b.scene_id = "PROJECT_B_SCENE";
    // A session id names its second and its device, so another device keeps this process's
    // own session apart from the backlog's within the second the test runs in.
    b.device_id = "robot02";
    auto transport = std::make_shared<FakeTransport>();
    auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull << 20,
                                             std::make_shared<NullLogger>());
    auto log = std::make_shared<CapturingLogger>();
    auto session = Session::Create(b, transport, spool, log);
    // Its start and opening properties, its camera part, its work and end, and its closing
    // properties.
    ASSERT_TRUE(WaitFor([&] { return session->stats().chunks_held == 5; }))
        << session->stats().chunks_held << " held";
    ASSERT_TRUE(session->Begin());
    session->RecordEvent("project_b_work", 2000.0, Vec3{});
    session->End();
    ASSERT_TRUE(WaitFor([&] { return spool->Count() == 5; }))
        << "this process's own chunks are stuck behind the held ones";
    for (const auto& r : transport->requests()) {
      EXPECT_EQ(r.body.find(recorded), std::string::npos)
          << "project A's session was posted under project B's key to " << r.url;
      EXPECT_EQ(r.url.find("/SCENEGUID"), std::string::npos) << r.url;
    }
    EXPECT_EQ(session->stats().chunks_discarded, 0u) << "held means kept";
    EXPECT_EQ(log->Count("another application key"), 1) << "said once, not per chunk or pass";
  }

  Config again = MakeConfig(dir_.string());
  again.application_key = kKeyOfProjectA;
  again.scene_id = "PROJECT_B_SCENE";
  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull << 20,
                                           std::make_shared<NullLogger>());
  auto session = Session::Create(again, transport, spool, std::make_shared<NullLogger>());
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; }));
  ASSERT_EQ(transport->count(), 5u);
  for (const auto& r : transport->requests()) {
    EXPECT_TRUE(CarriesKey(r, kKeyOfProjectA)) << r.url;
    // A camera part is packed Cap'n Proto, whose text is not contiguous.
    if (r.url.find("/camera") != std::string::npos) continue;
    EXPECT_NE(r.body.find(recorded), std::string::npos) << r.url;
  }
}

// A session that crashed recording for one project, on a robot then moved to another. The
// abnormalEnd recovery writes for it is held with the rest of its data, under its own key.
TEST_F(ChunkDestinationTest, ACrashedSessionMovedToAnotherProjectIsClosedUnderItsOwnKey) {
  const std::string dead = CrashWhileRecording(dir_, kKeyOfProjectA, "SCENEGUID");
  ASSERT_FALSE(dead.empty());
  {
    Config b = MakeConfig(dir_.string());
    b.application_key = kKeyOfProjectB;
    b.scene_id = "PROJECT_B_SCENE";
    auto transport = std::make_shared<FakeTransport>();
    auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull << 20,
                                             std::make_shared<NullLogger>());
    auto log = std::make_shared<CapturingLogger>();
    auto session = Session::Create(b, transport, spool, log);
    EXPECT_EQ(session->RecoverOrphanedSessions(), 1);
    EXPECT_TRUE(spool->OpenMarkers().empty());
    // Its start, its properties, its work, its camera part and its end.
    ASSERT_TRUE(WaitFor([&] { return session->stats().chunks_held == 5; }))
        << session->stats().chunks_held << " held";
    EXPECT_EQ(transport->count(), 0u) << "a request for the crashed session went out";
    EXPECT_EQ(log->Count("than this process has"), 1) << "the hold is said once";
    EXPECT_EQ(log->Count("its end is kept with its data for a process that has that key"), 1);
  }

  Config there = MakeConfig(dir_.string());
  there.application_key = kKeyOfProjectA;
  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull << 20,
                                           std::make_shared<NullLogger>());
  auto session = Session::Create(there, transport, spool, std::make_shared<NullLogger>());
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; }));
  bool closed = false;
  for (const auto& r : transport->requests()) {
    if (r.body.find("abnormalEnd") != std::string::npos) closed = CarriesKey(r, kKeyOfProjectA);
  }
  EXPECT_TRUE(closed) << "the crashed session was never closed under its own key";
}

// A key rotated within one project: another key, the same scene. That is the project's own data,
// and the old key may already be revoked, so the backlog -- a crashed session's abnormalEnd and
// its camera part included -- is sent under the current key, to the scene it was recorded for.
TEST_F(ChunkDestinationTest, AKeyRotatedWithinAProjectSendsTheBacklogUnderTheNewKey) {
  const std::string recorded = RecordABacklog(dir_, kKeyOfProjectA, "SCENEGUID");
  ASSERT_FALSE(recorded.empty());
  const std::string dead = CrashWhileRecording(dir_, kKeyOfProjectA, "SCENEGUID");
  ASSERT_FALSE(dead.empty());
  ASSERT_NE(dead, recorded);

  Config rotated = MakeConfig(dir_.string());
  rotated.application_key = kKeyOfProjectB;
  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull << 20,
                                           std::make_shared<NullLogger>());
  auto log = std::make_shared<CapturingLogger>();
  auto session = Session::Create(rotated, transport, spool, log);
  EXPECT_EQ(session->RecoverOrphanedSessions(), 1);
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; }))
      << spool->Count() << " chunk(s) left, " << session->stats().chunks_held << " held";
  EXPECT_EQ(session->stats().chunks_held, 0u);
  EXPECT_EQ(log->Count("another application key"), 0);
  // Five of the orderly session, five of the crashed one.
  ASSERT_EQ(transport->count(), 10u);
  bool closed = false;
  for (const auto& r : transport->requests()) {
    EXPECT_TRUE(CarriesKey(r, kKeyOfProjectB)) << r.url;
    if (r.url.find("/camera") == std::string::npos) {
      EXPECT_NE(r.url.find("/SCENEGUID"), std::string::npos) << r.url;
    }
    if (r.body.find("abnormalEnd") != std::string::npos) closed = r.body.find(dead) != std::string::npos;
  }
  EXPECT_TRUE(closed) << "the crashed session was not closed under the rotated key";
}

// Another scene under the same key is the same project: the backlog is sent, each chunk to the
// scene it was recorded for.
TEST_F(ChunkDestinationTest, AnotherSceneUnderTheSameKeyIsSentToTheSceneItWasRecordedFor) {
  const std::string recorded = RecordABacklog(dir_, kKey, "SCENE_ONE");
  ASSERT_FALSE(recorded.empty());
  Config next = MakeConfig(dir_.string());
  next.scene_id = "SCENE_TWO";
  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull << 20,
                                           std::make_shared<NullLogger>());
  auto session = Session::Create(next, transport, spool, std::make_shared<NullLogger>());
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; }))
      << spool->Count() << " chunk(s) left, " << session->stats().chunks_held << " held";
  EXPECT_EQ(session->stats().chunks_held, 0u);
  ASSERT_EQ(transport->count(), 5u);
  for (const auto& r : transport->requests()) {
    if (r.url.find("/camera") != std::string::npos) continue;
    EXPECT_NE(r.url.find("/SCENE_ONE?"), std::string::npos) << r.url;
  }
}

TEST(ParseChunkUrl, ReadsBackWhatBuildUrlWrote) {
  std::string host, scene, version;
  ASSERT_TRUE(ParseChunkUrl(BuildUrl("https", "Data.Example.Test:8443", "0", Stream::kGaze, "SCN",
                                     "12"),
                            &host, &scene, &version));
  EXPECT_EQ(host, "data.example.test:8443");
  EXPECT_EQ(scene, "SCN");
  EXPECT_EQ(version, "12");
  ASSERT_TRUE(ParseChunkUrl(BuildUrl("http", "h", "0", Stream::kLidar, "ignored", "1"), &host,
                            &scene, &version));
  EXPECT_EQ(host, "h");
  EXPECT_TRUE(scene.empty());
  EXPECT_TRUE(version.empty());
  for (const char* bad : {"", "h/v0/events", "https://", "https://h", "https://h?x",
                          "https://u@h/v0/events/S?version=1", "https://h/x"}) {
    EXPECT_FALSE(ParseChunkUrl(bad, &host, nullptr, nullptr)) << bad;
  }
}
