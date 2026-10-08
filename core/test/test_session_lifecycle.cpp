// How a session opens and closes, and what the threads recording into it can wait behind.
//
// End() spools the end, removes the marker and returns: a ROS service, a lifecycle transition
// and SIGINT all call it on the executor thread, and none of them may wait out an upload. The
// destructor gives the tail a bounded chance. A Record* call takes only the lock the batches
// live under, which is never held across serialisation or a disk write.

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <sys/wait.h>
#include <unistd.h>

#include <csignal>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <limits>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "c3d/session.hpp"
#include "c3d/spool.hpp"
#include "c3d/wall_clock.hpp"
#include "fake_transport.hpp"
#include "throttled_http_server.hpp"

using namespace c3d;
using c3d::testing_support::ThrottledHttpServer;
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

double SecondsSince(std::chrono::steady_clock::time_point t) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count();
}

bool WaitFor(const std::function<bool()>& done, double seconds = 3.0) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (std::chrono::steady_clock::now() < deadline) {
    if (done()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return done();
}

class CapturingLogger : public ILogger {
 public:
  void Log(LogLevel, const std::string& message) override {
    std::lock_guard<std::mutex> lock(mu_);
    messages_.push_back(message);
    times_.push_back(std::chrono::steady_clock::now());
  }
  bool Contains(const std::string& fragment) const {
    return TimeOf(fragment).has_value();
  }
  // The first line containing `fragment`, or empty.
  std::string Line(const std::string& fragment) const {
    std::lock_guard<std::mutex> lock(mu_);
    for (const auto& m : messages_) {
      if (m.find(fragment) != std::string::npos) return m;
    }
    return {};
  }
  // When the first line containing `fragment` was logged.
  std::optional<std::chrono::steady_clock::time_point> TimeOf(const std::string& fragment) const {
    std::lock_guard<std::mutex> lock(mu_);
    for (std::size_t i = 0; i < messages_.size(); ++i) {
      if (messages_[i].find(fragment) != std::string::npos) return times_[i];
    }
    return std::nullopt;
  }

 private:
  mutable std::mutex mu_;
  std::vector<std::string> messages_;
  std::vector<std::chrono::steady_clock::time_point> times_;
};

// A spool whose chunk writes can be held at the door, or refused, on demand.
class GatedSpool : public ISpool {
 public:
  explicit GatedSpool(std::shared_ptr<ISpool> inner) : inner_(std::move(inner)) {}

  bool Write(const Chunk& c) override {
    if (refuse_.load()) return false;
    if (throw_.load()) throw std::runtime_error("the consumer's spool threw");
    if (hold_next_.exchange(false)) {
      held_.store(true);
      while (!release_.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    {
      std::lock_guard<std::mutex> lock(mu_);
      written_.push_back(c.body);
      operations_.push_back("chunk " + c.session_id);
    }
    const std::size_t before = inner_->Count();
    const bool written = inner_->Write(c);
    if (written && c.body.find("c3d.sessionEnd") != std::string::npos &&
        hold_behind_end_.exchange(false)) {
      HoldNextWrite();
    }
    // Returns only once the uploader has already taken the end away again.
    if (written && return_after_end_sent_.load() &&
        c.body.find("c3d.sessionEnd") != std::string::npos) {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
      while (inner_->Count() > before && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    return written;
  }
  using ISpool::PeekOldest;
  bool PeekOldest(Chunk* out, const std::set<std::string>& skip) override {
    return inner_->PeekOldest(out, skip);
  }
  void Remove(const std::string& id) override { inner_->Remove(id); }
  std::uint64_t TotalBytes() const override { return inner_->TotalBytes(); }
  std::size_t Count() const override { return inner_->Count(); }
  void WriteMarker(const SessionMarker& m) override {
    {
      std::lock_guard<std::mutex> lock(mu_);
      operations_.push_back("marker " + m.session_id);
      markers_.push_back(m);
    }
    inner_->WriteMarker(m);
  }
  std::vector<SessionMarker> OpenMarkers() const override { return inner_->OpenMarkers(); }
  void CloseMarker(const std::string& id) override { inner_->CloseMarker(id); }

  std::vector<std::string> operations() const {
    std::lock_guard<std::mutex> lock(mu_);
    return operations_;
  }
  std::vector<SessionMarker> markers() const {
    std::lock_guard<std::mutex> lock(mu_);
    return markers_;
  }
  void HoldNextWrite() {
    release_.store(false);
    held_.store(false);
    hold_next_.store(true);
  }
  // Holds the first write after the one carrying a session's end: End() part-way through the
  // parts behind its end.
  void HoldTheWriteBehindTheEnd() { hold_behind_end_.store(true); }
  bool Holding() const { return held_.load(); }
  void Release() { release_.store(true); }
  void Refuse(bool refuse) { refuse_.store(refuse); }
  void Throw(bool do_throw) { throw_.store(do_throw); }
  void ReturnAfterTheEndIsSent() { return_after_end_sent_.store(true); }
  std::vector<std::string> written() const {
    std::lock_guard<std::mutex> lock(mu_);
    return written_;
  }

 private:
  std::shared_ptr<ISpool> inner_;
  std::atomic<bool> hold_next_{false};
  std::atomic<bool> hold_behind_end_{false};
  std::atomic<bool> held_{false};
  std::atomic<bool> release_{false};
  std::atomic<bool> refuse_{false};
  std::atomic<bool> throw_{false};
  std::atomic<bool> return_after_end_sent_{false};
  mutable std::mutex mu_;
  std::vector<std::string> written_;
  std::vector<std::string> operations_;
  std::vector<SessionMarker> markers_;
};

double MarkerLastSeen(const fs::path& dir, const std::string& session_id) {
  std::ifstream in(dir / "sessions" / (session_id + ".c3dsession"));
  if (!in) return -1.0;
  json j;
  try {
    in >> j;
  } catch (...) {
    return -1.0;
  }
  return j.value("last_seen", -1.0);
}

bool MarkerExists(const fs::path& dir, const std::string& session_id) {
  return fs::exists(dir / "sessions" / (session_id + ".c3dsession"));
}

int CountInChunks(const fs::path& dir, const std::string& needle) {
  int n = 0;
  std::error_code ec;
  for (const auto& e : fs::directory_iterator(dir / "chunks", ec)) {
    std::ifstream in(e.path());
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    for (std::size_t at = text.find(needle); at != std::string::npos; at = text.find(needle, at + 1)) {
      ++n;
    }
  }
  return n;
}

class SessionLifecycleTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() /
           ("c3d_life_" + std::to_string(::getpid()) + "_" + std::to_string(counter_++));
    fs::create_directories(dir_);
  }
  void TearDown() override {
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }
  std::shared_ptr<DiskSpool> Spool() {
    return std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                       std::make_shared<NullLogger>());
  }
  fs::path dir_;
  static int counter_;
};
int SessionLifecycleTest::counter_ = 0;

}  // namespace

// ---------------------------------------------------------------------------
// End() never waits for an upload
// ---------------------------------------------------------------------------

TEST_F(SessionLifecycleTest, EndReturnsAtOnceAndTheBacklogUploadsBehindIt) {
  auto transport = std::make_shared<FakeTransport>();
  auto spool = Spool();
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
  ASSERT_TRUE(session->Begin());
  const std::string id = session->session_id();
  for (int i = 0; i < 10; ++i) {
    session->RecordEvent("queued", 1000.0 + i, Vec3{});
    session->Flush();
  }
  transport->Stall();  // an upload that will not finish until this test says so
  session->SetUploadsPaused(false);
  ASSERT_TRUE(transport->WaitUntilStalled(std::chrono::seconds(5))) << "the uploader never started";

  // End() writes its own last parts, which under disk load takes what the disk takes; the bound
  // is a watchdog, since an End() that waited for the stuck upload would never return.
  auto ended = std::async(std::launch::async, [&] { session->End("orderly"); });
  const bool returned = ended.wait_for(std::chrono::seconds(5)) == std::future_status::ready;
  const bool upload_still_stuck = transport->stalled_posts() == 1;
  const bool marker_gone = !MarkerExists(dir_, id);
  const std::size_t backlog = spool->Count();
  transport->Unstall();
  ended.wait();
  EXPECT_TRUE(returned) << "End() waited for an upload";
  EXPECT_TRUE(upload_still_stuck) << "the upload finished first; the test proves nothing";
  EXPECT_TRUE(marker_gone) << "the marker must be closed when End() returns";
  EXPECT_GT(backlog, 1u) << "the backlog was drained synchronously after all";

  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; }, 5.0))
      << "the backlog never uploaded after End()";
  int ends = 0;
  for (const auto& r : transport->requests()) {
    if (r.body.find("c3d.sessionEnd") != std::string::npos) ++ends;
  }
  EXPECT_EQ(ends, 1);
}

// SIGINT destroys the node and the node destroys the session. Waiting out a stalled POST there
// would take the 60 s stall window, and launch sends SIGKILL at 10 s.
TEST_F(SessionLifecycleTest, DestroyingAnActiveSessionBehindAStalledUploadIsBounded) {
  ThrottledHttpServer::Options options;
  options.stall_after_bytes = 1;
  ThrottledHttpServer server(options);
  Config cfg = MakeConfig(dir_.string());
  cfg.protocol = "http";
  cfg.gateway = server.host();
  cfg.http_low_speed_time_s = 60;
  cfg.http_timeout_s = 120;
  cfg.retry_min_s = 60.0;
  cfg.retry_max_s = 60.0;
  cfg.marker_heartbeat_s = 5.0;
  cfg.orphan_stale_after_s = 30.0;
  cfg.shutdown_drain_s = 0.5;
  auto spool = Spool();
  auto log = std::make_shared<CapturingLogger>();
  auto session = Session::Create(cfg, std::make_shared<CurlTransport>(UploadTimeouts(cfg)), spool,
                                 log);
  ASSERT_TRUE(session->Begin());
  const std::string id = session->session_id();
  std::this_thread::sleep_for(std::chrono::milliseconds(300));  // the uploader is now stuck

  session.reset();
  const auto destroyed = std::chrono::steady_clock::now();
  // Measured from the end of the End() the destructor runs: writing the last parts takes what
  // the disk takes. What follows is the drain's 0.5 s and curl noticing the interrupt, which
  // it polls about once a second -- against a stall that would otherwise hold for 60 s.
  const auto ended_at = log->TimeOf("ended (destructor)");
  ASSERT_TRUE(ended_at.has_value()) << "the destructor never ended the session";
  const double after_end = std::chrono::duration<double>(destroyed - *ended_at).count();
  EXPECT_LT(after_end, cfg.shutdown_drain_s + 1.5) << "destruction waited " << after_end
                                                   << " s after the session ended";
  EXPECT_FALSE(MarkerExists(dir_, id)) << "an orderly end must not be left for recovery";
  EXPECT_GT(spool->Count(), 0u) << "what did not go must stay spooled for the next start";
  EXPECT_EQ(server.completed(), 0u);
}

TEST_F(SessionLifecycleTest, TheDestructorGivesTheTailItsChanceToUpload) {
  auto transport = std::make_shared<FakeTransport>();
  auto spool = Spool();
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  cfg.shutdown_drain_s = 10.0;  // a ceiling: the wait ends as soon as the tail has gone
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
  ASSERT_TRUE(session->Begin());
  for (int i = 0; i < 5; ++i) {
    session->RecordEvent("tail", 1000.0 + i, Vec3{});
    session->Flush();
  }
  transport->SetPostDelay(std::chrono::milliseconds(40));
  session->SetUploadsPaused(false);
  session.reset();
  EXPECT_EQ(spool->Count(), 0u) << "the session's last parts were abandoned at destruction";
  bool end_sent = false;
  for (const auto& r : transport->requests()) {
    if (r.body.find("c3d.sessionEnd") != std::string::npos) end_sent = true;
  }
  EXPECT_TRUE(end_sent);
}

// The destructor waits only while something can go: not offline and backing off, not with
// uploads paused, not with the credentials refused. Each time it stops it says what it leaves
// behind and why, since the process may never start again. End() runs first, so the time
// measured is the destructor's own and not the disk's for the last parts.
TEST_F(SessionLifecycleTest, TheDestructorDoesNotWaitWhenNothingCanBeSent) {
  struct Case {
    const char* name;
    const char* reason;
  };
  for (const Case c : {Case{"backing off", "uploads are backing off after a failure"},
                       Case{"uploads paused", "uploads are paused"},
                       Case{"credentials refused", "the credentials are refused"}}) {
    SCOPED_TRACE(c.name);
    const std::string name = c.name;
    const fs::path sub = dir_ / name;
    fs::create_directories(sub);
    auto transport = std::make_shared<FakeTransport>();
    Config cfg = MakeConfig(sub.string());
    cfg.retry_min_s = 30.0;
    cfg.retry_max_s = 30.0;
    cfg.unauthorized_attempts = 1;
    cfg.shutdown_drain_s = 5.0;
    cfg.uploads_paused = name == "uploads paused";
    if (name == "backing off") transport->SetOffline(true);
    if (name == "credentials refused") {
      HttpResponse refused;
      refused.status = 401;
      transport->SetResponder([refused](const FakeTransport::Request&) { return refused; });
    }
    auto log = std::make_shared<CapturingLogger>();
    auto session = Session::Create(
        cfg, transport,
        std::make_shared<DiskSpool>(sub.string(), 64ull << 20, std::make_shared<NullLogger>()),
        log);
    ASSERT_TRUE(session->Begin());
    if (name == "backing off") ASSERT_TRUE(WaitFor([&] { return session->stats().backing_off; }));
    if (name == "credentials refused") {
      ASSERT_TRUE(WaitFor([&] { return session->stats().uploads_auth_paused; }));
    }
    const std::string id = session->session_id();
    session->End("orderly");
    const auto start = std::chrono::steady_clock::now();
    session.reset();
    EXPECT_LT(SecondsSince(start), 1.0) << "shutdown waited for uploads that cannot go";
    EXPECT_TRUE(log->Contains(std::string("chunk(s) still spooled in ") + sub.string() + " (" +
                              c.reason + ")"));
    EXPECT_TRUE(log->Contains("the end of session " + id + " among them"));
    EXPECT_TRUE(log->Contains("next start"));
  }
}

TEST_F(SessionLifecycleTest, TheDestructorDoesNotWaitForChunksItWillNeverSend) {
  {
    DiskSpool planter(dir_.string(), 64ull << 20, std::make_shared<NullLogger>());
    Chunk foreign;
    foreign.url = "http://127.0.0.1:8787/v0/events/SCENEGUID?version=2";
    foreign.scene_id = "SCENEGUID";
    foreign.scene_version = "2";
    foreign.body = "{}";
    foreign.stream = Stream::kEvents;
    ASSERT_TRUE(planter.Write(foreign));
  }
  Config cfg = MakeConfig(dir_.string());
  cfg.shutdown_drain_s = 5.0;
  auto log = std::make_shared<CapturingLogger>();
  auto session = Session::Create(cfg, std::make_shared<FakeTransport>(), Spool(), log);
  // A session that ends and is delivered, end and all, leaves only the held chunk.
  ASSERT_TRUE(session->Begin());
  session->End("orderly");
  ASSERT_TRUE(WaitFor([&] { return session->stats().chunks_held == 1 &&
                                   session->stats().spool_count == 1; }));
  const auto start = std::chrono::steady_clock::now();
  session.reset();
  EXPECT_LT(SecondsSince(start), 1.0) << "shutdown waited for a chunk held for another host";
  EXPECT_TRUE(log->Contains("1 chunk(s) still spooled in " + dir_.string() +
                            " (none of them can be sent by this process now)"));
  EXPECT_TRUE(log->Contains("none of them the end of a session this process ran"))
      << "the delivered end was reported as still spooled";
}

// Chunks of a stream this build does not send, left by another build between chunks it does
// send, are discarded unsent and cost the drain nothing: every sendable chunk goes before the
// process does, and none is reported as one this process cannot send.
TEST_F(SessionLifecycleTest, TheDestructorDrainsPastChunksOfAStreamItDoesNotSend) {
  const std::string gateway = MakeConfig(dir_.string()).gateway;
  constexpr int kPlanted = 20;
  {
    DiskSpool planter(dir_.string(), 64ull << 20, std::make_shared<NullLogger>());
    for (int i = 0; i < kPlanted; ++i) {
      Chunk chunk;
      chunk.url = "https://" + gateway + "/v0/events/SCENEGUID?version=2";
      chunk.scene_id = "SCENEGUID";
      chunk.scene_version = "2";
      chunk.body = "{\"planted\":" + std::to_string(i) + "}";
      chunk.stream = Stream::kEvents;
      ASSERT_TRUE(planter.Write(chunk));
    }
  }
  // Each sorts between two of the chunks above, as `<seq>-foreignStream` follows `<seq>-events`.
  for (int i = 0; i < kPlanted; ++i) {
    const std::string seq = std::to_string(i);
    std::ofstream(dir_ / "chunks" /
                  (std::string(12 - seq.size(), '0') + seq + "-foreignStream.c3dchunk"))
        << R"({"schema":4,"url":"https://)" << gateway
        << R"(/v0/foreignStream/SCENEGUID?version=2","scene_id":"SCENEGUID","scene_version":"2",)"
        << R"("key_fingerprint":"","session_id":"other","stream":"foreignStream",)"
        << R"("created_at":1.0,"body":"{\"foreignRecord\":[]}"})";
  }
  auto transport = std::make_shared<FakeTransport>();
  auto spool = Spool();
  ASSERT_EQ(spool->Count(), 2u * kPlanted);
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  cfg.shutdown_drain_s = 10.0;  // a ceiling: the wait ends as soon as the tail has gone
  auto log = std::make_shared<CapturingLogger>();
  auto session = Session::Create(cfg, transport, spool, log);
  ASSERT_TRUE(session->Begin());
  session->End("orderly");
  session->SetUploadsPaused(false);
  session.reset();
  EXPECT_EQ(spool->Count(), 0u) << "sendable chunks were left behind";
  EXPECT_FALSE(log->Contains("none of them can be sent by this process now"))
      << log->Line("still spooled");
  int planted_sent = 0;
  for (const auto& request : transport->requests()) {
    if (request.body.find("\"planted\"") != std::string::npos) ++planted_sent;
    EXPECT_EQ(request.body.find("foreignRecord"), std::string::npos)
        << "a chunk of a stream this build does not send was sent";
  }
  EXPECT_EQ(planted_sent, kPlanted);
}

// An end the uploader sends the moment it is spooled, before the write that spooled it has
// returned, is delivered: the destructor must not report it left behind.
TEST_F(SessionLifecycleTest, AnEndSentTheMomentItIsSpooledIsNotReportedLeftBehind) {
  {
    DiskSpool planter(dir_.string(), 64ull << 20, std::make_shared<NullLogger>());
    Chunk foreign;
    foreign.url = "http://127.0.0.1:8787/v0/events/SCENEGUID?version=2";
    foreign.scene_id = "SCENEGUID";
    foreign.scene_version = "2";
    foreign.body = "{}";
    ASSERT_TRUE(planter.Write(foreign));  // something left behind to report
  }
  auto spool = std::make_shared<GatedSpool>(Spool());
  Config cfg = MakeConfig(dir_.string());
  auto log = std::make_shared<CapturingLogger>();
  auto session = Session::Create(cfg, std::make_shared<FakeTransport>(), spool, log);
  ASSERT_TRUE(session->Begin());
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 1; })) << "the session's start never went";
  spool->ReturnAfterTheEndIsSent();
  session->End("orderly");
  ASSERT_EQ(CountInChunks(dir_, "c3d.sessionEnd"), 0)
      << "the end was not sent while its write was held";
  session.reset();
  EXPECT_TRUE(log->Contains("none of them the end of a session this process ran"))
      << "a delivered end was reported as still spooled";
}

// An end the spool refused was never spooled: its marker is left for recovery, and the
// destructor must not count it among what it leaves to upload.
TEST_F(SessionLifecycleTest, AnEndTheSpoolRefusedIsNotReportedAmongWhatIsLeft) {
  auto spool = std::make_shared<GatedSpool>(Spool());
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  cfg.marker_heartbeat_s = 5.0;
  cfg.orphan_stale_after_s = 30.0;  // no recovery before the destructor runs
  auto log = std::make_shared<CapturingLogger>();
  auto session = Session::Create(cfg, std::make_shared<FakeTransport>(), spool, log);
  ASSERT_TRUE(session->Begin());
  spool->Refuse(true);
  session->End("orderly");
  ASSERT_TRUE(log->Contains("could not be spooled"));
  session.reset();
  EXPECT_TRUE(log->Contains("none of them the end of a session this process ran"))
      << "an end that never reached the spool was reported as waiting in it";
}

// Likewise an abnormalEnd recovery could not spool: the orphan keeps its marker, and nothing of
// it is waiting to upload.
TEST_F(SessionLifecycleTest, ARecoveredEndTheSpoolRefusedIsNotReportedAmongWhatIsLeft) {
  {
    fs::create_directories(dir_ / "sessions");
    json j;
    j["session_id"] = "1790000000_robot01";
    j["scene_id"] = "SCENEGUID";
    j["scene_version"] = "2";
    j["device_id"] = "robot01";
    j["started_at"] = 1790000000.0;
    j["last_seen"] = 1790000100.0;
    std::ofstream(dir_ / "sessions" / "1790000000_robot01.c3dsession") << j.dump();
  }
  auto spool = std::make_shared<GatedSpool>(Spool());
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  cfg.marker_heartbeat_s = 5.0;
  cfg.orphan_stale_after_s = 30.0;  // the recovery recheck comes after the destructor
  auto log = std::make_shared<CapturingLogger>();
  auto session = Session::Create(cfg, std::make_shared<FakeTransport>(), spool, log);
  session->Begin();  // something of its own to leave behind
  session->End("orderly");
  spool->Refuse(true);
  EXPECT_EQ(session->RecoverOrphanedSessions(), 0);
  session.reset();
  const std::string left = log->Line("still spooled");
  EXPECT_NE(left.find("the end of session"), std::string::npos) << left;  // its own
  EXPECT_EQ(left.find("1790000000_robot01"), std::string::npos)
      << "an abnormalEnd that never reached the spool was reported as waiting in it: " << left;
}

// Likewise one whose write threw, in a consumer-supplied spool.
TEST_F(SessionLifecycleTest, ARecoveredEndWhoseWriteThrewIsNotReportedAmongWhatIsLeft) {
  {
    fs::create_directories(dir_ / "sessions");
    json j;
    j["session_id"] = "1790000000_robot01";
    j["scene_id"] = "SCENEGUID";
    j["scene_version"] = "2";
    j["device_id"] = "robot01";
    j["started_at"] = 1790000000.0;
    j["last_seen"] = 1790000100.0;
    std::ofstream(dir_ / "sessions" / "1790000000_robot01.c3dsession") << j.dump();
  }
  auto spool = std::make_shared<GatedSpool>(Spool());
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  cfg.marker_heartbeat_s = 5.0;
  cfg.orphan_stale_after_s = 30.0;
  auto log = std::make_shared<CapturingLogger>();
  auto session = Session::Create(cfg, std::make_shared<FakeTransport>(), spool, log);
  session->Begin();
  session->End("orderly");
  spool->Throw(true);
  EXPECT_ANY_THROW(session->RecoverOrphanedSessions());
  spool->Throw(false);
  session.reset();
  const std::string left = log->Line("still spooled");
  EXPECT_NE(left.find("the end of session"), std::string::npos) << left;
  EXPECT_EQ(left.find("1790000000_robot01"), std::string::npos)
      << "an abnormalEnd whose write threw was reported as waiting in the spool: " << left;
}

// An end the spool cap evicted is gone: the destructor names only the ends still spooled.
TEST_F(SessionLifecycleTest, AnEndTheCapEvictedIsNotReportedAmongWhatIsLeft) {
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 4 * 1024, std::make_shared<NullLogger>());
  auto log = std::make_shared<CapturingLogger>();
  std::string first;
  std::string last;
  {
    auto session = Session::Create(cfg, std::make_shared<FakeTransport>(), spool, log);
    ASSERT_TRUE(session->Begin());
    first = session->session_id();
    session->End();
    // Events evict events: later sessions' events push the first one's end out.
    for (int s = 0; s < 40; ++s) {
      ASSERT_TRUE(session->Begin());
      last = session->session_id();
      for (int i = 0; i < 5; ++i) session->RecordEvent("e" + std::to_string(i), 1.8e9 + i, Vec3{});
      session->End();
    }
    ASSERT_GT(spool->evicted(), 0u);
    ASSERT_EQ(CountInChunks(dir_, "\"session_id\":\"" + first + "\""), 0)
        << "a chunk of the first session outlived the cap; the test proves nothing";
    ASSERT_GT(CountInChunks(dir_, "\"session_id\":\"" + last + "\""), 0);
  }
  const std::string left = log->Line("shutting down with");
  ASSERT_FALSE(left.empty());
  EXPECT_EQ(left.find(first), std::string::npos)
      << "an evicted end was reported as still spooled: " << left;
  EXPECT_NE(left.find(last), std::string::npos) << "the last session's end is spooled: " << left;
}

namespace {
// What a consumer's logger may do on any line: ask the Session it logs for about itself.
class CallingBackLogger : public ILogger {
 public:
  void Log(LogLevel, const std::string&) override {
    if (Session* session = session_.load()) {
      (void)session->session_id();
      (void)session->active();
      (void)session->stats();
    }
  }
  void CallBackInto(Session* session) { session_.store(session); }

 private:
  std::atomic<Session*> session_{nullptr};
};
}  // namespace

// Every line is logged with the Session's lock and the spool's released, so a logger that calls
// back into the Session cannot deadlock it. Each step below reaches one such line. In a child,
// under a watchdog, so a deadlock is a result naming the step it happened in. The child never
// returns into the test runner: whatever it throws is a result too.
TEST_F(SessionLifecycleTest, ALoggerThatCallsBackIntoTheSessionNeverDeadlocksIt) {
  const fs::path progress = dir_.parent_path() / (dir_.filename().string() + ".progress");
  const pid_t child = ::fork();
  if (child == 0) {
    auto step = [&](const char* name) { std::ofstream(progress, std::ios::app) << name << "\n"; };
    try {
      auto log = std::make_shared<CallingBackLogger>();
      auto transport = std::make_shared<FakeTransport>();
      HttpResponse refused;
      refused.status = 401;
      transport->Script(refused);
      Config cfg = MakeConfig(dir_.string());
      cfg.unauthorized_retry_s = 0.01;
      auto spool = std::make_shared<GatedSpool>(
          std::make_shared<DiskSpool>(dir_.string(), 8 * 1024, log));
      auto session = Session::Create(cfg, transport, spool, log);
      if (!session) ::_exit(3);
      log->CallBackInto(session.get());
      step("Begin");
      if (!session->Begin()) ::_exit(4);
      step("Begin while a session is active");
      session->Begin();
      step("a 401 within the allowance");
      if (!WaitFor([&] {
            return session->stats().unauthorized_retries >= 1 && spool->Count() == 0;
          }, 5.0)) {
        ::_exit(5);
      }
      step("End");
      session->End();
      step("an End whose end cannot be spooled");
      session->Begin();
      spool->Refuse(true);
      session->End();
      spool->Refuse(false);
      // The worker's recheck closes that session with an end of its own; nothing else may be
      // writing a chunk while the directory goes.
      if (!WaitFor([&] { return spool->OpenMarkers().empty(); }, 10.0)) ::_exit(6);
      step("a chunk write after the chunk directory was removed");
      session->SetUploadsPaused(true);
      session->Begin();
      std::error_code removed;
      fs::remove_all(dir_ / "chunks", removed);
      if (removed) ::_exit(7);
      session->RecordEvent("after", 1.8e9, Vec3{});
      session->Flush();
      step("a gaze part refused at the cap");
      for (int i = 0; i < 60; ++i) {
        session->RecordEvent("fill", 1.8e9 + i, Vec3{});
        session->Flush();
      }
      session->RecordGaze(1.8e9, Vec3{}, Quat{});
      session->Flush();
      step("done");
      log->CallBackInto(nullptr);
      session.reset();
    } catch (...) {
      step("threw");
    }
    ::_exit(0);
  }
  int status = 0;
  bool exited = false;
  for (int i = 0; i < 300 && !exited; ++i) {
    exited = ::waitpid(child, &status, WNOHANG) == child;
    if (!exited) std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (!exited) {
    ::kill(child, SIGKILL);
    ::waitpid(child, &status, 0);
  }
  std::string last;
  {
    std::ifstream in(progress);
    for (std::string line; std::getline(in, line);) last = line;
  }
  std::error_code ec;
  fs::remove(progress, ec);
  EXPECT_TRUE(exited) << "the process deadlocked during: " << last;
  EXPECT_TRUE(exited && WIFEXITED(status) && WEXITSTATUS(status) == 0)
      << "the child stopped during: " << last << " (status " << status << ")";
  EXPECT_EQ(last, "done");
}

// A one-shot process exiting over a slow link. When shutdown_drain_s runs out it says how much
// it leaves, where, and that the session's end is among it: that session stays open on the
// platform until a later start drains the spool.
TEST_F(SessionLifecycleTest, TheDestructorSaysWhatItLeavesWhenTheDrainRunsOut) {
  auto transport = std::make_shared<FakeTransport>();
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  cfg.shutdown_drain_s = 0.3;
  auto log = std::make_shared<CapturingLogger>();
  auto spool = Spool();
  auto session = Session::Create(cfg, transport, spool, log);
  ASSERT_TRUE(session->Begin());
  const std::string id = session->session_id();
  for (int i = 0; i < 5; ++i) {
    session->RecordEvent("tail", 1000.0 + i, Vec3{});
    session->Flush();
  }
  transport->SetPostDelay(std::chrono::milliseconds(200));
  session->SetUploadsPaused(false);
  session.reset();
  ASSERT_GT(spool->Count(), 0u) << "everything went; the test proves nothing";
  EXPECT_TRUE(log->Contains(" chunk(s) still spooled in " + dir_.string() +
                            " (shutdown_drain_s ran out)"));
  EXPECT_TRUE(log->Contains("the end of session " + id + " among them"));
  EXPECT_TRUE(log->Contains("They upload at the next start that uses this spool_dir"));
}

// A spool that can hold the uploader inside a peek that found nothing, and then keep every
// later peek waiting, so a pass can be made to report "nothing to send" after a part has been
// spooled behind its back.
class PeekGatedSpool : public ISpool {
 public:
  explicit PeekGatedSpool(std::shared_ptr<ISpool> inner) : inner_(std::move(inner)) {}

  bool PeekOldest(Chunk* out, const std::set<std::string>& skip) override {
    std::unique_lock<std::mutex> lock(mu_);
    cv_.wait(lock, [&] { return !blocked_; });
    lock.unlock();
    const bool found = inner_->PeekOldest(out, skip);
    lock.lock();
    if (!found && hold_empty_) {
      hold_empty_ = false;
      holding_ = true;
      cv_.notify_all();
      cv_.wait(lock, [&] { return !holding_; });
    }
    return found;
  }
  bool Write(const Chunk& c) override {
    const bool written = inner_->Write(c);
    std::lock_guard<std::mutex> lock(mu_);
    ++writes_;
    cv_.notify_all();
    return written;
  }
  using ISpool::PeekOldest;
  void Remove(const std::string& id) override { inner_->Remove(id); }
  std::uint64_t TotalBytes() const override { return inner_->TotalBytes(); }
  std::size_t Count() const override { return inner_->Count(); }
  void WriteMarker(const SessionMarker& m) override { inner_->WriteMarker(m); }
  std::vector<SessionMarker> OpenMarkers() const override { return inner_->OpenMarkers(); }
  void CloseMarker(const std::string& id) override { inner_->CloseMarker(id); }

  // Holds the next peek that finds nothing until Release, and returns once it is held.
  void HoldNextEmptyPeek() {
    std::unique_lock<std::mutex> lock(mu_);
    hold_empty_ = true;
    cv_.wait(lock, [&] { return holding_; });
  }
  std::size_t writes() const {
    std::lock_guard<std::mutex> lock(mu_);
    return writes_;
  }
  void WaitForWritesAbove(std::size_t n) {
    std::unique_lock<std::mutex> lock(mu_);
    cv_.wait(lock, [&] { return writes_ > n; });
  }
  // Lets the held peek return, with every later peek kept waiting until Unblock.
  void ReleaseAndBlock() {
    std::lock_guard<std::mutex> lock(mu_);
    blocked_ = true;
    holding_ = false;
    cv_.notify_all();
  }
  void Unblock() {
    std::lock_guard<std::mutex> lock(mu_);
    blocked_ = false;
    cv_.notify_all();
  }

 private:
  std::shared_ptr<ISpool> inner_;
  mutable std::mutex mu_;
  std::condition_variable cv_;
  bool hold_empty_ = false;
  bool holding_ = false;
  bool blocked_ = false;
  std::size_t writes_ = 0;
};

// The destructor stops waiting once a pass finds nothing it can send. A pass that looked just
// before End() spooled the session's end, and said so just after, must not make the destructor
// abandon that end.
TEST_F(SessionLifecycleTest, APassThatLookedBeforeTheEndWasSpooledCannotAbandonIt) {
  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<PeekGatedSpool>(Spool());
  Config cfg = MakeConfig(dir_.string());
  cfg.shutdown_drain_s = 10.0;
  auto log = std::make_shared<CapturingLogger>();
  auto session = Session::Create(cfg, transport, spool, log);
  ASSERT_TRUE(session->Begin());
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; })) << "the session's start never went";

  spool->HoldNextEmptyPeek();  // a pass has found the spool empty and not yet said so
  const std::size_t before = spool->writes();
  std::atomic<bool> destroyed{false};
  std::thread destroyer([&] {
    session.reset();  // End() spools the end, then the tail gets its chance
    destroyed.store(true);
  });
  spool->WaitForWritesAbove(before);
  spool->ReleaseAndBlock();  // the pass says "nothing to send" now, with the end spooled
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const bool gave_up = destroyed.load();
  spool->Unblock();
  destroyer.join();

  EXPECT_FALSE(gave_up) << "the destructor gave up on an end spooled after the pass looked";
  EXPECT_FALSE(log->Contains("none of them can be sent by this process now"));
  EXPECT_EQ(spool->Count(), 0u) << "the session's end was left behind";
}

// ros2 launch sends SIGINT, then SIGTERM at +5 s and SIGKILL at +10 s. A kill landing after End()
// returns, with an upload in flight and the session's last parts not yet sent, must not leave the
// marker open, or the next start appends a recovered abnormalEnd to a session that had already
// ended in order: two ends, one of them false.
TEST_F(SessionLifecycleTest, AKillJustAfterEndAddsNoRecoveredAbnormalEnd) {
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    auto transport = std::make_shared<FakeTransport>();
    transport->Stall();  // a dead uplink: no upload finishes
    Config cfg = MakeConfig(dir_.string());
    cfg.uploads_paused = true;
    auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull << 20,
                                             std::make_shared<NullLogger>());
    auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
    if (!session || !session->Begin()) ::_exit(2);
    session->RecordEvent("work", 1000.0, Vec3{});
    session->SetUploadsPaused(false);
    if (!transport->WaitUntilStalled(std::chrono::milliseconds(5000))) ::_exit(3);
    session->End("shutdown");
    ::_exit(0);  // the SIGKILL, in miniature, with an upload still in flight
  }
  int status = 0;
  ::waitpid(child, &status, 0);
  ASSERT_TRUE(WIFEXITED(status));
  ASSERT_EQ(WEXITSTATUS(status), 0) << "the child never reached End() with an upload in flight";
  std::this_thread::sleep_for(std::chrono::milliseconds(200));  // past orphan_stale_after_s
  {
    Config cfg = MakeConfig(dir_.string());
    cfg.uploads_paused = true;
    auto session = Session::Create(cfg, std::make_shared<FakeTransport>(), Spool(),
                                   std::make_shared<NullLogger>());
    EXPECT_EQ(session->RecoverOrphanedSessions(), 0);
  }
  EXPECT_EQ(CountInChunks(dir_, "c3d.sessionEnd"), 1);
  EXPECT_EQ(CountInChunks(dir_, "abnormalEnd"), 0);
}

namespace {

// Ends the process at the first chunk write after the one carrying a session's end: a kill landing
// while End() writes the parts behind its end.
class KilledBehindTheEndSpool : public ISpool {
 public:
  explicit KilledBehindTheEndSpool(std::shared_ptr<ISpool> inner) : inner_(std::move(inner)) {}

  bool Write(const Chunk& c) override {
    if (end_written_.load()) ::_exit(0);
    const bool written = inner_->Write(c);
    if (written && c.body.find("c3d.sessionEnd") != std::string::npos) end_written_.store(true);
    return written;
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

 private:
  std::shared_ptr<ISpool> inner_;
  std::atomic<bool> end_written_{false};
};

}  // namespace

// The session is durable once the part carrying its end is in the spool, so a kill landing while
// End() still writes the parts behind it leaves nothing for recovery to close either.
TEST_F(SessionLifecycleTest, AKillWhileEndWritesThePartsBehindItsEndAddsNoRecoveredAbnormalEnd) {
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    Config cfg = MakeConfig(dir_.string());
    cfg.uploads_paused = true;
    auto spool = std::make_shared<KilledBehindTheEndSpool>(Spool());
    auto session = Session::Create(cfg, std::make_shared<FakeTransport>(), spool,
                                   std::make_shared<NullLogger>());
    if (!session || !session->Begin()) ::_exit(2);
    session->RecordEvent("work", 1000.0, Vec3{});
    session->RecordGaze(1000.0, Vec3{}, Quat{});
    session->End("shutdown");
    ::_exit(3);
  }
  int status = 0;
  ::waitpid(child, &status, 0);
  ASSERT_TRUE(WIFEXITED(status));
  ASSERT_EQ(WEXITSTATUS(status), 0) << "End() wrote no part behind its end";
  std::this_thread::sleep_for(std::chrono::milliseconds(200));  // past orphan_stale_after_s
  {
    Config cfg = MakeConfig(dir_.string());
    cfg.uploads_paused = true;
    auto session = Session::Create(cfg, std::make_shared<FakeTransport>(), Spool(),
                                   std::make_shared<NullLogger>());
    EXPECT_EQ(session->RecoverOrphanedSessions(), 0)
        << "the marker outlived the end it was kept for";
  }
  EXPECT_EQ(CountInChunks(dir_, "c3d.sessionEnd"), 1);
  EXPECT_EQ(CountInChunks(dir_, "abnormalEnd"), 0);
}

// The marker closes with the end, while End() still writes the parts behind it and the session is
// still the one the marker names. A heartbeat in that window must find nothing to beat, or it
// writes the marker back and the next start appends a recovered abnormalEnd to an orderly end.
TEST_F(SessionLifecycleTest, AHeartbeatWhileEndWritesThePartsBehindItsEndCannotReopenTheMarker) {
  auto spool = std::make_shared<GatedSpool>(Spool());
  Config cfg = MakeConfig(dir_.string());
  auto session = Session::Create(cfg, std::make_shared<FakeTransport>(), spool,
                                 std::make_shared<NullLogger>());
  ASSERT_TRUE(session->Begin());
  const std::string id = session->session_id();
  session->RecordGaze(1000.0, Vec3{}, Quat{});

  spool->HoldTheWriteBehindTheEnd();
  std::thread ender([&] { session->End("orderly"); });
  const bool held = WaitFor([&] { return spool->Holding(); });
  bool reopened = false;
  if (held) {
    // Longer than marker_heartbeat_s since End() last wrote the marker, so a beat is due.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    session->PumpUploads();  // beats on this thread before it looks for a chunk
    reopened = MarkerExists(dir_, id);
  }
  spool->Release();
  ender.join();
  ASSERT_TRUE(held) << "End() wrote no part behind its end";
  EXPECT_FALSE(reopened) << "a heartbeat wrote back the marker End() had closed";
  EXPECT_FALSE(MarkerExists(dir_, id));
  session.reset();
  spool.reset();  // the spool directory takes one user at a time

  std::this_thread::sleep_for(std::chrono::milliseconds(200));  // past orphan_stale_after_s
  cfg.uploads_paused = true;
  auto next = Session::Create(cfg, std::make_shared<FakeTransport>(), Spool(),
                              std::make_shared<NullLogger>());
  ASSERT_NE(next, nullptr);
  EXPECT_EQ(next->RecoverOrphanedSessions(), 0);
  EXPECT_EQ(CountInChunks(dir_, "abnormalEnd"), 0);
}

TEST_F(SessionLifecycleTest, AnEndThatCannotBeSpooledLeavesTheMarkerForRecovery) {
  auto inner = Spool();
  auto spool = std::make_shared<GatedSpool>(inner);
  auto log = std::make_shared<CapturingLogger>();
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  auto session = Session::Create(cfg, std::make_shared<FakeTransport>(), spool, log);
  ASSERT_TRUE(session->Begin());
  const std::string id = session->session_id();
  spool->Refuse(true);
  session->End("orderly");
  EXPECT_TRUE(MarkerExists(dir_, id)) << "an end that never reached the spool closed the marker";
  EXPECT_TRUE(log->Contains("could not be spooled"));

  // This process looks again, once the marker has gone quiet, rather than leaving the
  // session open until some later start.
  spool->Refuse(false);
  EXPECT_TRUE(WaitFor([&] { return !MarkerExists(dir_, id); }))
      << "the unclosed session was never recovered";
  EXPECT_EQ(CountInChunks(dir_, "abnormalEnd"), 1);
}

// ---------------------------------------------------------------------------
// What a Record* call, and the heartbeat, can wait behind
// ---------------------------------------------------------------------------

TEST_F(SessionLifecycleTest, ARecordCallNeverWaitsBehindAPartBeingWritten) {
  auto spool = std::make_shared<GatedSpool>(Spool());
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  auto session = Session::Create(cfg, std::make_shared<FakeTransport>(), spool,
                                 std::make_shared<NullLogger>());
  ASSERT_TRUE(session->Begin());
  session->RecordEvent("part", 1000.0, Vec3{});
  spool->HoldNextWrite();
  std::thread flusher([&] { session->Flush(); });
  ASSERT_TRUE(WaitFor([&] { return spool->Holding(); })) << "the write was never reached";

  // A disk write is in progress. Everything a ROS callback calls must still return at once.
  auto record = std::async(std::launch::async, [&] {
    session->RecordGaze(1000.0, Vec3{}, Quat{});
    session->RecordEvent("during", 1000.5, Vec3{});
    session->RecordSensor("battery", 1000.5, 0.5);
    session->SetProperty("k", std::string("v"));
    (void)session->stats();
    (void)session->session_id();
    (void)session->active();
  });
  const bool returned = record.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready;
  spool->Release();
  flusher.join();
  record.wait();
  EXPECT_TRUE(returned) << "a Record* call waited behind a part being written to disk";
}

// The heartbeat is a marker write. Sharing a lock with the chunks the uploader unlinks and the
// parts the worker writes would let one slow unlink under I/O pressure stale the marker, and a
// reader with a short window would adopt a live session.
TEST_F(SessionLifecycleTest, TheHeartbeatNeverWaitsBehindAChunkBeingWritten) {
  auto spool = std::make_shared<GatedSpool>(Spool());
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  cfg.marker_heartbeat_s = 0.02;
  cfg.orphan_stale_after_s = 0.40;
  auto session = Session::Create(cfg, std::make_shared<FakeTransport>(), spool,
                                 std::make_shared<NullLogger>());
  ASSERT_TRUE(session->Begin());
  session->RecordGaze(1000.0, Vec3{}, Quat{});
  spool->HoldNextWrite();
  std::thread flusher([&] { session->Flush(); });
  ASSERT_TRUE(WaitFor([&] { return spool->Holding(); }));
  // Counted as each write starts, so a slow disk under the marker cannot hide a heartbeat that
  // did start; what is tested is whether one can start at all while a chunk is held.
  const std::size_t before = spool->markers().size();
  const bool beat = WaitFor([&] { return spool->markers().size() > before; }, 5.0);
  spool->Release();
  flusher.join();
  EXPECT_TRUE(beat) << "no heartbeat started while a chunk write was held";
  session->End();
}

// Parts are cut under the lock and written outside it, possibly by two threads at once. They
// must still reach the spool -- and so the uploader -- in the order they were cut.
TEST_F(SessionLifecycleTest, PartsReachTheSpoolInTheOrderTheyWereCut) {
  auto spool = std::make_shared<GatedSpool>(Spool());
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  auto session = Session::Create(cfg, std::make_shared<FakeTransport>(), spool,
                                 std::make_shared<NullLogger>());
  ASSERT_TRUE(session->Begin());
  session->RecordEvent("first", 1000.0, Vec3{});
  spool->HoldNextWrite();
  std::thread first([&] { session->Flush(); });
  ASSERT_TRUE(WaitFor([&] { return spool->Holding(); }));
  session->RecordEvent("second", 1001.0, Vec3{});
  std::thread second([&] { session->Flush(); });
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  spool->Release();
  first.join();
  second.join();

  std::vector<std::string> order;
  for (const auto& body : spool->written()) {
    if (body.find("\"first\"") != std::string::npos) order.push_back("first");
    if (body.find("\"second\"") != std::string::npos) order.push_back("second");
  }
  ASSERT_EQ(order.size(), 2u);
  EXPECT_EQ(order[0], "first") << "the part cut second was written first";

  Chunk chunk;
  std::vector<std::uint64_t> parts;
  std::set<std::string> skip;
  while (spool->PeekOldest(&chunk, skip)) {
    skip.insert(chunk.id);
    const json body = json::parse(chunk.body);
    if (body.contains("data") && chunk.stream == Stream::kEvents) parts.push_back(body["part"]);
  }
  EXPECT_TRUE(std::is_sorted(parts.begin(), parts.end())) << "events parts out of FIFO order";
  session->End();
}

// ---------------------------------------------------------------------------
// Session identity and the wall clock
// ---------------------------------------------------------------------------

TEST_F(SessionLifecycleTest, NoTwoSessionsOfOneProcessShareAnIdHoweverQuicklyTheyCome) {
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  auto session = Session::Create(cfg, std::make_shared<FakeTransport>(), Spool(),
                                 std::make_shared<NullLogger>());
  std::vector<std::string> ids;
  std::set<std::string> distinct;
  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(session->Begin());
    ids.push_back(session->session_id());
    distinct.insert(ids.back());
    session->End("quick");
  }
  EXPECT_EQ(distinct.size(), ids.size()) << "the platform merges sessions that share an id";
  for (std::size_t i = 1; i < ids.size(); ++i) {
    EXPECT_GT(std::stoll(ids[i]), std::stoll(ids[i - 1])) << ids[i - 1] << " then " << ids[i];
  }
}

// A part of a session spooled before its marker exists belongs to a session that no recovery
// would ever close, should the process die between the two.
TEST_F(SessionLifecycleTest, TheMarkerIsOnDiskBeforeAnyPartOfTheSession) {
  auto spool = std::make_shared<GatedSpool>(Spool());
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  cfg.marker_heartbeat_s = 3600.0;  // so only Begin can be the one writing it
  cfg.orphan_stale_after_s = 7200.0;
  auto session = Session::Create(cfg, std::make_shared<FakeTransport>(), spool,
                                 std::make_shared<NullLogger>());
  ASSERT_TRUE(session->Begin());
  const std::string id = session->session_id();
  const auto operations = spool->operations();
  const auto marker = std::find(operations.begin(), operations.end(), "marker " + id);
  const auto chunk = std::find(operations.begin(), operations.end(), "chunk " + id);
  ASSERT_NE(marker, operations.end()) << "Begin wrote no marker";
  ASSERT_NE(chunk, operations.end()) << "Begin spooled no sessionStart";
  EXPECT_LT(marker - operations.begin(), chunk - operations.begin());
  // Recovery closes the session on the host this first marker names, should the process die
  // before any other is written.
  EXPECT_EQ(spool->markers().front().gateway, cfg.gateway);
  session->End();
}

TEST_F(SessionLifecycleTest, BeginRefusesWhileTheWallClockHasNotBeenSet) {
  auto transport = std::make_shared<FakeTransport>();
  auto spool = Spool();
  auto log = std::make_shared<CapturingLogger>();
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  cfg.wall_clock_floor_epoch_s =
      std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count() +
      86400.0 * 365;
  auto session = Session::Create(cfg, transport, spool, log);
  ASSERT_NE(session, nullptr);
  EXPECT_FALSE(session->Begin());
  EXPECT_FALSE(session->active());
  EXPECT_TRUE(session->session_id().empty());
  EXPECT_TRUE(log->Contains("has not been set"));
  EXPECT_TRUE(log->Contains("NTP"));
  EXPECT_TRUE(spool->OpenMarkers().empty()) << "a refused Begin must leave nothing behind";
  EXPECT_EQ(spool->Count(), 0u);
}

TEST(WallClock, AClockBeforeTheFloorIsNamedWithTheFloorAndWhatToDo) {
  EXPECT_EQ(WallClockProblem(kWallClockFloorEpochSeconds), "");
  EXPECT_EQ(WallClockProblem(kWallClockFloorEpochSeconds + 1e6), "");
  const std::string problem = WallClockProblem(12.0);
  EXPECT_NE(problem.find("1970-01-01T00:00:12Z"), std::string::npos) << problem;
  EXPECT_NE(problem.find("2026-01-01T00:00:00Z"), std::string::npos) << problem;
  EXPECT_NE(problem.find("NTP"), std::string::npos) << problem;
  EXPECT_NE(WallClockProblem(std::numeric_limits<double>::quiet_NaN()), "");
  EXPECT_NE(WallClockProblem(kWallClockFloorEpochSeconds - 1.0), "");
}

// ---------------------------------------------------------------------------
// Recovery never closes a marker over an end it could not write
// ---------------------------------------------------------------------------

TEST_F(SessionLifecycleTest, RecoveryKeepsAnOrphansMarkerUntilItsEndIsWritten) {
  {
    fs::create_directories(dir_ / "sessions");
    json j;
    j["session_id"] = "1790000000_robot01";
    j["scene_id"] = "SCENEGUID";
    j["scene_version"] = "2";
    j["device_id"] = "robot01";
    j["started_at"] = 1790000000.0;
    j["last_seen"] = 1790000100.0;
    j["next_events_part"] = 4;
    std::ofstream(dir_ / "sessions" / "1790000000_robot01.c3dsession") << j.dump();
  }
  auto spool = std::make_shared<GatedSpool>(Spool());
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  auto session = Session::Create(cfg, std::make_shared<FakeTransport>(), spool,
                                 std::make_shared<NullLogger>());
  spool->Refuse(true);  // the disk is full, which is likely after the crash being recovered
  EXPECT_EQ(session->RecoverOrphanedSessions(), 0);
  EXPECT_EQ(spool->OpenMarkers().size(), 1u) << "closed over an abnormalEnd that was not written";

  spool->Refuse(false);
  EXPECT_TRUE(WaitFor([&] { return spool->OpenMarkers().empty(); }))
      << "the kept marker was never looked at again";
  EXPECT_EQ(CountInChunks(dir_, "abnormalEnd"), 1);
}

// ---------------------------------------------------------------------------
// Validation of the sizes and windows a Config carries
// ---------------------------------------------------------------------------

TEST(ConfigValidateSizes, ANegativeSizeThatWrappedOnTheWayInIsRefused) {
  Config cfg;
  cfg.application_key = "APPKEY";
  cfg.scene_id = "SCENEGUID";
  cfg.scene_version = "2";
  cfg.device_id = "robot01";
  cfg.spool_dir = "/tmp/does-not-matter";
  ASSERT_EQ(cfg.Validate(), "");

  const std::uint64_t minus_one_mib = static_cast<std::uint64_t>(-1) * 1024ull * 1024ull;
  auto refused = [&](const char* name, const std::function<void(Config&)>& set) {
    Config bad = cfg;
    set(bad);
    const std::string problem = bad.Validate();
    EXPECT_NE(problem.find(name), std::string::npos) << name << ": " << problem;
  };
  refused("spool_max_bytes", [&](Config& c) { c.spool_max_bytes = minus_one_mib; });
  refused("camera_flush_bytes", [](Config& c) { c.camera_flush_bytes = static_cast<std::uint64_t>(-5); });
  refused("lidar_flush_bytes", [](Config& c) { c.lidar_flush_bytes = static_cast<std::uint64_t>(-5); });
  refused("gaze_batch", [](Config& c) { c.gaze_batch = static_cast<std::size_t>(-1); });
  refused("max_records_per_batch",
          [](Config& c) { c.max_records_per_batch = static_cast<std::size_t>(-1); });
  refused("shutdown_drain_s", [](Config& c) { c.shutdown_drain_s = -1.0; });
  refused("shutdown_drain_s", [](Config& c) { c.shutdown_drain_s = 61.0; });
  refused("shutdown_drain_s",
          [](Config& c) { c.shutdown_drain_s = std::numeric_limits<double>::quiet_NaN(); });
  refused("wall_clock_floor_epoch_s",
          [](Config& c) { c.wall_clock_floor_epoch_s = std::numeric_limits<double>::infinity(); });
  // Each by its own name, and NaN as well as a non-positive value: NaN fails every comparison.
  for (const double bad : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
    Config camera = cfg;
    camera.camera_flush_s = bad;
    EXPECT_NE(camera.Validate().find("camera_flush_s"), std::string::npos) << bad;
    EXPECT_EQ(camera.Validate().find("lidar_flush_s"), std::string::npos) << camera.Validate();
    Config lidar = cfg;
    lidar.lidar_flush_s = bad;
    EXPECT_NE(lidar.Validate().find("lidar_flush_s"), std::string::npos) << bad;
    EXPECT_EQ(lidar.Validate().find("camera_flush_s"), std::string::npos) << lidar.Validate();
  }

  Config fine = cfg;
  fine.shutdown_drain_s = 0.0;
  EXPECT_EQ(fine.Validate(), "") << "0 means do not wait, which is legitimate";
}

// Every value Validate() passes is one Create may run with, so a value that crashes the first
// Record* call or wedges a thread is refused here, by the field's name.
TEST(ConfigValidateSizes, NoValueThatPassesCanCrashOrWedgeTheSession) {
  Config cfg;
  cfg.application_key = "APPKEY";
  cfg.scene_id = "SCENEGUID";
  cfg.scene_version = "2";
  cfg.device_id = "robot01";
  cfg.spool_dir = "/tmp/does-not-matter";
  ASSERT_EQ(cfg.Validate(), "");
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  auto refused = [&](const char* name, const std::function<void(Config&)>& set) {
    Config bad = cfg;
    set(bad);
    const std::string problem = bad.Validate();
    EXPECT_NE(problem.find(name), std::string::npos) << name << ": '" << problem << "'";
  };
  auto accepted = [&](const char* what, const std::function<void(Config&)>& set) {
    Config good = cfg;
    set(good);
    EXPECT_EQ(good.Validate(), "") << what;
  };

  refused("max_records_per_batch", [](Config& c) { c.max_records_per_batch = 0; });
  accepted("a cap of one record", [](Config& c) { c.max_records_per_batch = 1; });

  for (const double bad : {0.0, -0.01, nan, inf, 1.5}) {
    refused("worker_tick_s", [bad](Config& c) { c.worker_tick_s = bad; });
  }
  accepted("a one-second tick", [](Config& c) {
    c.worker_tick_s = 1.0;
    c.orphan_stale_after_s = 30.0;
  });

  for (const double bad : {0.0, -1.0, nan}) {
    refused("auto_send_interval_s", [bad](Config& c) { c.auto_send_interval_s = bad; });
  }
  accepted("parts cut on batch size and Flush alone",
           [inf](Config& c) { c.auto_send_interval_s = inf; });

  for (const double bad : {0.0, -0.1, nan, inf}) {
    refused("gaze_interval_s", [bad](Config& c) { c.gaze_interval_s = bad; });
  }

  for (const double bad : {nan, inf}) {
    refused("unauthorized_retry_s", [bad](Config& c) { c.unauthorized_retry_s = bad; });
  }
  accepted("an immediate retry", [](Config& c) { c.unauthorized_retry_s = 0.0; });
}

// What a cap of 0 did before Validate() refused it: the first RecordGaze erased the front of an
// empty batch. In a child, so a crash is a result rather than the end of the suite; the child
// exits 3 when Create refuses, which is the pass.
TEST(ConfigValidateSizes, ARecordCapOfZeroNeverReachesASession) {
  const fs::path dir = fs::temp_directory_path() / ("c3d_cap0_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  const pid_t child = ::fork();
  if (child == 0) {
    try {
      Config cfg = MakeConfig(dir.string());
      cfg.max_records_per_batch = 0;
      auto session = Session::Create(
          cfg, std::make_shared<FakeTransport>(),
          std::make_shared<DiskSpool>(cfg.spool_dir, 1 << 20, std::make_shared<NullLogger>()),
          std::make_shared<NullLogger>());
      if (!session) ::_exit(3);
      if (session->Begin()) {
        for (int i = 0; i < 1000; ++i) session->RecordGaze(1.8e9 + i, Vec3{}, Quat{});
      }
    } catch (...) {
      ::_exit(4);
    }
    ::_exit(0);
  }
  int status = 0;
  ::waitpid(child, &status, 0);
  std::error_code ec;
  fs::remove_all(dir, ec);
  ASSERT_FALSE(WIFSIGNALED(status)) << "RecordGaze crashed with signal " << WTERMSIG(status);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 3) << "Create accepted a record cap of 0";
}
