// The chaos tests: a ten minute network cut and a hard kill must lose nothing
// and must not leave an orphaned session.

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <sys/wait.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <chrono>
#include <algorithm>
#include <atomic>
#include <mutex>
#include <optional>
#include <limits>
#include <memory>
#include <set>
#include <thread>
#include <vector>

#include "c3d/session.hpp"
#include "c3d/spool.hpp"
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
  // Test-scale heartbeat and staleness. A real power loss leaves a marker whose
  // last_seen is minutes old; these tests simulate it in milliseconds, so the
  // window has to be scaled with them or "walked away" would read as "still
  // running". The relationship Validate() enforces is preserved.
  cfg.marker_heartbeat_s = 0.01;
  cfg.orphan_stale_after_s = 0.05;
  return cfg;
}

double NowSeconds() {
  return std::chrono::duration<double>(
             std::chrono::system_clock::now().time_since_epoch()).count();
}

// Written by hand with a chosen last_seen, which is the only way to test the
// boundary without sleeping across it.
void WriteMarkerWithLastSeen(const fs::path& dir, const std::string& session_id,
                             double started_at, double last_seen,
                             double max_silence_s = 0.0) {
  fs::create_directories(dir / "sessions");
  json j;
  j["session_id"] = session_id;
  j["scene_id"] = "SCENEGUID";
  j["scene_version"] = "2";
  j["device_id"] = "robot01";
  j["started_at"] = started_at;
  j["last_seen"] = last_seen;
  // 0 means the marker declares no threshold of its own, which is what a foreign
  // writer's marker looks like. Written only when asked for, so the fallback path
  // stays exercised by the tests that do not care.
  if (max_silence_s > 0.0) j["max_silence_s"] = max_silence_s;
  std::ofstream out(dir / "sessions" / (session_id + ".c3dsession"));
  out << j.dump();
}

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

bool WaitFor(const std::function<bool()>& done, double seconds) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (std::chrono::steady_clock::now() < deadline) {
    if (done()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return done();
}

std::string OnlyOpenMarkerId(const std::shared_ptr<DiskSpool>& spool) {
  const auto markers = spool->OpenMarkers();
  return markers.size() == 1 ? markers.front().session_id : std::string();
}

// Captures log lines so a test can read the numeric backoff a Warn call named,
// instead of inferring it from a sleep.
class CapturingLogger : public ILogger {
 public:
  void Log(LogLevel, const std::string& message) override {
    std::lock_guard<std::mutex> lock(mu_);
    messages_.push_back(message);
  }
  std::vector<std::string> messages() const {
    std::lock_guard<std::mutex> lock(mu_);
    return messages_;
  }

 private:
  mutable std::mutex mu_;
  std::vector<std::string> messages_;
};

double SteadyNow() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// A spool that timestamps every marker write as it starts and as it finishes, so a test reads
// the heartbeat's cadence -- what the SDK leaves between one write and the next -- apart from
// how long the filesystem took over each write.
class MarkerClockSpool : public ISpool {
 public:
  explicit MarkerClockSpool(std::shared_ptr<ISpool> inner) : inner_(std::move(inner)) {}

  void WriteMarker(const SessionMarker& m) override {
    const double started = SteadyNow();
    inner_->WriteMarker(m);
    const double finished = SteadyNow();
    std::lock_guard<std::mutex> lock(mu_);
    writes_.push_back(MarkerWrite{m.session_id, started, finished});
  }
  bool Write(const Chunk& c) override { return inner_->Write(c); }
  using ISpool::PeekOldest;
  bool PeekOldest(Chunk* out, const std::set<std::string>& skip) override {
    return inner_->PeekOldest(out, skip);
  }
  void Remove(const std::string& id) override { inner_->Remove(id); }
  std::uint64_t TotalBytes() const override { return inner_->TotalBytes(); }
  std::size_t Count() const override { return inner_->Count(); }
  std::vector<SessionMarker> OpenMarkers() const override { return inner_->OpenMarkers(); }
  void CloseMarker(const std::string& id) override { inner_->CloseMarker(id); }

  // The longest stretch within [from, to] in which no write of `id`'s marker was under way.
  double LongestIdle(const std::string& id, double from, double to) const {
    std::lock_guard<std::mutex> lock(mu_);
    double longest = 0.0;
    double idle_since = from;
    for (const auto& w : writes_) {
      if (w.id != id || w.finished < from) continue;
      if (w.started > to) break;
      longest = std::max(longest, w.started - idle_since);
      idle_since = std::max(idle_since, w.finished);
    }
    return std::max(longest, to - idle_since);
  }

 private:
  struct MarkerWrite {
    std::string id;
    double started;
    double finished;
  };
  std::shared_ptr<ISpool> inner_;
  mutable std::mutex mu_;
  std::vector<MarkerWrite> writes_;
};

class RecoveryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() /
           ("c3d_rec_" + std::to_string(::getpid()) + "_" + std::to_string(counter_++));
    fs::create_directories(dir_);
  }
  void TearDown() override {
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }
  fs::path dir_;
  static int counter_;
};
int RecoveryTest::counter_ = 0;

}  // namespace

TEST_F(RecoveryTest, NetworkOutageLosesNothing) {
  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(dir_.string());
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
  ASSERT_NE(session, nullptr);

  transport->SetOffline(true);
  ASSERT_TRUE(session->Begin());

  constexpr int kEvents = 200;
  for (int i = 0; i < kEvents; ++i) {
    session->RecordEvent("during_outage", 1000.0 + i, Vec3{});
    session->Flush();
  }
  session->PumpUploads();
  EXPECT_GT(spool->Count(), 0u);

  // Network returns.
  transport->SetOffline(false);
  for (int i = 0; i < 40 && spool->Count() > 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    session->PumpUploads();
  }

  int delivered = 0;
  for (const auto& r : transport->requests()) {
    if (r.url.find("/events/") == std::string::npos) continue;
    const json body = json::parse(r.body);
    for (const auto& rec : body["data"]) {
      if (rec["name"] == "during_outage") ++delivered;
    }
  }
  EXPECT_EQ(delivered, kEvents) << "every record buffered during the outage must arrive";
  session->End();
}

TEST_F(RecoveryTest, ReplayIsChronologicalAfterAnOutage) {
  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(dir_.string());
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
  transport->SetOffline(true);
  ASSERT_TRUE(session->Begin());

  for (int i = 0; i < 10; ++i) {
    session->RecordEvent("seq", 1000.0 + i, Vec3{});
    session->Flush();
  }
  transport->SetOffline(false);
  for (int i = 0; i < 40 && spool->Count() > 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    session->PumpUploads();
  }

  std::vector<double> times;
  for (const auto& r : transport->requests()) {
    if (r.url.find("/events/") == std::string::npos) continue;
    const json body = json::parse(r.body);
    for (const auto& rec : body["data"]) {
      if (rec["name"] == "seq") times.push_back(rec["time"].get<double>());
    }
  }
  ASSERT_GE(times.size(), 10u);
  EXPECT_TRUE(std::is_sorted(times.begin(), times.end()))
      << "recovered data must replay in the order it happened, not newest-first";
  session->End();
}

TEST_F(RecoveryTest, APayloadRejectionIsDiscardedWithoutWedgingTheQueue) {
  // Only the four answers that say the payload itself is wrong discard it: replaying the same
  // bytes can never succeed. The gateway here refuses one payload every time it sees it, so a
  // rejected chunk left in the spool would go again and again ahead of everything behind it.
  for (const long status : {400L, 413L, 415L, 422L}) {
    SCOPED_TRACE("status " + std::to_string(status));
    const fs::path sub_dir = dir_ / std::to_string(status);
    fs::create_directories(sub_dir);
    auto transport = std::make_shared<FakeTransport>();
    transport->SetResponder([status](const FakeTransport::Request& r) -> std::optional<HttpResponse> {
      if (r.body.find("rejected_payload") == std::string::npos) return std::nullopt;
      HttpResponse rejected;
      rejected.status = status;
      return rejected;
    });
    auto spool = std::make_shared<DiskSpool>(sub_dir.string(), 64ull * 1024 * 1024,
                                             std::make_shared<NullLogger>());
    auto cfg = MakeConfig(sub_dir.string());
    cfg.uploads_paused = true;
    auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
    ASSERT_TRUE(session->Begin());
    session->RecordEvent("rejected_payload", 2000.0, Vec3{});
    session->Flush();
    for (int i = 0; i < 5; ++i) {
      session->RecordEvent("after_rejection", 2001.0 + i, Vec3{});
      session->Flush();
    }
    session->SetUploadsPaused(false);
    EXPECT_TRUE(WaitFor([&] { return spool->Count() == 0; }, 3.0))
        << "a rejected chunk must not block the ones behind it";
    session->SetUploadsPaused(true);

    int rejected_posts = 0, delivered = 0;
    for (const auto& r : transport->requests()) {
      if (r.body.find("rejected_payload") != std::string::npos) ++rejected_posts;
      if (r.body.find("after_rejection") != std::string::npos) ++delivered;
    }
    EXPECT_EQ(rejected_posts, 1) << "the rejected chunk was sent again instead of discarded";
    EXPECT_EQ(delivered, 5);
    EXPECT_EQ(session->stats().chunks_discarded, 1u);
    session.reset();
  }
}

TEST_F(RecoveryTest, TheDiscardMessageCarriesWhatTheGatewaySaid) {
  // A 400 is the gateway's only channel for "this configuration is wrong": an
  // unknown colour, a scene version it cannot resolve for the key, a timestamp
  // below its floor. The chunk is discarded whatever the reason, so the body is
  // the only chance anyone gets to read which of them it was -- and a status code
  // beside a stream name reads as a transport problem, which is the one thing it
  // is not.
  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(dir_.string());
  auto logger = std::make_shared<CapturingLogger>();
  auto session = Session::Create(cfg, transport, spool, logger);
  ASSERT_TRUE(session->Begin());
  session->Flush();
  for (int i = 0; i < 40 && spool->Count() > 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    session->PumpUploads();
  }
  ASSERT_EQ(spool->Count(), 0u) << "start from a drained spool, or the scripted response "
                                   "lands on the wrong chunk";

  HttpResponse rejected;
  rejected.status = 400;
  // The reason first, then a control byte and far more than the log may carry:
  // a body from the network can be an HTML page from an intercepting proxy or an
  // echo of a packed Cap'n Proto request.
  rejected.body = "{\"error\":\"unknown color 'RGB565'\"}";
  rejected.body += '\n';
  rejected.body += std::string(400, 'z');
  rejected.body += "TAIL";
  transport->Script(rejected);

  session->RecordEvent("rejected", 8000.0, Vec3{});
  session->Flush();
  for (int i = 0; i < 40 && session->stats().chunks_discarded == 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    session->PumpUploads();
  }
  ASSERT_EQ(session->stats().chunks_discarded, 1u);

  bool named_the_reason = false, leaked_the_tail = false, said_it_truncated = false;
  bool raw_byte_reached_the_log = false;
  for (const auto& message : logger->messages()) {
    if (message.find("chunk discarded") == std::string::npos) continue;
    if (message.find("unknown color 'RGB565'") != std::string::npos) named_the_reason = true;
    if (message.find("TAIL") != std::string::npos) leaked_the_tail = true;
    if (message.find("truncated from " + std::to_string(rejected.body.size()) + " bytes") !=
        std::string::npos) {
      said_it_truncated = true;
    }
    for (char c : message) {
      if (static_cast<unsigned char>(c) < 0x20) raw_byte_reached_the_log = true;
    }
  }
  EXPECT_TRUE(named_the_reason) << "the discard line named the status and not the cause";
  EXPECT_FALSE(leaked_the_tail) << "the whole body reached the log";
  EXPECT_TRUE(said_it_truncated) << "a truncated body that does not say it was truncated reads "
                                    "as the gateway's whole answer";
  EXPECT_FALSE(raw_byte_reached_the_log);
  session->End();
}

TEST_F(RecoveryTest, CaptivePortalIsTreatedAsFailureNotSuccess) {
  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(dir_.string());
  // No retry inside the test, so the chunk the portal answered is still the one spooled.
  cfg.retry_min_s = 30.0;
  cfg.retry_max_s = 30.0;
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
  ASSERT_TRUE(session->Begin());
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; }, 5.0))
      << "start from a drained spool, or the scripted response lands on the wrong chunk";
  // A pass counts an upload after removing its chunk; this one waits for the uploader's to end.
  session->PumpUploads();
  const std::uint64_t uploaded = session->stats().chunks_uploaded;

  // A 200 with no cvr-request-time header came from an intercepting proxy, not
  // from the Cognitive3D gateway. Site wifi does this routinely.
  HttpResponse portal;
  portal.status = 200;
  portal.saw_request_time_header = false;
  EXPECT_FALSE(portal.IsSuccess());
  EXPECT_TRUE(portal.IsRetryable());

  transport->Script(portal);
  session->RecordEvent("x", 1.0, Vec3{});
  session->Flush();
  session->PumpUploads();
  EXPECT_EQ(spool->Count(), 1u) << "data must be retained, not discarded as delivered";
  const Stats stats = session->stats();
  EXPECT_EQ(stats.upload_failures, 1u);
  EXPECT_EQ(stats.chunks_uploaded, uploaded);
  session->End();
}

TEST_F(RecoveryTest, HardKillLeavesRecoverableStateAndIsClosedOnRestart) {
  const std::string spool_dir = dir_.string();
  std::string dead_session_id;

  // Run one: begin a session, write data, then die without calling End -- in a
  // child process that exits on the spot, so no destructor, no orderly End and no
  // heartbeat outlives it. This is a robot losing power, which is the scenario the
  // spool exists for. The kernel drops the spool's directory lock with the process,
  // exactly as it would after a real crash.
  //
  // The doomed session declares a long silence budget, which the marker must carry.
  dead_session_id = RunAndDieWithoutEnding([&] {
    auto transport = std::make_shared<FakeTransport>();
    transport->SetOffline(true);
    auto spool = std::make_shared<DiskSpool>(spool_dir, 64ull * 1024 * 1024,
                                             std::make_shared<NullLogger>());
    auto cfg = MakeConfig(spool_dir);
    cfg.marker_heartbeat_s = 3600.0;
    cfg.orphan_stale_after_s = 7200.0;
    auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
    if (!session || !session->Begin()) return std::string();
    session->RecordEvent("work", 5000.0, Vec3{});
    session->Flush();
    const std::string id = session->session_id();
    session.release();  // the child exits next; nothing may end the session first
    return id;
  });
  ASSERT_FALSE(dead_session_id.empty()) << "the child never began its session";
  // ...and then the marker goes unrefreshed for long enough that a lost robot is
  // the only explanation. Stamped rather than slept, and the number is not
  // arbitrary: the marker this session wrote DECLARES a 7200s silence budget,
  // and a reader honours it. So 80ms
  // of silence proves nothing about a writer that may legitimately be quiet for
  // two hours -- correctly, and that is the point of recording the budget --
  // while 30000s is unambiguous.
  //
  // Ageing the marker keeps every field the SDK actually wrote and tests the
  // declared-budget rule in the ADOPT direction. Shortening the window instead
  // would test the reader's threshold against itself.
  {
    auto spool = std::make_shared<DiskSpool>(spool_dir, 64ull * 1024 * 1024,
                                             std::make_shared<NullLogger>());
    const auto markers = spool->OpenMarkers();
    ASSERT_EQ(markers.size(), 1u);
    ASSERT_DOUBLE_EQ(markers.front().max_silence_s, 7200.0)
        << "the marker must record the silence budget its writer declared";
    WriteMarkerWithLastSeen(fs::path(spool_dir), dead_session_id, markers.front().started_at,
                            NowSeconds() - 30000.0, 7200.0);
  }

  {
    auto spool = std::make_shared<DiskSpool>(spool_dir, 64ull * 1024 * 1024,
                                             std::make_shared<NullLogger>());
    ASSERT_FALSE(spool->OpenMarkers().empty()) << "the dead session must still look open";
  }

  // Run two: a fresh process recovers.
  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(spool_dir, 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(spool_dir);
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
  const int recovered = session->RecoverOrphanedSessions();
  EXPECT_EQ(recovered, 1);
  EXPECT_TRUE(spool->OpenMarkers().empty()) << "recovery must close the marker";

  ASSERT_TRUE(session->Begin());
  for (int i = 0; i < 40 && spool->Count() > 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    session->PumpUploads();
  }

  bool saw_work = false;
  bool saw_abnormal_end = false;
  for (const auto& r : transport->requests()) {
    if (r.url.find("/events/") == std::string::npos) continue;
    const json body = json::parse(r.body);
    for (const auto& rec : body["data"]) {
      if (rec["name"] == "work") saw_work = true;
      if (rec["name"] == "c3d.sessionEnd" && body["sessionid"] == dead_session_id) {
        saw_abnormal_end = true;
        EXPECT_EQ(rec["properties"]["Reason"], "abnormalEnd");
        EXPECT_GE(rec["properties"]["sessionlength"].get<double>(), 0.0);
      }
    }
  }
  EXPECT_TRUE(saw_work) << "data written before the kill must still be delivered";
  EXPECT_TRUE(saw_abnormal_end)
      << "the interrupted session must be closed, not left open forever";
  session->End();
}

TEST_F(RecoveryTest, ConcurrentPumpsDoNotUploadTheSameChunkTwice) {
  // The worker thread drains the spool continuously, and callers drain it
  // explicitly on shutdown. Without a serialised pump both can peek the same
  // chunk and POST it, producing duplicate `part` numbers within one session,
  // which on the dashboard reads as duplicated telemetry.
  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(dir_.string());
  cfg.worker_tick_s = 0.001;  // maximise the overlap a race needs
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
  ASSERT_TRUE(session->Begin());

  std::atomic<bool> stop{false};
  std::thread hammer([&] {
    while (!stop.load()) session->PumpUploads();
  });

  for (int i = 0; i < 60; ++i) {
    session->RecordEvent("dup_check", 3000.0 + i, Vec3{});
    session->Flush();
  }
  for (int i = 0; i < 60 && spool->Count() > 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  stop.store(true);
  hammer.join();

  const std::string sid = session->session_id();
  std::set<std::uint64_t> seen_parts;
  std::size_t duplicates = 0;
  for (const auto& r : transport->requests()) {
    if (r.url.find("/events/") == std::string::npos) continue;
    const json body = json::parse(r.body);
    if (body["sessionid"] != sid) continue;
    if (!seen_parts.insert(body["part"].get<std::uint64_t>()).second) ++duplicates;
  }
  EXPECT_EQ(duplicates, 0u) << "the same chunk was uploaded more than once";
  session->End();
}

TEST_F(RecoveryTest, TransientUnauthorizedIsRetriedAndDelivers) {
  // One gaze batch meeting one transient 401 from the collector must cost one
  // bounded retry, not the batch.
  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(dir_.string());
  cfg.unauthorized_retry_s = 0.01;
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
  ASSERT_TRUE(session->Begin());
  session->Flush();
  session->PumpUploads();
  const std::uint64_t uploaded_before = session->stats().chunks_uploaded;

  HttpResponse unauthorized;
  unauthorized.status = 401;
  transport->Script(unauthorized);  // one 401, then the transport's default 200

  session->RecordEvent("transient_401", 2000.0, Vec3{});
  session->Flush();
  for (int i = 0; i < 40 && spool->Count() > 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    session->PumpUploads();
  }

  EXPECT_EQ(spool->Count(), 0u);
  EXPECT_GT(session->stats().chunks_uploaded, uploaded_before);
  EXPECT_EQ(session->stats().chunks_discarded, 0u);

  // Count the POSTs rather than scanning for the payload: FakeTransport records a
  // request BEFORE returning the scripted status, so a scan finds the batch after
  // the 401 alone and cannot fail. Two posts for one chunk is the retry, and the
  // spool emptying with nothing discarded is the delivery.
  int posts_for_chunk = 0;
  for (const auto& r : transport->requests()) {
    if (r.url.find("/events/") == std::string::npos) continue;
    const json body = json::parse(r.body);
    for (const auto& rec : body["data"]) {
      if (rec["name"] == "transient_401") ++posts_for_chunk;
    }
  }
  EXPECT_EQ(posts_for_chunk, 2) << "one 401 then one success: the batch was retried once";
  session->End();
}

TEST_F(RecoveryTest, PersistentUnauthorizedPausesEveryUploadAndKeepsTheData) {
  // A rotated or revoked key is not a bad payload: every chunk would meet the same answer, and
  // deleting each one after its allowance would wipe a whole backlog at about a chunk a second. The
  // allowance is spent, then ALL uploads pause with everything kept, and authentication is tried
  // again every retry_max_s until it is accepted.
  // Before the session: its uploader can call the responder until the session is destroyed.
  std::mutex attempts_mu;
  std::vector<double> attempt_times;
  std::atomic<bool> refused{true};
  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(dir_.string());
  cfg.unauthorized_retry_s = 0.01;
  cfg.retry_max_s = 0.6;
  auto logger = std::make_shared<CapturingLogger>();
  auto session = Session::Create(cfg, transport, spool, logger);
  ASSERT_TRUE(session->Begin());
  session->Flush();
  for (int i = 0; i < 40 && spool->Count() > 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    session->PumpUploads();
  }
  ASSERT_EQ(spool->Count(), 0u) << "start from a drained spool";

  // The key stays refused until the test has seen a probe refused, however long that takes, and
  // the probe after that is accepted.
  transport->SetResponder([&](const FakeTransport::Request& r) -> std::optional<HttpResponse> {
    if (r.body.find("persistent_401") == std::string::npos) return std::nullopt;
    {
      std::lock_guard<std::mutex> lock(attempts_mu);
      attempt_times.push_back(
          std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
              .count());
    }
    if (!refused.load()) return std::nullopt;
    HttpResponse unauthorized;
    unauthorized.status = 401;
    return unauthorized;
  });
  const auto attempts = [&] {
    std::lock_guard<std::mutex> lock(attempts_mu);
    return attempt_times.size();
  };

  session->RecordEvent("persistent_401", 3000.0, Vec3{});
  session->Flush();
  ASSERT_TRUE(WaitFor(
      [&] {
        session->PumpUploads();
        return session->stats().uploads_auth_paused;
      },
      5.0));
  Stats stats = session->stats();
  EXPECT_EQ(stats.auth_refused_streams, std::vector<Stream>{Stream::kEvents});
  EXPECT_EQ(stats.chunks_discarded, 0u) << "a refused key must not delete data that is fine";
  EXPECT_EQ(spool->Count(), 1u);

  // Paused means paused: new data spools, nothing of it is sent while the key is refused, and the
  // refused chunk is asked about again once per retry_max_s.
  for (int i = 0; i < 3; ++i) {
    session->RecordEvent("while_paused", 3001.0 + i, Vec3{});
    session->Flush();
  }
  ASSERT_TRUE(WaitFor(
      [&] {
        session->PumpUploads();
        return attempts() >= 3;
      },
      5.0))
      << "the refused chunk was never probed";
  EXPECT_EQ(spool->Count(), 4u);
  EXPECT_TRUE(session->stats().uploads_auth_paused);

  // Accepted again at the next probe, and then everything drains.
  refused.store(false);
  ASSERT_TRUE(WaitFor(
      [&] {
        session->PumpUploads();
        return spool->Count() == 0;
      },
      5.0))
      << "uploads never resumed after the key was accepted again";
  stats = session->stats();
  EXPECT_FALSE(stats.uploads_auth_paused);
  EXPECT_TRUE(stats.auth_refused_streams.empty());
  EXPECT_EQ(stats.chunks_discarded, 0u);

  // Every request for the refused chunk but the last was refused, and nothing recorded while
  // paused went before the probe that was accepted.
  const auto requests = transport->requests();
  int posts_for_chunk = 0;
  std::size_t accepted_probe = 0;
  std::size_t first_while_paused = requests.size();
  for (std::size_t i = 0; i < requests.size(); ++i) {
    if (requests[i].url.find("/events/") == std::string::npos) continue;
    const json body = json::parse(requests[i].body);
    for (const auto& rec : body["data"]) {
      if (rec["name"] == "persistent_401") {
        ++posts_for_chunk;
        accepted_probe = i;
      }
      if (rec["name"] == "while_paused") first_while_paused = std::min(first_while_paused, i);
    }
  }
  std::vector<double> times;
  {
    std::lock_guard<std::mutex> lock(attempts_mu);
    times = attempt_times;
  }
  ASSERT_GE(times.size(), 4u) << "two in the allowance, a refused probe, an accepted one";
  EXPECT_EQ(posts_for_chunk, static_cast<int>(times.size()));
  EXPECT_EQ(stats.auth_rejections, times.size() - 1);
  ASSERT_LT(first_while_paused, requests.size());
  EXPECT_GT(first_while_paused, accepted_probe) << "an upload went out while paused";
  for (std::size_t i = 2; i < times.size(); ++i) {
    EXPECT_GE(times[i] - times[i - 1], 0.9 * cfg.retry_max_s)
        << "probe " << i - 1 << " came before retry_max_s";
  }

  int paused_errors = 0;
  bool after_the_allowance = false;
  bool said_resumed = false;
  for (const auto& message : logger->messages()) {
    if (message.find("ALL uploads are paused") == std::string::npos) {
      if (message.find("authentication accepted again") != std::string::npos) said_resumed = true;
      continue;
    }
    ++paused_errors;
    if (message.find("after 2 attempt(s)") != std::string::npos) after_the_allowance = true;
  }
  EXPECT_EQ(paused_errors, 1) << "the pause is said once, not once per probe";
  EXPECT_TRUE(after_the_allowance) << "uploads paused before the allowance of two was spent";
  EXPECT_TRUE(said_resumed);
  session->End();
}

// An accepted probe lifts the pause itself, not only the chunk it carried. What is recorded after
// it goes at once rather than at a probe per retry_max_s, and the shutdown drain does not stop for
// credentials that are no longer refused. A chunk held for another host keeps the spool from
// emptying, so the destructor always looks at why it should stop.
TEST_F(RecoveryTest, AnAcceptedProbeLiftsThePauseForEverythingAfterIt) {
  {
    DiskSpool planter(dir_.string(), 64ull * 1024 * 1024, std::make_shared<NullLogger>());
    Chunk foreign;
    foreign.url = "http://127.0.0.1:8787/v0/events/SCENEGUID?version=2";
    foreign.scene_id = "SCENEGUID";
    foreign.scene_version = "2";
    foreign.body = "{}";
    foreign.stream = Stream::kEvents;
    ASSERT_TRUE(planter.Write(foreign));
  }
  std::atomic<bool> refused{true};
  auto transport = std::make_shared<FakeTransport>();
  transport->SetResponder([&](const FakeTransport::Request&) -> std::optional<HttpResponse> {
    if (!refused.load()) return std::nullopt;
    HttpResponse unauthorized;
    unauthorized.status = 401;
    return unauthorized;
  });
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(dir_.string());
  cfg.unauthorized_retry_s = 0.01;
  cfg.retry_max_s = 0.5;
  cfg.shutdown_drain_s = 5.0;
  auto logger = std::make_shared<CapturingLogger>();
  auto session = Session::Create(cfg, transport, spool, logger);
  ASSERT_TRUE(session->Begin());
  session->RecordEvent("refused_then_accepted", 1000.0, Vec3{});
  session->Flush();
  ASSERT_TRUE(WaitFor([&] { return session->stats().uploads_auth_paused; }, 5.0));

  refused.store(false);
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 1; }, 5.0))
      << "uploads never resumed after the key was accepted again";

  // Nothing gates a pass once the pause is lifted, so this one sends what was just recorded.
  session->RecordEvent("after_the_resume", 1001.0, Vec3{});
  session->Flush();
  session->PumpUploads();
  EXPECT_EQ(spool->Count(), 1u) << "an upload after the accepted probe waited for a probe's turn";

  session.reset();
  bool looked = false;
  for (const auto& message : logger->messages()) {
    if (message.find("chunk(s) still spooled in") != std::string::npos) looked = true;
    EXPECT_EQ(message.find("the credentials are refused"), std::string::npos)
        << "the shutdown drain stopped for a pause the accepted probe should have lifted: "
        << message;
  }
  EXPECT_TRUE(looked) << "the destructor never weighed what was left";
}

TEST_F(RecoveryTest, AForbiddenKeyAndARefusingProxyPauseUploadsTheSameWay) {
  for (const long status : {403L, 407L}) {
    SCOPED_TRACE("status " + std::to_string(status));
    const fs::path sub_dir = dir_ / std::to_string(status);
    fs::create_directories(sub_dir);
    auto transport = std::make_shared<FakeTransport>();
    auto spool = std::make_shared<DiskSpool>(sub_dir.string(), 64ull * 1024 * 1024,
                                             std::make_shared<NullLogger>());
    auto cfg = MakeConfig(sub_dir.string());
    cfg.uploads_paused = true;
    cfg.unauthorized_retry_s = 0.01;
    cfg.retry_max_s = 30.0;
    auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
    ASSERT_TRUE(session->Begin());
    session->RecordEvent("work", 1000.0, Vec3{});
    session->Flush();
    const std::size_t spooled = spool->Count();
    HttpResponse refused;
    refused.status = status;
    transport->SetResponder([refused](const FakeTransport::Request&) { return refused; });
    session->SetUploadsPaused(false);
    for (int i = 0; i < 30; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      session->PumpUploads();
    }
    EXPECT_TRUE(session->stats().uploads_auth_paused);
    EXPECT_EQ(session->stats().chunks_discarded, 0u);
    EXPECT_EQ(spool->Count(), spooled);
    EXPECT_EQ(transport->count(), 2u) << "the allowance, then nothing until the probe";
    session->End();
  }
}

TEST_F(RecoveryTest, AProbeIsOneRequestWhateverItIsAnswered) {
  // While paused for authentication, a pass is a single probe. Its answer decides for that one
  // chunk -- here a payload rejection discards it -- and only a success lets the rest go.
  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  cfg.unauthorized_attempts = 1;
  cfg.retry_max_s = 0.3;
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
  ASSERT_TRUE(session->Begin());
  for (int i = 0; i < 4; ++i) {
    session->RecordEvent("queued", 1000.0 + i, Vec3{});
    session->Flush();
  }
  const std::size_t spooled = spool->Count();
  HttpResponse unauthorized;
  unauthorized.status = 401;
  HttpResponse rejected;
  rejected.status = 400;
  transport->Script(unauthorized);
  transport->Script(rejected);
  for (int i = 0; i < 10; ++i) transport->Script(unauthorized);
  std::mutex times_mu;
  std::vector<double> post_times;
  transport->SetResponder([&](const FakeTransport::Request&) -> std::optional<HttpResponse> {
    std::lock_guard<std::mutex> lock(times_mu);
    post_times.push_back(SteadyNow());
    return std::nullopt;
  });
  session->SetUploadsPaused(false);

  ASSERT_TRUE(WaitFor([&] { return session->stats().chunks_discarded == 1; }, 5.0))
      << "the probe never met its 400";
  // Room for a burst behind the probe to show; the next probe is retry_max_s away.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  {
    std::lock_guard<std::mutex> lock(times_mu);
    ASSERT_GE(post_times.size(), 2u);
    for (std::size_t i = 2; i < post_times.size(); ++i) {
      EXPECT_GE(post_times[i] - post_times[i - 1], 0.9 * cfg.retry_max_s)
          << "the probe sent more than its one chunk";
    }
  }
  EXPECT_TRUE(session->stats().uploads_auth_paused);
  EXPECT_EQ(spool->Count(), spooled - 1);
  session->End();
}

// A refusal can be about one route: a stream the project is not entitled to, a proxy rule on one
// path. The pause begins as for a refused key, but the probes go to one stream after another, and
// once one is accepted every stream but the refused ones uploads. The refused streams' chunks are
// kept, one probe per retry_max_s asks about them in turn, and each stream goes once accepted.
TEST_F(RecoveryTest, AStreamWhoseCredentialsAreRefusedStopsOnlyItself) {
  std::mutex mu;
  std::vector<std::pair<double, std::string>> refused_posts;  // time, stream
  std::vector<std::string> delivered;
  std::atomic<bool> camera_refused{true};
  std::atomic<bool> sensors_refused{true};
  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  cfg.unauthorized_retry_s = 0.01;
  cfg.retry_max_s = 0.25;
  auto logger = std::make_shared<CapturingLogger>();
  auto session = Session::Create(cfg, transport, spool, logger);
  ASSERT_TRUE(session->Begin());
  for (int i = 0; i < 3; ++i) {
    session->RecordCameraAccessUnit(
        "cam", SensorEntry{1700000000000ull + i, static_cast<std::uint32_t>(i), Pose{}},
        VideoChunkContext{}, std::string(100, 'x'), true);
    session->RecordSensor("battery", 1000.0 + i, static_cast<double>(i));
    session->RecordEvent("event_" + std::to_string(i), 1000.0 + i, Vec3{});
    session->Flush();
  }

  transport->SetResponder([&](const FakeTransport::Request& r) -> std::optional<HttpResponse> {
    const double now =
        std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    const bool camera = r.url.find("/v0/camera") != std::string::npos;
    const bool sensors = r.url.find("/v0/sensors/") != std::string::npos;
    std::lock_guard<std::mutex> lock(mu);
    if ((camera && camera_refused.load()) || (sensors && sensors_refused.load())) {
      refused_posts.emplace_back(now, camera ? "camera" : "sensors");
      HttpResponse refused;
      refused.status = 403;
      return refused;
    }
    if (r.url.find("/events/") != std::string::npos) {
      const json body = json::parse(r.body);
      for (const auto& rec : body["data"]) delivered.push_back(rec["name"].get<std::string>());
    }
    return std::nullopt;
  });
  session->SetUploadsPaused(false);

  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 6; }, 3.0))
      << spool->Count() << " chunk(s) left: the accepted streams stayed paused behind the refused";
  Stats stats = session->stats();
  EXPECT_FALSE(stats.uploads_auth_paused) << "events and gaze are accepted";
  EXPECT_EQ(stats.auth_refused_streams, (std::vector<Stream>{Stream::kSensors, Stream::kCamera}));
  EXPECT_EQ(stats.chunks_discarded, 0u);
  {
    std::lock_guard<std::mutex> lock(mu);
    EXPECT_EQ(delivered,
              (std::vector<std::string>{"c3d.sessionStart", "event_0", "event_1", "event_2"}));
  }
  bool said_which = false;
  for (const auto& message : logger->messages()) {
    if (message.find("but sensors, camera") != std::string::npos) said_which = true;
  }
  EXPECT_TRUE(said_which) << "the resume did not name the streams still refused";

  // Kept, and asked about once per retry_max_s across both refused streams, never in a burst.
  ASSERT_TRUE(WaitFor(
      [&] {
        std::lock_guard<std::mutex> lock(mu);
        return refused_posts.size() >= 6;
      },
      5.0))
      << "the refused streams were not asked about again";
  EXPECT_EQ(spool->Count(), 6u);
  {
    std::lock_guard<std::mutex> lock(mu);
    int camera = 0, sensors = 0;
    double closest = std::numeric_limits<double>::infinity();
    for (std::size_t i = 2; i < refused_posts.size(); ++i) {
      const double gap = refused_posts[i].first - refused_posts[i - 1].first;
      EXPECT_GE(gap, 0.9 * cfg.retry_max_s) << "refused request " << i << " came before retry_max_s";
      closest = std::min(closest, gap);
      (refused_posts[i].second == "camera" ? camera : sensors)++;
    }
    // A late probe lengthens only its own gap, so the closest pair bounds the interval from above.
    EXPECT_LT(closest, 1.5 * cfg.retry_max_s) << "no probe came within 1.5 x retry_max_s of the last";
    EXPECT_GE(camera, 2);
    EXPECT_GE(sensors, 1) << "one refused stream was never asked about again";
  }

  // Camera is accepted first, while sensors is still refused: its turn comes round.
  camera_refused.store(false);
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 3; }, 3.0))
      << "the camera chunks never went once their stream was accepted";
  EXPECT_EQ(session->stats().auth_refused_streams, (std::vector<Stream>{Stream::kSensors}));
  sensors_refused.store(false);
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; }, 3.0));
  EXPECT_TRUE(session->stats().auth_refused_streams.empty());
  bool said_accepted = false;
  for (const auto& message : logger->messages()) {
    if (message.find("accepted again for stream camera") != std::string::npos) said_accepted = true;
  }
  EXPECT_TRUE(said_accepted);
  session->SetUploadsPaused(true);
  session.reset();
}

// A refused key refuses every stream. Probing a stream at a time must not turn into hammering:
// after the allowance there is one request per retry_max_s however many streams are spooled, and
// the probes take the streams in turn.
TEST_F(RecoveryTest, ARefusedKeyIsProbedOncePerIntervalAStreamAtATime) {
  std::mutex mu;
  std::vector<std::pair<double, std::string>> posts;  // time, stream
  std::atomic<bool> refused{true};
  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  cfg.unauthorized_retry_s = 0.01;
  cfg.retry_max_s = 0.2;
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
  ASSERT_TRUE(session->Begin());
  for (int i = 0; i < 3; ++i) {
    session->RecordCameraAccessUnit(
        "cam", SensorEntry{1700000000000ull + i, static_cast<std::uint32_t>(i), Pose{}},
        VideoChunkContext{}, std::string(100, 'x'), true);
    session->RecordEvent("event_" + std::to_string(i), 1000.0 + i, Vec3{});
    session->Flush();
  }
  const std::size_t spooled = spool->Count();

  transport->SetResponder([&](const FakeTransport::Request& r) -> std::optional<HttpResponse> {
    std::string stream = "?";
    for (const char* name : {"events", "gaze", "camera"}) {
      if (r.url.find(std::string("/v0/") + name) != std::string::npos) stream = name;
    }
    std::lock_guard<std::mutex> lock(mu);
    posts.emplace_back(
        std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(),
        stream);
    if (!refused.load()) return std::nullopt;
    HttpResponse unauthorized;
    unauthorized.status = 401;
    return unauthorized;
  });
  session->SetUploadsPaused(false);
  ASSERT_TRUE(WaitFor(
      [&] {
        std::lock_guard<std::mutex> lock(mu);
        return posts.size() >= 8;
      },
      5.0))
      << "the allowance and two rounds of probes, at least";
  EXPECT_TRUE(session->stats().uploads_auth_paused);
  EXPECT_EQ(spool->Count(), spooled) << "a refused key must not cost any data";
  EXPECT_EQ(session->stats().chunks_discarded, 0u);
  {
    std::lock_guard<std::mutex> lock(mu);
    double closest = std::numeric_limits<double>::infinity();
    for (std::size_t i = 2; i < posts.size(); ++i) {
      const double gap = posts[i].first - posts[i - 1].first;
      EXPECT_GE(gap, 0.9 * cfg.retry_max_s) << "request " << i << " came before retry_max_s";
      closest = std::min(closest, gap);
    }
    EXPECT_LT(closest, 1.5 * cfg.retry_max_s) << "no probe came within 1.5 x retry_max_s of the last";
    // Three streams are spooled -- events, the properties part on gaze, camera -- and each round
    // of probes asks about every one of them once.
    const std::set<std::string> every = {"events", "gaze", "camera"};
    for (std::size_t round = 0; round < 2; ++round) {
      std::set<std::string> asked;
      for (std::size_t i = 2 + 3 * round; i < 5 + 3 * round; ++i) asked.insert(posts[i].second);
      EXPECT_EQ(asked, every) << "round " << round << " of probes did not take the streams in turn";
    }
  }

  refused.store(false);
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; }, 3.0));
  EXPECT_FALSE(session->stats().uploads_auth_paused);
  session->SetUploadsPaused(true);
  session.reset();
}

TEST_F(RecoveryTest, AnUnauthorizedRetryDoesNotEscalateTheTransientBackoff) {
  // The 401 path must not touch `current_backoff`. That curve is floored at retry_min_s -- 60 s
  // in production -- so escalating it would turn a credential hiccup into a minute-long hole
  // for every other stream.
  //
  // THE ORDER HERE IS THE TEST. No success may come between the 401s and the outage, because
  // the success path zeroes `current_backoff` and would erase an escalation before the assertion
  // could see it. So the allowance is spent, uploads pause, and the probe itself -- the next
  // request made -- meets a genuine transient outage.
  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(dir_.string());
  cfg.unauthorized_retry_s = 0.01;
  cfg.retry_min_s = 1.0;   // distinguishable from a doubling of itself
  cfg.retry_max_s = 1.5;   // the probe interval
  auto logger = std::make_shared<CapturingLogger>();
  auto session = Session::Create(cfg, transport, spool, logger);
  ASSERT_TRUE(session->Begin());
  session->Flush();
  for (int i = 0; i < 40 && spool->Count() > 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    session->PumpUploads();
  }
  ASSERT_EQ(spool->Count(), 0u) << "start from a drained spool, or the scripted "
                                   "responses land on the wrong chunk";

  HttpResponse unauthorized;
  unauthorized.status = 401;
  HttpResponse outage;
  outage.status = 503;
  transport->Script(unauthorized);
  transport->Script(unauthorized);
  transport->Script(outage);

  session->RecordEvent("blip", 4000.0, Vec3{});
  session->Flush();
  session->PumpUploads();
  EXPECT_FALSE(session->stats().backing_off)
      << "a 401 must not raise the transient-outage flag";
  for (int i = 0; i < 40 && !session->stats().uploads_auth_paused; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    session->PumpUploads();
  }
  ASSERT_TRUE(session->stats().uploads_auth_paused);
  EXPECT_FALSE(session->stats().backing_off);

  for (int i = 0; i < 300 && !session->stats().backing_off; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    session->PumpUploads();
  }
  ASSERT_TRUE(session->stats().backing_off) << "the probe never met the outage";

  bool from_retry_min = false, escalated = false;
  for (const auto& message : logger->messages()) {
    if (message.find("backing off 1s") != std::string::npos) from_retry_min = true;
    if (message.find("backing off 2s") != std::string::npos) escalated = true;
  }
  EXPECT_TRUE(from_retry_min)
      << "the transient curve must start at retry_min_s, not wherever the 401 left it";
  EXPECT_FALSE(escalated) << "the 401 path escalated current_backoff";
  EXPECT_EQ(session->stats().chunks_discarded, 0u);
  session->End();
}

TEST_F(RecoveryTest, ThePauseMessageNamesHowManyAttemptsWereActuallyMade) {
  // The counter is cleared as the pause begins, so the message must read it first: "after 0
  // attempt(s)" reads as "the allowance never engaged", which is the wrong diagnosis.
  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(dir_.string());
  cfg.unauthorized_retry_s = 0.01;
  cfg.retry_max_s = 30.0;
  auto logger = std::make_shared<CapturingLogger>();
  auto session = Session::Create(cfg, transport, spool, logger);
  ASSERT_TRUE(session->Begin());
  session->Flush();
  for (int i = 0; i < 40 && spool->Count() > 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    session->PumpUploads();
  }

  HttpResponse unauthorized;
  unauthorized.status = 401;
  for (int i = 0; i < 4; ++i) transport->Script(unauthorized);

  session->RecordEvent("give_up", 7000.0, Vec3{});
  session->Flush();
  for (int i = 0; i < 40 && !session->stats().uploads_auth_paused; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    session->PumpUploads();
  }
  ASSERT_TRUE(session->stats().uploads_auth_paused);

  bool named_the_attempts = false, claimed_zero = false;
  for (const auto& message : logger->messages()) {
    if (message.find("after 2 attempt(s)") != std::string::npos) named_the_attempts = true;
    if (message.find("after 0 attempt(s)") != std::string::npos) claimed_zero = true;
  }
  EXPECT_TRUE(named_the_attempts) << "the pause message must say how many attempts happened";
  EXPECT_FALSE(claimed_zero) << "the counter was read after it had been cleared";
  session->End();
}

TEST_F(RecoveryTest, UnauthorizedRetryCanBeDisabled) {
  for (const int attempts_allowed : {0, 1}) {
    SCOPED_TRACE("cfg.unauthorized_attempts = " + std::to_string(attempts_allowed));
    const fs::path sub_dir = dir_ / std::to_string(attempts_allowed);
    fs::create_directories(sub_dir);

    auto transport = std::make_shared<FakeTransport>();
    auto spool = std::make_shared<DiskSpool>(sub_dir.string(), 64ull * 1024 * 1024,
                                             std::make_shared<NullLogger>());
    auto cfg = MakeConfig(sub_dir.string());
    cfg.unauthorized_attempts = attempts_allowed;
    cfg.unauthorized_retry_s = 0.01;
    cfg.retry_max_s = 30.0;
    auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
    ASSERT_TRUE(session->Begin());
    session->Flush();
    session->PumpUploads();

    HttpResponse unauthorized;
    unauthorized.status = 401;
    for (int i = 0; i < 5; ++i) transport->Script(unauthorized);

    session->RecordEvent("disabled_retry", 6000.0, Vec3{});
    session->Flush();
    for (int i = 0; i < 20; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      session->PumpUploads();
    }

    EXPECT_EQ(spool->Count(), 1u) << "the refused chunk is kept";
    EXPECT_EQ(session->stats().chunks_discarded, 0u);
    EXPECT_TRUE(session->stats().uploads_auth_paused);

    int posts_for_chunk = 0;
    for (const auto& r : transport->requests()) {
      if (r.url.find("/events/") == std::string::npos) continue;
      const json body = json::parse(r.body);
      for (const auto& rec : body["data"]) {
        if (rec["name"] == "disabled_retry") ++posts_for_chunk;
      }
    }
    EXPECT_EQ(posts_for_chunk, 1)
        << "unauthorized_attempts=" << attempts_allowed << " must mean one POST, then the pause";
    session->End();
  }
}

// --- Recovery must never adopt a session that is still alive ----------------
//
// `RecoverOrphanedSessions` closes a marker and writes an `abnormalEnd` for it.
// Matching a marker against this process's own session id guards nothing, since
// at configure time that id is EMPTY. What keeps a co-tenant on the same spool
// directory from having its running session declared abnormally ended and its
// marker deleted -- leaving a genuine later crash unrecoverable -- is the
// heartbeat, which answers "is this alive?" every few seconds.

TEST_F(RecoveryTest, LeavesAMarkerThatIsStillBeingHeartbeatenAlone) {
  WriteMarkerWithLastSeen(dir_, "live-session", NowSeconds() - 120.0, NowSeconds());

  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(dir_.string());
  cfg.orphan_stale_after_s = 30.0;      // realistic window; the marker beats now
  cfg.marker_heartbeat_s = 5.0;
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());

  EXPECT_EQ(session->RecoverOrphanedSessions(), 0) << "a live session was adopted";
  EXPECT_EQ(spool->OpenMarkers().size(), 1u)
      << "its marker must survive, or a genuine later crash is unrecoverable";
}

TEST_F(RecoveryTest, AdoptsAMarkerThatHasGoneSilent) {
  WriteMarkerWithLastSeen(dir_, "dead-session", NowSeconds() - 600.0, NowSeconds() - 300.0);

  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(dir_.string());
  cfg.marker_heartbeat_s = 5.0;
  cfg.orphan_stale_after_s = 30.0;
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());

  EXPECT_EQ(session->RecoverOrphanedSessions(), 1);
  EXPECT_TRUE(spool->OpenMarkers().empty());
}

TEST_F(RecoveryTest, AdoptsAMarkerThatNeverGotItsFirstHeartbeat) {
  // A session that died within one heartbeat of starting: last_seen is 0, which
  // must read as "never seen alive".
  //
  // The threshold here is deliberately larger than the epoch. At any realistic
  // threshold `now - 0` is ~1.8e9 seconds and already counts as stale, so the
  // explicit `last_seen > 0.0` clause decides nothing and a test at 30 s passes
  // whether or not that clause exists. This is the only
  // regime where the clause is load-bearing, so it is the only regime that
  // tests it. The clause stays because a caller may legitimately want a very
  // long window, and "never seen alive" should not become "seen very long ago".
  WriteMarkerWithLastSeen(dir_, "stillborn", NowSeconds() - 1.0, 0.0);

  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(dir_.string());
  cfg.marker_heartbeat_s = 5.0;
  cfg.orphan_stale_after_s = 1e12;
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());

  EXPECT_EQ(session->RecoverOrphanedSessions(), 1)
      << "a marker that was never heartbeaten must be adopted at any threshold";
}

TEST_F(RecoveryTest, TheHeartbeatIntervalIsTheConfiguredOne) {
  // Every other test's window is far shorter than either value, so without this
  // one a hardcoded 5.0 s interval passes the whole suite. A short configured
  // interval must produce a refreshed marker quickly.
  auto transport = std::make_shared<FakeTransport>();
  transport->SetOffline(true);
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(dir_.string());
  cfg.marker_heartbeat_s = 0.01;
  cfg.orphan_stale_after_s = 0.05;
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
  ASSERT_TRUE(session->Begin());

  // The SECOND refresh is the one that measures the interval. `last_marker_touch`
  // starts at 0, so the worker's very first tick satisfies `now - 0 > interval`
  // for ANY interval and touches immediately -- a test comparing against
  // Begin()'s touch sees that first tick and passes even with the interval
  // hardcoded to 5 s.
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  const auto after_first_tick = spool->OpenMarkers();
  ASSERT_EQ(after_first_tick.size(), 1u);
  const double baseline = after_first_tick[0].last_seen;

  // Many configured intervals, but far less than the 5 s default.
  EXPECT_TRUE(WaitFor(
      [&] {
        const auto later = spool->OpenMarkers();
        return later.size() == 1 && later[0].last_seen > baseline;
      },
      2.0))
      << "the marker was not refreshed again within 2 s, so the heartbeat is "
         "not using marker_heartbeat_s";
}

TEST_F(RecoveryTest, TreatsAMarkerFromAFasterClockAsAlive) {
  // Two containers can disagree about the time. A last_seen in the future makes
  // `now - last_seen` negative, which must read as live rather than as
  // infinitely stale -- the conservative direction, because waiting costs a
  // delay while guessing wrong corrupts a running measurement.
  WriteMarkerWithLastSeen(dir_, "skewed", NowSeconds(), NowSeconds() + 3600.0);

  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(dir_.string());
  cfg.marker_heartbeat_s = 5.0;
  cfg.orphan_stale_after_s = 30.0;
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());

  EXPECT_EQ(session->RecoverOrphanedSessions(), 0);
  EXPECT_EQ(spool->OpenMarkers().size(), 1u);
}

// A sessions/ that cannot be stat'd, under a live session. The heartbeat, the marker written
// before an events part, and the recovery the worker runs again later all meet it on a thread
// where an escaped exception ends the process. It must survive them, in a child of its own so a
// death is a result rather than the end of the run.
TEST_F(RecoveryTest, TheWorkerSurvivesASessionsDirectoryThatCannotBeStatted) {
  // A declared budget the child cannot outlast however slowly it starts; the recheck still comes
  // at the reader's own threshold.
  WriteMarkerWithLastSeen(dir_, "still-running", NowSeconds() - 60.0, NowSeconds(), 30.0);
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                             std::make_shared<NullLogger>());
    auto cfg = MakeConfig(dir_.string());
    cfg.uploads_paused = true;
    auto session = Session::Create(cfg, std::make_shared<FakeTransport>(), spool,
                                   std::make_shared<NullLogger>());
    // An entry that cannot be stat'd, beside a marker left alone for being fresh, so the
    // worker reads sessions/ again shortly...
    fs::create_symlink(dir_ / "sessions" / "loop.c3dsession",
                       dir_ / "sessions" / "loop.c3dsession");
    if (!session || session->RecoverOrphanedSessions() != 0 || !session->Begin()) ::_exit(2);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    // ...and then the whole directory, for the heartbeat and the marker before an events part.
    std::error_code ec;
    fs::remove_all(dir_ / "sessions", ec);
    fs::create_symlink(dir_ / "sessions", dir_ / "sessions", ec);
    session->RecordEvent("during", 1000.0, Vec3{});
    session->Flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    session.release();
    ::_exit(0);
  }
  int status = 0;
  ::waitpid(child, &status, 0);
  EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0)
      << (WIFSIGNALED(status) ? "the process was killed by signal " + std::to_string(WTERMSIG(status))
                              : "the child exited " + std::to_string(WEXITSTATUS(status)));
}

TEST(RecoveryConfig, RefusesAConfigurationThatWouldAdoptLiveSessions) {
  Config cfg;
  cfg.application_key = "APPKEY";
  cfg.scene_id = "SCENEGUID";
  cfg.scene_version = "2";
  cfg.device_id = "robot01";
  cfg.spool_dir = "/tmp/does-not-matter";

  cfg.marker_heartbeat_s = 5.0;
  cfg.orphan_stale_after_s = 5.0;   // equal: stale the instant it beats
  EXPECT_NE(cfg.Validate(), "");
  cfg.orphan_stale_after_s = 1.0;
  EXPECT_NE(cfg.Validate(), "");
  cfg.orphan_stale_after_s = 30.0;
  EXPECT_EQ(cfg.Validate(), "");
  cfg.marker_heartbeat_s = 0.0;
  EXPECT_NE(cfg.Validate(), "");
}

// ---------------------------------------------------------------------------
// The staleness gate is only as good as the heartbeat it reads. These pin the
// two ways the heartbeat could stop without the session being dead.
// ---------------------------------------------------------------------------

TEST_F(RecoveryTest, TheHeartbeatKeepsBeatingThroughALongUploadBacklog) {
  // The uploader sends up to 64 chunks a pass, and a heartbeat that has to wait for the pass
  // goes silent for the whole drain of a backlog over a slow-but-working link; a co-tenant then
  // reads the session as dead and adopts it. At production defaults that needs about 0.47 s per
  // POST across 64 queued chunks, which is an ordinary LTE link.
  //
  // What is measured is the SDK's own cadence: how long it leaves between finishing one marker
  // write and starting the next. How long the filesystem takes over each write is not the
  // SDK's to control, and under fsync load it alone exceeds any window this test could use.
  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<MarkerClockSpool>(std::make_shared<DiskSpool>(
      dir_.string(), 64ull * 1024 * 1024, std::make_shared<NullLogger>()));
  auto cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  cfg.marker_heartbeat_s = 0.02;
  cfg.orphan_stale_after_s = 0.40;
  cfg.gaze_batch = 1;

  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
  ASSERT_TRUE(session->Begin());
  const std::string live_id = session->session_id();

  // A slow-but-WORKING link, and a backlog of separate chunks to push through it: each Flush
  // cuts one part per stream. Built while paused rather than by going offline, because a
  // failed POST arms the back-off, which would make this measure the back-off instead.
  for (int i = 0; i < 40; ++i) {
    session->RecordGaze(1000.0 + i, Vec3{}, Quat{});
    session->Flush();
  }
  transport->SetPostDelay(std::chrono::milliseconds(15));
  const double drain_started = SteadyNow();
  session->SetUploadsPaused(false);
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; }, 30.0)) << "the backlog never drained";
  const double drained = SteadyNow();
  ASSERT_GT(drained - drain_started, 0.5) << "the drain was too quick to starve anything";

  // The healthy gap is a heartbeat interval plus a worker tick, about 30 ms; a heartbeat that
  // waits for the pass leaves one as long as the drain.
  EXPECT_LT(spool->LongestIdle(live_id, drain_started, drained), 0.30)
      << "the heartbeat waited behind the uploader";
  session->End();
}

TEST_F(RecoveryTest, EndClosesTheMarkerBeforeTheBacklogDrainsSoNothingCanAdoptIt) {
  // A co-tenant adopting a session during its orderly end writes an abnormalEnd that collides
  // with the real end on session id and replaces it. A drain with the marker open would leave
  // the heartbeat to cover the whole drain, so the marker goes as soon as the end is spooled:
  // End() returns with it gone, while an upload is stuck, and the backlog drains after.
  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  cfg.marker_heartbeat_s = 0.02;
  cfg.orphan_stale_after_s = 0.40;
  cfg.gaze_batch = 1;

  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
  ASSERT_TRUE(session->Begin());
  const std::string live_id = session->session_id();
  for (int i = 0; i < 10; ++i) {
    session->RecordGaze(1000.0 + i, Vec3{}, Quat{});
    session->Flush();
  }
  transport->Stall();
  session->SetUploadsPaused(false);
  ASSERT_TRUE(transport->WaitUntilStalled(std::chrono::seconds(5))) << "no upload ever started";

  // The bound is a watchdog: an End() that waited for the stuck upload would never return.
  auto ended = std::async(std::launch::async, [&] { session->End("orderly"); });
  const bool returned = ended.wait_for(std::chrono::seconds(5)) == std::future_status::ready;
  const bool marker_gone = MarkerLastSeen(dir_, live_id) < 0.0;
  const bool upload_still_stuck = transport->stalled_posts() == 1;
  const std::size_t backlog = spool->Count();
  // A co-tenant reading the same markers meanwhile finds nothing to adopt. Its uploads are
  // paused: it shares the spool, and an uploader of its own would drain the backlog through
  // its own transport whenever this thread is descheduled before the Unstall below.
  auto other_cfg = MakeConfig(dir_.string());
  other_cfg.uploads_paused = true;
  auto other = Session::Create(other_cfg, std::make_shared<FakeTransport>(), spool,
                               std::make_shared<NullLogger>());
  const int adopted = other->RecoverOrphanedSessions();
  transport->Unstall();
  ended.wait();

  EXPECT_TRUE(returned) << "End() waited for an upload";
  EXPECT_TRUE(upload_still_stuck) << "the upload finished first; the test proves nothing";
  EXPECT_TRUE(marker_gone) << "End() returned with the marker still open";
  EXPECT_GT(backlog, 1u) << "the backlog drained before End() returned; proves nothing";
  EXPECT_EQ(adopted, 0);
  EXPECT_TRUE(WaitFor([&] { return spool->Count() == 0; }, 5.0));
  int ends = 0;
  for (const auto& r : transport->requests()) {
    if (r.body.find("c3d.sessionEnd") != std::string::npos) ++ends;
  }
  EXPECT_EQ(ends, 1);
}

TEST_F(RecoveryTest, AMarkerLeftAloneForBeingFreshIsLookedAtAgain) {
  // Recovery runs once, from on_configure, so a marker deferred as "probably
  // still running" must be revisited later. Otherwise a node that crashes and
  // respawns inside the staleness window (systemd Restart=on-failure,
  // respawn:=True, both 1-5s) defers its OWN crashed session forever.
  WriteMarkerWithLastSeen(dir_, "crashed-a-moment-ago", NowSeconds() - 60.0, NowSeconds());

  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(dir_.string());
  cfg.marker_heartbeat_s = 0.02;
  // Longer than opening the spool and the session can take, so the marker is still fresh when
  // recovery first looks at it.
  cfg.orphan_stale_after_s = 1.0;
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());

  EXPECT_EQ(session->RecoverOrphanedSessions(), 0) << "too fresh to judge: must be deferred";
  EXPECT_EQ(OnlyOpenMarkerId(spool), "crashed-a-moment-ago");

  // Begin() starts the worker, which is what re-runs recovery once the deferred
  // marker has had its window. Nothing else touches that marker.
  ASSERT_TRUE(session->Begin());
  const bool closed = WaitFor(
      [&] {
        const auto markers = spool->OpenMarkers();
        return std::none_of(markers.begin(), markers.end(), [](const SessionMarker& m) {
          return m.session_id == "crashed-a-moment-ago";
        });
      },
      10.0);
  EXPECT_TRUE(closed) << "the deferred marker was never re-examined, so it leaks forever";
}

TEST_F(RecoveryTest, JudgesAMarkerByTheHeartbeatItsWriterPromised) {
  // Staleness judged with the RECOVERING process's threshold against a marker
  // written at the WRITER's heartbeat lets two configs that each pass Validate()
  // have one adopt the other's live session: writer at 60/120, reader at the
  // defaults 5/30. So the marker records the writer's budget.
  WriteMarkerWithLastSeen(dir_, "slow-beater", NowSeconds() - 600.0, NowSeconds() - 100.0, 300.0);
  WriteMarkerWithLastSeen(dir_, "undeclared", NowSeconds() - 600.0, NowSeconds() - 100.0);

  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(dir_.string());
  cfg.marker_heartbeat_s = 5.0;
  cfg.orphan_stale_after_s = 30.0;
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());

  // 100s of silence: past the reader's own 30s, but well inside the writer's
  // declared 300s budget. The one that declares nothing gets the reader's
  // threshold.
  EXPECT_EQ(session->RecoverOrphanedSessions(), 1) << "only the undeclared marker is judgeable";
  EXPECT_EQ(OnlyOpenMarkerId(spool), "slow-beater")
      << "a writer that declared a 300s silence budget was adopted after 100s of silence";
}

TEST_F(RecoveryTest, TheStalenessThresholdIsTheConfiguredOneNotAMultipleOfTheHeartbeat) {
  // Mutation guard. `silent_for < 6.0 * marker_heartbeat_s` passes any test
  // whose stale_after happens to be six heartbeats, which does not separate the
  // two values. Here they are far apart in
  // the direction that distinguishes them -- 0.5s of silence is inside the
  // configured 5.0s threshold but outside 6 x 0.01s.
  WriteMarkerWithLastSeen(dir_, "beats-rarely", NowSeconds() - 60.0, NowSeconds() - 0.5);

  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(dir_.string());
  cfg.worker_tick_s = 0.01;
  cfg.marker_heartbeat_s = 0.01;
  cfg.orphan_stale_after_s = 5.0;
  ASSERT_EQ(cfg.Validate(), "");
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());

  EXPECT_EQ(session->RecoverOrphanedSessions(), 0)
      << "orphan_stale_after_s is the threshold, not a multiple of the heartbeat";
}

TEST(RecoveryConfig, RefusesNonFiniteHeartbeatAndThreshold) {
  // NaN fails every comparison: `NaN <= 0.0` and `x <= NaN` are both false, so an
  // unchecked NaN passes BOTH checks and then makes `silent_for <
  // orphan_stale_after_s` false for every marker -- recovery adopting everything,
  // which is exactly the defect these checks exist to prevent. Inf is the mirror:
  // nothing is ever stale and dead markers leak forever.
  Config cfg;
  cfg.application_key = "APPKEY";
  cfg.scene_id = "SCENEGUID";
  cfg.scene_version = "2";
  cfg.device_id = "robot01";
  cfg.spool_dir = "/tmp/does-not-matter";
  ASSERT_EQ(cfg.Validate(), "") << "the defaults must be valid, or this proves nothing";

  const double nan_value = std::numeric_limits<double>::quiet_NaN();
  const double inf_value = std::numeric_limits<double>::infinity();
  for (double bad : {nan_value, inf_value}) {
    cfg.marker_heartbeat_s = bad;
    cfg.orphan_stale_after_s = 30.0;
    EXPECT_NE(cfg.Validate(), "") << "a non-finite marker_heartbeat_s was accepted";
    cfg.marker_heartbeat_s = 5.0;
    cfg.orphan_stale_after_s = bad;
    EXPECT_NE(cfg.Validate(), "") << "a non-finite orphan_stale_after_s was accepted";
  }
}

TEST(RecoveryConfig, TheMarginMustClearTheWorkerTick) {
  // A heartbeat becomes DUE on a tick boundary, so the real maximum interval
  // between two touches is marker_heartbeat_s + worker_tick_s. Requiring only
  // `stale > heartbeat` accepts 5.0 / 5.01 / 0.05 while a perfectly healthy
  // session can go 5.05s without touching -- a configuration that passes
  // validation and adopts live sessions anyway.
  Config cfg;
  cfg.application_key = "APPKEY";
  cfg.scene_id = "SCENEGUID";
  cfg.scene_version = "2";
  cfg.device_id = "robot01";
  cfg.spool_dir = "/tmp/does-not-matter";
  cfg.worker_tick_s = 0.05;
  cfg.marker_heartbeat_s = 5.0;

  cfg.orphan_stale_after_s = 5.01;
  EXPECT_NE(cfg.Validate(), "") << "a threshold inside one worker tick of the heartbeat";
  cfg.orphan_stale_after_s = 5.05;
  EXPECT_NE(cfg.Validate(), "") << "exactly one tick is still not a margin";
  cfg.orphan_stale_after_s = 5.10;
  EXPECT_EQ(cfg.Validate(), "");
}

// A spool that lets a test hold WriteMarker open, so the window between "decide
// to heartbeat" and "the marker hits disk" can be inspected rather than raced.
class GatedMarkerSpool : public ISpool {
 public:
  explicit GatedMarkerSpool(std::shared_ptr<ISpool> inner) : inner_(std::move(inner)) {}

  void WriteMarker(const SessionMarker& m) override {
    if (gate_.exchange(false)) {
      entered_.store(true);
      while (!release_.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    inner_->WriteMarker(m);
  }
  void CloseMarker(const std::string& id) override { inner_->CloseMarker(id); }
  std::vector<SessionMarker> OpenMarkers() const override { return inner_->OpenMarkers(); }
  bool Write(const Chunk& c) override { return inner_->Write(c); }
  using ISpool::PeekOldest;
  bool PeekOldest(Chunk* out, const std::set<std::string>& skip) override {
    return inner_->PeekOldest(out, skip);
  }
  void Remove(const std::string& id) override { inner_->Remove(id); }
  std::uint64_t TotalBytes() const override { return inner_->TotalBytes(); }
  std::size_t Count() const override { return inner_->Count(); }

  void ArmGate() { gate_.store(true); }
  bool Entered() const { return entered_.load(); }
  void Release() { release_.store(true); }

 private:
  std::shared_ptr<ISpool> inner_;
  std::atomic<bool> gate_{false};
  std::atomic<bool> entered_{false};
  std::atomic<bool> release_{false};
};

TEST_F(RecoveryTest, AHeartbeatInFlightCannotReopenAMarkerThatEndAlreadyClosed) {
  // A heartbeat decided before End() and written after it would put back the marker End()
  // removed, and an orderly-ended session would reappear as an open marker for the next
  // recovery to adopt: a false `abnormalEnd`. Every marker write and its removal are
  // serialised, and a write for a session this Session no longer owns is dropped.
  auto inner = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto spool = std::make_shared<GatedMarkerSpool>(inner);
  auto transport = std::make_shared<FakeTransport>();
  auto cfg = MakeConfig(dir_.string());
  cfg.marker_heartbeat_s = 0.01;
  cfg.orphan_stale_after_s = 0.05;

  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
  ASSERT_TRUE(session->Begin());
  const std::string id = session->session_id();

  // Catch the worker inside a heartbeat write, then end the session underneath it.
  spool->ArmGate();
  for (int i = 0; i < 200 && !spool->Entered(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  ASSERT_TRUE(spool->Entered()) << "never caught a heartbeat write; test proves nothing";

  std::thread ender([&] { session->End("orderly"); });
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  spool->Release();
  ender.join();

  // Give any in-flight write time to land before judging.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const auto markers = inner->OpenMarkers();
  EXPECT_TRUE(std::none_of(markers.begin(), markers.end(),
                           [&](const SessionMarker& m) { return m.session_id == id; }))
      << "an orderly-ended session was reopened by an in-flight heartbeat";

  auto reader = Session::Create(MakeConfig(dir_.string()), std::make_shared<FakeTransport>(),
                                inner, std::make_shared<NullLogger>());
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  EXPECT_EQ(reader->RecoverOrphanedSessions(), 0)
      << "the resurrected marker was adopted, writing abnormalEnd for a clean session";
}

TEST_F(RecoveryTest, TheHeartbeatKeepsBeatingWhileTheTransportIsBackingOff) {
  // The worker-tick heartbeat is the ONLY one during backoff: PumpUploadsInternal
  // returns at its backoff guard before reaching the per-POST touch. That is the
  // offline robot, which is the spool's primary scenario. A short retry ceiling
  // lets the pump's own touch mask a missing worker touch, so this test uses a
  // realistic one.
  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(dir_.string());
  cfg.marker_heartbeat_s = 0.02;
  cfg.orphan_stale_after_s = 0.10;
  cfg.retry_min_s = 30.0;  // a realistic ceiling: the pump will not run again
  cfg.retry_max_s = 30.0;  // within this test, so only the worker tick can beat

  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
  ASSERT_TRUE(session->Begin());
  const std::string id = session->session_id();

  transport->SetOffline(true);
  session->RecordGaze(1000.0, Vec3{}, Quat{});
  session->Flush();
  ASSERT_TRUE(WaitFor([&] { return session->stats().backing_off; }, 5.0))
      << "the pump never attempted; backoff never armed";

  const double before = MarkerLastSeen(dir_, id);
  EXPECT_TRUE(WaitFor([&] { return MarkerLastSeen(dir_, id) > before; }, 5.0))
      << "the marker went unrefreshed while the transport was backing off, so a "
         "co-tenant would adopt a live session that is merely offline";
}

TEST_F(RecoveryTest, TheReaderNeverJudgesFasterThanItsOwnThreshold) {
  // The floor in StaleAfterFor, pinned by a marker declaring a budget SMALLER
  // than the reader's threshold: returning the writer's budget alone passes
  // every test that lacks one. That floor is what keeps a
  // fast-declaring writer from being adopted early by a slow, careful reader.
  WriteMarkerWithLastSeen(dir_, "brief-budget", NowSeconds() - 600.0, NowSeconds() - 10.0, 1.0);

  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(dir_.string());
  cfg.marker_heartbeat_s = 5.0;
  cfg.orphan_stale_after_s = 30.0;
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());

  // 10s of silence: past the writer's declared 1s, but inside the reader's 30s.
  EXPECT_EQ(session->RecoverOrphanedSessions(), 0)
      << "the reader adopted sooner than its own orphan_stale_after_s allows";
}

TEST_F(RecoveryTest, DeferredRecoveryIsRetriedWithoutEverCallingBegin) {
  // `auto_start_session: false` is the node's orchestrated mode, and a node in it
  // may never call Begin(). The recheck runs on the worker, so a worker started
  // only by Begin() would leave a crashed-and-respawned session open on the
  // platform indefinitely, waiting for a Begin() that is not coming.
  WriteMarkerWithLastSeen(dir_, "crashed-no-begin", NowSeconds() - 60.0, NowSeconds());

  auto transport = std::make_shared<FakeTransport>();
  auto spool = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto cfg = MakeConfig(dir_.string());
  cfg.marker_heartbeat_s = 0.02;
  // Longer than opening the spool and the session can take, so the marker is still fresh when
  // recovery first looks at it.
  cfg.orphan_stale_after_s = 1.0;
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());

  EXPECT_EQ(session->RecoverOrphanedSessions(), 0) << "too fresh to judge: must be deferred";

  // Deliberately NO Begin().
  const bool closed = WaitFor(
      [&] {
        const auto markers = spool->OpenMarkers();
        return std::none_of(markers.begin(), markers.end(), [](const SessionMarker& m) {
          return m.session_id == "crashed-no-begin";
        });
      },
      10.0);
  EXPECT_TRUE(closed) << "the deferred marker is only revisited if Begin() is ever called";
}

// A spool whose marker scan takes long enough for two recovery passes to overlap.
class SlowMarkerScanSpool : public ISpool {
 public:
  explicit SlowMarkerScanSpool(std::shared_ptr<ISpool> inner) : inner_(std::move(inner)) {}
  bool Write(const Chunk& c) override { return inner_->Write(c); }
  using ISpool::PeekOldest;
  bool PeekOldest(Chunk* out, const std::set<std::string>& skip) override {
    return inner_->PeekOldest(out, skip);
  }
  void Remove(const std::string& id) override { inner_->Remove(id); }
  std::uint64_t TotalBytes() const override { return inner_->TotalBytes(); }
  std::size_t Count() const override { return inner_->Count(); }
  void WriteMarker(const SessionMarker& m) override { inner_->WriteMarker(m); }
  std::vector<SessionMarker> OpenMarkers() const override {
    auto markers = inner_->OpenMarkers();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return markers;
  }
  void CloseMarker(const std::string& id) override { inner_->CloseMarker(id); }

 private:
  std::shared_ptr<ISpool> inner_;
};

// Two recovery passes at once -- a consumer's call beside the worker's recheck, or two consumer
// calls -- adopt an orphan once between them. Each adopting it would spool an abnormalEnd of its
// own, a second end for one session.
TEST_F(RecoveryTest, TwoRecoveriesAtOnceAdoptAnOrphanOnce) {
  WriteMarkerWithLastSeen(dir_, "1700000000_robot01", 1700000000.0, 1700000100.0);
  auto cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  auto disk = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                          std::make_shared<NullLogger>());
  auto session = Session::Create(cfg, std::make_shared<FakeTransport>(),
                                 std::make_shared<SlowMarkerScanSpool>(disk),
                                 std::make_shared<NullLogger>());
  std::atomic<int> recovered{0};
  std::thread a([&] { recovered += session->RecoverOrphanedSessions(); });
  std::thread b([&] { recovered += session->RecoverOrphanedSessions(); });
  a.join();
  b.join();
  EXPECT_EQ(recovered.load(), 1);
  int ends = 0;
  Chunk chunk;
  std::set<std::string> seen;
  while (disk->PeekOldest(&chunk, seen)) {
    seen.insert(chunk.id);
    if (chunk.body.find("abnormalEnd") != std::string::npos) ++ends;
  }
  EXPECT_EQ(ends, 1) << "one orphan, two ends spooled";
}

// ---------------------------------------------------------------------------
// Validate() reports EVERY missing mandatory value, not one per round trip.
//
// Returning only the first costs the operator a lifecycle configure per unset
// field, and a fresh integration normally has several unset at once: three
// unset values would take three restarts to learn a static fact about a file.
// ---------------------------------------------------------------------------
TEST(ConfigValidate, ReportsEveryMissingMandatoryValueAtOnce) {
  Config cfg;
  cfg.application_key = "APPKEY";
  cfg.scene_id = "SCENEGUID";
  cfg.scene_version = "2";
  cfg.device_id = "robot01";
  cfg.spool_dir = "/tmp/does-not-matter";
  ASSERT_EQ(cfg.Validate(), "") << "the baseline must be valid, or this proves nothing";

  // One problem returns its BARE message. The single-problem case is by far the
  // common one and its wording is load-bearing in logs and in operator habit,
  // so it must not acquire a count prefix.
  cfg.application_key = "";
  EXPECT_EQ(cfg.Validate(), "application_key is empty");

  // Three problems: all three named, in one string, from one call.
  cfg.scene_id = "";
  cfg.scene_version = "";
  const std::string all = cfg.Validate();
  EXPECT_NE(all.find("application_key"), std::string::npos) << all;
  EXPECT_NE(all.find("scene_id"), std::string::npos) << all;
  EXPECT_NE(all.find("scene_version"), std::string::npos) << all;

  // The separator must not collide with the "; why it matters" clause several
  // of these messages carry. Joining with "; " would make three problems read
  // as five.
  EXPECT_NE(all.find(" | "), std::string::npos)
      << "problems must be separated by \" | \": " << all;
  EXPECT_NE(all.find("3 configuration problems"), std::string::npos)
      << "the count tells the operator how many restarts they just saved: " << all;

  // A range check still short-circuits: those are not independent, so the first
  // failure is the useful one.
  cfg.application_key = "APPKEY";
  cfg.scene_id = "SCENEGUID";
  cfg.scene_version = "2";
  cfg.http_timeout_s = -1.0;
  EXPECT_NE(cfg.Validate(), "");
}
