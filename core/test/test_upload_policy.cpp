// What each answer from the gateway does to the data. Only a payload rejection discards; a
// refused credential pauses everything (test_recovery.cpp); every other answer keeps the chunk
// and backs off. A chunk the gateway fails on its own is moved out of the way of the rest.

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <functional>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "c3d/session.hpp"
#include "c3d/spool.hpp"
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

bool WaitFor(const std::function<bool()>& done, double seconds) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (std::chrono::steady_clock::now() < deadline) {
    if (done()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return done();
}

// Every event name the transport saw delivered with a 200, in the order it saw them.
std::vector<std::string> EventNamesPosted(const FakeTransport& transport) {
  std::vector<std::string> names;
  for (const auto& r : transport.requests()) {
    if (r.url.find("/events/") == std::string::npos) continue;
    const json body = json::parse(r.body);
    for (const auto& rec : body["data"]) names.push_back(rec["name"].get<std::string>());
  }
  return names;
}

class UploadPolicyTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() /
           ("c3d_policy_" + std::to_string(::getpid()) + "_" + std::to_string(counter_++));
    fs::create_directories(dir_);
  }
  void TearDown() override {
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }
  // A session whose spool holds `events` chunks, one event each, with uploads paused so the
  // test decides when the first request goes out.
  std::unique_ptr<Session> SessionWithBacklog(const Config& base, int events,
                                              std::shared_ptr<ITransport> transport,
                                              std::shared_ptr<ILogger> log,
                                              std::shared_ptr<DiskSpool>* spool_out) {
    Config cfg = base;
    cfg.uploads_paused = true;
    auto spool = std::make_shared<DiskSpool>(cfg.spool_dir, 64ull * 1024 * 1024,
                                             std::make_shared<NullLogger>());
    auto session = Session::Create(cfg, transport, spool, log);
    EXPECT_TRUE(session->Begin());
    session->Flush();
    for (int i = 0; i < events; ++i) {
      session->RecordEvent("event_" + std::to_string(i), 1000.0 + i, Vec3{});
      session->Flush();
    }
    *spool_out = spool;
    return session;
  }
  fs::path dir_;
  static int counter_;
};
int UploadPolicyTest::counter_ = 0;

}  // namespace

// A rate limit, a gateway timeout, a redirect, a wrong route, a server error, a captive portal,
// no answer at all: none of them says anything is wrong with the payload, so none of them may
// cost it.
TEST_F(UploadPolicyTest, EveryAnswerThatIsNotAboutThePayloadKeepsTheChunk) {
  for (const long status : {0L, 204L, 301L, 302L, 308L, 404L, 408L, 409L, 429L, 500L, 503L}) {
    SCOPED_TRACE("status " + std::to_string(status));
    const fs::path sub = dir_ / std::to_string(status);
    fs::create_directories(sub);
    auto transport = std::make_shared<FakeTransport>();
    std::shared_ptr<DiskSpool> spool;
    auto session = SessionWithBacklog(MakeConfig(sub.string()), 1, transport,
                                      std::make_shared<NullLogger>(), &spool);
    HttpResponse answer;
    answer.status = status;
    // The 204 arrives unstamped, as a portal's does.
    answer.saw_request_time_header = status != 0 && status != 204;
    transport->Script(answer);
    session->SetUploadsPaused(false);
    ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; }, 3.0))
        << "the chunk was never delivered after the one " << status;
    EXPECT_EQ(session->stats().chunks_discarded, 0u);
    EXPECT_GE(session->stats().upload_failures, 1u);
    session->End();
  }
}

// The gateway's stamp is what says a request was received. A stamped 2xx other than 200 is a
// delivery; re-sending it would duplicate the part.
TEST_F(UploadPolicyTest, AnyStampedTwoHundredDeliversTheChunkOnce) {
  for (const long status : {201L, 202L, 204L}) {
    SCOPED_TRACE("status " + std::to_string(status));
    const fs::path sub = dir_ / std::to_string(status);
    fs::create_directories(sub);
    auto transport = std::make_shared<FakeTransport>();
    std::shared_ptr<DiskSpool> spool;
    auto session = SessionWithBacklog(MakeConfig(sub.string()), 1, transport,
                                      std::make_shared<NullLogger>(), &spool);
    const std::size_t spooled = spool->Count();
    HttpResponse answer;
    answer.status = status;
    answer.saw_request_time_header = true;
    transport->SetResponder([answer](const FakeTransport::Request&) { return answer; });
    session->SetUploadsPaused(false);
    ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; }, 3.0));
    EXPECT_EQ(transport->count(), spooled) << "a delivered chunk was sent again";
    EXPECT_EQ(session->stats().upload_failures, 0u);
    session->SetUploadsPaused(true);
    session.reset();
  }
}

// backing_off says a retryable failure's wait is running, and only that. A chunk that fails and is
// then discarded leaves nothing to back off from once the wait is over; and Begin, which clears
// the back-off so a new session's first parts go at once, clears it too.
TEST_F(UploadPolicyTest, BackingOffEndsWithTheWaitWhateverCameAfterTheFailure) {
  auto transport = std::make_shared<FakeTransport>();
  HttpResponse unavailable;
  unavailable.status = 503;
  HttpResponse rejected;
  rejected.status = 400;
  transport->Script(unavailable);
  transport->Script(rejected);
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  cfg.retry_min_s = 0.5;
  cfg.retry_max_s = 0.5;
  auto spool = std::make_shared<DiskSpool>(cfg.spool_dir, 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
  auto plant = [&] {
    Chunk c;
    c.url = "https://" + cfg.gateway + "/v0/events/SCENEGUID?version=2";
    c.scene_id = "SCENEGUID";
    c.scene_version = "2";
    c.body = "{}";
    c.session_id = "1_x";
    return spool->Write(c);
  };
  ASSERT_TRUE(plant());
  session->SetUploadsPaused(false);
  Stats failed;
  ASSERT_TRUE(WaitFor([&] {
    failed = session->stats();
    return failed.upload_failures >= 1;
  }, 3.0));
  EXPECT_TRUE(failed.backing_off) << "the wait after the 503 is running";
  // The spool empties as the chunk is removed, a moment before the uploader counts the discard.
  ASSERT_TRUE(WaitFor([&] {
    return spool->Count() == 0 && session->stats().chunks_discarded == 1u;
  }, 3.0));
  EXPECT_FALSE(session->stats().backing_off) << "the spool is empty and the wait is over";

  transport->Script(unavailable);
  ASSERT_TRUE(plant());
  ASSERT_TRUE(WaitFor([&] {
    failed = session->stats();
    return failed.upload_failures >= 2;
  }, 3.0));
  ASSERT_TRUE(failed.backing_off);
  ASSERT_TRUE(session->Begin());
  EXPECT_FALSE(session->stats().backing_off) << "Begin cleared the back-off, not what it reports";
  session->SetUploadsPaused(true);
  session.reset();
}

TEST_F(UploadPolicyTest, ARetryAfterLongerThanTheBackoffIsWaited) {
  std::mutex attempts_mu;
  std::vector<double> attempts;
  auto transport = std::make_shared<FakeTransport>();
  auto log = std::make_shared<CapturingLogger>();
  std::shared_ptr<DiskSpool> spool;
  auto session = SessionWithBacklog(MakeConfig(dir_.string()), 1, transport, log, &spool);
  transport->SetResponder([&](const FakeTransport::Request&) -> std::optional<HttpResponse> {
    std::lock_guard<std::mutex> lock(attempts_mu);
    attempts.push_back(
        std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count());
    if (attempts.size() > 1) return std::nullopt;
    HttpResponse limited;
    limited.status = 429;
    limited.retry_after_s = 0.6;
    return limited;
  });
  session->SetUploadsPaused(false);
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; }, 5.0));
  {
    std::lock_guard<std::mutex> lock(attempts_mu);
    ASSERT_GE(attempts.size(), 2u);
    EXPECT_GE(attempts[1] - attempts[0], 0.55)
        << "the retry came on the 0.02 s curve, not after the server's longer Retry-After";
  }
  EXPECT_EQ(log->Count("as the server's Retry-After asked"), 1);
  session.reset();
}

// A Retry-After never shortens the back-off. 0, a date the robot's clock has already passed and
// a value that is neither would otherwise retry at every worker tick, past retry_min_s, and a
// fleet multiplies that. Read off the wire by the real transport.
TEST_F(UploadPolicyTest, ARetryAfterNeverShortensTheBackoff) {
  struct Case {
    const char* value;
    const char* waited;
  };
  int n = 0;
  for (const Case& c : {Case{"0", "backing off 30s"},
                        Case{"Sun, 06 Nov 1994 08:49:37 GMT", "backing off 30s"},
                        Case{"soon", "backing off 30s"},
                        Case{"99999999999", "backing off 3600s"}}) {
    SCOPED_TRACE(std::string("Retry-After: ") + c.value);
    ThrottledHttpServer::Options options;
    options.response = std::string("HTTP/1.1 429 Too Many Requests\r\nRetry-After: ") + c.value +
                       "\r\nContent-Length: 0\r\n\r\n";
    ThrottledHttpServer server(options);
    const fs::path sub = dir_ / std::to_string(n++);
    fs::create_directories(sub);
    Config cfg = MakeConfig(sub.string());
    cfg.protocol = "http";
    cfg.gateway = server.host();
    cfg.retry_min_s = 30.0;
    cfg.retry_max_s = 30.0;
    auto log = std::make_shared<CapturingLogger>();
    std::shared_ptr<DiskSpool> spool;
    auto session = SessionWithBacklog(cfg, 1, std::make_shared<CurlTransport>(UploadTimeouts(cfg)),
                                      log, &spool);
    session->SetUploadsPaused(false);
    ASSERT_TRUE(WaitFor([&] { return server.completed() >= 1; }, 3.0));
    EXPECT_TRUE(WaitFor([&] { return log->Count(c.waited) >= 1; }, 5.0));
    // Room for a shortened back-off to retry.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    EXPECT_EQ(server.completed(), 1u) << "the back-off was shortened to the server's value";
    EXPECT_EQ(log->Count(c.waited), 1);
    session->SetUploadsPaused(true);
    session.reset();
  }
}

TEST_F(UploadPolicyTest, AnAbsurdRetryAfterIsCappedAtAnHour) {
  auto transport = std::make_shared<FakeTransport>();
  auto log = std::make_shared<CapturingLogger>();
  std::shared_ptr<DiskSpool> spool;
  auto session = SessionWithBacklog(MakeConfig(dir_.string()), 1, transport, log, &spool);
  HttpResponse limited;
  limited.status = 503;
  limited.retry_after_s = 1e9;
  transport->Script(limited);
  const std::size_t spooled = spool->Count();
  session->SetUploadsPaused(false);
  EXPECT_TRUE(WaitFor([&] { return log->Count("backing off 3600s") >= 1; }, 3.0));
  EXPECT_EQ(session->stats().upload_failures, 1u);
  EXPECT_EQ(log->Count("backing off 3600s"), 1);
  EXPECT_EQ(spool->Count(), spooled) << "everything is kept";
  session.reset();
}

TEST_F(UploadPolicyTest, A404IsKeptAndNamedAConfigurationProblem) {
  auto transport = std::make_shared<FakeTransport>();
  auto log = std::make_shared<CapturingLogger>();
  std::shared_ptr<DiskSpool> spool;
  auto session = SessionWithBacklog(MakeConfig(dir_.string()), 1, transport, log, &spool);
  HttpResponse missing;
  missing.status = 404;
  missing.saw_request_time_header = true;
  transport->Script(missing);
  session->SetUploadsPaused(false);
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; }, 3.0));
  EXPECT_EQ(session->stats().chunks_discarded, 0u);
  EXPECT_GE(log->Count("configuration problem"), 1);
  session->End();
}

// The gateway answers 500 to one particular payload every time, as an unhandled exception in a
// handler does. Retried at the head of the FIFO for ever, it would hold every chunk behind it
// until eviction began deleting data at the cap.
TEST_F(UploadPolicyTest, AChunkThatAlwaysFailsDoesNotBlockTheQueueAndIsNeverDeleted) {
  auto transport = std::make_shared<FakeTransport>();
  std::atomic<bool> poisoned{true};
  transport->SetResponder([&poisoned](const FakeTransport::Request& r) -> std::optional<HttpResponse> {
    if (!poisoned.load() || r.body.find("poison") == std::string::npos) return std::nullopt;
    HttpResponse failure;
    failure.status = 500;
    failure.saw_request_time_header = true;
    return failure;
  });
  auto log = std::make_shared<CapturingLogger>();
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  auto spool = std::make_shared<DiskSpool>(cfg.spool_dir, 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto session = Session::Create(cfg, transport, spool, log);
  ASSERT_TRUE(session->Begin());
  session->RecordEvent("poison", 1000.0, Vec3{});
  session->Flush();
  for (int i = 0; i < 5; ++i) {
    session->RecordEvent("healthy_" + std::to_string(i), 1001.0 + i, Vec3{});
    session->Flush();
  }
  session->SetUploadsPaused(false);

  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 1; }, 5.0))
      << spool->Count() << " chunk(s) left: the queue is stuck behind the failing one";
  const auto names = EventNamesPosted(*transport);
  for (int i = 0; i < 5; ++i) {
    EXPECT_NE(std::find(names.begin(), names.end(), "healthy_" + std::to_string(i)), names.end());
  }
  Stats stats = session->stats();
  EXPECT_EQ(stats.chunks_discarded, 0u) << "a 500 is never a reason to delete";
  EXPECT_EQ(stats.chunks_deferred, 1u);
  EXPECT_TRUE(stats.deferred_streams.empty()) << "one chunk failing on its own deferred its stream";
  EXPECT_EQ(log->Count("waits behind the rest of the spool"), 1);

  // It is still being tried, on its own interval, and goes once the gateway stops failing it.
  const auto tries = [&] {
    const auto posted = EventNamesPosted(*transport);
    return std::count(posted.begin(), posted.end(), std::string("poison"));
  };
  const auto before = tries();
  ASSERT_TRUE(WaitFor([&] { return tries() > before; }, 3.0)) << "the deferred chunk was abandoned";
  EXPECT_FALSE(session->stats().backing_off)
      << "a deferred chunk failing again must not hold back the rest of the spool";
  poisoned.store(false);
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; }, 10.0))
      << "the deferred chunk never went once the gateway accepted it";
  // The spool empties as the chunk is removed, a moment before the uploader counts again.
  EXPECT_TRUE(WaitFor([&] { return session->stats().chunks_deferred == 0u; }, 3.0))
      << session->stats().chunks_deferred << " deferred with nothing spooled";
  session->End();
}

// What a gateway saw, request by request: the stream, the event names an events part carried,
// and whether it was accepted.
struct Seen {
  std::string stream;
  std::vector<std::string> names;
  bool accepted = false;
};

std::string StreamOf(const std::string& url) {
  for (const char* stream : {"events", "gaze", "sensors", "dynamics", "camera", "lidar"}) {
    if (url.find(std::string("/") + stream + "/") != std::string::npos) return stream;
  }
  return "?";
}

Seen Describe(const FakeTransport::Request& r, bool accepted) {
  Seen seen;
  seen.stream = StreamOf(r.url);
  seen.accepted = accepted;
  if (seen.stream == "events") {
    const json body = json::parse(r.body);
    for (const auto& rec : body["data"]) seen.names.push_back(rec["name"].get<std::string>());
  }
  return seen;
}

// The requests in order, for a failure message.
std::string Trace(const std::vector<Seen>& seen) {
  std::string out;
  for (const auto& s : seen) {
    out += "\n  " + s.stream + (s.accepted ? " accepted" : " failed");
    for (const auto& name : s.names) out += " " + name;
  }
  return out;
}

std::vector<std::string> AcceptedEventNames(const std::vector<Seen>& seen) {
  std::vector<std::string> names;
  for (const auto& s : seen) {
    if (s.accepted) names.insert(names.end(), s.names.begin(), s.names.end());
  }
  return names;
}

// A network that is down, or a load balancer with nothing behind it, is nobody's chunk: no
// strike may be counted for it, or an ordinary outage would reorder the backlog behind it.
TEST_F(UploadPolicyTest, AnOutageDefersNoChunkAndTheBacklogStaysInOrder) {
  for (const long status : {0L, 502L, 503L, 504L}) {
    SCOPED_TRACE("status " + std::to_string(status));
    const fs::path sub = dir_ / std::to_string(status);
    fs::create_directories(sub);
    auto transport = std::make_shared<FakeTransport>();
    std::atomic<bool> down{true};
    std::mutex seen_mu;
    std::vector<Seen> seen;
    transport->SetResponder([&](const FakeTransport::Request& r) -> std::optional<HttpResponse> {
      std::lock_guard<std::mutex> lock(seen_mu);
      const bool is_down = down.load();
      seen.push_back(Describe(r, !is_down));
      if (!is_down) return std::nullopt;
      HttpResponse failure;
      failure.status = status;
      failure.saw_request_time_header = status != 0;
      return failure;
    });
    std::shared_ptr<DiskSpool> spool;
    auto session = SessionWithBacklog(MakeConfig(sub.string()), 5, transport,
                                      std::make_shared<NullLogger>(), &spool);
    session->SetUploadsPaused(false);
    ASSERT_TRUE(WaitFor([&] { return session->stats().upload_failures >= 12; }, 5.0));
    EXPECT_EQ(session->stats().chunks_deferred, 0u);
    down.store(false);
    ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; }, 5.0));
    session->SetUploadsPaused(true);

    session.reset();
    std::lock_guard<std::mutex> lock(seen_mu);
    const std::vector<std::string> expected = {"c3d.sessionStart", "event_0", "event_1",
                                               "event_2", "event_3", "event_4"};
    EXPECT_EQ(AcceptedEventNames(seen), expected);
    // Nothing but the head was ever tried during the outage.
    for (const auto& attempt : seen) {
      if (attempt.accepted) break;
      EXPECT_EQ(attempt.names, std::vector<std::string>{"c3d.sessionStart"});
    }
  }
}

// A gateway answering 500 to everything strikes every chunk alike. The head becomes the
// deferral candidate, but a witness sent in its place fails too, so nothing is deferred, the
// head is tried again, and each stream arrives in the order it was spooled.
TEST_F(UploadPolicyTest, AServerErrorOnEveryChunkIsAnOutageAndDefersNothing) {
  auto transport = std::make_shared<FakeTransport>();
  std::atomic<bool> down{true};
  std::mutex seen_mu;
  std::vector<Seen> seen;
  transport->SetResponder([&](const FakeTransport::Request& r) -> std::optional<HttpResponse> {
    std::lock_guard<std::mutex> lock(seen_mu);
    const bool is_down = down.load();
    seen.push_back(Describe(r, !is_down));
    if (!is_down) return std::nullopt;
    HttpResponse failure;
    failure.status = 500;
    failure.saw_request_time_header = true;
    return failure;
  });
  std::shared_ptr<DiskSpool> spool;
  auto session = SessionWithBacklog(MakeConfig(dir_.string()), 5, transport,
                                    std::make_shared<NullLogger>(), &spool);
  session->SetUploadsPaused(false);
  ASSERT_TRUE(WaitFor([&] { return session->stats().upload_failures >= 16; }, 5.0));
  EXPECT_EQ(session->stats().chunks_deferred, 0u) << "an outage deferred a chunk";
  down.store(false);
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; }, 5.0));
  EXPECT_EQ(session->stats().chunks_deferred, 0u);
  session->SetUploadsPaused(true);

  session.reset();
  std::lock_guard<std::mutex> lock(seen_mu);
  const std::vector<std::string> expected = {"c3d.sessionStart", "event_0", "event_1",
                                             "event_2", "event_3", "event_4"};
  EXPECT_EQ(AcceptedEventNames(seen), expected);
  int head = 0, witness = 0;
  for (const auto& attempt : seen) {
    if (attempt.accepted) break;
    if (attempt.names == std::vector<std::string>{"c3d.sessionStart"}) {
      ++head;
    } else {
      ++witness;
      EXPECT_EQ(attempt.stream, "gaze") << "the witness was not taken from another stream";
    }
  }
  EXPECT_GE(witness, 2) << "the head's strikes never led to a witness";
  EXPECT_GE(head, 2 * witness) << "after a failed witness the head was not tried again";
}

// An outage that ends just as the witness goes. The witness is accepted, but the candidate is
// tried again before anything is decided, and it is accepted too: nothing was wrong with it.
TEST_F(UploadPolicyTest, AnOutageThatEndsOnTheWitnessDefersNothing) {
  auto transport = std::make_shared<FakeTransport>();
  std::mutex seen_mu;
  std::vector<Seen> seen;
  int head_failures = 0;
  bool up = false;
  transport->SetResponder([&](const FakeTransport::Request& r) -> std::optional<HttpResponse> {
    const bool is_head = r.body.find("c3d.sessionStart") != std::string::npos;
    std::lock_guard<std::mutex> lock(seen_mu);
    if (!up && !is_head && head_failures >= 3) up = true;
    seen.push_back(Describe(r, up));
    if (up) return std::nullopt;
    if (is_head) ++head_failures;
    HttpResponse failure;
    failure.status = 500;
    failure.saw_request_time_header = true;
    return failure;
  });
  auto log = std::make_shared<CapturingLogger>();
  std::shared_ptr<DiskSpool> spool;
  auto session = SessionWithBacklog(MakeConfig(dir_.string()), 2, transport, log, &spool);
  session->SetUploadsPaused(false);
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; }, 5.0));
  session->SetUploadsPaused(true);
  EXPECT_EQ(log->Count("waits behind the rest of the spool"), 0)
      << "a chunk was deferred on a witness alone, without failing again after it";
  session.reset();
  std::lock_guard<std::mutex> lock(seen_mu);
  EXPECT_TRUE(up) << "the witness was never sent";
  EXPECT_EQ(AcceptedEventNames(seen),
            (std::vector<std::string>{"c3d.sessionStart", "event_0", "event_1"}))
      << "the head went behind the backlog";
}

// A witness is accepted and then the gateway goes down again, first with a 503 and then with
// 500s. The 503 says outage, so the candidate's next 500 is not held against it.
TEST_F(UploadPolicyTest, AnOutageAfterTheWitnessWithdrawsWhatItSaid) {
  auto transport = std::make_shared<FakeTransport>();
  std::mutex phase_mu;
  int phase = 0;  // 0: only the head fails; 1: the head meets a 503; 2: everything 500s; 3: up
  int head_failures = 0;
  int outage_requests = 0;
  std::vector<Seen> seen;
  transport->SetResponder([&](const FakeTransport::Request& r) -> std::optional<HttpResponse> {
    const bool is_head = r.body.find("c3d.sessionStart") != std::string::npos;
    std::lock_guard<std::mutex> lock(phase_mu);
    seen.push_back(Describe(r, phase == 3 || (phase == 0 && !is_head)));
    HttpResponse failure;
    failure.saw_request_time_header = true;
    failure.status = 500;
    switch (phase) {
      case 0:
        if (!is_head) {
          if (head_failures >= 3) phase = 1;
          return std::nullopt;
        }
        ++head_failures;
        return failure;
      case 1:
        phase = 2;
        failure.status = 503;
        return failure;
      case 2:
        if (++outage_requests >= 3) phase = 3;
        return failure;
      default:
        return std::nullopt;
    }
  });
  auto log = std::make_shared<CapturingLogger>();
  std::shared_ptr<DiskSpool> spool;
  auto session = SessionWithBacklog(MakeConfig(dir_.string()), 2, transport, log, &spool);
  session->SetUploadsPaused(false);
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 0; }, 5.0));
  session->SetUploadsPaused(true);
  EXPECT_EQ(log->Count("waits behind the rest of the spool"), 0)
      << "a witness from before the outage got the head deferred during it";
  session.reset();
  std::lock_guard<std::mutex> lock(phase_mu);
  EXPECT_EQ(phase, 3) << "the scenario did not run as scripted";
  EXPECT_EQ(AcceptedEventNames(seen),
            (std::vector<std::string>{"c3d.sessionStart", "event_0", "event_1"}))
      << Trace(seen);
}

// The candidate's own stream keeps its order when it can: the witness is the oldest chunk of
// another stream, not simply the next chunk.
TEST_F(UploadPolicyTest, TheWitnessComesFromAnotherStreamWhenOneIsSpooled) {
  auto transport = std::make_shared<FakeTransport>();
  std::mutex seen_mu;
  std::vector<Seen> seen;
  transport->SetResponder([&](const FakeTransport::Request& r) -> std::optional<HttpResponse> {
    const bool poison = r.body.find("poison") != std::string::npos;
    std::lock_guard<std::mutex> lock(seen_mu);
    seen.push_back(Describe(r, !poison));
    if (!poison) return std::nullopt;
    HttpResponse failure;
    failure.status = 500;
    failure.saw_request_time_header = true;
    return failure;
  });
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  auto spool = std::make_shared<DiskSpool>(cfg.spool_dir, 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
  ASSERT_TRUE(session->Begin());
  session->RecordEvent("poison", 1000.0, Vec3{});
  session->Flush();
  session->RecordEvent("healthy_0", 1001.0, Vec3{});
  session->Flush();
  session->RecordGaze(1002.0, Vec3{}, Quat{});
  session->Flush();
  session->RecordEvent("healthy_1", 1003.0, Vec3{});
  session->Flush();
  session->SetUploadsPaused(false);
  ASSERT_TRUE(WaitFor([&] { return session->stats().chunks_deferred == 1; }, 5.0));
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 1; }, 5.0));
  session->SetUploadsPaused(true);

  session.reset();
  std::lock_guard<std::mutex> lock(seen_mu);
  int poison_failures = 0;
  std::size_t at = 0;
  while (at < seen.size() && poison_failures < 3) {
    if (seen[at].names == std::vector<std::string>{"poison"}) ++poison_failures;
    ++at;
  }
  ASSERT_LT(at, seen.size());
  EXPECT_EQ(seen[at].stream, "gaze") << "the witness came from the candidate's own stream";
  EXPECT_EQ(AcceptedEventNames(seen),
            (std::vector<std::string>{"c3d.sessionStart", "healthy_0", "healthy_1"}));
}

// The responses a test's gateway gives sensors chunks, keyed by the one sample value each carries.
std::string SensorValueOf(const FakeTransport::Request& r) {
  const json body = json::parse(r.body);
  for (const auto& series : body["data"]) {
    for (const auto& sample : series["data"]) return sample[1].dump();
  }
  return "?";
}

HttpResponse ServerError() {
  HttpResponse failure;
  failure.status = 500;
  failure.saw_request_time_header = true;
  return failure;
}

// A gateway that fails one stream's every chunk chunk-shaped -- a schema mismatch on one route,
// say -- and accepts the rest. Judged one chunk at a time, each of that stream's chunks costs three
// strikes under the global back-off before it steps aside, and every stream waits behind it. The
// first chunk is judged as any chunk is; the second, of a stream already seen failing on its own,
// at its first failure; then the whole stream steps aside and the rest flow at once.
TEST_F(UploadPolicyTest, AStreamWhoseEveryChunkFailsStepsAsideAndTheRestFlowAtOnce) {
  auto transport = std::make_shared<FakeTransport>();
  std::mutex seen_mu;
  std::vector<std::string> sensors_tried;
  int events_delivered = 0;
  transport->SetResponder([&](const FakeTransport::Request& r) -> std::optional<HttpResponse> {
    std::lock_guard<std::mutex> lock(seen_mu);
    if (r.url.find("/sensors/") != std::string::npos) {
      sensors_tried.push_back(SensorValueOf(r));
      return ServerError();
    }
    if (r.url.find("/events/") != std::string::npos) ++events_delivered;
    return std::nullopt;
  });
  auto log = std::make_shared<CapturingLogger>();
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  // Strikes back off 0.1, 0.2 and 0.4 s. A deferred chunk or stream is next tried 5 s on, after
  // this test has looked; one sensors chunk at a time would take about 30 s to clear the events.
  cfg.retry_min_s = 0.1;
  cfg.retry_max_s = 5.0;
  auto spool = std::make_shared<DiskSpool>(cfg.spool_dir, 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto session = Session::Create(cfg, transport, spool, log);
  ASSERT_TRUE(session->Begin());
  // Two sensors parts for every events part, as a fast stream beside a slow one.
  for (int i = 0; i < 20; ++i) {
    session->RecordSensor("battery", 1000.0 + i, double(i));
    session->Flush();
    session->RecordSensor("battery", 1000.5 + i, double(i) + 100.0);
    session->Flush();
    session->RecordEvent("event_" + std::to_string(i), 1000.0 + i, Vec3{});
    session->Flush();
  }
  session->SetUploadsPaused(false);

  // Counted as each request arrives, so the spool is read as well: the last events chunk leaves
  // it once its answer is in.
  EXPECT_TRUE(WaitFor([&] {
    std::lock_guard<std::mutex> lock(seen_mu);
    return events_delivered == 21 && spool->Count() == 40u;
  }, 4.0)) << "the events are still stuck behind the failing stream";
  const Stats stats = session->stats();
  EXPECT_EQ(stats.deferred_streams, std::vector<Stream>{Stream::kSensors});
  EXPECT_EQ(stats.chunks_deferred, 2u) << "the two chunks the stream was judged on";
  EXPECT_EQ(stats.chunks_discarded, 0u) << "a 500 is never a reason to delete";
  EXPECT_EQ(spool->Count(), 40u) << "every sensors chunk is kept";
  EXPECT_EQ(log->Count("the whole sensors stream now waits behind the rest of the spool"), 1);
  {
    std::lock_guard<std::mutex> lock(seen_mu);
    // The first chunk: three strikes, then once more after a witness was accepted. The second:
    // one strike, and once more after a witness. No third chunk was tried.
    EXPECT_EQ(sensors_tried, (std::vector<std::string>{"0.0", "0.0", "0.0", "0.0", "100.0",
                                                       "100.0"}));
  }
  session->SetUploadsPaused(true);
  session.reset();
}

// A deferred stream is tried again on its own interval, and the first of its chunks the gateway
// accepts resumes the whole stream. The retry is never a chunk deferred on its own account, so one
// that always fails cannot keep its stream deferred once the rest are accepted again.
TEST_F(UploadPolicyTest, ADeferredStreamResumesOnceTheGatewayAcceptsItAgain) {
  auto transport = std::make_shared<FakeTransport>();
  std::atomic<bool> healed{false};
  std::atomic<int> sensors_requests{0};
  transport->SetResponder([&](const FakeTransport::Request& r) -> std::optional<HttpResponse> {
    if (r.url.find("/sensors/") == std::string::npos) return std::nullopt;
    ++sensors_requests;
    if (healed.load() && SensorValueOf(r) != "666.0") return std::nullopt;
    return ServerError();
  });
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  cfg.retry_min_s = 0.05;
  cfg.retry_max_s = 0.1;
  auto spool = std::make_shared<DiskSpool>(cfg.spool_dir, 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto log = std::make_shared<CapturingLogger>();
  auto session = Session::Create(cfg, transport, spool, log);
  ASSERT_TRUE(session->Begin());
  session->RecordSensor("battery", 1000.0, 666.0);  // fails whatever else happens
  session->Flush();
  for (int i = 0; i < 10; ++i) {
    session->RecordSensor("battery", 1001.0 + i, double(i));
    session->Flush();
    session->RecordEvent("event_" + std::to_string(i), 1001.0 + i, Vec3{});
    session->Flush();
  }
  session->SetUploadsPaused(false);
  ASSERT_TRUE(WaitFor([&] { return !session->stats().deferred_streams.empty(); }, 5.0))
      << "the stream was never deferred";
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 11u; }, 5.0))
      << "the events did not flow past the deferred stream";
  // The next failure is the stream's own retry. Read in the snapshot that first counts it, the
  // global back-off says whether that failure held back every other stream.
  const std::uint64_t failures_at_deferral = session->stats().upload_failures;
  Stats retried;
  EXPECT_TRUE(WaitFor([&] {
    retried = session->stats();
    return retried.upload_failures > failures_at_deferral;
  }, 5.0)) << "the deferred stream was never tried again";
  EXPECT_FALSE(retried.backing_off) << "a deferred stream failing again backed off every stream";
  EXPECT_FALSE(retried.deferred_streams.empty());

  healed.store(true);
  // chunks_deferred is counted a moment after the chunk leaves the spool.
  EXPECT_TRUE(WaitFor([&] {
    const Stats stats = session->stats();
    return spool->Count() == 1 && stats.deferred_streams.empty() && stats.chunks_deferred == 1u;
  }, 5.0)) << spool->Count() << " chunk(s) left, "
           << session->stats().deferred_streams.size() << " stream(s) deferred, "
           << session->stats().chunks_deferred << " chunk(s) deferred; one, which always fails, "
           << "should be";
  EXPECT_EQ(log->Count("deferred stream sensors accepted again"), 1)
      << "the stream was not resumed by the chunk the gateway accepted";
  session->SetUploadsPaused(true);
  session.reset();
}

// A gateway that starts failing everything after a stream was seen failing on its own. The
// stream's next chunk is judged at its first failure, but the witness fails too: an outage, so no
// stream is deferred and every stream keeps its order.
TEST_F(UploadPolicyTest, AnOutageAfterAStreamWasSeenFailingDefersNoStream) {
  auto transport = std::make_shared<FakeTransport>();
  std::atomic<int> phase{0};  // 0: only the poison fails; 1: everything fails; 2: up again
  std::mutex seen_mu;
  std::vector<Seen> seen;
  transport->SetResponder([&](const FakeTransport::Request& r) -> std::optional<HttpResponse> {
    const bool poison = r.url.find("/sensors/") != std::string::npos && SensorValueOf(r) == "666.0";
    const bool fails = poison || phase.load() == 1;
    std::lock_guard<std::mutex> lock(seen_mu);
    seen.push_back(Describe(r, !fails));
    if (fails) return ServerError();
    return std::nullopt;
  });
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  auto spool = std::make_shared<DiskSpool>(cfg.spool_dir, 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
  ASSERT_TRUE(session->Begin());
  session->RecordSensor("battery", 1000.0, 666.0);
  session->Flush();
  session->RecordEvent("witness", 1000.5, Vec3{});
  session->Flush();
  session->SetUploadsPaused(false);
  ASSERT_TRUE(WaitFor([&] { return session->stats().chunks_deferred == 1 && spool->Count() == 1; },
                      5.0));
  ASSERT_TRUE(session->stats().deferred_streams.empty());

  session->SetUploadsPaused(true);
  for (int i = 0; i < 3; ++i) {
    session->RecordSensor("battery", 1001.0 + i, double(i));
    session->Flush();
    session->RecordEvent("event_" + std::to_string(i), 1001.0 + i, Vec3{});
    session->Flush();
  }
  phase.store(1);
  const std::uint64_t failures = session->stats().upload_failures;
  session->SetUploadsPaused(false);
  ASSERT_TRUE(WaitFor([&] { return session->stats().upload_failures >= failures + 12; }, 5.0));
  EXPECT_TRUE(session->stats().deferred_streams.empty()) << "an outage deferred a stream";
  EXPECT_EQ(session->stats().chunks_deferred, 1u) << "an outage deferred a chunk";
  phase.store(2);
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 1; }, 5.0));
  session->SetUploadsPaused(true);
  session.reset();
  std::lock_guard<std::mutex> lock(seen_mu);
  EXPECT_EQ(AcceptedEventNames(seen),
            (std::vector<std::string>{"c3d.sessionStart", "witness", "event_0", "event_1",
                                      "event_2"}))
      << Trace(seen);
}

// A chunk of another stream starts failing on its own while a deferred stream's retry is due. It
// is still judged with a witness, and steps aside, rather than being retried in the witness's place
// under the global back-off for as long as the retry stays due behind it.
TEST_F(UploadPolicyTest, ACandidateIsStillWitnessedWhileADeferredStreamIsDue) {
  auto transport = std::make_shared<FakeTransport>();
  std::mutex seen_mu;
  std::vector<Seen> seen;
  transport->SetResponder([&](const FakeTransport::Request& r) -> std::optional<HttpResponse> {
    const bool fails = r.url.find("/sensors/") != std::string::npos ||
                       r.body.find("poison") != std::string::npos;
    std::lock_guard<std::mutex> lock(seen_mu);
    seen.push_back(Describe(r, !fails));
    if (fails) return ServerError();
    return std::nullopt;
  });
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  cfg.retry_min_s = 0.02;
  cfg.retry_max_s = 0.1;
  auto spool = std::make_shared<DiskSpool>(cfg.spool_dir, 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
  ASSERT_TRUE(session->Begin());
  for (int i = 0; i < 2; ++i) {
    session->RecordSensor("battery", 1000.0 + i, double(i));
    session->Flush();
    session->RecordEvent("witness_" + std::to_string(i), 1000.5 + i, Vec3{});
    session->Flush();
  }
  session->RecordEvent("poison", 1003.0, Vec3{});
  session->Flush();
  for (int i = 0; i < 2; ++i) {
    session->RecordEvent("healthy_" + std::to_string(i), 1004.0 + i, Vec3{});
    session->Flush();
    session->RecordSensor("battery", 1004.5 + i, double(i) + 10.0);
    session->Flush();
  }
  session->SetUploadsPaused(false);
  EXPECT_TRUE(WaitFor([&] {
    const auto names = [&] {
      std::lock_guard<std::mutex> lock(seen_mu);
      return AcceptedEventNames(seen);
    }();
    return std::find(names.begin(), names.end(), "healthy_1") != names.end();
  }, 3.0)) << "the events stayed behind a chunk that was never witnessed";
  EXPECT_EQ(session->stats().deferred_streams, std::vector<Stream>{Stream::kSensors});
  session->SetUploadsPaused(true);
  session.reset();
  std::lock_guard<std::mutex> lock(seen_mu);
  const auto names = AcceptedEventNames(seen);
  EXPECT_EQ(std::count(names.begin(), names.end(), "poison"), 0) << Trace(seen);
}

// Two chunks of one stream that fail on their own, with the stream's other chunks accepted
// between them, are two bad chunks and not a bad stream: each steps aside on its own and the
// stream keeps flowing.
TEST_F(UploadPolicyTest, TwoChunksThatFailApartDoNotDeferTheirStream) {
  auto transport = std::make_shared<FakeTransport>();
  transport->SetResponder([](const FakeTransport::Request& r) -> std::optional<HttpResponse> {
    if (r.body.find("poison") == std::string::npos) return std::nullopt;
    return ServerError();
  });
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  // A stream wrongly deferred would be tried again only after this test has looked.
  cfg.retry_max_s = 5.0;
  auto spool = std::make_shared<DiskSpool>(cfg.spool_dir, 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
  ASSERT_TRUE(session->Begin());
  for (const char* name : {"poison_0", "healthy_0", "healthy_1", "poison_1", "healthy_2",
                           "healthy_3"}) {
    session->RecordEvent(name, 1000.0, Vec3{});
    session->Flush();
  }
  session->SetUploadsPaused(false);
  ASSERT_TRUE(WaitFor([&] { return spool->Count() == 2; }, 5.0))
      << spool->Count() << " chunk(s) left";
  const Stats stats = session->stats();
  EXPECT_EQ(stats.chunks_deferred, 2u);
  EXPECT_TRUE(stats.deferred_streams.empty()) << "two chunks failing apart deferred their stream";
  session->SetUploadsPaused(true);
  session.reset();
}

// What the uploader remembers about a chunk goes with the chunk. One evicted at the cap, or
// removed by another user of the spool, no longer counts as held or deferred.
TEST_F(UploadPolicyTest, HeldAndDeferredCountsForgetAChunkThatLeftTheSpool) {
  auto transport = std::make_shared<FakeTransport>();
  transport->SetResponder([](const FakeTransport::Request& r) -> std::optional<HttpResponse> {
    if (r.body.find("poison") == std::string::npos) return std::nullopt;
    HttpResponse failure;
    failure.status = 500;
    failure.saw_request_time_header = true;
    return failure;
  });
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  auto spool = std::make_shared<DiskSpool>(cfg.spool_dir, 64ull * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  Chunk foreign;
  foreign.url = "https://collector.example.test/v0/events/SCENEGUID?version=2";
  foreign.scene_id = "SCENEGUID";
  foreign.scene_version = "2";
  foreign.body = "{}";
  ASSERT_TRUE(spool->Write(foreign));
  auto session = Session::Create(cfg, transport, spool, std::make_shared<NullLogger>());
  ASSERT_TRUE(session->Begin());
  session->RecordEvent("poison", 1000.0, Vec3{});
  session->Flush();
  session->RecordEvent("healthy", 1001.0, Vec3{});
  session->Flush();
  session->SetUploadsPaused(false);
  ASSERT_TRUE(WaitFor([&] {
    const Stats stats = session->stats();
    return stats.chunks_held == 1 && stats.chunks_deferred == 1;
  }, 5.0));

  // Both leave the spool behind the session's back.
  std::set<std::string> skip;
  Chunk chunk;
  while (spool->PeekOldest(&chunk, skip)) {
    skip.insert(chunk.id);
    if (chunk.url == foreign.url || chunk.body.find("poison") != std::string::npos) {
      spool->Remove(chunk.id);
    }
  }
  ASSERT_EQ(spool->Count(), 0u);
  EXPECT_TRUE(WaitFor([&] {
    const Stats stats = session->stats();
    return stats.chunks_held == 0 && stats.chunks_deferred == 0;
  }, 3.0)) << "held " << session->stats().chunks_held << ", deferred "
           << session->stats().chunks_deferred << " for chunks no longer spooled";
  session->SetUploadsPaused(true);
  session.reset();
}

// Under a refused key the cap is the one path by which data is lost, so its evictions are counted
// where the node and doctor read everything else.
TEST_F(UploadPolicyTest, ChunksEvictedAtTheCapAreInTheStats) {
  Config cfg = MakeConfig(dir_.string());
  cfg.uploads_paused = true;
  auto spool = std::make_shared<DiskSpool>(cfg.spool_dir, 4096, std::make_shared<NullLogger>());
  auto session = Session::Create(cfg, std::make_shared<FakeTransport>(), spool,
                                 std::make_shared<NullLogger>());
  ASSERT_TRUE(session->Begin());
  for (int i = 0; i < 30; ++i) {
    session->RecordGaze(1000.0 + i, Vec3{}, Quat{});
    session->Flush();
  }
  ASSERT_GT(spool->evicted(), 0u) << "the cap never evicted; the test proves nothing";
  EXPECT_EQ(session->stats().chunks_evicted, spool->evicted());
  session.reset();
}
