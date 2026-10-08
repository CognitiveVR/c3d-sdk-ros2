// Strings the SDK does not control reach every JSON stream: driver diagnostics, event names and
// properties, session names, participant ids, object names. DDS does not validate UTF-8, so one
// Latin-1 byte from a C++ driver -- a degree sign, 0xB0 -- must cost nothing worse than a
// replacement character, and a part that cannot be written for any reason must cost only itself.

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <functional>
#include <map>
#include <mutex>
#include <stdexcept>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "c3d/session.hpp"
#include "c3d/spool.hpp"
#include "fake_transport.hpp"

using namespace c3d;
using json = nlohmann::json;
namespace fs = std::filesystem;

namespace {

const std::string kReplacement = "\xEF\xBF\xBD";  // U+FFFD in UTF-8

class LinesLogger : public ILogger {
 public:
  void Log(LogLevel, const std::string& message) override {
    std::lock_guard<std::mutex> lock(mu_);
    lines_.push_back(message);
  }
  bool Contains(const std::string& fragment) const { return Count(fragment) > 0; }
  int Count(const std::string& fragment) const {
    std::lock_guard<std::mutex> lock(mu_);
    int n = 0;
    for (const auto& line : lines_) {
      if (line.find(fragment) != std::string::npos) ++n;
    }
    return n;
  }

 private:
  mutable std::mutex mu_;
  std::vector<std::string> lines_;
};

// Throws from Write for one stream while armed, and from CloseMarker while close_throws is set, the
// way a consumer-supplied spool or a failing allocation would, and forwards everything else. Counts
// what it wrote per stream: the session's uploader removes chunks from the moment of Create, so
// what REMAINS spooled is no measure of what was written.
class ThrowingSpool : public ISpool {
 public:
  explicit ThrowingSpool(std::shared_ptr<ISpool> inner) : inner_(std::move(inner)) {}

  bool Write(const Chunk& c) override {
    if (armed_.load() && c.stream == Stream::kEvents) {
      if (non_standard_.load()) throw 42;
      throw std::runtime_error("disk exploded");
    }
    if (!inner_->Write(c)) return false;
    std::lock_guard<std::mutex> lock(written_mu_);
    ++written_[c.stream];
    return true;
  }
  std::size_t Written(Stream stream) const {
    std::lock_guard<std::mutex> lock(written_mu_);
    const auto it = written_.find(stream);
    return it == written_.end() ? 0 : it->second;
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
  void CloseMarker(const std::string& id) override {
    if (close_throws.load()) throw std::runtime_error("close exploded");
    inner_->CloseMarker(id);
  }

  std::atomic<bool> close_throws{false};

  void Arm(bool non_standard = false) {
    non_standard_.store(non_standard);
    armed_.store(true);
  }
  void Disarm() { armed_.store(false); }

 private:
  std::shared_ptr<ISpool> inner_;
  std::atomic<bool> armed_{false};
  std::atomic<bool> non_standard_{false};
  mutable std::mutex written_mu_;
  std::map<Stream, std::size_t> written_;
};

class SerializationGuardTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() /
           ("c3d_guard_" + std::to_string(::getpid()) + "_" + std::to_string(counter_++));
    fs::create_directories(dir_);
    cfg_.application_key = "APPKEY";
    cfg_.scene_id = "SCENEGUID";
    cfg_.scene_version = "2";
    cfg_.device_id = "robot01";
    cfg_.spool_dir = dir_.string();
    cfg_.auto_send_interval_s = 3600.0;
    cfg_.worker_tick_s = 0.01;
    cfg_.retry_min_s = 0.05;
    cfg_.retry_max_s = 0.2;
    transport_ = std::make_shared<FakeTransport>();
    disk_ = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                        std::make_shared<NullLogger>());
  }
  void TearDown() override {
    session_.reset();
    disk_.reset();
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }
  // Every uploaded body for a stream, parsed with nlohmann's strict parser: a body holding an
  // invalid UTF-8 sequence does not parse, so a parse is itself the assertion.
  std::vector<json> BodiesFor(const std::string& stream) {
    std::vector<json> out;
    for (const auto& r : transport_->requests()) {
      if (r.url.find("/" + stream + "/") == std::string::npos) continue;
      out.push_back(json::parse(r.body));
    }
    return out;
  }
  bool WaitFor(const std::function<bool()>& done) {
    for (int i = 0; i < 300; ++i) {
      if (done()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return done();
  }

  fs::path dir_;
  Config cfg_;
  std::shared_ptr<FakeTransport> transport_;
  std::shared_ptr<DiskSpool> disk_;
  std::unique_ptr<Session> session_;
  static int counter_;
};
int SerializationGuardTest::counter_ = 0;

}  // namespace

TEST_F(SerializationGuardTest, ALatin1ByteReachesTheWireReplacedThroughEveryStringTheSdkCarries) {
  session_ = Session::Create(cfg_, transport_, disk_, std::make_shared<NullLogger>());
  ASSERT_NE(session_, nullptr);
  ASSERT_TRUE(session_->Begin());

  session_->SetSessionName("lab \xB0");
  session_->SetParticipantId("operator \xB0");
  session_->SetProperty("rack \xB0", std::string("85\xB0" "C"));
  Properties event_properties;
  event_properties.emplace_back("unit \xB0", std::string("\xB0" "C"));
  session_->RecordEvent(std::string("temperature 85\xB0" "C"), 1000.0, Vec3{}, event_properties,
                        "arm \xB0");
  session_->RecordSensor("battery \xB0", 1000.0, 0.5);
  session_->RegisterDynamic("object \xFF", "name \xFF", "mesh \xFF", 1000.0, Vec3{}, Quat{});
  Properties dynamic_properties;
  dynamic_properties.emplace_back("state \xFE", std::string("open \xFE"));
  session_->RecordDynamic("object \xFF", 1000.5, Vec3{}, Quat{}, std::nullopt, dynamic_properties);
  session_->RecordGaze(1000.0, Vec3{}, Quat{});

  ASSERT_NO_THROW(session_->Flush());
  session_->PumpUploads();

  bool event_seen = false;
  for (const auto& body : BodiesFor("events")) {
    for (const auto& record : body["data"]) {
      if (record["name"] != "temperature 85" + kReplacement + "C") continue;
      event_seen = true;
      EXPECT_EQ(record["dynamicId"], "arm " + kReplacement);
      EXPECT_EQ(record["properties"]["unit " + kReplacement], kReplacement + "C");
    }
  }
  EXPECT_TRUE(event_seen) << "the event never reached the wire";

  bool properties_seen = false;
  for (const auto& body : BodiesFor("gaze")) {
    const auto props = body.value("properties", json::object());
    if (!props.contains("c3d.sessionname")) continue;
    properties_seen = true;
    EXPECT_EQ(props["c3d.sessionname"], "lab " + kReplacement);
    EXPECT_EQ(props["c3d.participant.id"], "operator " + kReplacement);
    EXPECT_EQ(props["rack " + kReplacement], "85" + kReplacement + "C");
  }
  EXPECT_TRUE(properties_seen) << "the session properties never reached the wire";

  const auto sensors = BodiesFor("sensors");
  ASSERT_EQ(sensors.size(), 1u);
  EXPECT_EQ(sensors[0]["data"][0]["name"], "battery " + kReplacement);

  const auto dynamics = BodiesFor("dynamics");
  ASSERT_EQ(dynamics.size(), 1u);
  EXPECT_TRUE(dynamics[0]["manifest"].contains("object " + kReplacement));
  EXPECT_EQ(dynamics[0]["manifest"]["object " + kReplacement]["mesh"], "mesh " + kReplacement);

  const Stats stats = session_->stats();
  EXPECT_EQ(stats.serialization_failures, 0u);
  EXPECT_EQ(stats.records_dropped, 0u) << "a replaced byte must not cost the record";
}

// The worker thread cuts parts on its own schedule, and an exception escaping it is
// std::terminate: the node aborts, the respawn starts it again, the driver publishes the same
// diagnostic, and the robot crash-loops. Surviving is the assertion.
TEST_F(SerializationGuardTest, ABadByteCutOnTheWorkerThreadIsUploadedRatherThanEndingTheProcess) {
  cfg_.auto_send_interval_s = 0.05;
  session_ = Session::Create(cfg_, transport_, disk_, std::make_shared<NullLogger>());
  ASSERT_TRUE(session_->Begin());
  session_->RecordEvent(std::string("\xFF"), 1000.0, Vec3{});
  EXPECT_TRUE(WaitFor([&] {
    for (const auto& body : BodiesFor("events")) {
      for (const auto& record : body["data"]) {
        if (record["name"] == kReplacement) return true;
      }
    }
    return false;
  })) << "the worker never cut and uploaded the part";
}

TEST_F(SerializationGuardTest, AStreamThatThrowsCostsItsOwnPartCountedAndLogged) {
  auto spool = std::make_shared<ThrowingSpool>(disk_);
  auto log = std::make_shared<LinesLogger>();
  session_ = Session::Create(cfg_, transport_, spool, log);
  ASSERT_TRUE(session_->Begin());
  const std::size_t gaze_before = spool->Written(Stream::kGaze);

  spool->Arm();
  session_->RecordEvent("lost", 1000.0, Vec3{});
  session_->RecordGaze(1000.0, Vec3{}, Quat{});
  ASSERT_NO_THROW(session_->Flush());
  Stats stats = session_->stats();
  EXPECT_EQ(stats.serialization_failures, 1u);
  EXPECT_EQ(stats.records_dropped, 1u) << "the one event in the part that threw";
  EXPECT_EQ(spool->Written(Stream::kGaze), gaze_before + 1)
      << "the gaze part behind it must still spool";
  EXPECT_TRUE(log->Contains("could not spool a events part (disk exploded)"));

  // The batch is gone, not stuck: the next part of the same stream spools normally.
  spool->Disarm();
  session_->RecordEvent("kept", 1001.0, Vec3{});
  ASSERT_NO_THROW(session_->Flush());
  EXPECT_EQ(session_->stats().serialization_failures, 1u);
  session_->PumpUploads();
  bool kept = false, lost = false;
  for (const auto& body : BodiesFor("events")) {
    for (const auto& record : body["data"]) {
      if (record["name"] == "kept") kept = true;
      if (record["name"] == "lost") lost = true;
    }
  }
  EXPECT_TRUE(kept);
  EXPECT_FALSE(lost);
}

// The marker closes between the part carrying the end and the parts behind it, on the thread that
// called End(). A spool that throws from CloseMarker costs the close, said once, and neither those
// parts nor the caller.
TEST_F(SerializationGuardTest, AMarkerCloseThatThrowsCostsTheCloseNotThePartsBehindTheEnd) {
  cfg_.uploads_paused = true;
  auto spool = std::make_shared<ThrowingSpool>(disk_);
  auto log = std::make_shared<LinesLogger>();
  session_ = Session::Create(cfg_, transport_, spool, log);
  ASSERT_TRUE(session_->Begin());
  session_->RecordGaze(1000.0, Vec3{}, Quat{});
  const std::size_t gaze_before = spool->Written(Stream::kGaze);

  spool->close_throws.store(true);
  ASSERT_NO_THROW(session_->End("orderly"));
  EXPECT_EQ(spool->Written(Stream::kGaze), gaze_before + 1)
      << "the gaze part behind the end must still spool";
  EXPECT_EQ(log->Count("a session marker write threw (close exploded)"), 1);
  spool->close_throws.store(false);
}

TEST_F(SerializationGuardTest, ANonStandardExceptionIsCaughtToo) {
  auto spool = std::make_shared<ThrowingSpool>(disk_);
  auto log = std::make_shared<LinesLogger>();
  session_ = Session::Create(cfg_, transport_, spool, log);
  ASSERT_TRUE(session_->Begin());
  spool->Arm(/*non_standard=*/true);
  session_->RecordEvent("lost", 1000.0, Vec3{});
  ASSERT_NO_THROW(session_->Flush());
  EXPECT_EQ(session_->stats().serialization_failures, 1u);
  EXPECT_TRUE(log->Contains("non-standard exception"));
  spool->Disarm();
}

TEST_F(SerializationGuardTest, AThrowOnTheWorkerThreadIsCountedAndTheWorkerCarriesOn) {
  cfg_.auto_send_interval_s = 0.05;
  auto spool = std::make_shared<ThrowingSpool>(disk_);
  session_ = Session::Create(cfg_, transport_, spool, std::make_shared<NullLogger>());
  ASSERT_TRUE(session_->Begin());
  spool->Arm();
  session_->RecordEvent("lost", 1000.0, Vec3{});
  ASSERT_TRUE(WaitFor([&] { return session_->stats().serialization_failures >= 1; }));
  spool->Disarm();
  session_->RecordEvent("kept", 1001.0, Vec3{});
  EXPECT_TRUE(WaitFor([&] {
    for (const auto& body : BodiesFor("events")) {
      for (const auto& record : body["data"]) {
        if (record["name"] == "kept") return true;
      }
    }
    return false;
  })) << "the worker stopped cutting parts after the throw";
}

TEST(ConfigValidateUtf8, ADeviceIdThatIsNotUtf8IsRefused) {
  Config cfg;
  cfg.application_key = "APPKEY";
  cfg.scene_id = "SCENEGUID";
  cfg.scene_version = "2";
  cfg.device_id = "robot01";
  cfg.spool_dir = "/tmp/does-not-matter";
  ASSERT_EQ(cfg.Validate(), "");
  cfg.device_id = "robot\xB0";
  EXPECT_NE(cfg.Validate().find("device_id"), std::string::npos) << cfg.Validate();
  cfg.device_id = "robot \xC3\xA9";
  EXPECT_EQ(cfg.Validate(), "") << "valid UTF-8 beyond ASCII is a legitimate device id";
}

namespace {

// Throws from the calls the worker and the uploader make on their own, as a consumer-supplied
// spool might, and forwards everything to the real spool otherwise. A chunk whose body carries
// "remove_poison" throws on every removal and stays spooled.
class ThreadCallThrowingSpool : public ISpool {
 public:
  explicit ThreadCallThrowingSpool(std::shared_ptr<DiskSpool> inner) : inner_(std::move(inner)) {}

  bool Write(const Chunk& c) override { return inner_->Write(c); }
  using ISpool::PeekOldest;
  bool PeekOldest(Chunk* out, const std::set<std::string>& skip) override {
    if (peek_throws.load()) throw std::runtime_error("peek exploded");
    if (!inner_->PeekOldest(out, skip)) return false;
    std::lock_guard<std::mutex> lock(mu_);
    bodies_[out->id] = out->body;
    return true;
  }
  void Remove(const std::string& id) override {
    {
      std::lock_guard<std::mutex> lock(mu_);
      const auto it = bodies_.find(id);
      if (it != bodies_.end() && it->second.find("remove_poison") != std::string::npos) {
        throw std::runtime_error("remove exploded");
      }
    }
    inner_->Remove(id);
  }
  std::uint64_t TotalBytes() const override { return inner_->TotalBytes(); }
  std::size_t Count() const override { return inner_->Count(); }
  void WriteMarker(const SessionMarker& m) override {
    if (marker_throws.load()) throw 7;
    inner_->WriteMarker(m);
  }
  std::vector<SessionMarker> OpenMarkers() const override {
    if (open_markers_throws.load()) throw std::runtime_error("markers exploded");
    return inner_->OpenMarkers();
  }
  void CloseMarker(const std::string& id) override { inner_->CloseMarker(id); }
  bool Contains(const std::string& id) const override {
    if (contains_throws.load()) throw std::runtime_error("contains exploded");
    return inner_->Contains(id);
  }

  std::atomic<bool> peek_throws{false};
  std::atomic<bool> marker_throws{false};
  std::atomic<bool> contains_throws{false};
  std::atomic<bool> open_markers_throws{false};

 private:
  std::shared_ptr<DiskSpool> inner_;
  mutable std::mutex mu_;
  std::map<std::string, std::string> bodies_;
};

}  // namespace

// The heartbeat runs on the worker, and the marker write before an events part on whichever
// thread cuts it. A spool that throws from WriteMarker costs those writes and nothing else.
TEST_F(SerializationGuardTest, AMarkerWriteThatThrowsCostsTheWriteNotThePartOrTheProcess) {
  cfg_.marker_heartbeat_s = 0.02;
  auto spool = std::make_shared<ThreadCallThrowingSpool>(disk_);
  auto log = std::make_shared<LinesLogger>();
  session_ = Session::Create(cfg_, transport_, spool, log);
  ASSERT_TRUE(session_->Begin());
  spool->marker_throws.store(true);
  ASSERT_TRUE(WaitFor([&] { return session_->stats().thread_exceptions >= 1; }))
      << "the heartbeat never met the throwing marker write";
  session_->RecordEvent("after_the_marker_failed", 1000.0, Vec3{});
  ASSERT_NO_THROW(session_->Flush());
  EXPECT_TRUE(WaitFor([&] {
    for (const auto& body : BodiesFor("events")) {
      for (const auto& record : body["data"]) {
        if (record["name"] == "after_the_marker_failed") return true;
      }
    }
    return false;
  })) << "the part behind the failed marker write never reached the wire";
  EXPECT_GE(session_->stats().thread_exceptions, 2u);
  EXPECT_EQ(log->Count("a session marker write threw (a non-standard exception)"), 1);
  spool->marker_throws.store(false);
}

TEST_F(SerializationGuardTest, AReadOfTheNextChunkThatThrowsIsSaidOnceAndUploadsGoOnAfterIt) {
  cfg_.uploads_paused = true;
  auto spool = std::make_shared<ThreadCallThrowingSpool>(disk_);
  auto log = std::make_shared<LinesLogger>();
  session_ = Session::Create(cfg_, transport_, spool, log);
  ASSERT_TRUE(session_->Begin());
  session_->RecordEvent("queued", 1000.0, Vec3{});
  ASSERT_NO_THROW(session_->Flush());
  spool->peek_throws.store(true);
  session_->SetUploadsPaused(false);
  EXPECT_TRUE(WaitFor([&] { return session_->stats().thread_exceptions >= 2; }))
      << "every pass that threw is counted";
  EXPECT_EQ(transport_->count(), 0u);
  EXPECT_EQ(log->Count("an upload pass threw (peek exploded)"), 1);
  spool->peek_throws.store(false);
  EXPECT_TRUE(WaitFor([&] {
    for (const auto& body : BodiesFor("events")) {
      for (const auto& record : body["data"]) {
        if (record["name"] == "queued") return true;
      }
    }
    return false;
  })) << "the uploader stopped after the throw";
}

// Sent again, a delivered chunk is a duplicate on the platform, and one whose removal throws every
// time would be sent at the uploader's tick for good.
TEST_F(SerializationGuardTest, ADeliveredChunkWhoseRemovalThrowsIsNeverSentAgain) {
  cfg_.uploads_paused = true;
  auto spool = std::make_shared<ThreadCallThrowingSpool>(disk_);
  auto log = std::make_shared<LinesLogger>();
  session_ = Session::Create(cfg_, transport_, spool, log);
  ASSERT_TRUE(session_->Begin());
  session_->RecordEvent("remove_poison", 1000.0, Vec3{});
  ASSERT_NO_THROW(session_->Flush());
  session_->RecordEvent("behind_it", 1001.0, Vec3{});
  ASSERT_NO_THROW(session_->Flush());
  session_->SetUploadsPaused(false);
  EXPECT_TRUE(WaitFor([&] {
    for (const auto& body : BodiesFor("events")) {
      for (const auto& record : body["data"]) {
        if (record["name"] == "behind_it") return true;
      }
    }
    return false;
  })) << "the queue stopped at the chunk whose removal threw";
  EXPECT_TRUE(WaitFor([&] { return disk_->Count() == 1; }))
      << "the chunk whose removal threw stays spooled, and only it";
  // Room for a resend to show: a chunk left in the queue would go again at the next tick.
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  int poisoned = 0;
  for (const auto& r : transport_->requests()) {
    if (r.body.find("remove_poison") != std::string::npos) ++poisoned;
  }
  EXPECT_EQ(poisoned, 1) << "a delivered chunk was sent again";
  EXPECT_EQ(disk_->Count(), 1u) << "the chunk whose removal threw stays spooled";
  EXPECT_GE(session_->stats().thread_exceptions, 1u);
  EXPECT_EQ(log->Count("the removal of a chunk that left the queue threw (remove exploded)"), 1);
}

TEST_F(SerializationGuardTest, AnyOtherSpoolCallThatThrowsOnTheUploaderCostsOnlyThatPass) {
  // A chunk held for another host, so the uploader asks the spool whether it still holds it.
  Chunk foreign;
  foreign.url = "https://other.example.test/v0/events/SCENEGUID?version=2";
  foreign.scene_id = "SCENEGUID";
  foreign.scene_version = "2";
  foreign.body = "{}";
  foreign.session_id = "1_other";
  ASSERT_TRUE(disk_->Write(foreign));
  auto spool = std::make_shared<ThreadCallThrowingSpool>(disk_);
  auto log = std::make_shared<LinesLogger>();
  session_ = Session::Create(cfg_, transport_, spool, log);
  ASSERT_TRUE(session_->Begin());
  spool->contains_throws.store(true);
  ASSERT_TRUE(WaitFor([&] { return session_->stats().thread_exceptions >= 1; }))
      << "the uploader never asked about the held chunk";
  session_->RecordEvent("after_the_throw", 1000.0, Vec3{});
  ASSERT_NO_THROW(session_->Flush());
  EXPECT_TRUE(WaitFor([&] {
    for (const auto& body : BodiesFor("events")) {
      for (const auto& record : body["data"]) {
        if (record["name"] == "after_the_throw") return true;
      }
    }
    return false;
  })) << "the uploader stopped after the throw";
  EXPECT_EQ(log->Count("an upload pass threw (contains exploded)"), 1);
  spool->contains_throws.store(false);
}

// Recovery's second look at a marker left alone for being fresh runs on the worker. One that
// throws is said, and looked at again rather than forgotten.
TEST_F(SerializationGuardTest, ARecoveryRecheckThatThrowsOnTheWorkerIsTriedAgain) {
  cfg_.marker_heartbeat_s = 0.1;
  // Longer than opening the session and arming the spool can take, so the marker is fresh when
  // first looked at and the recheck meets the armed spool.
  cfg_.orphan_stale_after_s = 1.0;
  SessionMarker other;
  other.session_id = "1700000000_other";
  other.scene_id = cfg_.scene_id;
  other.scene_version = cfg_.scene_version;
  other.device_id = "other";
  other.started_at = 1700000000.0;
  other.last_seen =
      std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  disk_->WriteMarker(other);
  auto spool = std::make_shared<ThreadCallThrowingSpool>(disk_);
  auto log = std::make_shared<LinesLogger>();
  session_ = Session::Create(cfg_, transport_, spool, log);
  ASSERT_EQ(session_->RecoverOrphanedSessions(), 0) << "a fresh marker is left alone";
  spool->open_markers_throws.store(true);
  const std::string threw = "a pass of the worker threw (markers exploded)";
  ASSERT_TRUE(WaitFor([&] { return log->Count(threw) >= 1; }))
      << "the worker never looked at the marker again";
  EXPECT_GE(session_->stats().thread_exceptions, 1u);
  EXPECT_EQ(log->Count(threw), 1);
  spool->open_markers_throws.store(false);
  EXPECT_TRUE(WaitFor([&] {
    for (const auto& r : transport_->requests()) {
      if (r.body.find("1700000000_other") != std::string::npos &&
          r.body.find("abnormalEnd") != std::string::npos) {
        return true;
      }
    }
    return false;
  })) << "the session was not recovered once its markers could be read again";
}
