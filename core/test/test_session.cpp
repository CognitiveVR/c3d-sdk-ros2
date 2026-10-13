#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <functional>
#include <map>
#include <thread>

#include "c3d/remote_variables.hpp"
#include "c3d/session.hpp"
#include "c3d/version.hpp"
#include "fake_transport.hpp"

using namespace c3d;
using json = nlohmann::json;
namespace fs = std::filesystem;

namespace {

class SessionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() /
           ("c3d_sess_" + std::to_string(::getpid()) + "_" + std::to_string(counter_++));
    fs::create_directories(dir_);
    cfg_.application_key = "APPKEY";
    cfg_.scene_id = "SCENEGUID";
    cfg_.scene_version = "2";
    cfg_.device_id = "robot01";
    // DISTINCT from kSdkVersion on purpose. Both default to the same string, so with the default
    // in place `EXPECT_EQ(props["c3d.version"], kSdkVersion)` passes even when `c3d.version` is
    // wired to the integrator's app version instead of the SDK's -- a different fact that drifts
    // the moment an integrator sets one.
    cfg_.app_version = "9.9.9-app";
    cfg_.spool_dir = dir_.string();
    cfg_.auto_send_interval_s = 3600.0;  // tests drive flushing explicitly
    cfg_.worker_tick_s = 0.01;
    cfg_.retry_min_s = 0.05;
    cfg_.retry_max_s = 0.2;
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
    session_ = Session::Create(cfg_, transport_, spool_, std::make_shared<NullLogger>(), &err);
    ASSERT_NE(session_, nullptr) << err;
  }
  std::vector<json> BodiesFor(const std::string& stream_fragment) {
    std::vector<json> out;
    for (const auto& r : transport_->requests()) {
      if (r.url.find("/" + stream_fragment + "/") != std::string::npos) {
        out.push_back(json::parse(r.body));
      }
    }
    return out;
  }

  fs::path dir_;
  Config cfg_;
  std::shared_ptr<FakeTransport> transport_;
  std::shared_ptr<DiskSpool> spool_;
  std::unique_ptr<Session> session_;
  static int counter_;
};
int SessionTest::counter_ = 0;

bool WaitFor(const std::function<bool()>& done, double seconds) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (std::chrono::steady_clock::now() < deadline) {
    if (done()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return done();
}

// What one session delivered, read off the requests the transport accepted.
struct Delivered {
  bool start = false;
  bool end = false;
  int gaze_records = 0;
  json properties = json::object();  // every property any of its gaze parts carried
};

std::map<std::string, Delivered> DeliveredBySession(const FakeTransport& transport) {
  std::map<std::string, Delivered> out;
  for (const auto& r : transport.requests()) {
    const json body = json::parse(r.body);
    Delivered& d = out[body.value("sessionid", "")];
    if (r.url.find("/gaze/") != std::string::npos) {
      d.gaze_records += static_cast<int>(body["data"].size());
      if (body.contains("properties")) d.properties.update(body["properties"]);
    }
    if (r.url.find("/events/") != std::string::npos) {
      for (const auto& rec : body["data"]) {
        if (rec["name"] == "c3d.sessionStart") d.start = true;
        if (rec["name"] == "c3d.sessionEnd") d.end = true;
      }
    }
  }
  return out;
}

}  // namespace

// A participant id longer than the cap is cut, and the cut must land on a code-point boundary.
// A byte-exact cut inside a multi-byte sequence leaves invalid UTF-8 in a session property, which
// reaches the wire as a replacement character at the end of the id.
TEST_F(SessionTest, TruncatesAParticipantIdOnAUtf8Boundary) {
  Build();
  ASSERT_TRUE(session_->Begin());
  // 65 bytes: the 65th and 64th are the two halves of one code point, so a cut at 64 splits it.
  session_->SetParticipantId(std::string(63, 'a') + "\xC3\xA9");

  session_->RecordGaze(1000.0, Vec3{0, 0, 0}, Quat{0, 0, 0, 1});
  ASSERT_NO_THROW({
    session_->Flush();
    session_->PumpUploads();
  });

  bool saw = false;
  for (const auto& body : BodiesFor("gaze")) {
    const auto props = body.value("properties", json::object());
    if (!props.contains("c3d.participant.id")) continue;
    saw = true;
    EXPECT_EQ(props["c3d.participant.id"], std::string(63, 'a'));
  }
  EXPECT_TRUE(saw) << "no gaze body carried the participant id: the batch never serialized";
}

// The other side of the back-off: a sequence that ends EXACTLY at the cap was not split, and
// dropping it would silently shorten an id that was already legal.
TEST_F(SessionTest, KeepsACompleteTrailingCodePointAtTheBoundary) {
  Build();
  ASSERT_TRUE(session_->Begin());
  // 66 bytes: the cut at 64 falls between the two code points, splitting neither.
  session_->SetParticipantId(std::string(62, 'a') + "\xC3\xA9" + "\xC3\xA9");

  session_->RecordGaze(1000.0, Vec3{0, 0, 0}, Quat{0, 0, 0, 1});
  session_->Flush();
  session_->PumpUploads();

  const std::string expected = std::string(62, 'a') + "\xC3\xA9";
  ASSERT_EQ(expected.size(), 64u);
  bool saw = false;
  for (const auto& body : BodiesFor("gaze")) {
    const auto props = body.value("properties", json::object());
    if (!props.contains("c3d.participant.id")) continue;
    saw = true;
    EXPECT_EQ(props["c3d.participant.id"], expected);
  }
  EXPECT_TRUE(saw) << "no gaze body carried the participant id";
}

TEST_F(SessionTest, KeepsACompleteFourByteCodePointAtTheBoundary) {
  // Four-byte sequences are where an off-by-one in the scan shows up: three continuation bytes
  // have to be walked back before the lead is even seen.
  Build();
  ASSERT_TRUE(session_->Begin());
  session_->SetParticipantId(std::string(60, 'a') + "\xF0\x9F\x98\x80" + "\xF0\x9F\x98\x80");

  session_->RecordGaze(1000.0, Vec3{0, 0, 0}, Quat{0, 0, 0, 1});
  session_->Flush();
  session_->PumpUploads();

  const std::string expected = std::string(60, 'a') + "\xF0\x9F\x98\x80";
  ASSERT_EQ(expected.size(), 64u);
  bool saw = false;
  for (const auto& body : BodiesFor("gaze")) {
    const auto props = body.value("properties", json::object());
    if (!props.contains("c3d.participant.id")) continue;
    saw = true;
    EXPECT_EQ(props["c3d.participant.id"], expected);
  }
  EXPECT_TRUE(saw) << "no gaze body carried the participant id";
}

TEST_F(SessionTest, RefusesToStartWithoutASceneVersion) {
  // Accepting this would mean a warning once and then 100% of telemetry
  // silently discarded for the life of the process, so Create refuses it.
  cfg_.scene_version = "";
  std::string err;
  auto s = Session::Create(cfg_, transport_, spool_, std::make_shared<NullLogger>(), &err);
  EXPECT_EQ(s, nullptr);
  EXPECT_NE(err.find("scene_version"), std::string::npos);
}

TEST_F(SessionTest, RefusesToStartWithoutASceneId) {
  cfg_.scene_id = "";
  std::string err;
  EXPECT_EQ(Session::Create(cfg_, transport_, spool_, std::make_shared<NullLogger>(), &err),
            nullptr);
}

TEST_F(SessionTest, SessionIdEmbedsTruncatedEpochSeconds) {
  Build();
  ASSERT_TRUE(session_->Begin());
  const std::string id = session_->session_id();
  EXPECT_NE(id.find("_robot01"), std::string::npos);
  const std::string secs = id.substr(0, id.find('_'));
  EXPECT_EQ(secs, std::to_string(static_cast<std::int64_t>(session_->session_timestamp())));
  session_->End();
}

// The DEFAULT, not the fixture's override. `Config::app_version` is derived from `kSdkVersion`
// rather than declared as a second copy of it, and nothing else asserts the derivation,
// because every other test sets `app_version` explicitly -- including this file's own fixture,
// which sets "9.9.9-app" precisely so the two are distinguishable on the wire.
TEST(ConfigDefaults, TheApplicationVersionDefaultsToTheSdksOwnVersion) {
  EXPECT_EQ(c3d::Config{}.app_version, std::string(c3d::kSdkVersion));
}

// The spool's cap is the documented 1 GiB unless a consumer sets another.
TEST(ConfigDefaults, TheSpoolCapDefaultsToOneGibibyte) {
  EXPECT_EQ(c3d::Config{}.spool_max_bytes, 1024ull * 1024 * 1024);
}

TEST_F(SessionTest, BeginEmitsSessionStartAndProperties) {
  Build();
  session_->SetProperty("site", std::string("warehouse-3"));
  ASSERT_TRUE(session_->Begin());
  session_->PumpUploads();

  const auto events = BodiesFor("events");
  ASSERT_FALSE(events.empty());
  EXPECT_EQ(events[0]["data"][0]["name"], "c3d.sessionStart");

  const auto gaze = BodiesFor("gaze");
  ASSERT_FALSE(gaze.empty()) << "properties must ship even with no gaze records";
  const auto& props = gaze[0]["properties"];
  EXPECT_EQ(props["c3d.app.engine"], "ROS2");
  EXPECT_EQ(props["c3d.deviceid"], "robot01");
  EXPECT_EQ(props["site"], "warehouse-3");
  // ON THE WIRE, not just declared. A version string that is declared and never transmitted
  // leaves the platform unable to tell which SDK produced a session, and `c3d.app.sdktype` must
  // name what produced the session rather than how this library is built. Both are asserted
  // against the serialized payload for that reason; a test on the constants passes whether or
  // not they are sent.
  EXPECT_EQ(props["c3d.app.sdktype"], "ros");
  EXPECT_EQ(props["c3d.version"], c3d::kSdkVersion);
  EXPECT_TRUE(props["c3d.version"].get<std::string>().find('.') != std::string::npos)
      << "c3d.version must be a semver, not a name: " << props["c3d.version"];
  session_->End();
}

TEST_F(SessionTest, EndEmitsSessionEndWithLength) {
  Build();
  ASSERT_TRUE(session_->Begin());
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  session_->End("test");
  session_->PumpUploads();

  bool found = false;
  for (const auto& body : BodiesFor("events")) {
    for (const auto& rec : body["data"]) {
      if (rec["name"] == "c3d.sessionEnd") {
        found = true;
        EXPECT_EQ(rec["properties"]["Reason"], "test");
        EXPECT_GT(rec["properties"]["sessionlength"].get<double>(), 0.0);
      }
    }
  }
  EXPECT_TRUE(found);
}

// A caller whose records run on their own timeline ends the session on it: the end event and the
// length both follow the time it names, not wall clock at the call.
TEST_F(SessionTest, AnExplicitEndTimeSetsTheEndEventAndTheLength) {
  Build();
  ASSERT_TRUE(session_->Begin());
  const double start = session_->session_timestamp();
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  session_->End("test", start + 5.0);
  session_->PumpUploads();

  bool found = false;
  for (const auto& body : BodiesFor("events")) {
    for (const auto& rec : body["data"]) {
      if (rec["name"] == "c3d.sessionEnd") {
        found = true;
        EXPECT_NEAR(rec["time"].get<double>(), start + 5.0, 1e-3);
        EXPECT_NEAR(rec["properties"]["sessionlength"].get<double>(), 5.0, 1e-6);
      }
    }
  }
  EXPECT_TRUE(found);
}

// An end time before the start would give a negative length and an end event ahead of the start.
TEST_F(SessionTest, AnExplicitEndTimeIsNeverBeforeTheStart) {
  Build();
  ASSERT_TRUE(session_->Begin());
  const double start = session_->session_timestamp();
  session_->End("test", start - 100.0);
  session_->PumpUploads();

  bool found = false;
  for (const auto& body : BodiesFor("events")) {
    for (const auto& rec : body["data"]) {
      if (rec["name"] == "c3d.sessionEnd") {
        found = true;
        EXPECT_NEAR(rec["time"].get<double>(), start, 1e-3);
        EXPECT_NEAR(rec["properties"]["sessionlength"].get<double>(), 0.0, 1e-6);
      }
    }
  }
  EXPECT_TRUE(found);
}

// Omitted, the length is the steady clock's: a session held open for 300 ms measures at least that.
TEST_F(SessionTest, WithoutAnEndTimeTheLengthIsTheSteadyClocks) {
  Build();
  ASSERT_TRUE(session_->Begin());
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  session_->End("test");
  session_->PumpUploads();

  bool found = false;
  for (const auto& body : BodiesFor("events")) {
    for (const auto& rec : body["data"]) {
      if (rec["name"] == "c3d.sessionEnd") {
        found = true;
        EXPECT_GE(rec["properties"]["sessionlength"].get<double>(), 0.29);
      }
    }
  }
  EXPECT_TRUE(found);
}

// Without an end time the end is stamped with the wall clock at the call, as the start and every
// record are. On the steady clock it would reach the wire as seconds since boot, decades before
// the session's own start.
TEST_F(SessionTest, WithoutAnEndTimeTheEndIsWallClockNow) {
  const auto wall = [] {
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch())
        .count();
  };
  Build();
  ASSERT_TRUE(session_->Begin());
  const double started = session_->session_timestamp();
  // Bracketed rather than bounded, so a slow disk under End cannot fail it. The slack is the
  // wire's rounding to five decimals.
  const double before = wall();
  session_->End("test");
  const double after = wall();
  session_->PumpUploads();

  bool found = false;
  for (const auto& body : BodiesFor("events")) {
    for (const auto& rec : body["data"]) {
      if (rec["name"] != "c3d.sessionEnd") continue;
      found = true;
      const double t = rec["time"].get<double>();
      EXPECT_GE(t, started);
      EXPECT_GE(t, before - 1e-4) << "the end is not on the wall clock";
      EXPECT_LE(t, after + 1e-4) << "the end is not on the wall clock";
    }
  }
  EXPECT_TRUE(found);
}

TEST_F(SessionTest, RecordingIsIgnoredOutsideASession) {
  Build();
  session_->RecordSensor("battery", 1.0, 50.0);
  session_->Flush();
  session_->PumpUploads();
  EXPECT_EQ(transport_->count(), 0u);
}

TEST_F(SessionTest, PartCountersResetBetweenSessions) {
  // Carried across sessions, these would have a long-lived process send
  // part: 500 on a fresh session's first upload.
  Build();
  ASSERT_TRUE(session_->Begin());
  for (int i = 0; i < 3; ++i) {
    session_->RecordEvent("tick", 1.0 + i, Vec3{});
    session_->Flush();
  }
  session_->End();
  session_->PumpUploads();

  ASSERT_TRUE(session_->Begin());
  session_->RecordEvent("tick", 100.0, Vec3{});
  session_->Flush();
  session_->PumpUploads();
  const std::string second_id = session_->session_id();
  session_->End();

  std::uint64_t first_part_of_second_session = 999;
  for (const auto& r : transport_->requests()) {
    if (r.url.find("/events/") == std::string::npos) continue;
    const json body = json::parse(r.body);
    if (body["sessionid"] == second_id) {
      first_part_of_second_session =
          std::min<std::uint64_t>(first_part_of_second_session, body["part"].get<std::uint64_t>());
    }
  }
  EXPECT_EQ(first_part_of_second_session, 1u);
}

TEST_F(SessionTest, SensorSamplesAreChangeGated) {
  Build();
  ASSERT_TRUE(session_->Begin());
  // Identical values inside the keepalive window collapse to one sample.
  for (int i = 0; i < 10; ++i) session_->RecordSensor("battery", 1000.0 + i * 0.1, 55.0);
  session_->Flush();
  session_->PumpUploads();

  std::size_t samples = 0;
  for (const auto& body : BodiesFor("sensors")) {
    for (const auto& entry : body["data"]) {
      if (entry["name"] == "battery") samples += entry["data"].size();
    }
  }
  EXPECT_EQ(samples, 1u) << "unchanged values inside keepalive must be gated";
  session_->End();
}

TEST_F(SessionTest, SensorKeepaliveStillEmits) {
  Build();
  ASSERT_TRUE(session_->Begin());
  session_->RecordSensor("battery", 1000.0, 55.0);
  session_->RecordSensor("battery", 1005.0, 55.0);  // past the 2s keepalive
  session_->Flush();
  session_->PumpUploads();

  std::size_t samples = 0;
  for (const auto& body : BodiesFor("sensors")) {
    for (const auto& entry : body["data"]) {
      if (entry["name"] == "battery") samples += entry["data"].size();
    }
  }
  EXPECT_EQ(samples, 2u);
  session_->End();
}

TEST_F(SessionTest, RecordNeverTouchesTheNetwork) {
  // The central contract. A synchronous flush from inside Record* would let a
  // sensor sample block a ROS callback thread on a POST.
  cfg_.event_batch = 2;  // small enough that a naive implementation would flush
  Build();
  ASSERT_TRUE(session_->Begin());
  // By thread, not by count: the uploader legitimately posts Begin's parts meanwhile.
  const std::thread::id caller = std::this_thread::get_id();
  std::atomic<bool> posted_on_caller{false};
  transport_->SetResponder([&](const FakeTransport::Request&) -> std::optional<HttpResponse> {
    if (std::this_thread::get_id() == caller) posted_on_caller.store(true);
    return std::nullopt;
  });
  for (int i = 0; i < 50; ++i) session_->RecordEvent("e", i, Vec3{});
  EXPECT_FALSE(posted_on_caller.load()) << "Record* must not perform I/O on the caller thread";
  session_->End();
  transport_->SetResponder(nullptr);
}

TEST_F(SessionTest, DataReachesDiskBeforeUploadIsAttempted) {
  transport_->SetOffline(true);
  Build();
  ASSERT_TRUE(session_->Begin());
  session_->RecordEvent("offline", 1.0, Vec3{});
  session_->Flush();
  EXPECT_GT(spool_->Count(), 0u) << "payloads must be durable before the first POST";
  session_->End("offline");
}

TEST_F(SessionTest, AlwaysSendsTheSessionPropertiesTheBackendRequires) {
  // The platform accepts a gaze payload whose properties omit
  // `c3d.app.version` with HTTP 200 and a valid `cvr-request-time` header, and
  // then discards the entire session. It never appears in the session list or
  // the dashboard's counts. There is no error and no warning.
  //
  // Identical sessions differing only in the presence of this one property
  // either appear or vanish.
  Build();
  ASSERT_TRUE(session_->Begin());
  session_->PumpUploads();

  const auto gaze = BodiesFor("gaze");
  ASSERT_FALSE(gaze.empty());
  const auto& props = gaze[0]["properties"];
  EXPECT_TRUE(props.contains("c3d.app.version"))
      << "without c3d.app.version the backend silently discards the session";
  // The VALUE, not just presence: with only a non-empty check, wiring c3d.app.version to
  // kSdkVersion passes -- the mirror image of the swap the fixture's distinct app_version catches.
  EXPECT_EQ(props["c3d.app.version"], "9.9.9-app");
  EXPECT_TRUE(props.contains("c3d.app.name"));
  session_->End();
}

// The session's last gaze part carries the whole property set, not the delta since the last part:
// a session whose opening part is lost to the spool cap delivers its properties regardless.
TEST_F(SessionTest, TheLastGazePartCarriesEveryPropertyTheSessionHas) {
  Build();
  session_->SetProperty("site", std::string("warehouse-3"));
  ASSERT_TRUE(session_->Begin());
  session_->SetSessionName("run 7");
  session_->RecordGaze(1.8e9, Vec3{}, Quat{});
  session_->Flush();
  session_->SetProperty("battery_pack", std::string("B"));
  session_->RecordGaze(1.8e9 + 1, Vec3{}, Quat{});
  session_->End();
  session_->PumpUploads();

  const auto gaze = BodiesFor("gaze");
  ASSERT_GE(gaze.size(), 3u);
  ASSERT_TRUE(gaze.back().contains("properties"));
  const auto& props = gaze.back()["properties"];
  EXPECT_EQ(props.value("c3d.app.version", ""), "9.9.9-app");
  EXPECT_EQ(props.value("c3d.app.name", ""), "cognitive3d_ros");
  EXPECT_EQ(props.value("site", ""), "warehouse-3");
  EXPECT_EQ(props.value("c3d.sessionname", ""), "run 7");
  EXPECT_EQ(props.value("battery_pack", ""), "B");
}

// At the spool cap gaze is evicted first, and a gaze part carrying the session's properties is
// what makes the platform keep the session: one that delivers its events and none of its
// properties is accepted and then discarded whole. Here a second session's poses evict every
// pose the first one recorded.
TEST_F(SessionTest, EverySessionThatOutlivesTheCapStillDeliversItsAppVersion) {
  spool_.reset();
  spool_ = std::make_shared<DiskSpool>(dir_.string(), 24 * 1024, std::make_shared<NullLogger>());
  cfg_.uploads_paused = true;
  Build();
  ASSERT_TRUE(session_->Begin());
  const std::string first = session_->session_id();
  for (int part = 0; part < 6; ++part) {
    for (int i = 0; i < 10; ++i) {
      session_->RecordGaze(1.8e9 + part * 10 + i, Vec3{double(i), 0, 0}, Quat{});
    }
    session_->RecordEvent("waypoint_" + std::to_string(part), 1.8e9 + part * 10, Vec3{});
    session_->Flush();
  }
  session_->End();
  ASSERT_TRUE(session_->Begin());
  for (int part = 0; part < 200; ++part) {
    for (int i = 0; i < 10; ++i) {
      session_->RecordGaze(1.8e9 + 100 + part * 10 + i, Vec3{double(i), 0, 0}, Quat{});
    }
    session_->Flush();
  }
  session_->End();
  ASSERT_GT(spool_->evicted(), 0u) << "the spool never reached its cap; the test proves nothing";

  session_->SetUploadsPaused(false);
  ASSERT_TRUE(WaitFor([&] { return spool_->Count() == 0; }, 10.0));
  const auto delivered = DeliveredBySession(*transport_);
  ASSERT_EQ(delivered.size(), 2u);
  ASSERT_EQ(delivered.count(first), 1u);
  EXPECT_EQ(delivered.at(first).gaze_records, 0)
      << "a pose of the first session outlived the cap; the test proves nothing";
  EXPECT_TRUE(delivered.at(first).start);
  EXPECT_TRUE(delivered.at(first).end);
  for (const auto& [id, d] : delivered) {
    EXPECT_EQ(d.properties.value("c3d.app.version", ""), "9.9.9-app")
        << "session " << id << " delivered data and no c3d.app.version, so the platform "
        << "discards it whole";
  }
}

// Once only events are left to evict, the events tier evicts its own oldest: a long run recorded
// offline loses its opening chunks, the opening property part among them, and its end still
// delivers the whole set.
TEST_F(SessionTest, ASessionWhoseOpeningPartsWereEvictedStillDeliversItsProperties) {
  spool_.reset();
  spool_ = std::make_shared<DiskSpool>(dir_.string(), 16 * 1024, std::make_shared<NullLogger>());
  cfg_.uploads_paused = true;
  Build();
  ASSERT_TRUE(session_->Begin());
  const std::string id = session_->session_id();
  session_->SetSessionName("long offline run");
  for (int part = 0; part < 150; ++part) {
    session_->RecordEvent("waypoint_" + std::to_string(part), 1.8e9 + part, Vec3{});
    session_->Flush();
  }
  session_->End();
  ASSERT_GT(spool_->evicted(), 0u) << "the spool never reached its cap; the test proves nothing";

  session_->SetUploadsPaused(false);
  ASSERT_TRUE(WaitFor([&] { return spool_->Count() == 0; }, 10.0));
  const auto delivered = DeliveredBySession(*transport_);
  ASSERT_EQ(delivered.count(id), 1u);
  const Delivered& d = delivered.at(id);
  EXPECT_FALSE(d.start) << "the opening events part outlived the cap; the test proves nothing";
  EXPECT_TRUE(d.end);
  EXPECT_EQ(d.properties.value("c3d.app.version", ""), "9.9.9-app")
      << "the session delivered its end and no c3d.app.version, so the platform discards it whole";
  EXPECT_EQ(d.properties.value("c3d.sessionname", ""), "long offline run");
}

TEST_F(SessionTest, KeepsRemoteVariablesOnTheWireTyped) {
  // The dashboard slices `c3d.remote_variable.*` as A/B dimensions and indexes numeric
  // session properties separately from textual ones. A 30 sent as "30" is a session in no arm.
  Build();
  ASSERT_TRUE(session_->Begin());

  RemoteVariableSet set;
  set.fetched = true;
  set.variables.push_back({"nav_speed_centimeters_per_second", PropertyValue(std::int64_t{30})});
  set.variables.push_back({"headlights_on", PropertyValue(true)});
  set.variables.push_back({"greeting", PropertyValue(std::string("hello"))});
  ApplyRemoteVariables(*session_, set);

  session_->RecordGaze(1000.0, Vec3{0, 0, 0}, Quat{0, 0, 0, 1});
  session_->Flush();
  session_->PumpUploads();

  bool saw = false;
  for (const auto& body : BodiesFor("gaze")) {
    const auto props = body.value("properties", json::object());
    if (!props.contains("c3d.remote_variable.nav_speed_centimeters_per_second")) continue;
    saw = true;
    const auto& speed = props["c3d.remote_variable.nav_speed_centimeters_per_second"];
    EXPECT_TRUE(speed.is_number_integer()) << "sent as " << speed.dump();
    EXPECT_EQ(speed, 30);
    EXPECT_EQ(props["c3d.remote_variable.headlights_on"], true);
    EXPECT_EQ(props["c3d.remote_variable.greeting"], "hello");
  }
  EXPECT_TRUE(saw) << "no gaze payload carried the remote variables";
  session_->End();
}

// The draw is keyed on a digest of the identifier; the participant the session records is the
// identifier itself, because that is the name an operator searches the dashboard by.
TEST_F(SessionTest, TheParticipantIsRecordedRawWhileTheDrawIsSentItsDigest) {
  Build();
  ASSERT_TRUE(session_->Begin());
  const std::string participant = "robot-01";
  session_->SetParticipantId(participant);
  FetchRemoteVariables(*transport_, cfg_, participant);

  session_->RecordGaze(1000.0, Vec3{0, 0, 0}, Quat{0, 0, 0, 1});
  session_->Flush();
  session_->PumpUploads();

  bool saw = false;
  for (const auto& body : BodiesFor("gaze")) {
    const auto props = body.value("properties", json::object());
    if (!props.contains("c3d.participant.id")) continue;
    saw = true;
    EXPECT_EQ(props["c3d.participant.id"], participant);
  }
  EXPECT_TRUE(saw) << "no gaze payload carried the participant";
  const auto gets = transport_->gets();
  ASSERT_EQ(gets.size(), 1u);
  EXPECT_NE(gets[0].url.find("identifier=1d2186ba0fd0a394cc2e45c9ef3e103bbc4dea5c3a9a1248ac15a48bb147ef0c"),
            std::string::npos)
      << gets[0].url;
  EXPECT_EQ(gets[0].url.find(participant), std::string::npos) << gets[0].url;
  session_->End();
}

TEST_F(SessionTest, RefusesAnEmptyAppVersion) {
  cfg_.app_version = "";
  std::string err;
  EXPECT_EQ(Session::Create(cfg_, transport_, spool_, std::make_shared<NullLogger>(), &err),
            nullptr);
  EXPECT_NE(err.find("app_version"), std::string::npos);
}

TEST_F(SessionTest, APropertySetDuringASessionDoesNotRideTheNextSessionsOpeningPayload) {
  // Begin flushes BEFORE the caller can apply this session's identity or
  // properties, so `known_properties` carried over from session 1 would send
  // session 2's opening gaze payload out with session 1's values.
  //
  // Asserted on the payload CONTENT of session 2's FIRST gaze body, not on the
  // stored session: the platform keeps the last write, so the stored session is
  // right either way and a test that read it back would pass against the
  // defect.
  Build();
  ASSERT_TRUE(session_->Begin());
  session_->SetProperty("arm", std::string("thirty"));
  session_->RecordGaze(1000.0, Vec3{0, 0, 0}, Quat{0, 0, 0, 1});
  session_->Flush();
  session_->PumpUploads();
  session_->End("normal");
  session_->PumpUploads();

  const std::size_t after_first = BodiesFor("gaze").size();

  // Begin flushes internally; PumpUploads is what puts that flush on the wire.
  ASSERT_TRUE(session_->Begin());
  session_->PumpUploads();
  const auto opening = BodiesFor("gaze");
  ASSERT_GT(opening.size(), after_first) << "Begin must emit its own properties flush";
  const json& first_of_second = opening[after_first];
  const auto props = first_of_second.value("properties", json::object());
  EXPECT_FALSE(props.contains("arm"))
      << "session 2 opened carrying session 1's property: " << props.dump();

  // And the mechanism is scoped, not blanket: this session can still set its own.
  session_->SetProperty("arm", std::string("fifteen"));
  session_->RecordGaze(2000.0, Vec3{0, 0, 0}, Quat{0, 0, 0, 1});
  session_->Flush();
  session_->PumpUploads();
  bool saw_fifteen = false;
  for (const auto& body : BodiesFor("gaze")) {
    const auto p = body.value("properties", json::object());
    if (p.contains("arm") && p["arm"] == "fifteen") saw_fifteen = true;
  }
  EXPECT_TRUE(saw_fifteen) << "the second session must still be able to set the property";
  session_->End("normal");
}

TEST_F(SessionTest, MetadataSetBeforeTheFirstSessionSurvivesEverySession) {
  // The other half, and the reason End restores a snapshot rather than clearing.
  // Begin's comment calls pre-session properties "legitimate session metadata";
  // a blanket clear would drop them from session 2 onward.
  Build();
  session_->SetProperty("fleet", std::string("warehouse-3"));
  ASSERT_TRUE(session_->Begin());
  session_->RecordGaze(1000.0, Vec3{0, 0, 0}, Quat{0, 0, 0, 1});
  session_->Flush();
  session_->PumpUploads();
  session_->End("normal");
  session_->PumpUploads();
  const std::size_t after_first = BodiesFor("gaze").size();

  ASSERT_TRUE(session_->Begin());
  session_->PumpUploads();
  const auto bodies = BodiesFor("gaze");
  ASSERT_GT(bodies.size(), after_first);
  const auto props = bodies[after_first].value("properties", json::object());
  EXPECT_TRUE(props.contains("fleet"))
      << "pre-session metadata was dropped from the second session: " << props.dump();
  session_->End("normal");
}

TEST(SessionIdentity, TwoSessionsInTheSameSecondGetDifferentIds) {
  // The id is `<unix seconds>_<device id>`, truncated to whole seconds, so
  // ending a session and immediately starting another could produce two
  // sessions the platform cannot tell apart -- no corner case once a session
  // can be bounded by a service call: an orchestrator closing one trial and
  // opening the next does it in milliseconds.
  const fs::path dir = fs::temp_directory_path() /
                       ("c3d_ident_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  Config cfg;
  cfg.application_key = "K"; cfg.scene_id = "S"; cfg.scene_version = "1";
  cfg.device_id = "robot"; cfg.spool_dir = dir.string();
  cfg.auto_send_interval_s = 3600.0; cfg.worker_tick_s = 0.01;
  auto session = Session::Create(cfg, std::make_shared<FakeTransport>(),
                                 std::make_shared<DiskSpool>(dir.string(), 64ull << 20,
                                                             std::make_shared<NullLogger>()),
                                 std::make_shared<NullLogger>());
  ASSERT_TRUE(session->Begin());
  const std::string first = session->session_id();
  session->End("first");
  ASSERT_TRUE(session->Begin());
  const std::string second = session->session_id();
  session->End("second");

  EXPECT_FALSE(first.empty());
  EXPECT_NE(first, second) << "two sessions shared an id, so the platform sees one";
  // The format is the Unity SDK's and is not ours to change.
  EXPECT_NE(first.find("_robot"), std::string::npos);
  EXPECT_NE(second.find("_robot"), std::string::npos);

  std::error_code ec;
  fs::remove_all(dir, ec);
}

// A spool that cannot be used -- another writer holds its directory -- must stop the session
// being created at all, with the spool's reason, rather than record into nothing.
TEST_F(SessionTest, RefusesASpoolThatAnotherWriterHolds) {
  auto contender = std::make_shared<DiskSpool>(dir_.string(), 64ull * 1024 * 1024,
                                               std::make_shared<NullLogger>());
  ASSERT_FALSE(contender->error().empty()) << "the fixture's spool should hold the directory";
  std::string error;
  auto session =
      Session::Create(cfg_, transport_, contender, std::make_shared<NullLogger>(), &error);
  EXPECT_EQ(session, nullptr);
  EXPECT_NE(error.find("already in use"), std::string::npos) << error;
}
