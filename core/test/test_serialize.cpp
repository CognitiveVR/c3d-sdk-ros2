// Wire conformance suite.
//
// These assert SCHEMA AND SEMANTIC equivalence with the reference
// implementations, not byte identity. Byte identity is neither achievable nor
// desirable: nlohmann orders object keys by its own rules, and this library
// deliberately fixes defects that the reference implementations contain. Every
// intentional divergence is named in a test below.
//
// Provenance: each assertion is derived from the sources of Unity SDK 2.5.0
// (SharedCore.cs, CognitiveStatics.cs, NetworkManager.cs) and cvr-sdk-cpp.

#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>
#include <nlohmann/json.hpp>

#include "serialize.hpp"

using namespace c3d;
using json = nlohmann::json;

namespace {

EnvelopeContext Ctx(Stream s) {
  EnvelopeContext ctx;
  ctx.session_id = "1787010742_robot01";
  ctx.device_id = "robot01";
  ctx.session_timestamp = 1787010742.798;
  ctx.device_type = "ros2";
  ctx.gaze_interval = 0.1;
  ctx.part = PartBase(s);
  return ctx;
}

}  // namespace

// ---------------------------------------------------------------------------
// URL and auth
// ---------------------------------------------------------------------------

TEST(Url, MatchesReferenceShape) {
  EXPECT_EQ(BuildUrl("https", "data.cognitive3d.com", "0", Stream::kGaze, "SCENEGUID", "2"),
            "https://data.cognitive3d.com/v0/gaze/SCENEGUID?version=2");
  EXPECT_EQ(BuildUrl("https", "data.cognitive3d.com", "0", Stream::kSensors, "SCENEGUID", "2"),
            "https://data.cognitive3d.com/v0/sensors/SCENEGUID?version=2");
  EXPECT_EQ(BuildUrl("https", "data.cognitive3d.com", "0", Stream::kDynamics, "S", "10"),
            "https://data.cognitive3d.com/v0/dynamics/S?version=10");
  EXPECT_EQ(BuildUrl("https", "data.cognitive3d.com", "0", Stream::kEvents, "S", "1"),
            "https://data.cognitive3d.com/v0/events/S?version=1");
}

TEST(Url, TheBinaryStreamsAreSessionKeyedAndCarryNoSceneInThePath) {
  // Scene id and version number travel in the Cap'n Proto body instead, because
  // the gateway resolves the number to a version id it alone can look up.
  EXPECT_EQ(BuildUrl("https", "data.cognitive3d.com", "0", Stream::kCamera, "SCENEGUID", "2"),
            "https://data.cognitive3d.com/v0/camera");
  EXPECT_EQ(BuildUrl("https", "data.cognitive3d.com", "0", Stream::kLidar, "SCENEGUID", "2"),
            "https://data.cognitive3d.com/v0/lidar");
}

TEST(Headers, AuthSchemeIsApiKeyData) {
  const auto headers = BuildHeaders("APPKEY123", Stream::kGaze);
  ASSERT_EQ(headers.size(), 3u);
  EXPECT_EQ(headers[0], "Content-Type: application/json");
  EXPECT_EQ(headers[1], "Authorization: APIKEY:DATA APPKEY123");
  EXPECT_EQ(headers[2], "X-HTTP-Method-Override: POST");
}

TEST(Headers, TheBinaryStreamsAnnounceOctetStreamAndDropTheUnityOverride) {
  for (Stream s : {Stream::kCamera, Stream::kLidar}) {
    const auto headers = BuildHeaders("APPKEY123", s);
    ASSERT_EQ(headers.size(), 2u) << StreamName(s);
    EXPECT_EQ(headers[0], "Content-Type: application/octet-stream");
    EXPECT_EQ(headers[1], "Authorization: APIKEY:DATA APPKEY123");
  }
}

// ---------------------------------------------------------------------------
// Part counters and formatversion, as the Unity SDK sends them
// ---------------------------------------------------------------------------

TEST(PartBase, EachJsonStreamStartsWhereTheUnitySdkStartsIt) {
  EXPECT_EQ(PartBase(Stream::kGaze), 0u);
  EXPECT_EQ(PartBase(Stream::kDynamics), 0u);
  EXPECT_EQ(PartBase(Stream::kEvents), 1u);
  EXPECT_EQ(PartBase(Stream::kSensors), 1u);
}

TEST(PartBase, TheBinaryStreamsStartAtOne) {
  // No reference implementation has these routes: docs/data-format.md fixes the base at 1.
  EXPECT_EQ(PartBase(Stream::kCamera), 1u);
  EXPECT_EQ(PartBase(Stream::kLidar), 1u);
}

TEST(StreamNames, TheTwoBinaryStreamsAreNamedAndFlagged) {
  // The name is the url segment, the chunk-id suffix the spool sorts and evicts
  // by, and what StreamFromName reads back on recovery -- one string, three jobs.
  EXPECT_STREQ(StreamName(Stream::kCamera), "camera");
  EXPECT_STREQ(StreamName(Stream::kLidar), "lidar");
  EXPECT_TRUE(IsBinaryStream(Stream::kCamera));
  EXPECT_TRUE(IsBinaryStream(Stream::kLidar));
  for (Stream s : {Stream::kGaze, Stream::kEvents, Stream::kSensors, Stream::kDynamics}) {
    EXPECT_FALSE(IsBinaryStream(s)) << StreamName(s);
  }
}

TEST(FormatVersion, SensorsAreTheOnlyStreamAtFormatVersionTwo) {
  EXPECT_STREQ(FormatVersion(Stream::kSensors), "2.0");
  EXPECT_STREQ(FormatVersion(Stream::kGaze), "1.0");
  EXPECT_STREQ(FormatVersion(Stream::kEvents), "1.0");
  EXPECT_STREQ(FormatVersion(Stream::kDynamics), "1.0");
  EXPECT_STREQ(FormatVersion(Stream::kCamera), "1.0");
  EXPECT_STREQ(FormatVersion(Stream::kLidar), "1.0");
}

// ---------------------------------------------------------------------------
// Gaze
// ---------------------------------------------------------------------------

TEST(Gaze, EnvelopeAndRecordShape) {
  std::vector<GazeRecord> records;
  GazeRecord r;
  r.t = 1787010742.5;
  r.p = Vec3{1, 2, 3};
  r.r = Quat{0, 0, 0, 1};
  r.has_gaze = true;
  r.g = Vec3{4, 5, 6};
  records.push_back(r);

  const json j = json::parse(SerializeGaze(Ctx(Stream::kGaze), records, {}));

  EXPECT_EQ(j["userid"], "robot01");
  EXPECT_EQ(j["sessionid"], "1787010742_robot01");
  EXPECT_EQ(j["part"], 0);
  EXPECT_EQ(j["formatversion"], "1.0");
  EXPECT_EQ(j["hmdtype"], "ros2");
  EXPECT_DOUBLE_EQ(j["interval"].get<double>(), 0.1);
  // Gaze truncates the session timestamp to whole seconds; events do not.
  EXPECT_EQ(j["timestamp"].get<std::int64_t>(), 1787010742);

  ASSERT_EQ(j["data"].size(), 1u);
  const auto& rec = j["data"][0];
  EXPECT_EQ(rec["p"], json::array({1.0, 2.0, 3.0}));
  // Quaternion is [x, y, z, w] with w last, matching geometry_msgs field order.
  EXPECT_EQ(rec["r"], json::array({0.0, 0.0, 0.0, 1.0}));
  EXPECT_EQ(rec["g"], json::array({4.0, 5.0, 6.0}));
  EXPECT_FALSE(j.contains("properties"));
}

TEST(Gaze, OmitsGazePointWhenPoseOnly) {
  std::vector<GazeRecord> records;
  GazeRecord r;
  r.t = 1.0;
  r.has_gaze = false;
  records.push_back(r);
  const json j = json::parse(SerializeGaze(Ctx(Stream::kGaze), records, {}));
  EXPECT_FALSE(j["data"][0].contains("g"));
}

TEST(Gaze, CarriesSessionPropertiesWithAnEmptyDataArray) {
  // This is the one that is not obvious and that a naive implementation gets
  // wrong: session and participant metadata reaches the platform ONLY on the
  // gaze stream. There is no /sessions or /participants endpoint. A payload
  // with no gaze records but populated properties is legal and necessary.
  Properties props;
  props.emplace_back("c3d.app.engine", std::string("ROS2"));
  props.emplace_back("c3d.participant.id", std::string("robot01"));
  props.emplace_back("site", std::string("warehouse-3"));
  props.emplace_back("payload_kg", 12.5);
  props.emplace_back("teleop", true);
  props.emplace_back("run_index", 7);

  const json j = json::parse(SerializeGaze(Ctx(Stream::kGaze), {}, props));

  ASSERT_TRUE(j.contains("properties"));
  EXPECT_TRUE(j["data"].is_array());
  EXPECT_EQ(j["data"].size(), 0u);
  EXPECT_EQ(j["properties"]["c3d.app.engine"], "ROS2");
  EXPECT_EQ(j["properties"]["site"], "warehouse-3");
  EXPECT_DOUBLE_EQ(j["properties"]["payload_kg"].get<double>(), 12.5);
  EXPECT_EQ(j["properties"]["teleop"], true);
  EXPECT_EQ(j["properties"]["run_index"], 7);
}

// ---------------------------------------------------------------------------
// Events
// ---------------------------------------------------------------------------

TEST(Events, EnvelopeKeepsFractionalSessionTimestamp) {
  EventRecord e;
  e.name = "c3d.sessionStart";
  e.t = 1787010742.798;
  const json j = json::parse(SerializeEvents(Ctx(Stream::kEvents), {e}));

  EXPECT_EQ(j["part"], 1);
  EXPECT_EQ(j["formatversion"], "1.0");
  EXPECT_EQ(j["userid"], "robot01");
  // Verified divergence from every other stream: events keep the fraction.
  EXPECT_NEAR(j["timestamp"].get<double>(), 1787010742.798, 1e-3);
  EXPECT_EQ(j["data"][0]["name"], "c3d.sessionStart");
  EXPECT_EQ(j["data"][0]["point"], json::array({0.0, 0.0, 0.0}));
  EXPECT_FALSE(j["data"][0].contains("properties"));
  EXPECT_FALSE(j["data"][0].contains("dynamicId"));
}

TEST(Events, PropertiesAreAnObjectNotAnArray) {
  // Custom event properties are an OBJECT. Dynamic object snapshot properties
  // are an ARRAY. Mixing them up on dynamics silently drops the properties.
  EventRecord e;
  e.name = "nav.goal.aborted";
  e.t = 100.0;
  e.dynamic_id = "robot";
  e.properties.emplace_back("goal_id", std::string("abc-123"));
  e.properties.emplace_back("recoveries", 3);
  const json j = json::parse(SerializeEvents(Ctx(Stream::kEvents), {e}));
  const auto& rec = j["data"][0];
  EXPECT_TRUE(rec["properties"].is_object());
  EXPECT_EQ(rec["properties"]["goal_id"], "abc-123");
  EXPECT_EQ(rec["properties"]["recoveries"], 3);
  EXPECT_EQ(rec["dynamicId"], "robot");
}

// ---------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------

TEST(Sensors, DeviceKeyIsNameNotUserid) {
  SensorSeries s;
  s.name = "battery_percent";
  s.hz_limit = 1.0;
  s.samples.push_back(SensorSample{1787010742.798, 42.5});
  s.samples.push_back(SensorSample{1787010743.798, 42.4});

  const json j = json::parse(SerializeSensors(Ctx(Stream::kSensors), {s}));

  // Sensors are the only stream that keys the device on "name", and they carry
  // no "userid" field at all. Verified in both reference implementations.
  EXPECT_EQ(j["name"], "robot01");
  EXPECT_FALSE(j.contains("userid"));
  EXPECT_EQ(j["formatversion"], "2.0");
  EXPECT_EQ(j["part"], 1);
  EXPECT_EQ(j["timestamp"].get<std::int64_t>(), 1787010742);

  const auto& entry = j["data"][0];
  EXPECT_EQ(entry["name"], "battery_percent");
  // Both of these are strings on the wire, including the boolean.
  EXPECT_EQ(entry["sensorHzLimitType"], "1.00");
  EXPECT_EQ(entry["sensorHzLimited"], "true");
  ASSERT_EQ(entry["data"].size(), 2u);
  EXPECT_EQ(entry["data"][0].size(), 2u);
  EXPECT_DOUBLE_EQ(entry["data"][0][1].get<double>(), 42.5);
}

TEST(Sensors, OmitsHzFieldsAboveTenHertz) {
  SensorSeries s;
  s.name = "joint_effort";
  s.hz_limit = 50.0;
  s.samples.push_back(SensorSample{1.0, 0.5});
  const json j = json::parse(SerializeSensors(Ctx(Stream::kSensors), {s}));
  EXPECT_FALSE(j["data"][0].contains("sensorHzLimitType"));
  EXPECT_FALSE(j["data"][0].contains("sensorHzLimited"));
}

// ---------------------------------------------------------------------------
// Dynamics
// ---------------------------------------------------------------------------

TEST(Dynamics, ManifestIsAMapAndDataIsAnArray) {
  ManifestEntry m;
  m.id = "robot";
  m.name = "TurtleBot4";
  m.mesh = "robot_placeholder";

  DynamicSnapshot s;
  s.id = "robot";
  s.t = 1787010742.798;
  s.p = Vec3{1, 2, 3};
  s.r = Quat{0, 0, 0, 1};
  s.scale = Vec3{1, 1, 1};
  s.properties.emplace_back("enabled", true);

  const json j = json::parse(SerializeDynamics(Ctx(Stream::kDynamics), {m}, {s}));

  EXPECT_EQ(j["part"], 0);
  EXPECT_EQ(j["formatversion"], "1.0");
  ASSERT_TRUE(j["manifest"].is_object());
  EXPECT_EQ(j["manifest"]["robot"]["name"], "TurtleBot4");
  EXPECT_EQ(j["manifest"]["robot"]["mesh"], "robot_placeholder");
  EXPECT_EQ(j["manifest"]["robot"]["fileType"], "gltf");

  ASSERT_TRUE(j["data"].is_array());
  const auto& rec = j["data"][0];
  EXPECT_EQ(rec["id"], "robot");
  EXPECT_EQ(rec["s"], json::array({1.0, 1.0, 1.0}));
  // Snapshot properties are an ARRAY of single-key objects. This asymmetry with
  // custom events is real, and getting it wrong drops the properties silently.
  ASSERT_TRUE(rec["properties"].is_array());
  EXPECT_TRUE(rec["properties"][0].is_object());
  EXPECT_EQ(rec["properties"][0]["enabled"], true);
}

TEST(Dynamics, OmitsScaleWhenUnset) {
  DynamicSnapshot s;
  s.id = "robot";
  s.t = 1.0;
  const json j = json::parse(SerializeDynamics(Ctx(Stream::kDynamics), {}, {s}));
  EXPECT_FALSE(j["data"][0].contains("s"));
  // A manifest-only payload with no data array is legal, and so is the inverse.
  EXPECT_FALSE(j.contains("manifest"));
}

TEST(Dynamics, ManifestOnlyPayloadIsLegal) {
  ManifestEntry m;
  m.id = "robot";
  m.name = "r";
  m.mesh = "mesh";
  const json j = json::parse(SerializeDynamics(Ctx(Stream::kDynamics), {m}, {}));
  EXPECT_TRUE(j.contains("manifest"));
  EXPECT_FALSE(j.contains("data"));
}

// ---------------------------------------------------------------------------
// Numeric formatting
// ---------------------------------------------------------------------------

TEST(Numbers, RoundedToFiveDecimalsAndNeverScientific) {
  std::vector<GazeRecord> records;
  GazeRecord r;
  r.t = 1787010742.123456789;
  r.p = Vec3{0.000001234, -1.9999999, 12345.678901};
  records.push_back(r);
  const std::string body = SerializeGaze(Ctx(Stream::kGaze), records, {});

  // Scientific notation for tiny values is the failure mode this guards. Look
  // for an exponent marker attached to a digit, not for the letter e anywhere,
  // since key names legitimately contain one.
  for (std::size_t i = 1; i < body.size(); ++i) {
    const char c = body[i];
    if (c != 'e' && c != 'E') continue;
    EXPECT_FALSE(std::isdigit(static_cast<unsigned char>(body[i - 1])))
        << "scientific notation in payload: " << body;
  }

  const json j = json::parse(body);
  EXPECT_DOUBLE_EQ(j["data"][0]["p"][0].get<double>(), 0.0);
  EXPECT_DOUBLE_EQ(j["data"][0]["p"][1].get<double>(), -2.0);
  EXPECT_DOUBLE_EQ(j["data"][0]["p"][2].get<double>(), 12345.6789);
}

TEST(Numbers, TimestampKeepsMillisecondResolution) {
  // Epoch seconds at 5 decimals still needs 15 significant digits. If this
  // regresses, every record in a session collapses onto the same instant.
  std::vector<GazeRecord> records;
  GazeRecord a, b;
  a.t = 1787010742.100;
  b.t = 1787010742.101;
  records.push_back(a);
  records.push_back(b);
  const json j = json::parse(SerializeGaze(Ctx(Stream::kGaze), records, {}));
  EXPECT_NE(j["data"][0]["time"].get<double>(), j["data"][1]["time"].get<double>());
}

