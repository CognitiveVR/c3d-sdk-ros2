#include <gtest/gtest.h>

#include <string>
#include <utility>
#include <vector>

#include "cognitive3d_ros/topic_plan.hpp"

using namespace cognitive3d_ros;

TEST(TopicPlan, ExactAndWildcardMatching) {
  EXPECT_TRUE(TopicMatches("/odom", "/odom"));
  EXPECT_FALSE(TopicMatches("/odom", "/odometry"));
  EXPECT_TRUE(TopicMatches("/robot1/*", "/robot1/scan"));
  EXPECT_TRUE(TopicMatches("/robot1/*", "/robot1/"));
  EXPECT_FALSE(TopicMatches("/robot1/*", "/robot2/scan"));
}

// A '*' is a wildcard ONLY as the last character. Pinned because the node
// REFUSES to configure an allowlist entry with an interior '*', and that refusal
// is only correct while this remains true: an interior star falls through to the
// exact compare below and can never match a real topic name, so such an entry
// would subscribe to nothing, be skipped by the missing-topic check, and still
// print in the plan dump with an ingest policy.
TEST(TopicPlan, AnInteriorStarIsNotAWildcard) {
  EXPECT_FALSE(TopicMatches("/rosbot*/odom", "/rosbot1/odom"));
  EXPECT_FALSE(TopicMatches("/rosbot*/odom", "/rosbot/odom"));
  EXPECT_FALSE(TopicMatches("/rosbot*/odom", "/rosbot*/odom_literal"));
  // It is a plain string, so only the identical string matches.
  EXPECT_TRUE(TopicMatches("/rosbot*/odom", "/rosbot*/odom"));
  // A trailing star still globs, which is what makes the interior case a bug
  // rather than a documented limitation.
  EXPECT_TRUE(TopicMatches("/rosbot*", "/rosbot1/odom"));
}

TEST(TopicPlan, LongestPatternWins) {
  TopicPlan plan;
  TopicRule wide;
  wide.topic = "/robot1/*";
  wide.ingest = Ingest::kDownsample;
  wide.rate_hz = 1.0;
  TopicRule narrow;
  narrow.topic = "/robot1/joint_states";
  narrow.ingest = Ingest::kDownsample;
  narrow.rate_hz = 20.0;
  plan.AddRule(wide);
  plan.AddRule(narrow);

  const auto r = plan.Resolve("/robot1/joint_states", "sensor_msgs/msg/JointState");
  EXPECT_DOUBLE_EQ(r.rate_hz, 20.0);
}

TEST(TopicPlan, BulkTypesAreRefusedEvenWhenAllowlisted) {
  // A wildcard rule must never be able to opt a point cloud in by accident.
  // Even a best-effort subscription that discards on arrival still copies tens
  // of MB/s through the RMW.
  TopicPlan plan;
  TopicRule wide;
  wide.topic = "/*";
  wide.ingest = Ingest::kFullIngest;
  plan.AddRule(wide);

  EXPECT_EQ(plan.Resolve("/points", "sensor_msgs/msg/PointCloud2").ingest, Ingest::kRefuse);
  EXPECT_EQ(plan.Resolve("/camera/image_raw", "sensor_msgs/msg/Image").ingest, Ingest::kRefuse);
  EXPECT_EQ(plan.Resolve("/map", "nav_msgs/msg/OccupancyGrid").ingest, Ingest::kRefuse);
}

TEST(TopicPlan, BulkTypeCanBeOptedInByExactTopic) {
  TopicPlan plan;
  TopicRule exact;
  exact.topic = "/points";
  exact.ingest = Ingest::kFullIngest;
  plan.AddRule(exact);
  EXPECT_EQ(plan.Resolve("/points", "sensor_msgs/msg/PointCloud2").ingest,
            Ingest::kFullIngest);
}

TEST(TopicPlan, TypeDefaultsAreConservative) {
  double rate = 0.0;
  EXPECT_EQ(DefaultIngestForType("sensor_msgs/msg/PointCloud2", &rate), Ingest::kRefuse);
  EXPECT_EQ(DefaultIngestForType("sensor_msgs/msg/LaserScan", &rate), Ingest::kDeriveMetrics);
  EXPECT_EQ(DefaultIngestForType("sensor_msgs/msg/CameraInfo", &rate), Ingest::kMetadataOnce);
  // Diagnostics is the exception: highest signal to bandwidth ratio on the robot.
  EXPECT_EQ(DefaultIngestForType("diagnostic_msgs/msg/DiagnosticArray", &rate),
            Ingest::kFullIngest);
}

TEST(TopicPlan, UnknownTopicIsNotAllowed) {
  TopicPlan plan;
  TopicRule rule;
  rule.topic = "/odom";
  plan.AddRule(rule);
  EXPECT_TRUE(plan.Allowed("/odom"));
  EXPECT_FALSE(plan.Allowed("/cmd_vel")) << "the allowlist must be a real allowlist";
}

TEST(TopicPlan, MatchFollowsResolveAndNotAllowed) {
  // The two disagree by design: `Allowed` takes the FIRST pattern that matches and
  // `Resolve` the LONGEST. A topic named in `topics` under a wildcard that refuses
  // everything else resolves on its own rule, so anything reporting where the
  // decision came from has to ask `Match` -- asking `Allowed` labels a configured
  // topic "discovered, not in `topics`".
  TopicPlan plan;
  TopicRule wide;
  wide.topic = "/odom*";
  wide.ingest = Ingest::kRefuse;
  TopicRule narrow;
  narrow.topic = "/odometry/filtered";
  narrow.ingest = Ingest::kDownsample;
  plan.AddRule(wide);
  plan.AddRule(narrow);

  EXPECT_FALSE(plan.Allowed("/odometry/filtered"));
  ASSERT_NE(plan.Match("/odometry/filtered"), nullptr)
      << "a topic named in the allowlist was not found in it";
  EXPECT_EQ(plan.Match("/odometry/filtered")->topic, "/odometry/filtered");
  EXPECT_EQ(plan.Resolve("/odometry/filtered", "nav_msgs/msg/Odometry").ingest,
            Ingest::kDownsample);
  EXPECT_EQ(plan.Match("/cmd_vel"), nullptr);
}

// A `series` names ONE channel, so it is not inherited from a wildcard -- the
// same test the bulk-type guard applies to full_ingest, for the same reason.
// Inherited, a single `/robot1/*` rule would record every Float32 under the
// namespace into one series: a rangefinder and a battery voltage interleaved
// into a channel that describes neither, with no way to separate them
// afterwards, because SeriesNameFor returns the name verbatim and nothing
// downstream knows a wildcard was involved.
TEST(TopicPlan, SeriesIsNotInheritedThroughAWildcard) {
  TopicPlan plan;
  TopicRule wide;
  wide.topic = "/robot1/*";
  wide.ingest = Ingest::kDownsample;
  wide.rate_hz = 4.0;
  wide.series = "ultrasonic.distance_centimeters";
  plan.AddRule(wide);

  const auto range = plan.Resolve("/robot1/ultrasonic", "std_msgs/msg/Float32");
  const auto volts = plan.Resolve("/robot1/battery_voltage", "std_msgs/msg/Float32");
  EXPECT_EQ(range.series, "") << "two unrelated sensors would share one series name";
  EXPECT_EQ(volts.series, "");
  // Only the NAME is refused. Everything else about the wildcard's rule still
  // applies, or refusing it would be a policy change rather than a fix.
  EXPECT_EQ(range.ingest, Ingest::kDownsample);
  EXPECT_DOUBLE_EQ(range.rate_hz, 4.0);
}

TEST(TopicPlan, SeriesSurvivesOnTheTopicsOwnRule) {
  // The narrow rule wins on length, and because it names the topic exactly its
  // series is the operator's own statement about that one channel.
  TopicPlan plan;
  TopicRule wide;
  wide.topic = "/robot1/*";
  wide.series = "everything";
  TopicRule narrow;
  narrow.topic = "/robot1/ultrasonic";
  narrow.series = "ultrasonic.distance_centimeters";
  plan.AddRule(wide);
  plan.AddRule(narrow);

  EXPECT_EQ(plan.Resolve("/robot1/ultrasonic", "std_msgs/msg/Float32").series,
            "ultrasonic.distance_centimeters");
  EXPECT_EQ(plan.Resolve("/robot1/odom", "nav_msgs/msg/Odometry").series, "");
}

// A topic no rule matches carries no series either, which is what leaves
// SeriesNameFor's topic-derived fallback as the only path such a topic can take.
TEST(TopicPlan, AnUnmatchedTopicCarriesNoSeries) {
  TopicPlan plan;
  TopicRule rule;
  rule.topic = "/ultrasonic";
  rule.series = "ultrasonic.distance_centimeters";
  plan.AddRule(rule);

  EXPECT_EQ(plan.Resolve("/ultrasonic", "std_msgs/msg/Float32").series,
            "ultrasonic.distance_centimeters");
  EXPECT_EQ(plan.Resolve("/battery_state", "sensor_msgs/msg/BatteryState").series, "")
      << "a topic taken by type rather than by the allowlist has no rule to name it";
}

TEST(TopicPlan, ParseIngestRejectsGarbage) {
  Ingest out;
  EXPECT_TRUE(ParseIngest("full_ingest", &out));
  EXPECT_EQ(out, Ingest::kFullIngest);
  EXPECT_FALSE(ParseIngest("everything_please", &out));
}

// The rates the documentation states for each message type. The robot's motion and its plain
// sensors record at the pose timeline's 30 Hz; batteries and laser scan summaries at 1 Hz, as does
// every other type, tf among them.
TEST(TopicPlan, TypeDefaultRates) {
  const auto rate_of = [](const std::string& type) {
    double rate = 0.0;
    DefaultIngestForType(type, &rate);
    return rate;
  };
  for (const char* type : {"nav_msgs/msg/Odometry", "sensor_msgs/msg/JointState",
                           "sensor_msgs/msg/Imu", "std_msgs/msg/Float32",
                           "std_msgs/msg/Int32MultiArray"}) {
    EXPECT_DOUBLE_EQ(rate_of(type), 30.0) << type;
  }
  for (const char* type : {"sensor_msgs/msg/BatteryState", "sensor_msgs/msg/LaserScan",
                           "nav2_msgs/msg/BehaviorTreeLog", "tf2_msgs/msg/TFMessage",
                           "geometry_msgs/msg/Twist"}) {
    EXPECT_DOUBLE_EQ(rate_of(type), 1.0) << type;
  }
}

// An allowlist entry with no rule of its own takes its message type's default, so `/odom` in the
// minimal file records at odometry's 30 Hz rather than a flat 1 Hz.
TEST(TopicPlan, AValueTheRuleLeavesOutIsTheMessageTypesDefault) {
  TopicPlan plan;
  TopicRule odom;
  odom.topic = "/odom";
  odom.ingest_from_type = true;
  odom.rate_from_type = true;
  plan.AddRule(odom);
  TopicRule joints;
  joints.topic = "/joint_states";
  joints.ingest = Ingest::kDownsample;
  joints.rate_hz = 2.0;
  joints.ingest_from_type = true;
  plan.AddRule(joints);

  const TopicRule resolved_odom = plan.Resolve("/odom", "nav_msgs/msg/Odometry");
  EXPECT_DOUBLE_EQ(resolved_odom.rate_hz, 30.0);
  EXPECT_EQ(resolved_odom.ingest, Ingest::kDownsample);
  EXPECT_TRUE(resolved_odom.rate_from_type);
  EXPECT_TRUE(resolved_odom.ingest_from_type);

  const TopicRule resolved_joints = plan.Resolve("/joint_states", "sensor_msgs/msg/JointState");
  EXPECT_DOUBLE_EQ(resolved_joints.rate_hz, 2.0) << "a configured rate stays";
  EXPECT_FALSE(resolved_joints.rate_from_type);

  const TopicRule scan = plan.Resolve("/scan", "sensor_msgs/msg/LaserScan");
  EXPECT_EQ(scan.ingest, Ingest::kDeriveMetrics) << "no rule at all: the type's default";
  EXPECT_TRUE(scan.rate_from_type);
}

// A QoS override the resolver does not act on would be ignored while the subscription still said
// it was overridden, so each field takes only the values ResolveQos applies, and the refusal names
// the parameter and those values.
TEST(QosOverrideValidation, EachFieldTakesOnlyTheValuesTheResolverApplies) {
  EXPECT_EQ(ValidateQosOverride(QosOverride{}, "topic_config.scan"), "") << "unset is valid";
  QosOverride every;
  every.reliability = "best_effort";
  every.durability = "transient_local";
  every.history = "keep_all";
  every.depth = 1;
  EXPECT_EQ(ValidateQosOverride(every, "topic_config.scan"), "");
  every.reliability = "reliable";
  every.durability = "volatile";
  every.history = "keep_last";
  every.depth = 100;
  EXPECT_EQ(ValidateQosOverride(every, "topic_config.scan"), "");

  struct Case {
    QosOverride qos;
    std::string parameter;
    std::string accepted;
  };
  const auto with = [](void (*set)(QosOverride&)) {
    QosOverride qos;
    set(qos);
    return qos;
  };
  const std::vector<Case> cases = {
      {with([](QosOverride& q) { q.reliability = "best-effort"; }), "camera.front.qos.reliability",
       "'reliable' or 'best_effort'"},
      {with([](QosOverride& q) { q.reliability = "BEST_EFFORT"; }), "camera.front.qos.reliability",
       "'reliable' or 'best_effort'"},
      {with([](QosOverride& q) { q.durability = "transient-local"; }),
       "camera.front.qos.durability", "'volatile' or 'transient_local'"},
      {with([](QosOverride& q) { q.history = "keepall"; }), "camera.front.qos.history",
       "'keep_last' or 'keep_all'"},
      {with([](QosOverride& q) { q.depth = 0; }), "camera.front.qos.depth", "1 or more"},
      {with([](QosOverride& q) { q.depth = -2; }), "camera.front.qos.depth", "1 or more"},
  };
  for (const auto& c : cases) {
    const std::string problem = ValidateQosOverride(c.qos, "camera.front");
    EXPECT_NE(problem.find(c.parameter), std::string::npos) << problem;
    EXPECT_NE(problem.find(c.accepted), std::string::npos) << problem;
  }
}

// Only `refuse`, and `full_ingest` on a bulk type, change what is recorded. An ingest value beside a
// subscribed topic reads as a mode in force, so the log names one only where it does something, and
// marks one set where it does nothing.
TEST(IngestText, TheLogNamesAnIngestValueOnlyWhereItHasAnEffectOrWasSet) {
  TopicPlan plan;
  TopicRule odom;
  odom.topic = "/odom";
  odom.ingest_from_type = true;
  odom.rate_from_type = true;
  plan.AddRule(odom);
  TopicRule imu;
  imu.topic = "/imu";
  imu.ingest = Ingest::kMetadataOnce;
  plan.AddRule(imu);
  TopicRule camera;
  camera.topic = "/camera/image_raw";
  camera.ingest = Ingest::kFullIngest;
  plan.AddRule(camera);
  TopicRule battery;
  battery.topic = "/battery_state";
  battery.ingest = Ingest::kFullIngest;
  plan.AddRule(battery);
  TopicRule quiet;
  quiet.topic = "/quiet";
  quiet.ingest = Ingest::kRefuse;
  plan.AddRule(quiet);

  EXPECT_EQ(PlannedIngestText(odom), "(message type default)");
  EXPECT_EQ(PlannedIngestText(imu), "metadata_once (configured, has no effect)");
  EXPECT_EQ(PlannedIngestText(camera), "full_ingest (configured)");
  EXPECT_EQ(PlannedIngestText(quiet), "refuse (configured)");

  for (const auto& [topic, type] : std::vector<std::pair<std::string, std::string>>{
           {"/odom", "nav_msgs/msg/Odometry"},
           {"/scan", "sensor_msgs/msg/LaserScan"},
           {"/diagnostics", "diagnostic_msgs/msg/DiagnosticArray"},
           {"/info", "sensor_msgs/msg/CameraInfo"}}) {
    EXPECT_EQ(SubscribedIngestText(plan.Resolve(topic, type), type), "") << topic;
  }
  EXPECT_EQ(SubscribedIngestText(plan.Resolve("/imu", "sensor_msgs/msg/Imu"),
                                 "sensor_msgs/msg/Imu"),
            "metadata_once (configured, has no effect)");
  EXPECT_EQ(SubscribedIngestText(plan.Resolve("/battery_state", "sensor_msgs/msg/BatteryState"),
                                 "sensor_msgs/msg/BatteryState"),
            "full_ingest (configured, has no effect)");
  EXPECT_EQ(SubscribedIngestText(plan.Resolve("/camera/image_raw", "sensor_msgs/msg/Image"),
                                 "sensor_msgs/msg/Image"),
            "full_ingest (configured)");
}
