// What the node says about its uploads. A refused key keeps every chunk and sends none, so the
// one thing that tells it apart from a slow link is what is said here.

#include <gtest/gtest.h>

#include <map>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include "cognitive3d_ros/upload_health.hpp"

namespace {

using cognitive3d_ros::DescribeUploads;
using cognitive3d_ros::UploadHealth;
using diagnostic_msgs::msg::DiagnosticStatus;

std::string Value(const UploadHealth& health, const std::string& key) {
  for (const auto& [k, v] : health.values) {
    if (k == key) return v;
  }
  return "(absent)";
}

bool Has(const std::string& text, const std::string& needle) {
  return text.find(needle) != std::string::npos;
}

TEST(UploadHealth, ARefusedKeyIsAnErrorThatSaysEverythingIsKept) {
  c3d::Stats stats;
  stats.uploads_auth_paused = true;
  stats.auth_refused_streams = {c3d::Stream::kEvents};
  stats.auth_rejections = 3;
  const UploadHealth health = DescribeUploads(stats, 240.0);
  EXPECT_EQ(health.level, DiagnosticStatus::ERROR);
  EXPECT_TRUE(Has(health.message, "PAUSED because the credentials were refused")) << health.message;
  EXPECT_TRUE(Has(health.message, "everything stays spooled")) << health.message;
  EXPECT_TRUE(Has(health.message, "every 240s")) << health.message;
  EXPECT_EQ(health.report, " [UPLOADS PAUSED: credentials refused]");
  EXPECT_EQ(Value(health, "uploads_auth_paused"), "true");
  EXPECT_EQ(Value(health, "auth_rejections"), "3");
}

// Some routes refused while the rest upload is not a pause, and must not read as healthy either.
TEST(UploadHealth, AStreamRefusedWhileTheRestUploadIsAWarningNamingIt) {
  c3d::Stats stats;
  stats.auth_refused_streams = {c3d::Stream::kSensors, c3d::Stream::kCamera};
  const UploadHealth health = DescribeUploads(stats, 240.0);
  EXPECT_EQ(health.level, DiagnosticStatus::WARN);
  EXPECT_TRUE(Has(health.message, "refused for sensors, camera")) << health.message;
  EXPECT_TRUE(Has(health.message, "every other stream uploads")) << health.message;
  EXPECT_EQ(health.report, " [CREDENTIALS REFUSED: sensors, camera]");
  EXPECT_EQ(Value(health, "uploads_auth_paused"), "false");
  EXPECT_EQ(Value(health, "auth_refused_streams"), "sensors, camera");
}

// A stream the gateway keeps failing while it accepts the others waits behind the rest of the
// spool. Its data is not uploading, so it must not read as healthy, and it is named.
TEST(UploadHealth, AStreamDeferredForItsOwnFailuresIsAWarningNamingIt) {
  c3d::Stats stats;
  stats.deferred_streams = {c3d::Stream::kCamera};
  stats.chunks_deferred = 2;
  const UploadHealth health = DescribeUploads(stats, 240.0);
  EXPECT_EQ(health.level, DiagnosticStatus::WARN);
  EXPECT_TRUE(Has(health.message, "keeps failing the chunks of camera")) << health.message;
  EXPECT_TRUE(Has(health.message, "from every 240s doubling to every hour")) << health.message;
  EXPECT_EQ(health.report, " [STREAMS DEFERRED: camera] deferred=2");
  EXPECT_EQ(Value(health, "deferred_streams"), "camera");
  EXPECT_EQ(Value(health, "chunks_deferred"), "2");
}

TEST(UploadHealth, AHealthyRunIsOkAndFlagsNothing) {
  const UploadHealth health = DescribeUploads(c3d::Stats{}, 240.0);
  EXPECT_EQ(health.level, DiagnosticStatus::OK);
  EXPECT_EQ(health.message, "uploading");
  EXPECT_EQ(health.report, "");
}

TEST(UploadHealth, TheWorstStateSetsTheLevelAndEveryStateIsSaid) {
  c3d::Stats stats;
  stats.uploads_paused = true;
  stats.backing_off = true;
  UploadHealth health = DescribeUploads(stats, 240.0);
  EXPECT_EQ(health.level, DiagnosticStatus::WARN);
  EXPECT_EQ(health.report, " [UPLOADS PAUSED] [BACKING OFF]");
  EXPECT_TRUE(Has(health.message, "uploads_paused: true")) << health.message;

  stats.uploads_auth_paused = true;
  health = DescribeUploads(stats, 240.0);
  EXPECT_EQ(health.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(health.report, " [UPLOADS PAUSED: credentials refused] [UPLOADS PAUSED] [BACKING OFF]");
}

// Held and deferred chunks wait and are still sent; an evicted one is gone.
TEST(UploadHealth, HeldDeferredAndEvictedChunksAreCountedAndOnlyAnEvictionWarns) {
  c3d::Stats stats;
  stats.chunks_held = 2;
  stats.chunks_deferred = 1;
  UploadHealth health = DescribeUploads(stats, 240.0);
  EXPECT_EQ(health.level, DiagnosticStatus::OK);
  EXPECT_EQ(health.report, " held=2 deferred=1");
  EXPECT_EQ(Value(health, "chunks_held"), "2");
  EXPECT_EQ(Value(health, "chunks_deferred"), "1");

  stats.chunks_evicted = 4;
  health = DescribeUploads(stats, 240.0);
  EXPECT_EQ(health.level, DiagnosticStatus::WARN);
  EXPECT_TRUE(Has(health.report, " EVICTED=4")) << health.report;
  EXPECT_TRUE(Has(health.message, "4 chunk(s) evicted")) << health.message;
  EXPECT_EQ(Value(health, "chunks_evicted"), "4");
}

TEST(UploadSeries, EveryCounterCoreKeepsHasItsOwnSeries) {
  c3d::Stats stats;
  stats.auth_rejections = 3;
  stats.uploads_auth_paused = true;
  stats.auth_refused_streams = {c3d::Stream::kSensors, c3d::Stream::kCamera};
  stats.chunks_held = 4;
  stats.chunks_deferred = 5;
  stats.deferred_streams = {c3d::Stream::kGaze};
  stats.chunks_evicted = 6;
  stats.serialization_failures = 7;
  stats.spool_write_failures = 8;
  stats.thread_exceptions = 9;
  stats.camera_access_units_dropped = 10;
  std::map<std::string, double> series;
  for (const auto& [name, value] : cognitive3d_ros::UploadSeries(stats)) {
    EXPECT_TRUE(series.emplace(name, value).second) << name << " twice";
  }
  const std::map<std::string, double> expected = {
      {"c3d.upload_auth_rejections", 3.0},     {"c3d.upload_auth_paused", 1.0},
      {"c3d.upload_auth_refused_streams", 2.0}, {"c3d.chunks_held", 4.0},
      {"c3d.chunks_deferred", 5.0},             {"c3d.upload_deferred_streams", 1.0},
      {"c3d.chunks_evicted", 6.0},
      {"c3d.serialization_failures", 7.0},      {"c3d.spool_write_failures", 8.0},
      {"c3d.thread_exceptions", 9.0},          {"c3d.camera_access_units_dropped", 10.0},
  };
  EXPECT_EQ(series, expected);
}

}  // namespace
