#include "cognitive3d_ros/topic_plan.hpp"

#include <algorithm>

namespace cognitive3d_ros {

const char* IngestName(Ingest i) {
  switch (i) {
    case Ingest::kRefuse: return "refuse";
    case Ingest::kMetadataOnce: return "metadata_once";
    case Ingest::kDeriveMetrics: return "derive_metrics";
    case Ingest::kDownsample: return "downsample";
    case Ingest::kFullIngest: return "full_ingest";
  }
  return "downsample";
}

bool ParseIngest(const std::string& text, Ingest* out) {
  if (text == "refuse") { *out = Ingest::kRefuse; return true; }
  if (text == "metadata_once") { *out = Ingest::kMetadataOnce; return true; }
  if (text == "derive_metrics") { *out = Ingest::kDeriveMetrics; return true; }
  if (text == "downsample") { *out = Ingest::kDownsample; return true; }
  if (text == "full_ingest") { *out = Ingest::kFullIngest; return true; }
  return false;
}

std::string ValidateQosOverride(const QosOverride& qos, const std::string& key) {
  const auto refuse = [&key](const char* field, const char* accepted, const std::string& value) {
    return key + ".qos." + field + " must be " + accepted +
           ", or empty to follow the publishers; got '" + value + "'";
  };
  if (!qos.reliability.empty() && qos.reliability != "reliable" &&
      qos.reliability != "best_effort") {
    return refuse("reliability", "'reliable' or 'best_effort'", qos.reliability);
  }
  if (!qos.durability.empty() && qos.durability != "volatile" &&
      qos.durability != "transient_local") {
    return refuse("durability", "'volatile' or 'transient_local'", qos.durability);
  }
  if (!qos.history.empty() && qos.history != "keep_last" && qos.history != "keep_all") {
    return refuse("history", "'keep_last' or 'keep_all'", qos.history);
  }
  // rclcpp reads a keep-last depth of 0 as the system default rather than as a queue.
  if (qos.depth == 0 || qos.depth < -1) {
    return key + ".qos.depth must be 1 or more, or -1 to follow the publishers; got " +
           std::to_string(qos.depth);
  }
  return "";
}

bool IsBulkType(const std::string& type) {
  // Never subscribe to these by default. Even a best-effort subscription that
  // discards on arrival still copies every megabyte through the RMW.
  static const char* kBulk[] = {
      "sensor_msgs/msg/PointCloud2",
      "sensor_msgs/msg/PointCloud",
      "sensor_msgs/msg/Image",
      "sensor_msgs/msg/CompressedImage",
      "sensor_msgs/msg/MultiEchoLaserScan",
      "octomap_msgs/msg/Octomap",
      "nav_msgs/msg/OccupancyGrid",
  };
  for (const char* b : kBulk) {
    if (type == b) return true;
  }
  return false;
}

Ingest DefaultIngestForType(const std::string& type, double* rate_hz_out) {
  double rate = 1.0;
  Ingest ingest = Ingest::kDownsample;

  if (IsBulkType(type)) {
    ingest = Ingest::kRefuse;
  } else if (type == "tf2_msgs/msg/TFMessage") {
    // tf aggregates well past 100 Hz. Sampling it at rate is the fastest way to
    // blow the bandwidth budget, and analytics needs 10 Hz at most.
    ingest = Ingest::kDownsample;
    rate = 10.0;
  } else if (type == "sensor_msgs/msg/JointState") {
    ingest = Ingest::kDownsample;
    rate = 5.0;
  } else if (type == "nav_msgs/msg/Odometry") {
    ingest = Ingest::kDownsample;
    rate = 10.0;
  } else if (type == "sensor_msgs/msg/BatteryState") {
    ingest = Ingest::kDownsample;
    rate = 1.0;
  } else if (type == "diagnostic_msgs/msg/DiagnosticArray") {
    // Highest signal to bandwidth ratio on the robot, but only on transitions.
    ingest = Ingest::kFullIngest;
    rate = 1.0;
  } else if (type == "sensor_msgs/msg/LaserScan") {
    ingest = Ingest::kDeriveMetrics;
    rate = 1.0;
  } else if (type == "sensor_msgs/msg/CameraInfo" ||
             type == "std_msgs/msg/String") {
    // Static in practice. Take it once and stop.
    ingest = Ingest::kMetadataOnce;
  } else if (type == "action_msgs/msg/GoalStatusArray") {
    ingest = Ingest::kFullIngest;
  }

  if (rate_hz_out) *rate_hz_out = rate;
  return ingest;
}

std::string PlannedIngestText(const TopicRule& rule) {
  if (rule.ingest_from_type) return "(message type default)";
  if (rule.ingest == Ingest::kRefuse || rule.ingest == Ingest::kFullIngest) {
    return std::string(IngestName(rule.ingest)) + " (configured)";
  }
  return std::string(IngestName(rule.ingest)) + " (configured, has no effect)";
}

std::string SubscribedIngestText(const TopicRule& resolved, const std::string& type) {
  const bool effect = resolved.ingest == Ingest::kRefuse ||
                      (resolved.ingest == Ingest::kFullIngest && IsBulkType(type));
  if (effect) {
    return std::string(IngestName(resolved.ingest)) +
           (resolved.ingest_from_type ? " (type default)" : " (configured)");
  }
  if (resolved.ingest_from_type) return "";
  return std::string(IngestName(resolved.ingest)) + " (configured, has no effect)";
}

bool TopicMatches(const std::string& pattern, const std::string& topic) {
  if (pattern.empty()) return false;
  if (pattern.back() != '*') return pattern == topic;
  const std::string prefix = pattern.substr(0, pattern.size() - 1);
  return topic.compare(0, prefix.size(), prefix) == 0 && topic.size() >= prefix.size();
}

void TopicPlan::AddRule(const TopicRule& rule) { rules_.push_back(rule); }

bool TopicPlan::Allowed(const std::string& topic) const {
  for (const auto& r : rules_) {
    if (TopicMatches(r.topic, topic)) return r.ingest != Ingest::kRefuse;
  }
  return false;
}

const TopicRule* TopicPlan::Match(const std::string& topic) const {
  const TopicRule* best = nullptr;
  std::size_t best_len = 0;
  for (const auto& r : rules_) {
    if (!TopicMatches(r.topic, topic)) continue;
    // Longest pattern wins, so /robot1/scan beats /robot1/*.
    const std::size_t len = r.topic.size();
    if (best == nullptr || len > best_len) {
      best = &r;
      best_len = len;
    }
  }
  return best;
}

TopicRule TopicPlan::Resolve(const std::string& topic, const std::string& type) const {
  const TopicRule* best = Match(topic);

  TopicRule result;
  result.topic = topic;
  double type_rate = 1.0;
  const Ingest type_ingest = DefaultIngestForType(type, &type_rate);
  if (best != nullptr) {
    result = *best;
    result.topic = topic;
    // A series name is a name for ONE channel, so it is not inherited from a
    // wildcard -- the same rule the bulk-type guard below applies to
    // full_ingest, for the same reason: what a wildcard says about a whole
    // namespace is not what the operator said about this topic. Inherited, one
    // `/robot1/*` rule would record a rangefinder and a battery voltage into a
    // single series that describes neither, and SeriesNameFor has no way to
    // tell them apart afterwards. Left empty, each topic falls back to its own
    // dotted name and stays distinguishable.
    if (best->topic != topic) result.series.clear();
  } else {
    result.ingest_from_type = true;
    result.rate_from_type = true;
  }
  if (result.ingest_from_type) result.ingest = type_ingest;
  if (result.rate_from_type) result.rate_hz = type_rate;

  // A bulk type is refused unless the operator asked for it by name and by
  // value. Inheriting full_ingest from a wildcard rule does not count.
  if (IsBulkType(type) && !(best != nullptr && best->ingest == Ingest::kFullIngest &&
                            best->topic == topic)) {
    result.ingest = Ingest::kRefuse;
  }
  return result;
}

}  // namespace cognitive3d_ros
