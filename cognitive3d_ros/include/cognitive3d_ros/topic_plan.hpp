#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace cognitive3d_ros {

// How much of a topic we are willing to take.
//
// The default is never kFullIngest. A 32-beam LIDAR publishes roughly 2 MB per
// message at 10 to 20 Hz, and an unclassified allowlist is how a telemetry node
// saturates a robot's network. Full ingest is opt-in, per topic, always.
enum class Ingest {
  kRefuse,        // do not even create the subscription
  kMetadataOnce,  // take the first message, then unsubscribe
  kDeriveMetrics, // never store the payload, only scalars derived from it
  kDownsample,    // take at rate_hz
  kFullIngest,    // take everything
};

const char* IngestName(Ingest i);
bool ParseIngest(const std::string& text, Ingest* out);

struct QosOverride {
  // Empty means "do not override, resolve from the publishers".
  std::string reliability;  // "reliable" | "best_effort"
  std::string durability;   // "volatile" | "transient_local"
  std::string history;      // "keep_last" | "keep_all"
  int depth = -1;
  bool any() const {
    return !reliability.empty() || !durability.empty() || !history.empty() || depth >= 0;
  }
};

// Empty when each field of `qos` is unset or a value ResolveQos applies, otherwise the refusal,
// naming `<key>.qos.<field>` and the values it takes. Any other value would be ignored while the
// subscription still reported the profile as overridden by configuration.
std::string ValidateQosOverride(const QosOverride& qos, const std::string& key);

struct TopicRule {
  std::string topic;
  Ingest ingest = Ingest::kDownsample;
  double rate_hz = 1.0;
  QosOverride qos;
  // The Cognitive3D sensor-series name values from this topic are recorded
  // under. EMPTY means "derive it from the topic name", which is all a
  // translator keyed by a bare std_msgs type can do on its own -- the type
  // carries no unit and no meaning. This is where a unit gets STATED, e.g.
  // `ultrasonic.distance_centimeters`. Translators that record more than one
  // number per message treat it as a base and append (`<series>.0`).
  //
  // Only ever read off a rule whose `topic` is the topic itself: `Resolve`
  // clears it when the rule that matched was a wildcard, because one name
  // cannot describe a whole namespace.
  std::string series;
  // The parameter group this rule was read from, `topic_config.<key>`, for messages that name the
  // line to change. Empty for a rule nothing configured.
  std::string config_key;
  // On a configured rule: the operator set no `ingest` / no `rate_hz`, so each is taken from the
  // message type's default when the rule is resolved against a topic. On a resolved rule: that
  // value came from the type. Either way the log can say where a value came from.
  bool ingest_from_type = false;
  bool rate_from_type = false;
};

// Message types we refuse by default regardless of what the allowlist says,
// unless the operator explicitly sets full_ingest for that topic.
bool IsBulkType(const std::string& type_name);

// Built-in per-type defaults. These encode the per-type bandwidth rules so an
// operator gets a safe configuration without having to know them.
Ingest DefaultIngestForType(const std::string& type_name, double* rate_hz_out);

// `ingest` as the log prints it. Only `refuse`, and `full_ingest` on a bulk type, change what is
// recorded: under any other value a subscribed topic's translator records it as it would anyway,
// so a value set there is printed as having no effect, and one nobody set is not printed.
//
// The configure plan's text for a configured rule, before the topic's type is known.
std::string PlannedIngestText(const TopicRule& rule);
// A `subscribed` line's text for the rule a topic of `type` resolved to; empty for no text.
std::string SubscribedIngestText(const TopicRule& resolved, const std::string& type);

class TopicPlan {
 public:
  void AddRule(const TopicRule& rule);
  // Longest-prefix / exact match against configured rules. A value the matched rule leaves to the
  // type, and every value when no rule matches, is the message type's default.
  TopicRule Resolve(const std::string& topic, const std::string& type) const;
  bool Allowed(const std::string& topic) const;
  // The configured rule Resolve() resolves against, or null when no configured
  // pattern matches the topic at all. `Allowed` answers a different question with a
  // different rule -- the FIRST pattern that matches, and whether it refuses -- so
  // anything that has to agree with what was RESOLVED asks this rather than that.
  const TopicRule* Match(const std::string& topic) const;
  const std::vector<TopicRule>& rules() const { return rules_; }

 private:
  std::vector<TopicRule> rules_;
};

// Glob matching supporting a single trailing '*', which covers the namespaced
// multi-robot case (/robot1/*) without pulling in a regex dependency.
bool TopicMatches(const std::string& pattern, const std::string& topic);

}  // namespace cognitive3d_ros
