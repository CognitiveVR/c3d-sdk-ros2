#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cognitive3d_ros/event_rate_limiter.hpp"
#include "cognitive3d_ros/rate_gate.hpp"
#include "cognitive3d_ros/session_runner.hpp"
#include "cognitive3d_ros/topic_plan.hpp"

namespace cognitive3d_ros {

// Everything a translator needs that is not the message itself.
struct TranslationContext {
  std::string topic;
  std::string type;
  Ingest ingest = Ingest::kDownsample;
  double rate_hz = 1.0;
  // The sensor-series name the operator configured for this topic, or empty.
  // Read it through SeriesNameFor() rather than directly, so every translator
  // falls back the same way.
  std::string series;

  // Epoch wall-clock seconds. This is what goes on the wire, always. See
  // ClockPolicy: the Cognitive3D session id embeds an epoch timestamp and sim
  // clocks start near zero.
  double wire_time = 0.0;
  // ROS time in seconds when the message arrived. No shipped translator records it, and it must
  // never reach the wire.
  double ros_time = 0.0;
  // When the message arrived, on the clock every rate gate runs on: what a translator's RateGate
  // reads, never `wire_time`.
  double gate_time = 0.0;
  // Age of the message according to its own header, or negative when the
  // message type has no header. Useful as a health metric in its own right:
  // it measures publisher backlog and cross-machine clock skew.
  double header_age = -1.0;

  // Where the robot BODY was at the last pose sample, in ROS axes and in the
  // reference frame in force. An event with no position is a row in a table; an
  // event with one is a point on the map, which is most of what this platform is
  // for. `position_known` is false until the first sample has landed, and an event
  // recorded before then goes to the origin carrying a property that says so
  // rather than quietly claiming it.
  c3d::Vec3 body_position;
  bool position_known = false;

  // The per-source event cap, or null for none. Applied by RecordTranslatorEvent
  // and keyed on `topic`, so one action server in a loop spends its own budget
  // and not the session's.
  EventRateLimiter* event_limiter = nullptr;
};

// Every event a translator produces goes through here: `RecordCappedEvent` with the
// context's own topic as the cap's source key and its position as the point.
//
// Returns whether the event reached the session, and a translator that carries a
// state machine has to READ it. See RecordCappedEvent in session_runner.hpp.
bool RecordTranslatorEvent(SessionRunner& runner, const TranslationContext& ctx,
                           const std::string& name, c3d::Properties properties);

// The sensor-series name to record under.
//
// `topic_config.<topic>.series` wins when the operator set one. Otherwise the
// topic is dotted -- `/line_sensor` becomes `line_sensor`, `/a/b` becomes `a.b`
// -- and `suffix` is appended when it is non-empty, so a single-value
// translator produces `<topic>.value` rather than a bare `<topic>` with no dot.
// The dot matters downstream: the replay viewer groups custom sensors by their
// first dot-segment and drops anything without one into an undifferentiated
// "Custom" bucket.
std::string SeriesNameFor(const TranslationContext& ctx, const std::string& suffix);

// One translator per message type. Deserializing from the generic
// subscription's serialized buffer rather than taking typed subscriptions is
// what lets the same code path serve live topics and, later, bag replay.
class ITranslator {
 public:
  virtual ~ITranslator() = default;
  virtual const char* type_name() const = 0;
  virtual void Translate(const rclcpp::SerializedMessage& message,
                         const TranslationContext& ctx, SessionRunner& runner) = 0;
  // Called on session start so a translator can reset any change-detection
  // state it holds, rather than carrying it across a session boundary.
  virtual void Reset() {}
  // The series this translator writes when their names are the same for every topic of its
  // type -- `imu.*` -- or empty when each topic gets series of its own. One instance serves every
  // topic of a type, so two topics of such a type interleave in one series.
  virtual std::string fixed_series() const { return {}; }
  // Whether this translator records under `topic_config.<key>.series` when one is set. On any
  // other translator's topic the name is ignored, which the node says when it subscribes.
  virtual bool reads_series() const { return false; }
};

class TranslatorRegistry {
 public:
  void Register(std::unique_ptr<ITranslator> translator);
  ITranslator* Find(const std::string& type_name) const;
  bool Handles(const std::string& type_name) const { return Find(type_name) != nullptr; }
  void ResetAll();
  std::vector<std::string> known_types() const;
  // The types whose translator reads_series().
  std::vector<std::string> series_types() const;

 private:
  std::map<std::string, std::unique_ptr<ITranslator>> by_type_;
};

// What to say when `second_topic` of `type` is recorded while `first_topic` of the same type
// already is, and that type's translator writes fixed series names: both topics, and that only
// one should be allowlisted. Empty for any other type.
std::string SharedSeriesWarning(const TranslatorRegistry& registry, const std::string& type,
                                const std::string& first_topic, const std::string& second_topic);

// Builds the registry for the message types every robot has. Nav2 and MoveIt
// translators live in their own optional packages, so a robot running neither
// does not have to install both.
void RegisterCoreTranslators(TranslatorRegistry& registry);

// What an optional package hands the node to add its own types. The dependency
// runs one way only -- the optional package includes this header, never the
// reverse -- so `cognitive3d_ros` builds and runs with none of them installed,
// and the composition happens in whichever package has the message types.
using TranslatorRegistrar = void (*)(TranslatorRegistry&);

// Helper for translators: deserialize into a typed message.
template <typename MessageT>
bool Deserialize(const rclcpp::SerializedMessage& serialized, MessageT* out) {
  try {
    static rclcpp::Serialization<MessageT> serializer;
    serializer.deserialize_message(&serialized, out);
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

}  // namespace cognitive3d_ros
