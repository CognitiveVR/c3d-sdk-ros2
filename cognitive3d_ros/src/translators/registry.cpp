#include "cognitive3d_ros/translators.hpp"

#include <string>

#include "cognitive3d_ros/translators/battery_translator.hpp"
#include "cognitive3d_ros/translators/diagnostics_translator.hpp"
#include "cognitive3d_ros/translators/float32_translator.hpp"
#include "cognitive3d_ros/translators/int32_multi_array_translator.hpp"
#include "cognitive3d_ros/translators/imu_translator.hpp"
#include "cognitive3d_ros/translators/joint_state_translator.hpp"
#include "cognitive3d_ros/translators/laser_scan_translator.hpp"
#include "cognitive3d_ros/translators/objective_tracker.hpp"
#include "cognitive3d_ros/translators/odometry_translator.hpp"

namespace cognitive3d_ros {

void TranslatorRegistry::Register(std::unique_ptr<ITranslator> translator) {
  if (!translator) return;
  by_type_[translator->type_name()] = std::move(translator);
}

ITranslator* TranslatorRegistry::Find(const std::string& type_name) const {
  auto it = by_type_.find(type_name);
  return it == by_type_.end() ? nullptr : it->second.get();
}

void TranslatorRegistry::ResetAll() {
  for (auto& kv : by_type_) kv.second->Reset();
}

std::vector<std::string> TranslatorRegistry::known_types() const {
  std::vector<std::string> out;
  out.reserve(by_type_.size());
  for (const auto& kv : by_type_) out.push_back(kv.first);
  return out;
}

std::vector<std::string> TranslatorRegistry::series_types() const {
  std::vector<std::string> out;
  for (const auto& kv : by_type_) {
    if (kv.second->reads_series()) out.push_back(kv.first);
  }
  return out;
}

std::string SeriesNameFor(const TranslationContext& ctx, const std::string& suffix) {
  // An explicit name is used EXACTLY as written, suffix and all -- the point of
  // setting one is to say what the channel is, and appending to it would undo
  // that.
  if (!ctx.series.empty()) return ctx.series;
  std::string dotted;
  dotted.reserve(ctx.topic.size());
  for (const char c : ctx.topic) {
    if (c != '/') {
      dotted.push_back(c);
    } else if (!dotted.empty()) {
      dotted.push_back('.');
    }
  }
  if (dotted.empty()) dotted = "topic";
  return suffix.empty() ? dotted : dotted + "." + suffix;
}

std::string SharedSeriesWarning(const TranslatorRegistry& registry, const std::string& type,
                                const std::string& first_topic, const std::string& second_topic) {
  const ITranslator* translator = registry.Find(type);
  if (translator == nullptr || first_topic == second_topic) return {};
  const std::string series = translator->fixed_series();
  if (series.empty()) return {};
  return second_topic + " and " + first_topic + " are both " + type +
         ", whose translator records every topic of that type under the same series (" + series +
         "): their samples interleave there, and anything accumulated across messages mixes the "
         "two. Allowlist only one topic of this type.";
}

bool RecordTranslatorEvent(SessionRunner& runner, const TranslationContext& ctx,
                           const std::string& name, c3d::Properties properties) {
  return RecordCappedEvent(runner, ctx.event_limiter, ctx.topic, ctx.wire_time, name,
                           ctx.body_position, ctx.position_known, std::move(properties));
}

void RegisterCoreTranslators(TranslatorRegistry& registry) {
  registry.Register(std::make_unique<BatteryTranslator>());
  registry.Register(std::make_unique<JointStateTranslator>());
  registry.Register(std::make_unique<OdometryTranslator>());
  registry.Register(std::make_unique<ImuTranslator>());
  registry.Register(std::make_unique<LaserScanTranslator>());
  // Bare std_msgs types. These carry no unit and no meaning, so they are named
  // from `topic_config.<topic>.series` and record their values unconverted.
  registry.Register(std::make_unique<Float32Translator>());
  registry.Register(std::make_unique<Int32MultiArrayTranslator>());
  registry.Register(std::make_unique<DiagnosticsTranslator>());
  registry.Register(std::make_unique<ObjectiveTracker>());
}

}  // namespace cognitive3d_ros
