#pragma once

#include <map>
#include <string>

#include <std_msgs/msg/float32.hpp>

#include "cognitive3d_ros/translators.hpp"

namespace cognitive3d_ros {

// std_msgs/Float32 to one Cognitive3D sensor series.
//
// This translator is keyed by a BARE type. `std_msgs/Float32` is "a number":
// the type carries no unit, no frame and no meaning, so this code cannot invent
// any. It therefore does exactly two things -- it names the series after the
// topic, and it records the value UNCONVERTED. A translator that "helpfully"
// scaled centimetres to metres here would be silently wrong on every other
// Float32 topic in the world, which is the failure mode this SDK is built to
// avoid.
//
// The unit belongs in the NAME, and the name is the operator's to set:
// `topic_config.<topic>.series`. Without one the series is `<dotted topic>.value`,
// which is honest but says nothing about units.
//
// ONE instance serves every Float32 topic on the robot, because the registry
// keys translators by type. The rate gate is therefore keyed BY TOPIC -- a
// shared one would let a fast topic starve a slow one, which is the same trap
// BatteryTranslator documents for its change detection.
class Float32Translator : public ITranslator {
 public:
  const char* type_name() const override { return "std_msgs/msg/Float32"; }
  bool reads_series() const override { return true; }
  void Translate(const rclcpp::SerializedMessage& message, const TranslationContext& ctx,
                 SessionRunner& runner) override;
  void Reset() override;

 private:
  std::map<std::string, RateGate> gates_;
};

}  // namespace cognitive3d_ros
