#pragma once

#include <map>
#include <string>

#include <std_msgs/msg/int32_multi_array.hpp>

#include "cognitive3d_ros/translators.hpp"

namespace cognitive3d_ros {

// std_msgs/Int32MultiArray to one Cognitive3D sensor series PER ELEMENT.
//
// Element i is recorded as `<series>.<i>`, which is what makes the four bits of
// a line-follower array four independently selectable channels in the replay
// viewer rather than one number nobody can read.
//
// `layout` is deliberately IGNORED. A MultiArrayLayout can declare strides and
// multiple dimensions, and honouring it would mean inventing a naming scheme for
// n-dimensional indices that no consumer of this data has asked for. The flat
// index is the one thing that is unambiguous. Anything that needs the layout
// should be its own message type with its own translator.
//
// The element count is recorded as `<series>.count` on every sample. That is
// not padding: it is what makes the cap below visible in the DATA rather than
// only in this comment, and it is what tells an analyst that a variable-length
// array changed length mid-run.
class Int32MultiArrayTranslator : public ITranslator {
 public:
  // A bound, because nothing stops a publisher putting 100k elements here and
  // each element is a named series the front end has to enumerate.
  static constexpr std::size_t kMaxElements = 32;

  const char* type_name() const override { return "std_msgs/msg/Int32MultiArray"; }
  bool reads_series() const override { return true; }
  void Translate(const rclcpp::SerializedMessage& message, const TranslationContext& ctx,
                 SessionRunner& runner) override;
  void Reset() override;

 private:
  std::map<std::string, RateGate> gates_;
};

}  // namespace cognitive3d_ros
