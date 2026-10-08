#pragma once

#include <sensor_msgs/msg/imu.hpp>

#include "cognitive3d_ros/translators.hpp"

namespace cognitive3d_ros {

// sensor_msgs/Imu to derived scalars. SENSORS ONLY: nothing here reaches the
// pose, gaze or dynamics channels. tf2 is the single source of pose, and an IMU
// feeding a second one would let two subsystems disagree about where the robot
// was.
//
// REP-145 makes each of the three estimates individually optional, and signals
// an absent one by setting the FIRST element of its covariance to -1 while the
// data fields stay at zero. A driver with no magnetometer therefore publishes a
// perfectly formed identity orientation, so taking the fields without reading
// the covariance records a robot that is permanently level rather than a robot
// whose attitude is unknown.
//
// An IMU publishes at 100 to 400 Hz, well past what the session model carries,
// so this decimates before deserializing the way the other high-rate
// translators do.
class ImuTranslator : public ITranslator {
 public:
  const char* type_name() const override { return "sensor_msgs/msg/Imu"; }
  std::string fixed_series() const override { return "imu.*"; }
  void Translate(const rclcpp::SerializedMessage& message, const TranslationContext& ctx,
                 SessionRunner& runner) override;
  void Reset() override;

 private:
  RateGate gate_;
};

}  // namespace cognitive3d_ros
