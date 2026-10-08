#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>

#include "cognitive3d_ros/translators.hpp"

namespace cognitive3d_ros {

// sensor_msgs/LaserScan to three scalars. The RANGES ARE NEVER STORED: a scan
// is 360 to 1800 floats at 5 to 40 Hz, which is the bandwidth budget of a whole
// session spent on data the platform has nowhere to put.
//
// What survives is what an analyst asks of a scan after the fact: how close the
// nearest obstacle came, how open the space was, and how much of the sweep
// returned anything at all. The last one is the interesting failure signal --
// a lidar looking at glass, an open doorway or nothing at all returns inf, and
// a valid fraction collapsing mid-run is a sensor or an environment problem
// that no other channel reports.
class LaserScanTranslator : public ITranslator {
 public:
  const char* type_name() const override { return "sensor_msgs/msg/LaserScan"; }
  std::string fixed_series() const override { return "scan.*"; }
  void Translate(const rclcpp::SerializedMessage& message, const TranslationContext& ctx,
                 SessionRunner& runner) override;
  void Reset() override;

 private:
  RateGate gate_;
};

}  // namespace cognitive3d_ros
