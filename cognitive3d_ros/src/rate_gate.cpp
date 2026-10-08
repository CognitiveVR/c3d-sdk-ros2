#include "cognitive3d_ros/rate_gate.hpp"

#include <algorithm>

namespace cognitive3d_ros {

bool RateGate::Accept(double time_seconds, double rate_hz) {
  if (!(rate_hz > 0.0)) return true;
  const double period = 1.0 / rate_hz;
  if (!primed_) {
    primed_ = true;
    next_ = time_seconds + period;
    return true;
  }
  if (time_seconds < next_) return false;
  next_ = std::max(next_ + period, time_seconds + 0.5 * period);
  return true;
}

}  // namespace cognitive3d_ros
