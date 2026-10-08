#include "c3d/wall_clock.hpp"

#include <cmath>
#include <ctime>

namespace c3d {
namespace {

std::string Iso8601(double epoch_s) {
  const std::time_t whole = static_cast<std::time_t>(std::floor(epoch_s));
  std::tm utc{};
  if (gmtime_r(&whole, &utc) == nullptr) return std::to_string(epoch_s);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &utc);
  return buf;
}

}  // namespace

std::string WallClockProblem(double now_epoch_s, double floor_epoch_s) {
  if (std::isfinite(now_epoch_s) && now_epoch_s >= floor_epoch_s) return {};
  return "the wall clock reads " + Iso8601(now_epoch_s) + ", before " + Iso8601(floor_epoch_s) +
         ", so it has not been set. Wait for time synchronisation (NTP) before beginning a "
         "session: one stamped from this clock is accepted and never appears";
}

}  // namespace c3d
