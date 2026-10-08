#pragma once

#include <string>

namespace c3d {

// The earliest wall-clock time a session may begin at: 2026-01-01T00:00:00Z, before any build
// of this SDK existed, so a clock that reads earlier has not been set. A board without a
// real-time clock boots at 1970, and a session stamped from that clock is accepted by the
// platform and never appears. Only a reading before the floor is caught: a clock restored from
// the last time it saved reads later than that, and passes however stale it is.
constexpr double kWallClockFloorEpochSeconds = 1767225600.0;

// Empty when `now_epoch_s` is at or after `floor_epoch_s`; otherwise what is wrong and what to
// do, for a log line or a preflight check.
std::string WallClockProblem(double now_epoch_s,
                             double floor_epoch_s = kWallClockFloorEpochSeconds);

}  // namespace c3d
