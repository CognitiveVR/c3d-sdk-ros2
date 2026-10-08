#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "c3d/session.hpp"

namespace cognitive3d_ros {

// What the node says about its uploads, from core's Stats, in `~/diagnostics` and in the periodic
// report. A refused key keeps everything and sends nothing, so without this a revoked key reads as
// a run whose spool only grows.
struct UploadHealth {
  // A diagnostic_msgs/DiagnosticStatus level: the worst of the states in force.
  std::uint8_t level = 0;
  std::string message;
  // What the report line adds after its counts, each item with a leading space; empty when
  // there is nothing to flag.
  std::string report;
  std::vector<std::pair<std::string, std::string>> values;
};
UploadHealth DescribeUploads(const c3d::Stats& stats, double retry_max_s);

// The upload counters core keeps and cannot record itself, because under `sim_time_on_wire` a
// series is stamped on the wire clock, which only the node knows. Name and value, in `c3d.*`.
std::vector<std::pair<std::string, double>> UploadSeries(const c3d::Stats& stats);

}  // namespace cognitive3d_ros
