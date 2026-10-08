#include "cognitive3d_ros/upload_health.hpp"

#include <algorithm>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace cognitive3d_ros {
namespace {

using diagnostic_msgs::msg::DiagnosticStatus;

std::string StreamNames(const std::vector<c3d::Stream>& streams) {
  std::string names;
  for (const c3d::Stream stream : streams) {
    if (!names.empty()) names += ", ";
    names += c3d::StreamName(stream);
  }
  return names;
}

}  // namespace

UploadHealth DescribeUploads(const c3d::Stats& stats, double retry_max_s) {
  UploadHealth health;
  health.level = DiagnosticStatus::OK;
  const std::string every = std::to_string(static_cast<int>(retry_max_s)) + "s";
  const std::string refused = StreamNames(stats.auth_refused_streams);
  const std::string deferred = StreamNames(stats.deferred_streams);
  std::vector<std::string> said;
  auto raise = [&health](std::uint8_t level) { health.level = std::max(health.level, level); };

  if (stats.uploads_auth_paused) {
    raise(DiagnosticStatus::ERROR);
    said.push_back("every upload is PAUSED because the credentials were refused; everything stays "
                   "spooled, and authentication is tried again every " + every);
    health.report += " [UPLOADS PAUSED: credentials refused]";
  } else if (!stats.auth_refused_streams.empty()) {
    raise(DiagnosticStatus::WARN);
    said.push_back("the credentials are refused for " + refused + ", whose chunks stay spooled "
                   "and are asked about again every " + every + "; every other stream uploads");
    health.report += " [CREDENTIALS REFUSED: " + refused + "]";
  }
  if (!stats.deferred_streams.empty()) {
    raise(DiagnosticStatus::WARN);
    said.push_back("the gateway keeps failing the chunks of " + deferred + " while it accepts the "
                   "other streams'; they stay spooled behind the rest and are tried again on their "
                   "own, from every " + every + " doubling to every hour");
    health.report += " [STREAMS DEFERRED: " + deferred + "]";
  }
  if (stats.uploads_paused) {
    raise(DiagnosticStatus::WARN);
    said.push_back("uploads are paused (uploads_paused: true): recording and spooling, sending "
                   "nothing");
    health.report += " [UPLOADS PAUSED]";
  }
  if (stats.backing_off) {
    raise(DiagnosticStatus::WARN);
    said.push_back("backing off after a failed upload; everything stays spooled");
    health.report += " [BACKING OFF]";
  }
  if (stats.chunks_held > 0) health.report += " held=" + std::to_string(stats.chunks_held);
  if (stats.chunks_deferred > 0) {
    health.report += " deferred=" + std::to_string(stats.chunks_deferred);
  }
  if (stats.chunks_evicted > 0) {
    raise(DiagnosticStatus::WARN);
    said.push_back(std::to_string(stats.chunks_evicted) + " chunk(s) evicted at spool_max_mb, "
                   "never to be uploaded");
    health.report += " EVICTED=" + std::to_string(stats.chunks_evicted);
  }
  if (said.empty()) said.push_back("uploading");
  for (std::size_t i = 0; i < said.size(); ++i) health.message += (i ? "; " : "") + said[i];

  health.values = {
      {"spool_chunks", std::to_string(stats.spool_count)},
      {"chunks_uploaded", std::to_string(stats.chunks_uploaded)},
      {"chunks_discarded", std::to_string(stats.chunks_discarded)},
      {"chunks_held", std::to_string(stats.chunks_held)},
      {"chunks_deferred", std::to_string(stats.chunks_deferred)},
      {"deferred_streams", deferred},
      {"chunks_evicted", std::to_string(stats.chunks_evicted)},
      {"upload_failures", std::to_string(stats.upload_failures)},
      {"auth_rejections", std::to_string(stats.auth_rejections)},
      {"uploads_auth_paused", stats.uploads_auth_paused ? "true" : "false"},
      {"auth_refused_streams", refused},
      {"uploads_paused", stats.uploads_paused ? "true" : "false"},
      {"backing_off", stats.backing_off ? "true" : "false"},
  };
  return health;
}

std::vector<std::pair<std::string, double>> UploadSeries(const c3d::Stats& stats) {
  return {
      {"c3d.upload_auth_rejections", static_cast<double>(stats.auth_rejections)},
      {"c3d.upload_auth_paused", stats.uploads_auth_paused ? 1.0 : 0.0},
      {"c3d.upload_auth_refused_streams", static_cast<double>(stats.auth_refused_streams.size())},
      {"c3d.chunks_held", static_cast<double>(stats.chunks_held)},
      {"c3d.chunks_deferred", static_cast<double>(stats.chunks_deferred)},
      {"c3d.upload_deferred_streams", static_cast<double>(stats.deferred_streams.size())},
      {"c3d.chunks_evicted", static_cast<double>(stats.chunks_evicted)},
      {"c3d.serialization_failures", static_cast<double>(stats.serialization_failures)},
      {"c3d.spool_write_failures", static_cast<double>(stats.spool_write_failures)},
      {"c3d.thread_exceptions", static_cast<double>(stats.thread_exceptions)},
  };
}

}  // namespace cognitive3d_ros
