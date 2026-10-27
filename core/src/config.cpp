#include "c3d/config.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "util.hpp"

namespace c3d {
namespace {

// A part larger than the gateway's request body cap is answered 413, which is
// not retryable, so the whole part is discarded and every frame in it is gone.
// The cap is about 49 MiB; this leaves room for the header and the packing.
constexpr std::uint64_t kMaxFlushBytes = 32ull * 1024 * 1024;

}  // namespace

std::string Config::Validate() const {
  // The mandatory-value checks report TOGETHER. Returning only the first one
  // costs a configure round trip per unset field, and a fresh integration
  // normally has several unset at once -- so an operator would learn about
  // application_key, fix it, learn about scene_id, fix it, and so on,
  // discovering one static fact about a file at a time. Everything below this
  // block is a range or consistency check, where the first failure is the
  // useful one because later checks can be meaningless once one is violated.
  std::vector<std::string> missing;
  if (application_key.empty()) missing.emplace_back("application_key is empty");
  // These two are the difference between a working integration and one that
  // silently uploads nothing, so they refuse the start rather than warn and
  // then drop every payload.
  if (scene_id.empty()) missing.emplace_back("scene_id is empty; every payload would be discarded");
  if (scene_version.empty())
    missing.emplace_back("scene_version is empty; every payload would be discarded");
  if (device_id.empty()) missing.emplace_back("device_id is empty");
  if (spool_dir.empty()) missing.emplace_back("spool_dir is empty");
  // Not cosmetic. The platform silently drops any session whose properties omit
  // c3d.app.version, so an empty value here produces a run that appears to
  // succeed and stores nothing.
  if (app_version.empty())
    missing.emplace_back("app_version is empty; the backend would silently discard the session");
  if (app_name.empty()) missing.emplace_back("app_name is empty");
  if (missing.size() == 1) return missing.front();
  if (!missing.empty()) {
    // " | ", not "; ", because several of the messages above carry their own
    // "; why it matters" clause -- joining with "; " would make three problems
    // read as five. One problem still returns its bare message, unchanged.
    std::string joined = std::to_string(missing.size()) + " configuration problems: " + missing.front();
    for (size_t i = 1; i < missing.size(); ++i) joined += " | " + missing[i];
    return joined;
  }
  // The session id embeds it, and the session id travels as JSON text on four streams and as
  // raw bytes on the binary two: an invalid sequence would reach those as two different ids.
  if (!IsValidUtf8(device_id)) return "device_id is not valid UTF-8";
  // An unsigned size above INT64_MAX is a negative number that wrapped on its way
  // in, and it would read as "unbounded" rather than be refused.
  for (const auto& [name, value] : {std::make_pair("spool_max_bytes", spool_max_bytes),
                                    std::make_pair("camera_flush_bytes", camera_flush_bytes),
                                    std::make_pair("lidar_flush_bytes", lidar_flush_bytes),
                                    std::make_pair("gaze_batch", std::uint64_t{gaze_batch}),
                                    std::make_pair("event_batch", std::uint64_t{event_batch}),
                                    std::make_pair("sensor_batch", std::uint64_t{sensor_batch}),
                                    std::make_pair("dynamic_batch", std::uint64_t{dynamic_batch}),
                                    std::make_pair("max_records_per_batch",
                                                   std::uint64_t{max_records_per_batch})}) {
    if (value > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
      return std::string(name) + " is negative; a size cannot be";
    }
  }
  if (gaze_batch == 0 || event_batch == 0 || sensor_batch == 0 || dynamic_batch == 0)
    return "batch sizes must be non-zero";
  // The drop-oldest path erases the front of a batch that is at the cap, and a cap of 0 makes
  // every batch, the empty one included, at the cap.
  if (max_records_per_batch == 0) return "max_records_per_batch must be non-zero";
  // The worker and the uploader both wait one tick per pass: 0 or less spins them, and NaN
  // reaches a timed wait whose behaviour is undefined. Above a second the heartbeat, the
  // part-cutting bounds and the uploader all run late by up to a tick.
  if (!std::isfinite(worker_tick_s) || worker_tick_s <= 0.0 || worker_tick_s > 1.0)
    return "worker_tick_s must be a number of seconds above 0 and at most 1";
  // NaN fails every comparison, so a NaN interval would never cut a part on the timer.
  // Infinity is a choice: parts are cut on the batch sizes and on Flush alone.
  if (std::isnan(auto_send_interval_s) || auto_send_interval_s <= 0.0)
    return "auto_send_interval_s must be a positive number of seconds";
  // Every gaze part advertises it. JSON has no NaN or infinity, so either would reach the
  // gateway as `null` on the part that carries the session's properties.
  if (!std::isfinite(gaze_interval_s) || gaze_interval_s <= 0.0)
    return "gaze_interval_s must be a finite, positive number of seconds";
  // The camera and lidar payloads carry the scene version as a NUMBER (UInt32),
  // because the gateway resolves it to the platform's internal version id and
  // has nothing to resolve a word with. The JSON streams put the same value in a
  // query string, where a non-numeric one is equally wrong and merely quieter --
  // so the check belongs to the config rather than to one serializer.
  std::uint32_t scene_version_number = 0;
  if (!SceneVersionNumber(scene_version, &scene_version_number))
    return "scene_version must be the version NUMBER as a string of digits (e.g. \"2\"), got '" +
           scene_version + "'";
  // Scene versions are numbered from 1, so there is nothing for the gateway to
  // resolve a 0 to and it answers 400 -- once per part, for the life of a run
  // whose params file looks filled in.
  if (scene_version_number == 0)
    return "scene_version must be 1 or greater; scene versions are numbered from 1 and the "
           "gateway rejects 0";
  // NaN fails `<= 0.0`, as it fails every comparison.
  if (std::isnan(camera_flush_s) || camera_flush_s <= 0.0)
    return "camera_flush_s must be a positive number of seconds";
  if (std::isnan(lidar_flush_s) || lidar_flush_s <= 0.0)
    return "lidar_flush_s must be a positive number of seconds";
  // A bound below one frame cuts a payload per frame, which is the shape that
  // makes the part counter and the HTTP overhead the dominant cost of the stream.
  if (camera_flush_bytes < 1024 || lidar_flush_bytes < 1024)
    return "camera_flush_bytes and lidar_flush_bytes are implausibly small";
  if (camera_flush_bytes > kMaxFlushBytes || lidar_flush_bytes > kMaxFlushBytes)
    return "camera_flush_bytes and lidar_flush_bytes cannot exceed 32 MiB; a part above the "
           "gateway's body cap is rejected whole and discarded, taking every record in it";
  if (spool_max_bytes < 1024) return "spool_max_bytes is implausibly small";
  // The block format's range, and NaN fails both comparisons.
  if (!(pose_block_interval_s >= 0.1 && pose_block_interval_s <= 10.0))
    return "pose_block_interval_s must be between 0.1 and 10 seconds";
  if (!std::isfinite(retry_min_s) || !std::isfinite(retry_max_s))
    return "retry_min_s and retry_max_s must be finite numbers";
  if (retry_min_s <= 0 || retry_max_s < retry_min_s)
    return "invalid retry window: need 0 < retry_min_s <= retry_max_s";
  // The quick allowance is bounded by construction: past it, uploads pause. A
  // negative wait is bounded and harmless, but it is not what anyone meant to
  // configure.
  if (unauthorized_attempts < 0) return "unauthorized_attempts cannot be negative";
  if (unauthorized_attempts > 8)
    return "unauthorized_attempts above 8 turns a rejected key into sustained "
           "authentication traffic; there is no uncapped mode";
  // Infinity would hold every upload until the next session, and NaN fails the check below.
  if (!std::isfinite(unauthorized_retry_s))
    return "unauthorized_retry_s must be a finite number of seconds";
  if (unauthorized_retry_s < 0.0) return "unauthorized_retry_s cannot be negative";
  // 0 is UNBOUNDED in curl, not instant, for every one of these: a zero would
  // silently remove the bound it names rather than make it strict.
  if (http_timeout_s <= 0) return "http_timeout_s must be positive; 0 means no timeout at all";
  if (http_connect_timeout_s <= 0)
    return "http_connect_timeout_s must be positive; 0 means no connect timeout at all";
  if (http_connect_timeout_s > http_timeout_s)
    return "http_connect_timeout_s cannot exceed http_timeout_s, the whole-request ceiling";
  if (http_low_speed_limit_bytes_per_s <= 0 || http_low_speed_time_s <= 0)
    return "http_low_speed_limit_bytes_per_s and http_low_speed_time_s must be positive; 0 "
           "disables stall detection and leaves a dead link to the whole-request ceiling";
  // A window at or past the ceiling can never fire: the ceiling always gets there
  // first, and the configuration reads as stall detection that does not exist.
  if (http_low_speed_time_s >= http_timeout_s)
    return "http_low_speed_time_s must be shorter than http_timeout_s, or the stall detector "
           "can never fire before the ceiling does";
  // isfinite FIRST, and explicitly. NaN fails every comparison below -- `NaN <= 0.0`
  // and `x <= NaN` are both false -- so a NaN would pass both checks and then make
  // `silent_for < orphan_stale_after_s` false for every marker, i.e. recovery would
  // adopt EVERYTHING, the exact defect these checks exist to prevent.
  // Inf is the mirror: every marker reads as live and dead ones leak forever.
  if (!std::isfinite(marker_heartbeat_s)) return "marker_heartbeat_s must be a finite number";
  if (!std::isfinite(orphan_stale_after_s)) return "orphan_stale_after_s must be a finite number";
  if (marker_heartbeat_s <= 0.0) return "marker_heartbeat_s must be positive";
  if (!std::isfinite(shutdown_drain_s) || shutdown_drain_s < 0.0 || shutdown_drain_s > 60.0)
    return "shutdown_drain_s must be between 0 and 60 seconds";
  if (!std::isfinite(wall_clock_floor_epoch_s) || wall_clock_floor_epoch_s < 0.0)
    return "wall_clock_floor_epoch_s must be a finite, non-negative epoch time";
  // A threshold at or below the heartbeat means startup recovery adopts sessions
  // that are refreshing their marker normally -- telling the platform a running
  // measurement ended abnormally, and deleting the marker that would have let it
  // recover for real. Refused rather than trusted.
  //
  // The margin must clear worker_tick_s as well, because a heartbeat becomes DUE
  // on a tick boundary: the real maximum interval between two touches is
  // marker_heartbeat_s + worker_tick_s, so `stale > heartbeat` alone accepts
  // 5.0/5.01/0.05 while a healthy session can go 5.05s without touching. That
  // configuration passes validation and adopts live sessions anyway, which is
  // precisely what Validate() is here to make impossible.
  //
  // No HTTP timeout is in this sum: the heartbeat runs on the serializing thread,
  // and uploads run on a thread of their own, so a POST however slow cannot
  // silence it.
  if (orphan_stale_after_s <= marker_heartbeat_s + worker_tick_s)
    return "orphan_stale_after_s must exceed marker_heartbeat_s + worker_tick_s, or startup "
           "recovery would adopt sessions that are still alive";
  return "";
}

}  // namespace c3d
