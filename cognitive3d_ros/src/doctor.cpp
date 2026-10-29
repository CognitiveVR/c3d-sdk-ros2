#include "cognitive3d_ros/doctor.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <iomanip>

#include <nlohmann/json.hpp>

#include "c3d/environment.hpp"
#include "c3d/version.hpp"
#include "c3d/wall_clock.hpp"
#include "cognitive3d_ros/media_config.hpp"
#include "cognitive3d_ros/qos_resolver.hpp"
#include <sstream>

namespace cognitive3d_ros::doctor {
namespace {

// Below this, even the last four characters are a large part of the value, so only the length is
// reported.
constexpr std::size_t kShortestMaskable = 16;


constexpr double kDegreesPerRadian = 57.295779513082320876798154814105;

// Frames whose transform is planar by construction on a standard ROS 2 mobile stack, compared
// bare so that a namespaced `robot1/odom` is recognised too.
bool PlanarByConstruction(const std::string& frame) {
  const std::size_t slash = frame.rfind('/');
  const std::string bare = slash == std::string::npos ? frame : frame.substr(slash + 1);
  return bare == "odom" || bare == "map";
}

// The distinct values, sorted and comma-joined, for a "it has: ..." remedy. DISTINCT because a
// project carries one object row per scene version, so six versions of one robot would list the
// same name six times -- a remedy that reads as broken.
std::string DistinctJoined(const std::vector<std::string>& values) {
  std::vector<std::string> distinct = values;
  std::sort(distinct.begin(), distinct.end());
  distinct.erase(std::unique(distinct.begin(), distinct.end()), distinct.end());
  std::ostringstream joined;
  for (std::size_t i = 0; i < distinct.size(); ++i) {
    joined << (i ? ", " : "") << distinct[i];
  }
  return joined.str();
}

std::string Joined(const std::vector<std::string>& values, const char* separator) {
  std::string joined;
  for (const std::string& value : values) {
    if (!joined.empty()) joined += separator;
    joined += value;
  }
  return joined;
}

// A JSON integer or non-empty string, rendered; false for anything else.
bool ScalarText(const nlohmann::json& value, std::string* text) {
  if (value.is_number_integer()) {
    *text = std::to_string(value.get<long long>());
    return true;
  }
  if (value.is_string() && !value.get<std::string>().empty()) {
    *text = value.get<std::string>();
    return true;
  }
  return false;
}

// Two version numbers or ids compared as numbers, as configure reads scene_version: "01" is 1.
// A value that is not a string of digits compares as text.
bool SameNumber(const std::string& a, const std::string& b) {
  const auto digits = [](const std::string& text) {
    return !text.empty() &&
           std::all_of(text.begin(), text.end(), [](char c) { return c >= '0' && c <= '9'; });
  };
  if (!digits(a) || !digits(b)) return a == b;
  const auto significant = [](const std::string& text) {
    const std::size_t first = text.find_first_not_of('0');
    return first == std::string::npos ? std::string("0") : text.substr(first);
  };
  return significant(a) == significant(b);
}

std::string Fixed(double value, int places) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(places) << value;
  return out.str();
}

const char* Label(Severity severity) {
  switch (severity) {
    case Severity::kOk:       return "ok  ";
    case Severity::kAdvisory: return "note";
    case Severity::kFatal:    return "FAIL";
  }
  return "????";
}


// Undo `Config::Validate`'s join. The count prefix is required to be digits immediately followed by
// the marker, so a validation message that merely contains the words cannot be truncated.
std::vector<std::string> SplitConfigurationProblems(const std::string& configure_error) {
  std::vector<std::string> problems;
  if (configure_error.empty()) return problems;

  std::string remainder = configure_error;
  const std::string marker = " configuration problems: ";
  const std::size_t marker_at = remainder.find(marker);
  if (marker_at != std::string::npos && marker_at > 0 &&
      remainder.find_first_not_of("0123456789") == marker_at) {
    remainder = remainder.substr(marker_at + marker.size());
  }

  const std::string separator = " | ";
  std::size_t from = 0;
  while (true) {
    const std::size_t at = remainder.find(separator, from);
    if (at == std::string::npos) {
      problems.push_back(remainder.substr(from));
      return problems;
    }
    problems.push_back(remainder.substr(from, at - from));
    from = at + separator.size();
  }
}

// The text between `open` and the next `close` after it, searched from the first `open`.
std::string Between(const std::string& text, const std::string& open, const std::string& close) {
  const std::size_t at = text.find(open);
  if (at == std::string::npos) return {};
  const std::size_t from = at + open.size();
  const std::size_t to = text.find(close, from);
  if (to == std::string::npos) return {};
  return text.substr(from, to - from);
}

std::string WithArticle(const std::string& type) {
  if (type.empty()) return "a value of another type";
  const bool vowel = std::string("aeiou").find(type.front()) != std::string::npos;
  return std::string(vowel ? "an " : "a ") + type;
}

}  // namespace

std::string MaskSecret(const std::string& value) {
  if (value.empty()) return "<empty>";
  if (value.size() < kShortestMaskable) {
    return "<" + std::to_string(value.size()) + " characters, too short to mask safely>";
  }
  return "ending ..." + value.substr(value.size() - 4);
}

Severity Aggregate(const std::vector<Check>& checks) {
  Severity worst = Severity::kOk;
  for (const auto& check : checks) {
    if (check.severity == Severity::kFatal) return Severity::kFatal;
    if (check.severity == Severity::kAdvisory) worst = Severity::kAdvisory;
  }
  return worst;
}

int ExitCode(const std::vector<Check>& checks) {
  // An empty list is not a pass. It means the caller gathered nothing, and reporting success for
  // it would make `doctor` a tool that says everything is fine when it has looked at nothing.
  if (checks.empty()) return 1;
  return Aggregate(checks) == Severity::kFatal ? 1 : 0;
}

Check CheckBuild(const BuildFacts& facts) {
  std::string detail = facts.package + "'s node";
  if (facts.package == "cognitive3d_ros_nav2") {
    detail += ", the one the launch file runs while that package is installed";
  } else if (facts.handoff_error.empty() && facts.index_error.empty()) {
    detail += ", the one the launch file runs while cognitive3d_ros_nav2 is not installed";
  }
  if (facts.constructed) {
    std::ostringstream types;
    for (std::size_t i = 0; i < facts.translated_types.size(); ++i) {
      types << (i ? ", " : "") << facts.translated_types[i];
    }
    detail += "; translating " + std::to_string(facts.translated_types.size()) +
              " message type(s): " + types.str();
  }
  if (!facts.index_error.empty()) {
    return {"build", Severity::kAdvisory,
            detail + ". The ament index could not be read, so which build the launch file runs "
                     "is unknown: " + facts.index_error,
            "Source the ROS 2 setup file of the install that holds cognitive3d_ros, for example "
            "/opt/ros/<distro>/setup.bash, and run doctor again. With cognitive3d_ros_nav2 "
            "installed, the launch file runs that package's node, and this report leaves out what "
            "it adds, such as nav2_msgs/msg/BehaviorTreeLog."};
  }
  if (!facts.handoff_error.empty()) {
    return {"build", Severity::kAdvisory,
            detail + ". The launch file runs cognitive3d_ros_nav2's node, and that package's "
                     "doctor did not start: " + facts.handoff_error,
            "Install cognitive3d_ros_nav2 at the same version as cognitive3d_ros. This report "
            "leaves out what the Nav2 build adds, such as nav2_msgs/msg/BehaviorTreeLog."};
  }
  return {"build", Severity::kOk, detail, ""};
}

Check CheckConstruction(const std::string& what) {
  const std::string name = Between(what, "parameter '", "'");
  if (!name.empty() && what.find("No parameter value set") != std::string::npos) {
    return {"parameters", Severity::kFatal,
            name + ": [] cannot be read -- ROS 2 cannot type an empty YAML list, so the node "
                   "cannot start",
            name == "action_names"
                ? "Delete the `action_names: []` line to keep the built-in list of actions. To "
                  "record no actions by name, set `action_discovery: false` instead."
                : "Delete the `" + name + ": []` line: this list defaults to empty, so leaving it "
                  "out is what an empty list means."};
  }
  if (!name.empty() && what.find("invalid type") != std::string::npos) {
    // Humble says "expected [string] got [integer]"; later distros say "is of type {string},
    // setting it to {integer}".
    std::string expected = Between(what, "expected [", "]");
    std::string given = Between(what, "got [", "]");
    if (expected.empty()) expected = Between(what, "is of type {", "}");
    if (given.empty()) given = Between(what, "setting it to {", "}");
    std::string remedy;
    if (expected == "string") {
      remedy = "Quote the value -- `" + name + ": \"1\"` in a params file, or `-p " + name +
               ":=\"'1'\"` on the command line, where the ROS 2 CLI reads an unquoted number as "
               "an integer.";
    } else if (expected == "bool") {
      remedy = "Write true or false, unquoted.";
    } else if (expected == "integer") {
      remedy = "Write a whole number, with no decimal point and no quotes.";
    } else if (expected.size() > 6 && expected.compare(expected.size() - 6, 6, "_array") == 0) {
      remedy = "Write a YAML list, e.g. `" + name + ": [\"a\", \"b\"]`.";
    } else {
      remedy = "Give " + WithArticle(expected) + ".";
    }
    return {"parameters", Severity::kFatal,
            name + " must be " + WithArticle(expected) + ", and the value given is " +
                WithArticle(given),
            remedy};
  }
  return {"parameters", Severity::kFatal, what,
          "The node could not be constructed from these parameters; the message above is "
          "rclcpp's own."};
}

Check CheckApplicationKey(const std::string& key) {
  if (key.empty()) {
    return {"application key", Severity::kFatal, "not set",
            "Set C3D_APPLICATION_API_KEY in the node's environment. It is the only place the SDK "
            "reads the key from: a ROS parameter is readable by anything on the graph, so there "
            "is none."};
  }
  return {"application key", Severity::kOk, "set (" + MaskSecret(key) + ")", ""};
}

SpoolLockFacts ProbeSpoolLock(const std::string& spool_dir) {
  SpoolLockFacts facts;
  if (spool_dir.empty()) return facts;
  const std::string path = spool_dir + "/.lock";
  struct stat st {};
  if (::lstat(path.c_str(), &st) != 0) return facts;
  if (S_ISLNK(st.st_mode)) {
    facts.refused = path + " is a symbolic link";
    return facts;
  }
  if (!S_ISREG(st.st_mode) || st.st_nlink != 1) {
    facts.refused = path + " is not a plain file of its own";
    return facts;
  }
  const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
  if (fd < 0) return facts;
  if (::flock(fd, LOCK_EX | LOCK_NB) == 0) {
    ::flock(fd, LOCK_UN);
  } else if (errno == EWOULDBLOCK) {
    facts.held = true;
    char buf[32] = {0};
    if (::pread(fd, buf, sizeof(buf) - 1, 0) > 0) facts.holder = buf;
    while (!facts.holder.empty() &&
           (facts.holder.back() == '\n' || facts.holder.back() == ' ')) {
      facts.holder.pop_back();
    }
  }
  ::close(fd);
  return facts;
}

Check CheckSpoolDirectory(const std::string& directory, bool writable, int orphaned_sessions,
                          const SpoolLockFacts& lock) {
  if (directory.empty()) {
    return {"spool directory", Severity::kFatal, "not set",
            "Set the `spool_dir` parameter. Without it there is nowhere to hold a payload the "
            "network could not take, so an upload failure becomes data loss instead of a retry."};
  }
  if (!writable) {
    return {"spool directory", Severity::kFatal, directory + " is not writable",
            "Create it, or point `spool_dir` somewhere this process can write. Every payload "
            "waits here before upload and after a failure."};
  }
  if (!lock.refused.empty()) {
    return {"spool directory", Severity::kFatal, lock.refused + ", which the node refuses",
            "Remove it, and give the node a spool_dir no other account can write to: the node "
            "will not write through a link it did not create."};
  }
  // Advisory: the holder is usually the node these parameters configure, already running.
  if (lock.held) {
    return {"spool directory", Severity::kAdvisory,
            directory + " is in use by " +
                (lock.holder.empty() ? std::string("another process") : "process " + lock.holder),
            "Nothing to do if that is the node this file configures. Any other node or process "
            "given this spool_dir is refused at configure, because two writers on one directory "
            "overwrite each other's chunks: give each node its own spool_dir."};
  }
  if (orphaned_sessions > 0) {
    return {"spool directory", Severity::kAdvisory,
            directory + " is writable, and holds " + std::to_string(orphaned_sessions) +
                " session(s) left open by a previous process",
            "Nothing to do -- the node closes these with an abnormalEnd reason at configure. "
            "Mentioned so an unexpected count is visible rather than silent."};
  }
  return {"spool directory", Severity::kOk, directory + " is writable", ""};
}

namespace {

Check CameraEncoderRefused(const std::string& detail) {
  return {"camera encoder", Severity::kFatal, detail,
          std::string(kFfmpegRemedy) + ". Configure refuses for the same reason."};
}

}  // namespace

std::vector<Check> ChecksForConfigurationRefusal(const std::string& configure_error) {
  std::vector<Check> checks;
  std::vector<std::string> unrecognised;
  bool recognised_any = false;
  const std::string ffmpeg_remedy = std::string(". ") + kFfmpegRemedy;

  for (const std::string& problem : SplitConfigurationProblems(configure_error)) {
    // A refused configure never reaches the gathering layer, so these are the only route to the
    // two checks' own remedies. Matched exactly: other refusals name these values too.
    if (problem.rfind("C3D_APPLICATION_API_KEY is not set", 0) == 0 ||
        problem == "application_key is empty") {
      checks.push_back(CheckApplicationKey(""));
      recognised_any = true;
    } else if (problem.rfind("pose_convention is not set", 0) == 0) {
      checks.push_back(CheckPoseConvention(""));
      recognised_any = true;
    } else if (problem == "spool_dir is empty") {
      checks.push_back(CheckSpoolDirectory("", true, 0));
      recognised_any = true;
    } else if (problem.size() > ffmpeg_remedy.size() &&
               problem.compare(problem.size() - ffmpeg_remedy.size(), ffmpeg_remedy.size(),
                               ffmpeg_remedy) == 0) {
      // The node's refusal is the problem and the remedy joined; the check carries them apart.
      checks.push_back(CameraEncoderRefused(problem.substr(0, problem.size() -
                                                                  ffmpeg_remedy.size())));
      recognised_any = true;
    } else {
      unrecognised.push_back(problem);
    }
  }

  if (!unrecognised.empty() || !recognised_any) {
    std::string detail;
    for (const std::string& problem : unrecognised) {
      if (!detail.empty()) detail += "; ";
      detail += problem;
    }
    checks.push_back({"configuration", Severity::kFatal, detail.empty() ? "rejected" : detail,
                      "The node refused to configure, so nothing downstream was checked. Fix this "
                      "first: every other check depends on a resolved configuration."});
  }
  return checks;
}

Check CheckPoseConvention(const std::string& convention) {
  if (convention.empty()) {
    return {"pose/convention", Severity::kFatal, "not set, and it has no default",
            "Set pose_convention to gltf_authored if your scene mesh -- and the robot mesh, when "
            "robot_dynamic_object is on -- came from tools/export_world_gltf.sh or any ROS -> glTF "
            "exporter mapping gltf.xyz = (ros.x, ros.z, -ros.y); to unity if the scene was "
            "authored in Unity against ROS-TCP-Connector axes; rep103 only for debugging. The two "
            "scene conventions differ by a 90 degree yaw, so the wrong one puts the trajectory "
            "off the floor plan and renders a wheeled robot's yaw as a roll -- and NOTHING "
            "downstream reports it: the platform stores poses verbatim and the gaze verifier "
            "passes under either."};
  }
  return {"pose/convention", Severity::kOk, convention, ""};
}

Check CheckSharedSeries(const std::vector<std::string>& warnings) {
  if (warnings.empty()) {
    return {"shared series", Severity::kOk, "no two recorded topics share a translator's series",
            ""};
  }
  std::string detail;
  for (const std::string& warning : warnings) detail += (detail.empty() ? "" : " ") + warning;
  return {"shared series", Severity::kAdvisory, detail,
          "Keep one topic of each of these types in `topics`. The odometry, IMU, joint-state and "
          "laser-scan translators name their series by message type, not by topic."};
}

Check CheckSensorTopic(const SensorTopicFacts& facts) {
  const SensorConfig& sensor = facts.sensor;
  Check check;
  check.name = "sensor/" + sensor.name;
  const std::string mismatch = SensorTypeMismatch(sensor, facts.type);
  if (!mismatch.empty()) {
    check.severity = Severity::kFatal;
    check.detail = mismatch;
    check.remedy = SensorTypeRemedy(sensor) + ".";
    return check;
  }
  if (facts.publishers == 0) {
    check.severity = Severity::kAdvisory;
    check.detail = sensor.topic + " has no publisher, so the " + sensor.kind +
                   " records nothing until one starts";
    check.remedy = "Start the sensor's driver, or check sensor." + sensor.name +
                   ".topic against `ros2 topic list`.";
    return check;
  }
  std::string series;
  for (const auto& name : sensor.series) series += (series.empty() ? "" : ",") + name;
  check.detail =
      sensor.kind + " on " + sensor.topic + " [" + facts.type + "], recorded as " + series;
  return check;
}

Check CheckWallClock(double now_epoch_s) {
  const std::string problem = c3d::WallClockProblem(now_epoch_s);
  if (!problem.empty()) {
    return {"wall clock", Severity::kFatal, problem,
            "Until the clock is set the node refuses every session start. A board with no "
            "real-time clock sets it from NTP once it has a network: `timedatectl` says when the "
            "system clock is synchronized."};
  }
  return {"wall clock", Severity::kOk, "set", ""};
}

Check CheckSimTime(bool use_sim_time, bool clock_publisher_present, bool sim_time_on_wire) {
  if (use_sim_time && !clock_publisher_present) {
    return {"clock", Severity::kFatal,
            "use_sim_time is true and nothing is publishing /clock",
            std::string("You have told the node it is running in simulation and no simulator is "
                        "running, so the robot you are about to record is not there. Start the "
                        "simulator first, or set use_sim_time:=false. ") +
                (sim_time_on_wire
                     ? "(sim_time_on_wire is on, so the wire advances only with /clock: until it "
                       "is published a session stays on wall clock, and no message can be "
                       "placed by its stamp, so each is placed at its arrival.)"
                     : "(sim_time_on_wire is off, so wall clock goes on the wire, but with ROS "
                       "time at zero no message can be placed by its stamp, so each is placed "
                       "at its arrival.)")};
  }
  if (!use_sim_time && clock_publisher_present) {
    return {"clock", Severity::kAdvisory,
            "/clock is being published but use_sim_time is false",
            "Probably wrong if you are running a simulator: set use_sim_time:=true so ROS time "
            "matches the sim. Wall clock goes on the wire here, since sim_time_on_wire requires "
            "use_sim_time, so telemetry lands in the right calendar year regardless -- what is "
            "affected is placement: a message stamped on the simulator's clock reads as decades "
            "old against this node's wall-clock ROS time, so it is placed at its arrival rather "
            "than at its stamp."};
  }
  if (use_sim_time) {
    return {"clock", Severity::kOk,
            sim_time_on_wire
                ? "use_sim_time is true and /clock has a publisher; sim_time_on_wire is on, so "
                  "the wire advances with ROS time from each session's wall-clock start"
                : "use_sim_time is true and /clock has a publisher; sim_time_on_wire is off, so "
                  "wall clock goes on the wire",
            ""};
  }
  return {"clock", Severity::kOk, "wall clock (use_sim_time is false, nothing publishing /clock)",
          ""};
}

Check CheckFrames(const std::string& preferred, const std::string& fallback,
                  const std::string& robot, bool preferred_resolved, bool fallback_resolved) {
  if (preferred_resolved) {
    return {"robot pose", Severity::kOk, preferred + " -> " + robot + " resolves", ""};
  }
  if (fallback.empty()) {
    return {"robot pose", Severity::kFatal,
            preferred + " -> " + robot + " does not resolve, and no fallback frame is configured",
            "Set `fallback_reference_frame` (normally `odom`) so the node can still record a "
            "timeline before localisation comes up, or fix the frame names against `ros2 run "
            "tf2_tools view_frames --wait-time 15`. WITHOUT A POSE THE SESSION HAS NO SPATIAL "
            "TIMELINE AT ALL."};
  }
  if (fallback_resolved) {
    return {"robot pose", Severity::kAdvisory,
            preferred + " -> " + robot + " does not resolve; falling back to " + fallback + " -> " +
                robot,
            "Normal before localisation converges, and the session will have a spatial timeline. "
            "But odometry drifts and never jumps, so positions will not agree with a map you "
            "upload later. If localisation IS running, it has not published a transform yet -- "
            "AMCL needs an initial pose before it broadcasts map -> odom."};
  }
  return {"robot pose", Severity::kFatal,
          "neither " + preferred + " -> " + robot + " nor " + fallback + " -> " + robot +
              " resolves",
          "Check the frame names against `ros2 run tf2_tools view_frames --wait-time 15`, and set "
          "`reference_frame` / `robot_frame` to match your robot. WITHOUT A POSE THE SESSION HAS "
          "NO SPATIAL TIMELINE AT ALL -- it will upload, appear in the dashboard, and be empty "
          "where it matters."};
}

bool FrameWaitSettled(bool preferred_resolved, bool fallback_resolved, bool preferred_configured,
                      double seconds_since_fallback_resolved, double grace_seconds) {
  if (preferred_resolved) return true;
  if (!fallback_resolved) return false;
  // An empty `reference_frame` is a deliberate fallback-only configuration, so there is nothing
  // to wait for.
  if (!preferred_configured) return true;
  return seconds_since_fallback_resolved >= grace_seconds;
}

Check CheckGazeFrame(const std::string& gaze_frame, bool resolved) {
  if (gaze_frame.empty()) {
    return {"gaze frame", Severity::kOk,
            "not configured; the gaze pose is the robot body's", ""};
  }
  if (!resolved) {
    return {"gaze frame", Severity::kFatal, gaze_frame + " does not resolve",
            "Fix the name or unset `gaze_frame`. This is the quietest misconfiguration this "
            "node has: nothing fails, the session's properties still claim the frame, and the "
            "replay viewpoint sits at the wrong height for the whole run."};
  }
  return {"gaze frame", Severity::kOk, gaze_frame + " resolves", ""};
}

Check CheckTopics(const TopicFacts& facts) {
  if (facts.allowlist.empty()) {
    // The same clauses, under the same conditions, as the node's own warning at configure. The
    // preflight and the run it is a preflight for cannot disagree about one configuration:
    // "nothing will be observed" is false in three ways at once, and sends an operator hunting a
    // subscription fault in a session recording battery, diagnostics, objectives and a camera.
    std::ostringstream still;
    if (facts.standard_types_enabled) still << ", battery and diagnostics by message type";
    if (facts.action_discovery_enabled) still << ", the action servers action_discovery enrols";
    if (facts.media_count > 0) still << ", the cameras and lidars subscribed by name";
    return {"topics", Severity::kAdvisory,
            "the allowlist is empty: this session records the pose timeline, the node's own c3d.* "
            "series" + still.str() + ", and nothing else from this robot",
            "Set the `topics` parameter for the robot's OWN sensors -- odometry, IMU, scans, joint "
            "states. None of what is recorded without it is odometry, and the c3d.* series mean a "
            "session with no sensor entry at all still shows sensor data in the dashboard."};
  }
  if (!facts.missing.empty()) {
    std::ostringstream names;
    for (std::size_t i = 0; i < facts.missing.size(); ++i) {
      names << (i ? ", " : "") << facts.missing[i];
    }
    return {"topics", Severity::kAdvisory,
            std::to_string(facts.missing.size()) + " of " + std::to_string(facts.allowlist.size()) +
                " allowlisted topic(s) are not on the graph: " + names.str(),
            "Fine if the robot is still starting up. If they never appear, the names do not match "
            "this robot and those series will be silently absent from the session -- check against "
            "`ros2 topic list`. Every missing topic is named here on purpose."};
  }
  return {"topics", Severity::kOk,
          "all " + std::to_string(facts.allowlist.size()) + " allowlisted topic(s) are present",
          ""};
}

Check CheckTopicQos(const std::vector<TopicQosFacts>& topics) {
  if (topics.empty()) {
    return {"topics/qos (NOT CHECKED)", Severity::kOk, "no allowlisted topic is on the graph", ""};
  }
  std::vector<std::string> problems;
  std::vector<std::string> fixes;
  // Topics whose publishers disagree and whose override already asks for the weaker value. They
  // are not a problem, and they are not publishers that agree either.
  std::vector<std::string> settled;
  for (const TopicQosFacts& topic : topics) {
    std::vector<PublisherQos> publishers = topic.publishers;
    std::sort(publishers.begin(), publishers.end(),
              [](const PublisherQos& a, const PublisherQos& b) { return a.node < b.node; });
    bool any_reliable = false, any_best_effort = false, any_transient = false, any_volatile = false;
    std::vector<std::string> listed;
    std::vector<std::string> unreliable_for_request;
    std::vector<std::string> volatile_for_request;
    for (const PublisherQos& publisher : publishers) {
      (publisher.reliable ? any_reliable : any_best_effort) = true;
      (publisher.transient_local ? any_transient : any_volatile) = true;
      listed.push_back(publisher.node + " " + (publisher.reliable ? "reliable" : "best_effort") +
                       "/" + (publisher.transient_local ? "transient_local" : "volatile"));
      if (topic.requests_reliable && !publisher.reliable) {
        unreliable_for_request.push_back(publisher.node);
      }
      if (topic.requests_transient_local && !publisher.transient_local) {
        volatile_for_request.push_back(publisher.node);
      }
    }
    const std::string publishers_named = " (" + Joined(listed, ", ") + ")";

    bool durability_fix = false;
    bool reliability_fix = false;
    if (any_transient && any_volatile && topic.override_durability != "volatile") {
      problems.push_back(topic.topic + ": its publishers disagree on durability" +
                         publishers_named + ", so a node that subscribes before it has "
                         "discovered a volatile one asks for transient_local and receives nothing "
                         "from it");
      durability_fix = true;
    }
    if (any_reliable && any_best_effort && topic.override_reliability != "best_effort") {
      problems.push_back(topic.topic + ": its publishers disagree on reliability" +
                         publishers_named + ", so a node that subscribes before it has "
                         "discovered a best-effort one asks for reliable and receives nothing "
                         "from it");
      reliability_fix = true;
    }
    if (!volatile_for_request.empty()) {
      problems.push_back(topic.topic + ": the subscription asks for " + topic.requested +
                         ", and nothing would arrive from " + Joined(volatile_for_request, ", ") +
                         ", which is volatile");
      durability_fix = true;
    }
    if (!unreliable_for_request.empty()) {
      problems.push_back(topic.topic + ": the subscription asks for " + topic.requested +
                         ", and nothing would arrive from " +
                         Joined(unreliable_for_request, ", ") + ", which is best_effort");
      reliability_fix = true;
    }
    if (durability_fix) {
      fixes.push_back(QosRelaxation(topic.override_key, RMW_QOS_POLICY_DURABILITY));
    }
    if (reliability_fix) {
      fixes.push_back(QosRelaxation(topic.override_key, RMW_QOS_POLICY_RELIABILITY));
    }
    if (durability_fix || reliability_fix) continue;
    std::vector<std::string> policies;
    std::vector<std::string> overrides;
    const std::string qos_key = topic.override_key.empty() ? "qos" : topic.override_key + ".qos";
    if (any_transient && any_volatile) {
      policies.push_back("durability");
      overrides.push_back(qos_key + ".durability: volatile");
    }
    if (any_reliable && any_best_effort) {
      policies.push_back("reliability");
      overrides.push_back(qos_key + ".reliability: best_effort");
    }
    if (!policies.empty()) {
      settled.push_back(topic.topic + ": its publishers disagree on " + Joined(policies, " and ") +
                        publishers_named + ", and your override" +
                        (overrides.size() > 1 ? "s (" : " (") + Joined(overrides, ", ") + ") " +
                        (overrides.size() > 1 ? "reach" : "reaches") + " every one");
    }
  }
  if (problems.empty()) {
    const std::size_t agreeing = topics.size() - settled.size();
    std::string detail = Joined(settled, "; ");
    if (agreeing > 0) {
      detail += std::string(settled.empty() ? "" : "; ") + "the publishers of " +
                (settled.empty() ? "" : "the other ") + std::to_string(agreeing) +
                " allowlisted topic(s) agree on reliability and durability, and the subscription "
                "the node would request reaches every one";
    }
    return {"topics/qos", Severity::kOk, detail, ""};
  }
  fixes.erase(std::remove(fixes.begin(), fixes.end(), std::string()), fixes.end());
  return {"topics/qos", Severity::kAdvisory, Joined(problems, "; "),
          Joined(fixes, " ") + (fixes.empty() ? "" : " ") +
              "Set it even when the node connects today: which publisher it discovers first can "
              "change at the next start."};
}

Check CheckActionEnrolment(const ActionEnrolmentFacts& facts) {
  Check check;
  check.name = "actions";

  std::ostringstream standard;
  for (std::size_t i = 0; i < facts.standard_topics.size(); ++i) {
    standard << (i ? ", " : "") << facts.standard_topics[i];
  }
  // With the switch off there is nothing to list, and an empty list is not the
  // same answer as "nothing is there": a robot whose battery topic is absent from
  // `topics` records no power events at all, and the switch is the only thing
  // that would have reached it.
  const std::string standard_note =
      !facts.standard_types_enabled
          ? "; auto_subscribe_standard_types is OFF, so battery and diagnostics are "
            "recorded only where `topics` names them"
      : facts.standard_topics.empty()
          ? std::string()
          : "; taken on type alone: " + standard.str();

  if (facts.actions.empty()) {
    check.detail = facts.discovery_enabled
                       ? "no action servers on the graph" + standard_note
                       : "action_discovery is off, and no action servers are on the graph" +
                             standard_note;
    return check;
  }

  std::ostringstream recorded;
  std::ostringstream skipped;
  std::ostringstream replan;
  std::size_t n_recorded = 0;
  std::size_t n_skipped = 0;
  std::size_t n_replan = 0;
  for (const ActionTopicVerdict& action : facts.actions) {
    if (action.enrolled) {
      recorded << (n_recorded++ ? ", " : "") << action.topic;
    } else if (action.replan_action) {
      replan << (n_replan++ ? ", " : "") << action.topic;
    } else {
      skipped << (n_skipped++ ? ", " : "") << action.topic << " (" << action.reason << ")";
    }
  }

  std::ostringstream detail;
  detail << n_recorded << " of " << facts.actions.size() << " action server(s) will be recorded";
  if (n_recorded > 0) detail << ": " << recorded.str();
  if (n_skipped > 0) detail << "; not recorded: " << skipped.str();
  if (n_replan > 0) {
    detail << "; left out by design: " << replan.str() << " (Nav2 runs these on every replan)";
  }
  detail << standard_note;
  check.detail = detail.str();

  // Advisory, not fatal, and worded as a fact rather than a fault: a robot runs actions nobody
  // wants in the session, and a preflight that failed on them would be muted within a day.
  if (n_skipped > 0) {
    check.severity = Severity::kAdvisory;
    check.remedy =
        "An action that is not recorded emits no events: its goals and outcomes are not in the "
        "session, and nothing during the run says so. To record one, add its name to "
        "`action_names` (matched by name, so one entry covers every namespace; setting the list "
        "replaces the built-in one) or its topic to `topics`. Leave it out if you do not want it "
        "recorded.";
    if (n_replan > 0) {
      check.remedy += " The Nav2 actions left out by design need nothing unless you want them.";
    }
  } else if (n_replan > 0) {
    check.severity = Severity::kAdvisory;
    check.remedy =
        "Nothing to do. These Nav2 actions are left out of the built-in list because each replan "
        "starts or ends a goal on them. To record one anyway, add its name to `action_names`; "
        "setting the list replaces the built-in one.";
  }
  return check;
}

Check CheckGateway(const std::string& protocol, const std::string& gateway, long status,
                   bool saw_request_time_header, const std::string& transport_error) {
  const std::string destination = protocol + "://" + gateway;
  if (status == 0) {
    return {"gateway", Severity::kFatal, destination + " unreachable: " + transport_error,
            "Check the host, the port and your network. Nothing can be delivered until this "
            "resolves, though the spool will hold payloads and retry rather than dropping them."};
  }
  if (status == 401 || status == 403) {
    return {"gateway", Severity::kFatal,
            destination + " answered " + std::to_string(status) + " to an unauthenticated probe",
            "Something in front of the collector requires credentials to talk to it at all -- a "
            "proxy or a gateway, since the probe sends none. Telemetry will not get out until "
            "that is resolved."};
  }
  if (status == 200 && !saw_request_time_header) {
    return {"gateway", Severity::kFatal,
            destination + " answered 200 but sent no cvr-request-time header",
            "Something is answering on the collector's behalf -- a captive portal, or an "
            "intercepting proxy. A 200 from it is not a receipt, and payloads will be accepted "
            "and lost. Get onto a network that reaches the host directly."};
  }
  if (status == 429 || (status >= 500 && status < 600)) {
    return {"gateway", Severity::kAdvisory,
            destination + " answered " + std::to_string(status),
            "The collector or something in front of it is refusing work right now. Payloads will "
            "spool and retry rather than being lost, so a session is still worth opening -- but "
            "expect nothing to appear until it recovers."};
  }
  // Anything else, including a 4xx on a deliberately empty probe, means the host is there and
  // speaking HTTP. That is all this check can honestly claim: the collector answers 200 to a
  // payload it is about to discard, and 200 for a scene id that does not exist, so a status code
  // from it is never a receipt. A 404 is the collector's own answer to this probe, so the line
  // says so rather than leave it to read as an error.
  return {"gateway", Severity::kOk,
          destination + " reachable (HTTP " + std::to_string(status) +
              (status == 404 ? ", the expected answer to this probe" : "") + ")",
          ""};
}

std::string OrganizationAuthorization(const std::string& organization_key) {
  if (organization_key.rfind("orgkey-", 0) == 0) return organization_key;
  return "APIKEY:ORGANIZATION " + organization_key;
}

ApiDestination ApiDestinationFor(const c3d::Config& config, const std::string& api_host) {
  if (api_host.empty()) return {};
  // `protocol` does not guard the DESTINATION -- the process environment decides that, and
  // nothing a params file can say reaches it. What it still says is whether this run is a loopback
  // capture, and a run whose telemetry deliberately is not going to Cognitive3D must not reach out
  // to Cognitive3D with a key that unlocks every project in the organisation.
  return {api_host, config.protocol == "https"};
}

std::string SceneLookupBase(const c3d::Config& config, const std::string& api_host) {
  const ApiDestination destination = ApiDestinationFor(config, api_host);
  return destination.permitted ? "https://" + destination.host : std::string();
}

Check CheckEnvironment(const std::string& description) {
  return {"environment", Severity::kOk, description, ""};
}

Check CheckProjectKey(const ProjectKeyFacts& facts) {
  // A check that did not run is named so that it cannot be skim-read as one that passed.
  //
  // NOT APPLICABLE is kOk; UNVERIFIED is advisory. The two are different facts and only one of
  // them is fine: nobody asking for the check is fine, whereas asking for it and not getting an
  // answer is the state this check exists to make visible. A severity-only consumer -- an exit
  // code, a CI gate -- never sees the name, so kOk on an unanswered check reads as "verified".
  const std::string skipped = "key/project (NOT CHECKED)";
  if (facts.expected_project.empty()) {
    return {skipped, Severity::kOk, "set C3D_PROJECT_ID to the project you expect", ""};
  }
  const std::string unverified = "the application key was not checked against project " +
                                 facts.expected_project;
  if (!facts.org_key_present) {
    return {skipped, Severity::kAdvisory, unverified,
            "Set C3D_ORG_API_KEY to check it; a developer key is refused by this route. An "
            "application key from another project ingests COMPLETELY into that project, with no "
            "error and all four data types present."};
  }
  if (!facts.destination_permitted) {
    return {skipped, Severity::kAdvisory,
            unverified + ": `protocol` is not https, so no organisation key was sent anywhere",
            "Deliberate. An organisation key unlocks every project in the org, and a run pointed "
            "at a loopback capture is not a run that should be reaching Cognitive3D with one. Set "
            "`protocol: https` when you want this checked -- the destination itself comes from "
            "the process environment (C3D_ENVIRONMENT, or C3D_DATA_HOST and C3D_API_HOST) and "
            "`gateway` cannot move it."};
  }
  // Below kShortestMaskable a key cannot be told apart from an ordinary JSON token, so a
  // containment test on the response would answer about the wrong thing.
  if (facts.application_key.size() < kShortestMaskable) {
    return {skipped, Severity::kAdvisory,
            unverified + ": it is too short to identify in the response safely",
            "Almost certainly a truncated or placeholder value in C3D_APPLICATION_API_KEY -- a "
            "real application key is longer than this. Check the value before running."};
  }

  const std::string name = "key belongs to project";
  if (facts.status == 0) {
    return {name, Severity::kAdvisory, "could not reach the API to check: " + facts.transport_error,
            "Not a verdict on the key -- the check simply did not run. Re-run when the API is "
            "reachable, or proceed knowing this one was skipped."};
  }
  if (facts.status == 401 || facts.status == 403) {
    return {name, Severity::kAdvisory,
            "the API refused the organisation key (HTTP " + std::to_string(facts.status) + ")",
            "Check C3D_ORG_API_KEY. This says nothing about the application key -- the check did "
            "not run."};
  }
  if (facts.status != 200 || !facts.body_is_a_key_list) {
    return {name, Severity::kAdvisory,
            "the API answered HTTP " + std::to_string(facts.status) +
                " without a key list, so the check did not run",
            "Re-run later. This is NOT a verdict on the application key: an absent or unreadable "
            "list is not the same as a key that is missing from one."};
  }
  if (!facts.key_listed) {
    return {name, Severity::kFatal,
            "the application key is NOT one of project " + facts.expected_project + "'s keys",
            "This session would ingest into whichever project this key DOES belong to, with no "
            "error and all four data types present, and the platform would auto-create a scene "
            "there for your scene id. Use the key from project " + facts.expected_project +
                ", or correct C3D_PROJECT_ID if you meant a different project."};
  }
  return {name, Severity::kOk,
          "yes -- the application key is a key of project " + facts.expected_project,
          ""};
}

bool OrganizationKeyProven(const ProjectKeyFacts& facts) {
  return facts.status == 200 && facts.body_is_a_key_list;
}

bool SceneDocumentIn(const std::string& body, std::string* project,
                     std::vector<SceneVersion>* versions) {
  project->clear();
  versions->clear();
  const nlohmann::json parsed = nlohmann::json::parse(body, nullptr, /*allow_exceptions=*/false);
  if (parsed.is_discarded() || !parsed.is_object()) return false;
  const auto project_id = parsed.find("projectId");
  if (project_id == parsed.end() || !ScalarText(*project_id, project)) return false;
  const auto listed = parsed.find("versions");
  if (listed == parsed.end() || !listed->is_array()) return false;
  for (const auto& entry : *listed) {
    SceneVersion version;
    if (!entry.is_object()) return false;
    const auto number = entry.find("versionNumber");
    if (number == entry.end() || !ScalarText(*number, &version.number)) return false;
    const auto id = entry.find("id");
    if (id != entry.end()) ScalarText(*id, &version.id);
    versions->push_back(version);
  }
  return true;
}

Check CheckSceneProject(const SceneProjectFacts& facts) {
  const char* kName = "scene/project";
  const char* kSkipped = "scene/project (NOT CHECKED)";
  if (!facts.org_key_present) {
    return {kSkipped, Severity::kOk,
            "set C3D_ORG_API_KEY to check that the scene is in your project and has version " +
                facts.scene_version,
            ""};
  }
  if (!facts.destination_permitted) {
    return {kSkipped, Severity::kAdvisory,
            "`protocol` is not https, so no organisation key was sent anywhere",
            "Deliberate. An organisation key unlocks every project in the org, and a run pointed "
            "at a loopback capture is not a run that should be reaching Cognitive3D with one. Set "
            "`protocol: https` when you want this checked -- the destination itself comes from "
            "the process environment (C3D_ENVIRONMENT, or C3D_DATA_HOST and C3D_API_HOST) and "
            "`gateway` cannot move it."};
  }
  if (facts.status == 0) {
    return {kName, Severity::kAdvisory, "could not reach the API: " + facts.transport_error,
            "Unverified rather than wrong. The run can proceed; if the scene is in another project "
            "or lacks this version, nothing at run time will say so."};
  }
  // The API answers 403 for a scene id it does not show this organisation key, whether the id
  // does not exist or belongs to another organisation; 404 means the same.
  if (facts.status == 403 || facts.status == 404) {
    const std::string answered = " (HTTP " + std::to_string(facts.status) + ")";
    const std::string empty_scene =
        "The platform accepts a session for a scene id it does not have, and records it against "
        "an empty scene it creates, so Session Replay draws the robot in an empty world.";
    if (facts.org_key_verified) {
      return {kName, Severity::kFatal,
              "no scene " + facts.scene_id + " is visible to the organisation that owns project " +
                  facts.expected_project + answered,
              "The scene id does not exist, or belongs to another organisation: key/project has "
              "just proved the organisation key, so this is not a credential fault. Set scene_id "
              "to the id of your scene on the Cognitive3D dashboard. " + empty_scene};
    }
    return {kName, Severity::kAdvisory,
            "no scene " + facts.scene_id + " is visible to this organisation key" + answered,
            "Either the scene id does not exist or belongs to another organisation, or "
            "C3D_ORG_API_KEY is not your organisation's key. Set C3D_PROJECT_ID to your "
            "application key's project: key/project then proves the organisation key, and this "
            "becomes a verdict. " + empty_scene};
  }
  if (facts.status == 401) {
    return {kName, Severity::kAdvisory, "the API answered 401 to the scene lookup",
            "This route needs an ORGANISATION key; a developer key is refused. Check "
            "C3D_ORG_API_KEY."};
  }
  if (facts.status != 200 || !facts.body_is_a_scene) {
    return {kName, Severity::kAdvisory,
            "the scene lookup answered " + std::to_string(facts.status) +
                " without a readable scene",
            "Unverified rather than wrong -- re-run, or look at the scene on the Cognitive3D "
            "dashboard before a long run."};
  }

  std::vector<std::string> wrong;
  std::vector<std::string> fixes;
  if (!facts.expected_project.empty() && facts.scene_project != facts.expected_project) {
    wrong.push_back("scene " + facts.scene_id + " belongs to project " + facts.scene_project +
                    ", not to project " + facts.expected_project + " (C3D_PROJECT_ID)");
    fixes.push_back(
        "The platform does not refuse a session for a scene in another project: it silently "
        "creates an empty shadow scene with this id in your project, and the session records "
        "against it, so Session Replay draws the robot in an empty world. Upload the scene to "
        "project " + facts.expected_project + ", set scene_id to a scene of that project, or "
        "correct C3D_PROJECT_ID if it names the wrong project. The Upload Web App files a scene "
        "under the project of the developer key you enter.");
  }
  const SceneVersion* configured = nullptr;
  const SceneVersion* by_id = nullptr;
  std::vector<std::string> numbers;
  for (const SceneVersion& version : facts.versions) {
    numbers.push_back(version.number);
    if (SameNumber(version.number, facts.scene_version)) configured = &version;
    if (!version.id.empty() && SameNumber(version.id, facts.scene_version)) by_id = &version;
  }
  if (configured == nullptr && by_id != nullptr) {
    wrong.push_back("scene_version \"" + facts.scene_version + "\" is the version id of version " +
                    by_id->number + ", not a version number");
    fixes.push_back("Set scene_version: \"" + by_id->number +
                    "\". It takes the version NUMBER, versions[].versionNumber, not the numeric "
                    "version id beside it.");
  } else if (configured == nullptr) {
    wrong.push_back("scene " + facts.scene_id + " has no version numbered " + facts.scene_version +
                    (numbers.empty() ? "; it lists none"
                                     : "; its versions are " + Joined(numbers, ", ")));
    fixes.push_back("Set scene_version to one of them, quoted. A session for a version the "
                    "scene does not have is accepted, and nothing at run time reports it.");
  }
  if (!wrong.empty()) {
    return {kName, Severity::kFatal, Joined(wrong, "; "), Joined(fixes, " ")};
  }

  const std::string found = "scene " + facts.scene_id + " has version " + facts.scene_version +
                            (configured->id.empty() ? "" : " (version id " + configured->id + ")") +
                            " and is in project " + facts.scene_project;
  if (facts.expected_project.empty()) {
    return {kName, Severity::kAdvisory,
            found + "; C3D_PROJECT_ID is not set, so the project was not compared with yours",
            "Set C3D_PROJECT_ID to your application key's project to check it. A session for a "
            "scene in another project is accepted, and records against an empty scene the "
            "platform creates in yours."};
  }
  return {kName, Severity::kOk, found, ""};
}

bool StringFieldIn(const std::string& body, const std::string& field,
                   std::vector<std::string>* values) {
  // Non-throwing parse: a malformed body is an unreadable ANSWER, not an exception for every
  // call site to handle. A substring scan cannot serve here -- it cannot distinguish an error
  // object that merely mentions `[` from an actual list, and it keeps a JSON escape verbatim
  // where a parser resolves it, so two spellings of one mesh name compare unequal.
  const nlohmann::json parsed = nlohmann::json::parse(body, nullptr, /*allow_exceptions=*/false);
  if (parsed.is_discarded() || !parsed.is_array()) return false;
  for (const auto& entry : parsed) {
    if (!entry.is_object()) continue;
    const auto found = entry.find(field);
    if (found != entry.end() && found->is_string()) {
      values->push_back(found->get<std::string>());
    }
  }
  return true;
}

bool VersionIdForNumber(const std::string& scene_body, const std::string& version_number,
                        std::string* version_id) {
  version_id->clear();
  const nlohmann::json parsed =
      nlohmann::json::parse(scene_body, nullptr, /*allow_exceptions=*/false);
  if (parsed.is_discarded() || !parsed.is_object()) return false;
  const auto versions = parsed.find("versions");
  if (versions == parsed.end() || !versions->is_array()) return false;
  for (const auto& version : *versions) {
    if (!version.is_object()) return false;
    const auto number = version.find("versionNumber");
    if (number == version.end()) return false;
    // The version number arrives as a JSON number or string and is configured as a string of
    // digits, so the two are compared as numbers.
    std::string rendered;
    if (!ScalarText(*number, &rendered)) return false;
    if (!SameNumber(rendered, version_number)) continue;
    const auto id = version.find("id");
    if (id == version.end()) return false;
    if (id->is_number_integer()) {
      std::ostringstream out;
      out << id->get<long long>();
      *version_id = out.str();
      return true;
    }
    if (id->is_string() && !id->get<std::string>().empty()) {
      *version_id = id->get<std::string>();
      return true;
    }
    // The version is listed and its id is unusable. That proves nothing either way.
    return false;
  }
  return true;
}

bool MeshFilesIn(const std::string& body, const std::string& mesh_name, bool* found,
                 std::vector<std::string>* files) {
  *found = false;
  const nlohmann::json parsed = nlohmann::json::parse(body, nullptr, /*allow_exceptions=*/false);
  if (parsed.is_discarded() || !parsed.is_array()) return false;
  for (const auto& entry : parsed) {
    // A row that cannot be read is a row that might have been the one, so absence stops being
    // provable. Skipping it instead would let a malformed row become a FATAL "not on this
    // version" for a mesh that is on it.
    if (!entry.is_object()) return false;
    const auto mesh = entry.find("meshName");
    if (mesh == entry.end() || !mesh->is_string()) return false;
    if (mesh->get<std::string>() != mesh_name) continue;
    const auto listed = entry.find("files");
    if (listed == entry.end() || !listed->is_array()) return false;
    for (const auto& file : *listed) {
      if (!file.is_object()) return false;
      const auto name = file.find("name");
      if (name == file.end() || !name->is_string()) return false;
      files->push_back(name->get<std::string>());
    }
    *found = true;
  }
  return true;
}

Check CheckRobotMeshFiles(const RobotMeshFilesFacts& facts) {
  const char* kName = "mesh/files";
  const std::string skipped = std::string(kName) + " (NOT CHECKED)";
  if (facts.robot_mesh.empty()) {
    return {skipped, Severity::kOk, "no robot_mesh is configured", ""};
  }
  if (facts.scene_id.empty() || facts.scene_version.empty()) {
    return {skipped, Severity::kAdvisory,
            "the scene id and version are needed to ask a version-scoped question", ""};
  }
  if (!facts.org_key_present) {
    return {skipped, Severity::kAdvisory, "set C3D_ORG_API_KEY to check it",
            "This route needs an ORGANISATION key; a developer key is refused. Without it the "
            "only available question is the project-scoped one, which cannot see whether the "
            "mesh FILES were ever stored."};
  }
  if (!facts.destination_permitted) {
    return {skipped, Severity::kAdvisory,
            "`protocol` is not https, so no organisation key was sent anywhere", ""};
  }
  if (facts.scene_status != 200) {
    return {skipped, Severity::kAdvisory,
            "the scene lookup answered " + std::to_string(facts.scene_status) +
                (facts.transport_error.empty() ? "" : " (" + facts.transport_error + ")"),
            "Unverified rather than wrong. The numeric version id has to come from the scene "
            "before the object listing can be asked for."};
  }
  if (!facts.scene_body_is_readable) {
    return {skipped, Severity::kAdvisory, "the scene document could not be read",
            "Unverified rather than wrong. The numeric version id has to be resolved out of the "
            "scene before the object listing can be asked for, and this body did not yield one."};
  }
  if (facts.version_id.empty() && !facts.scene_read_by_scene_project) {
    // Readable here and not as a scene there, so nothing else reports the missing version.
    return {kName, Severity::kFatal, "the scene has no version numbered " + facts.scene_version,
            "The run records against a version that does not exist, so the replay cannot have a "
            "robot in it whatever you upload. Set scene_version to a version number the scene "
            "lists on the Cognitive3D dashboard: the version NUMBER, not the numeric version id."};
  }
  if (facts.version_id.empty()) {
    // Readable, and the version is not in it. scene/project reads the same scene document and
    // fails on exactly this, naming the version to use, so here it is only the reason there is no
    // version to look the mesh up on.
    return {skipped, Severity::kAdvisory,
            "the scene has no version numbered " + facts.scene_version +
                ", so there is no version to look the mesh up on",
            "Fix scene_version first, as scene/project says; this check runs once it names a "
            "version the scene has."};
  }
  if (facts.status != 200 || !facts.body_is_an_object_list) {
    return {skipped, Severity::kAdvisory,
            "the object listing answered " + std::to_string(facts.status) +
                " without a readable list of objects",
            "Unverified rather than wrong -- treat the mesh as unconfirmed and look at the "
            "dashboard before a long run."};
  }
  if (!facts.mesh_listed) {
    return {kName, Severity::kFatal,
            "'" + facts.robot_mesh + "' is not an object of scene version " + facts.scene_version +
                " (version id " + facts.version_id + ")",
            "This is the version-scoped question, so it is the one that decides the replay. A "
            "mesh present elsewhere in the project does not help: dynamic objects are per scene "
            "VERSION, and a freshly uploaded scene version starts with none. Run call 1, the "
            "object creation, against THIS version."};
  }
  if (facts.files.empty()) {
    return {kName, Severity::kFatal,
            "'" + facts.robot_mesh + "' is registered on scene version " + facts.scene_version +
                " but has NO FILES",
            "Call 1 ran and call 2 did not. Registering the object and storing its mesh are two "
            "separate requests, and the second answers 200 for a post that stores nothing -- most "
            "often because its path segment took the object id where it wants the MESH NAME: "
            "`POST {data}/v0/objects/<scene_id>/" + facts.robot_mesh +
                "?version=" + facts.scene_version +
                "` with one `file` part per file. The replay will have NO ROBOT IN IT until this "
                "row has files, which is why it is fatal rather than a warning."};
  }
  // The viewer addresses an object's asset as `objects/{mesh}/{mesh}.gltf`, so that exact name is
  // the one file whose presence decides whether anything renders. Counting files cannot stand in
  // for it: a row carrying only its `.bin` is a row with files and no mesh.
  const std::string required = facts.robot_mesh + ".gltf";
  const bool has_required =
      std::any_of(facts.files.begin(), facts.files.end(),
                  [&required](const std::string& name) { return name == required; });
  if (!has_required) {
    return {kName, Severity::kFatal,
            "'" + facts.robot_mesh + "' has files on scene version " + facts.scene_version +
                " but not '" + required + "': " + DistinctJoined(facts.files),
            "The viewer addresses the asset as `objects/" + facts.robot_mesh + "/" + required +
                "`, so that name is the only one that resolves -- a `.bin` alone, or a glTF under "
                "any other basename, renders nothing. Re-export with the glTF named after the "
                "mesh and its buffer referenced by bare filename. Note the platform will not "
                "replace files already stored under a mesh name, so iterate on a NEW name and "
                "move `robot_mesh` with it."};
  }
  return {kName, Severity::kOk,
          "'" + facts.robot_mesh + "' is on scene version " + facts.scene_version + " with " +
              std::to_string(facts.files.size()) + " file(s): " + DistinctJoined(facts.files),
          ""};
}

Check CheckRobotMesh(const RobotMeshFacts& facts) {
  // Every decline path carries a DIFFERENT NAME. "not checked" rendered under the same name as a
  // pass is a check that reads as a verdict when it is a shrug.
  const char* kSkipped = "mesh/exists (NOT CHECKED)";

  // NOT APPLICABLE is kOk; UNVERIFIED is advisory -- the same rule CheckProjectKey follows.
  // A rejected org key and an absent one are both "I could not verify this", so they must
  // not carry different severities, and a severity-only consumer never sees the name.
  // Nothing to check is genuinely fine.
  if (facts.robot_mesh.empty()) {
    return {kSkipped, Severity::kOk, "no robot_mesh is configured, so there is nothing to look for",
            ""};
  }
  // A mesh IS named and could not be confirmed. That is a warning, not a pass.
  const std::string unverified =
      "'" + facts.robot_mesh + "' is named but was not checked against the platform";
  if (facts.project.empty()) {
    return {kSkipped, Severity::kAdvisory, unverified,
            "Set C3D_PROJECT_ID (and C3D_ORG_API_KEY) to check it. Without that, a mesh that was "
            "never uploaded produces a session that records perfectly and a replay with no robot "
            "in it."};
  }
  if (!facts.org_key_present) {
    return {kSkipped, Severity::kAdvisory, unverified,
            "Set C3D_ORG_API_KEY to check it. Reading the object list needs an organisation key "
            "with project-level access; the application key cannot do it."};
  }
  if (!facts.destination_permitted) {
    return {kSkipped, Severity::kAdvisory,
            unverified + ": `protocol` is not https, so no organisation key was sent anywhere",
            "Deliberate. An organisation key unlocks every project in the org, and a run pointed "
            "at a loopback capture is not a run that should be reaching Cognitive3D with one. Set "
            "`protocol: https` when you want this checked -- the destination itself comes from "
            "the process environment (C3D_ENVIRONMENT, or C3D_DATA_HOST and C3D_API_HOST) and "
            "`gateway` cannot move it."};
  }
  if (facts.status == 0) {
    return {"mesh/exists", Severity::kAdvisory, "could not reach the API: " + facts.transport_error,
            "Unverified rather than wrong. The run can proceed; if the mesh is absent the replay "
            "will have no robot in it."};
  }
  if (facts.status == 401 || facts.status == 403) {
    return {"mesh/exists", Severity::kAdvisory,
            "the API answered " + std::to_string(facts.status) + " to the object listing",
            "This route needs an ORGANISATION key with project-level access; a developer key is "
            "refused. Check C3D_ORG_API_KEY."};
  }
  if (facts.status != 200 || !facts.body_is_an_object_list) {
    return {"mesh/exists", Severity::kAdvisory,
            "the object listing answered " + std::to_string(facts.status) +
                " without a readable list of objects",
            "Unverified rather than wrong -- treat the mesh as unconfirmed and look at the "
            "dashboard before a long run."};
  }
  if (!facts.mesh_listed) {
    return {"mesh/exists", Severity::kFatal,
            "'" + facts.robot_mesh + "' is NOT one of project " + facts.project + "'s objects" +
                (facts.available.empty() ? " (it has none at all)"
                                         : "; it has: " + DistinctJoined(facts.available)),
            "The session will record correctly and the replay will have NO ROBOT IN IT, which is "
            "why this is fatal rather than a warning. Upload the mesh at upload.cognitive3d.com "
            "with your DEVELOPER key, or set `robot_mesh` to a name the project already has. Note "
            "that upload answers 200 for a format it discards -- it accepts a .gltf with its .bin "
            "and drops a .glb -- so re-run this check rather than trusting the upload's status. "
            "A scene in another project than your key also produces this, because this listing "
            "is project-scoped: scene/project above says whether that is the cause."};
  }
  // PROJECT scope, and that is weaker than the question you actually have. Dynamic objects are
  // per scene VERSION -- a project carries one row per version -- so a mesh present on some other
  // version of this scene passes here while the version this run targets has no robot in it.
  // A FAIL is scope-bound the other way: this listing cannot distinguish a missing mesh from a
  // scene that belongs to a different project than the key.
  // Saying so on the PASS is the point: a check that overstates what it verified is worse than
  // one that admits its scope, and this is the same false-OK shape the rest of this tool exists
  // to close. Narrowing it needs the numeric version id, which the node does not hold.
  return {"mesh/exists", Severity::kOk,
          "'" + facts.robot_mesh + "' is an object of project " + facts.project +
              " (project scope: not confirmed against scene version " + facts.scene_version + ")",
          ""};
}

Check CheckRobotObjectId(const RobotObjectIdFacts& facts) {
  const char* kName = "object id/registered";
  const char* kSkipped = "object id/registered (NOT CHECKED)";
  if (facts.robot_object_id.empty()) {
    return {kSkipped, Severity::kOk, "no robot_object_id is configured", ""};
  }
  const std::string unverified = "'" + facts.robot_object_id +
                                 "' is the id this run will emit and was not checked against the "
                                 "platform";
  if (facts.project.empty() || !facts.org_key_present || !facts.destination_permitted) {
    return {kSkipped, Severity::kAdvisory, unverified,
            "Set C3D_PROJECT_ID and C3D_ORG_API_KEY, with `protocol: https`, to check it."};
  }
  if (facts.status == 0) {
    return {kName, Severity::kAdvisory, "could not reach the API: " + facts.transport_error, unverified};
  }
  if (facts.status != 200 || !facts.body_is_an_object_list) {
    return {kName, Severity::kAdvisory,
            "the API answered HTTP " + std::to_string(facts.status) + " without an object list",
            unverified};
  }
  if (!facts.id_listed) {
    // ADVISORY, not fatal, and the distinction is the whole point: the mesh still renders.
    return {kName, Severity::kAdvisory,
            "'" + facts.robot_object_id + "' is NOT a registered object id of project " +
                facts.project +
                (facts.available.empty() ? " (no object of this project carries an sdkId)"
                                         : "; it has: " + DistinctJoined(facts.available)),
            "The replay will still show the robot -- the viewer reads the mesh name from the "
            "runtime manifest and fabricates an entry for an id it does not know -- so this will "
            "NOT look like a missing mesh. What is lost is the uploaded row: initial position, "
            "rotation, custom scale, thumbnail and group never apply, and aggregate gaze, which "
            "keys on the uploaded id, reports zero against a full timeline. The usual cause is "
            "that the upload chose the id and the run did not: the upload web app defaults to a "
            "random uuid, and a scripted uploader often derives one from the mesh name. Either "
            "set `robot_object_id` to the id listed above, or re-register the object under this "
            "one. They are one contract with a manual step in the middle."};
  }
  return {kName, Severity::kOk,
          "'" + facts.robot_object_id + "' is a registered object id of project " + facts.project +
              " (project scope: not confirmed against a scene version)",
          ""};
}

std::vector<Check> RobotObjectChecks(bool robot_dynamic_object, const RobotMeshFacts& mesh,
                                     const RobotObjectIdFacts& object_id,
                                     const RobotMeshFilesFacts& files) {
  if (!robot_dynamic_object) {
    return {{"robot dynamic object (NOT CHECKED)", Severity::kOk,
             "robot_dynamic_object is off, so this run registers no dynamic object and sends no "
             "dynamics samples: the robot is drawn from the pose timeline and no mesh has to be "
             "uploaded. mesh/exists, object id/registered and mesh/files are skipped",
             ""}};
  }
  return {CheckRobotMesh(mesh), CheckRobotObjectId(object_id), CheckRobotMeshFiles(files)};
}

double TiltDegrees(double qx, double qy, double qz, double qw) {
  const double norm_squared = qx * qx + qy * qy + qz * qz + qw * qw;
  if (norm_squared <= 0.0) return 0.0;
  // The z component of the rotated +Z axis, i.e. R(2,2), written so a non-unit quaternion
  // normalises rather than reporting a tilt it does not have.
  const double cos_tilt = (qw * qw - qx * qx - qy * qy + qz * qz) / norm_squared;
  return std::acos(std::clamp(cos_tilt, -1.0, 1.0)) * kDegreesPerRadian;
}

bool CarriesTilt(double qx, double qy) { return qx != 0.0 || qy != 0.0; }

Check CheckPoseChannel(const PoseChannelFacts& facts) {
  const char* kName = "pose channel";
  const char* kSkipped = "pose channel (NOT CHECKED)";
  if (facts.frame_in_force.empty() || facts.samples <= 0) {
    return {kSkipped, Severity::kOk,
            "no transform was read, so there is nothing to say about what it carries", ""};
  }

  const std::string observed = Fixed(facts.height_span_meters, 3) + " m of height across " +
                               std::to_string(facts.samples) + " samples, of which " +
                               std::to_string(facts.samples_with_tilt) +
                               " carried any tilt at all (worst " +
                               Fixed(facts.max_tilt_degrees, 2) + " degrees)";

  // A producer that carries tilt shows it even parked: its rotations are not EXACTLY pure yaws.
  // A planar one builds roll and pitch from literal zeros, so it reads 0 of N however long it
  // runs. That is the discriminator, and unlike the magnitudes it still works on a flat floor.
  // PROMOTION is the exact-zero test and nothing else: any non-zero height span, or any rotation
  // that is not EXACTLY a pure yaw, rules out a planar producer -- a planar one builds both from
  // literal zeros and reads zero however long it runs. That is the discriminator, and unlike the
  // magnitudes it still works on a flat floor.
  //
  // What splits below is only the WORDING. The discriminator fires at about 1e-6 degrees, which
  // the magnitudes print as `0.00`, so one verdict for both would say "carries a third dimension"
  // beside two printed zeros -- a sentence that refutes itself, and a reader who then disbelieves
  // the check is worse off than one without it. The thresholds are the RENDERING thresholds of
  // `Fixed(x, 3)` and `Fixed(x, 2)`, so the words can never disagree with the numbers beside them.
  if (facts.samples_with_tilt > 0 || facts.height_span_meters > 0.0) {
    if (facts.height_span_meters >= 0.0005 || facts.max_tilt_degrees >= 0.005) {
      return {kName, Severity::kOk,
              "the pose from " + facts.frame_in_force + " carries a third dimension: " + observed,
              ""};
    }
    // Non-planar, with nothing big enough to print. Name which signal fired, because they
    // establish different things: a tilt count says the ORIENTATION is not planar and says
    // nothing about height, and a sub-millimetre height span is the converse.
    std::string evidence;
    if (facts.samples_with_tilt > 0) {
      evidence = std::to_string(facts.samples_with_tilt) + " of " + std::to_string(facts.samples) +
                 " samples carry a roll or pitch that is not exactly zero";
    }
    if (facts.height_span_meters > 0.0) {
      if (!evidence.empty()) evidence += ", and ";
      evidence += "the height is not constant";
    }
    return {kName, Severity::kOk,
            "the pose from " + facts.frame_in_force + " is NOT planar -- " + evidence +
                " -- but nothing moved off the plane far enough to measure: " +
                Fixed(facts.height_span_meters, 3) + " m of height, worst tilt below 0.01 degrees",
            "A planar producer builds roll and pitch from literal zeros and reads 0 of N however "
            "long it runs, so this rules out a planar-BY-CONSTRUCTION source even on level "
            "ground -- which is also why the magnitudes beside it are zero. It does NOT establish "
            "that this run recorded anything off-plane, and a non-zero tilt count on its own says "
            "nothing about HEIGHT: a producer can jitter its orientation while holding z fixed. "
            "Drive the robot off the plane if you need the magnitudes to mean something."};
  }
  if (PlanarByConstruction(facts.frame_in_force)) {
    return {kName, Severity::kAdvisory,
            "the pose from " + facts.frame_in_force + " recorded " + observed,
            "EXPECTED on flat ground, and INVISIBLE when it is not. The usual producers for a "
            "frame of this name are planar by construction: `odom` from wheel encoders has z, "
            "roll and pitch of zero permanently, and nav2_amcl composes `map` from a 3-DOF filter "
            "state. A robot that climbs a ramp is then recorded flat, and every verifier still "
            "passes, because each compares a session against itself. If instead your `odom` comes "
            "from robot_localization in 3D mode, or your `map` from a 6-DOF SLAM, this note does "
            "not apply -- those carry a tilt in every sample and this check would have counted "
            "it."};
  }
  return {kSkipped, Severity::kOk,
          "the pose from " + facts.frame_in_force + " recorded " + observed +
              "; nothing is known about what this frame's producer can carry",
          ""};
}

namespace {

// The settings the size estimate reads, from what doctor found.
CameraConfig SizedCamera(const MediaSensorFacts& facts) {
  CameraConfig camera;
  camera.name = facts.name;
  camera.rate_hz = facts.rate_hz;
  camera.crf = facts.crf;
  camera.max_bitrate_kbps = facts.max_bitrate_kbps;
  if (facts.replay_max_mb > 0) camera.replay_max_mb = facts.replay_max_mb;
  return camera;
}

// A lidar's measured sample, or a camera's estimate, at its rate; 0 when there is neither.
double SensorBytesPerSecond(const MediaSensorFacts& facts) {
  if (!facts.sample_seen) return 0.0;
  if (facts.kind == "camera") {
    return CameraBytesPerSecond(SizedCamera(facts), facts.prepared_width, facts.prepared_height);
  }
  return static_cast<double>(facts.encoded_bytes) * facts.rate_hz;
}

std::string Bytes(std::uint64_t n) {
  std::ostringstream os;
  if (n >= 1024 * 1024) {
    os << std::fixed << std::setprecision(1) << static_cast<double>(n) / (1024.0 * 1024.0)
       << " MB";
  } else if (n >= 1024) {
    os << std::fixed << std::setprecision(1) << static_cast<double>(n) / 1024.0 << " KB";
  } else {
    os << n << " B";
  }
  return os.str();
}

}  // namespace

Check CheckMediaSensor(const MediaSensorFacts& facts) {
  Check check;
  check.name = facts.kind + "/" + facts.name;

  if (!facts.topic_present) {
    check.severity = Severity::kAdvisory;
    check.detail = facts.topic + " is not on the graph";
    check.remedy =
        "check the topic name against `ros2 topic list`. A robot mid-bring-up legitimately has "
        "no camera yet, but a name that is simply wrong records nothing for the whole run and "
        "nothing else reports it.";
    return check;
  }
  if (!facts.type_handled) {
    check.severity = Severity::kFatal;
    check.detail = facts.topic + " is " + facts.type + ", which this SDK does not handle";
    check.remedy = facts.kind == "camera"
                       ? "a camera takes sensor_msgs/msg/Image or sensor_msgs/msg/CompressedImage"
                       : "a lidar takes sensor_msgs/msg/LaserScan or sensor_msgs/msg/PointCloud2";
    return check;
  }
  if (facts.frame.empty()) {
    check.severity = Severity::kFatal;
    check.detail = facts.topic + " publishes no header.frame_id and no frame is configured";
    check.remedy = "set " + facts.kind + "." + facts.name +
                   ".frame. Every frame and scan carries a pose, and one whose frame is unknown "
                   "is dropped -- the stream would record nothing at all.";
    return check;
  }
  if (!facts.frame_resolves) {
    check.severity = Severity::kFatal;
    check.detail = "'" + facts.frame + "' does not resolve to the reference frame";
    check.remedy =
        "check the frame against `ros2 run tf2_tools view_frames --wait-time 15`. A record whose "
        "pose cannot be looked up is DROPPED, so this stream would be silent while the run looked "
        "healthy.";
    return check;
  }
  if (!facts.sample_seen) {
    check.severity = Severity::kAdvisory;
    check.detail = facts.topic + " [" + facts.type + "] qos=" + facts.qos +
                   " published nothing while doctor watched";
    check.remedy =
        "the topic exists and its frame resolves, so this may only be a slow publisher -- but a "
        "QoS mismatch looks exactly like this and produces silence rather than an error. Check "
        "`ros2 topic hz " + facts.topic + "`.";
    return check;
  }
  const bool camera = facts.kind == "camera";
  if (camera ? facts.prepared_width == 0 : facts.encoded_bytes == 0) {
    check.severity = Severity::kFatal;
    check.detail = std::string("a sample arrived and could not be ") +
                   (camera ? "prepared" : "encoded") + ": " + facts.encode_error;
    check.remedy = "nothing from this sensor would reach a session. Fix the topic or the "
                   "configuration named above.";
    return check;
  }

  std::ostringstream detail;
  detail << facts.topic << " [" << facts.type << "] qos=" << facts.qos << " frame=" << facts.frame;
  if (camera) {
    const CameraConfig sized = SizedCamera(facts);
    const double chunk_s = CameraChunkSeconds(sized, facts.prepared_width, facts.prepared_height);
    if (chunk_s == 0.0) {
      check.severity = Severity::kFatal;
      check.detail = "one frame prepares to " + std::to_string(facts.prepared_width) + "x" +
                     std::to_string(facts.prepared_height) + ", and " +
                     CameraChunkTooLarge(sized, facts.prepared_width, facts.prepared_height);
      check.remedy = "the node would refuse this camera at its first frame and record none of "
                     "its video. Change one of those parameters, then run doctor again.";
      return check;
    }
    const double per_second = SensorBytesPerSecond(facts);
    detail << ", one frame prepares to " << facts.prepared_width << "x" << facts.prepared_height
           << "; about " << Bytes(static_cast<std::uint64_t>(per_second))
           << "/s estimated (from that size, crf " << facts.crf << " and " << facts.rate_hz
           << " Hz";
    if (facts.max_bitrate_kbps > 0) detail << ", capped by max_bitrate_kbps";
    detail << ", by a model fitted to recorded footage; not measured on this camera); chunks of "
           << chunk_s << " s, about " << Bytes(static_cast<std::uint64_t>(per_second * chunk_s))
           << " each (sized to stay under "
           << Bytes(static_cast<std::uint64_t>(kCameraChunkTargetBytes)) << "); ";
    if (facts.max_bitrate_kbps > 0) {
      detail << "the bitrate is capped at max_bitrate_kbps " << facts.max_bitrate_kbps;
    } else if (ReplayNeedsBitrateCap(sized, facts.prepared_width, facts.prepared_height)) {
      detail << kReplayWindowFrames << " frames of this size overflow replay_max_mb "
             << sized.replay_max_mb << ", so the bitrate is capped at "
             << DefaultCameraBitrateKbps(chunk_s) << " kbps by default, and a chunk that still "
             << "reaches 8 MiB loses the frames the store could not keep";
    } else {
      detail << "no bitrate cap: a chunk that reaches 8 MiB ends and its last frames are encoded "
             << "again from the replay store";
    }
  } else {
    detail << ", one sample encodes to " << Bytes(facts.encoded_bytes) << " -- about "
           << Bytes(static_cast<std::uint64_t>(SensorBytesPerSecond(facts))) << "/s at "
           << facts.rate_hz << " Hz";
  }
  check.detail = detail.str();
  return check;
}

namespace {

bool EndsWith(const std::string& text, const std::string& suffix) {
  return text.size() >= suffix.size() &&
         text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// The link an optical frame is conventionally attached to: REP-103 names the pair
// `<x>_frame` and `<x>_optical_frame`. A GUESS, and printed as one -- the remedy
// has to name a starting point a person can check against their own tf tree, and
// "set it to the right frame" is not one.
std::string AttachedLinkGuess(const std::string& optical_frame) {
  if (EndsWith(optical_frame, "_optical_frame")) {
    return optical_frame.substr(0, optical_frame.size() - std::string("_optical_frame").size()) +
           "_frame";
  }
  // `<x>_optical` pairs with `<x>_link`, not with a bare `<x>`: dropping the suffix
  // and appending nothing names a frame that is generally not in the tree at all,
  // and the remedy is only useful if what it names is worth looking for.
  return optical_frame.substr(0, optical_frame.size() - std::string("_optical").size()) +
         "_link";
}

}  // namespace

Check CheckOpticalCloud(const MediaSensorFacts& facts) {
  const std::string name = facts.kind + "/" + facts.name + " optical frame";
  const std::string skipped = name + " (NOT CHECKED)";

  if (!EndsWith(facts.frame, "_optical_frame") && !EndsWith(facts.frame, "_optical")) {
    return {skipped, Severity::kOk,
            "'" + facts.frame + "' is not an optical frame, so there is no mismatch of this shape "
            "to look for",
            ""};
  }
  if (facts.finite_points < kOpticalCloudMinPoints) {
    return {skipped, Severity::kOk,
            std::to_string(facts.finite_points) + " finite points arrived from " + facts.topic +
                ", too few to read an axis out of",
            ""};
  }

  const double depth = static_cast<double>(facts.z_positive_points) /
                       static_cast<double>(facts.finite_points);
  const double forward = static_cast<double>(facts.x_positive_points) /
                         static_cast<double>(facts.finite_points);
  const std::string measured = "of " + std::to_string(facts.finite_points) +
                               " finite points, " + Fixed(depth * 100.0, 0) +
                               "% have z > 0 and " + Fixed(forward * 100.0, 0) + "% have x > 0";

  if (depth >= kOpticalDepthFraction) {
    return {name, Severity::kOk,
            facts.topic + " carries points that are in '" + facts.frame + "': " + measured, ""};
  }
  if (forward >= kBodyForwardFraction) {
    // ADVISORY rather than fatal, and the line is not the consequence -- a cloud
    // stored on its side is exactly the "uploads and is wrong" shape that makes
    // mesh/exists fatal. It is the EVIDENCE: this reads a statistical signature off
    // one sample of a third-party publisher, where mesh/exists reads a listing that
    // either contains a name or does not. A preflight that refuses to start a run
    // on an inference about geometry is one that will eventually refuse a healthy
    // robot, and then be muted. The remedy is one parameter.
    return {name, Severity::kAdvisory,
            facts.topic + " is posed against '" + facts.frame +
                "' and its points are not in it: " + measured +
                " -- depth is +Z in an optical frame and +X in the body frame the sensor is "
                "attached to",
            "Set lidar." + facts.name + ".frame to the link the sensor is attached to (probably '" +
                AttachedLinkGuess(facts.frame) +
                "' -- check it against `ros2 run tf2_tools view_frames --wait-time 15`). Gazebo's "
                "`rgbd_camera` writes its <optical_frame_id> into the PointCloud2 header and "
                "leaves the point data in the frame the sensor is attached to, so this SDK looks "
                "the pose up for a frame the points are not in and the stored cloud lies on its "
                "side. The session uploads, every point is present, and nothing downstream "
                "reports it. A real OAK-D publishes in the optical frame and needs no override, so "
                "this belongs in the simulation's params file rather than the robot's."};
  }
  return {skipped, Severity::kOk,
          facts.topic + " matches neither an optical frame nor the body frame one is attached to: " +
              measured,
          ""};
}

Check CheckCameraFov(const CameraFovFacts& facts) {
  Check check;
  check.name = "camera/" + facts.name + " field of view";
  const std::string parameters =
      "camera." + facts.name + ".hfov_degrees and vfov_degrees";
  const std::string consequence =
      "Its frames carry no field of view, and Session Replay draws no frustum for this camera.";
  std::ostringstream angles;
  angles << std::fixed << std::setprecision(1);

  if (facts.parameters_set) {
    angles << facts.hfov_degrees << " x " << facts.vfov_degrees;
    check.detail = angles.str() + " degrees, from " + parameters;
    return check;
  }
  check.severity = Severity::kAdvisory;
  const std::string set_them =
      "Set " + parameters + " from the lens datasheet or a measurement";
  if (facts.camera_info_topic.empty()) {
    check.detail = "no source: the parameters are unset and camera." + facts.name +
                   ".camera_info_topic is empty. " + consequence;
    check.remedy = set_them + ", or point camera_info_topic at the driver's CameraInfo.";
    return check;
  }
  if (!facts.topic_present) {
    check.detail = "no source: the parameters are unset and " + facts.camera_info_topic +
                   " is not on the graph. " + consequence;
    check.remedy = set_them + ", or set camera." + facts.name +
                   ".camera_info_topic to the topic the driver publishes its CameraInfo on.";
    return check;
  }
  if (facts.type != "sensor_msgs/msg/CameraInfo") {
    check.detail = "no source: " + facts.camera_info_topic + " is " + facts.type +
                   ", not sensor_msgs/msg/CameraInfo. " + consequence;
    check.remedy = set_them + ", or set camera." + facts.name +
                   ".camera_info_topic to the driver's CameraInfo topic.";
    return check;
  }
  if (!facts.sample_seen) {
    check.detail = facts.camera_info_topic + " published nothing while doctor watched";
    check.remedy = "a driver usually publishes CameraInfo with every image; check `ros2 topic hz " +
                   facts.camera_info_topic + "`. Without a message, " +
                   "frames carry no field of view. " + set_them + " to not depend on it.";
    return check;
  }
  if (facts.sample_hfov_degrees == 0.0f || facts.sample_vfov_degrees == 0.0f) {
    std::ostringstream fx;
    fx << facts.sample_fx;
    check.detail = "no usable source: " + facts.camera_info_topic +
                   " carries no usable intrinsics (fx " + fx.str() +
                   "), which is what a driver with no calibration publishes. " + consequence;
    check.remedy = set_them + ", or calibrate the camera.";
    return check;
  }
  check.severity = Severity::kOk;
  angles << facts.sample_hfov_degrees << " x " << facts.sample_vfov_degrees;
  check.detail = angles.str() + " degrees, from " + facts.camera_info_topic +
                 " (the pinhole model; a strongly distorted wide lens sees further at its edges)";
  return check;
}

Check CheckMediaBandwidth(const std::vector<MediaSensorFacts>& facts) {
  Check check;
  check.name = "camera/lidar bandwidth";

  double total = 0.0;
  int measured = 0;
  for (const auto& sensor : facts) {
    const double rate = SensorBytesPerSecond(sensor);
    if (rate <= 0.0) continue;
    total += rate;
    ++measured;
  }
  if (measured == 0) {
    check.name = "camera/lidar bandwidth (NOT CHECKED)";
    check.detail = facts.empty() ? "no cameras or lidars are configured"
                                 : "no sample was prepared or encoded, so there is nothing to size";
    return check;
  }

  std::ostringstream detail;
  detail << Bytes(static_cast<std::uint64_t>(total)) << "/s across " << measured << " sensor(s), "
         << Bytes(static_cast<std::uint64_t>(total * 3600.0)) << " per hour";
  check.detail = detail.str();
  if (total > kMediaBandwidthWarnBytesPerSecond) {
    check.severity = Severity::kAdvisory;
    check.remedy =
        "nothing on the platform rate-limits this, so the configuration IS the limit. Lower "
        "rate_hz or max_width_px, raise a camera's crf, or raise the point cloud's voxel_leaf_m.";
  }
  return check;
}

Check CheckCameraEncoder(const CameraEncoderFacts& facts) {
  if (facts.cameras == 0) {
    return {"camera encoder (NOT CHECKED)", Severity::kOk,
            "no camera is configured, so nothing runs " + facts.ffmpeg_path, ""};
  }
  if (!facts.problem.empty()) {
    return CameraEncoderRefused(FfmpegProblem(facts.ffmpeg_path, facts.problem));
  }
  return {"camera encoder", Severity::kOk, facts.ffmpeg_path + ": " + facts.version, ""};
}

std::string Render(const std::vector<Check>& checks, const std::string& runtime) {
  std::ostringstream out;
  out << "cognitive3d_ros " << c3d::kSdkVersion << " doctor";
  if (!runtime.empty()) out << " (" << runtime << ")";
  out << "\n\n";
  if (checks.empty()) {
    out << "  no checks ran, which is a defect in doctor itself rather than a pass\n";
    return out.str();
  }
  std::size_t width = 0;
  for (const auto& check : checks) width = std::max(width, check.name.size());
  for (const auto& check : checks) {
    out << "  [" << Label(check.severity) << "] " << check.name
        << std::string(width - check.name.size(), ' ') << "  " << check.detail << "\n";
    if (!check.remedy.empty()) out << "         -> " << check.remedy << "\n";
  }
  const Severity worst = Aggregate(checks);
  out << "\n";
  if (worst == Severity::kFatal) {
    out << "  FAIL: fix the items above before opening a session.\n";
  } else if (worst == Severity::kAdvisory) {
    out << "  OK, with notes. A session opened now will record a spatial timeline.\n";
  } else {
    out << "  OK.\n";
  }
  return out.str();
}

}  // namespace cognitive3d_ros::doctor
