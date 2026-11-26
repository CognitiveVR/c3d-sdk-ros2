#include "cognitive3d_ros/node.hpp"

#include "c3d/environment.hpp"

#include "c3d/properties.hpp"
#include "cognitive3d_ros/destination.hpp"
#include "cognitive3d_ros/identity.hpp"
#include "cognitive3d_ros/image_codec.hpp"
#include "cognitive3d_ros/translators/int32_multi_array_translator.hpp"
#include "cognitive3d_ros/upload_health.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/version.h>
#include <rclcpp_components/register_node_macro.hpp>

#include "c3d/h264_encoder.hpp"
#include "c3d/session.hpp"
#include "c3d/spool.hpp"
#include "c3d/transport.hpp"
#include "c3d/wall_clock.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace cognitive3d_ros {
namespace {

std::string SanitizeForParameter(const std::string& topic) {
  std::string out;
  out.reserve(topic.size());
  for (char c : topic) {
    if (c == '/' || c == '*') {
      if (!out.empty() && out.back() != '_') out.push_back('_');
    } else {
      out.push_back(c);
    }
  }
  if (!out.empty() && out.front() == '_') out.erase(out.begin());
  if (!out.empty() && out.back() == '_') out.pop_back();
  return out.empty() ? "root" : out;
}

double WallEpochSeconds() {
  using namespace std::chrono;
  return duration<double>(system_clock::now().time_since_epoch()).count();
}

double SteadySeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// The event-cap source keys for the events no topic produced. A translator's key is
// its topic, which neither of these can collide with.
constexpr const char* kUserEventSource = "~/events";
constexpr const char* kRemoteVariablesEventSource = "remote_variables";

// The property namespace the dashboard reads as an A/B arm. While the node fetches arms itself,
// a consumer-supplied value here is indistinguishable on the dashboard from a drawn one.
constexpr const char* kRemoteVariablePrefix = "c3d.remote_variable.";

bool NamesARemoteVariable(const std::string& key) {
  return key.rfind(kRemoteVariablePrefix, 0) == 0;
}

// The namespace the SDK records its own properties in. A session property supplied under it
// replaces the SDK's, and without `c3d.app.version` the platform discards the session.
constexpr const char* kSdkPropertyPrefix = "c3d.";

bool InSdkNamespace(const std::string& key) { return key.rfind(kSdkPropertyPrefix, 0) == 0; }

// Whether a consumer-supplied session property would replace one the SDK owns. The arms are the
// consumer's to supply while the node fetches none itself.
bool NamesAnSdkSessionProperty(const std::string& key, bool node_fetches_arms) {
  return InSdkNamespace(key) && (node_fetches_arms || !NamesARemoteVariable(key));
}

// The `session_properties` entry naming an SDK-owned key, or empty.
std::string SdkKeyInSessionProperties(const std::vector<std::string>& entries,
                                      bool node_fetches_arms) {
  for (const auto& entry : entries) {
    const std::string key = entry.substr(0, entry.find('='));
    if (NamesAnSdkSessionProperty(key, node_fetches_arms)) return key;
  }
  return {};
}

std::string SdkKeyRefusal(const std::string& where, const std::string& key) {
  if (NamesARemoteVariable(key)) {
    return where + " names " + key +
           ", which the SDK owns while remote_variables is true; set remote_variables: false to "
           "supply arms yourself";
  }
  return where + " names " + key + ", in the c3d. namespace the SDK records its own session "
         "properties in, where a value would replace the SDK's. Rename the key";
}

// The message types whose translators read `topic_config.<key>.series`, as one phrase.
std::string SeriesTypesText(const TranslatorRegistry& registry) {
  const std::vector<std::string> types = registry.series_types();
  std::string out;
  for (std::size_t i = 0; i < types.size(); ++i) {
    if (i > 0) out += i + 1 == types.size() ? " or " : ", ";
    out += types[i];
  }
  return out.empty() ? std::string("a type whose translator reads a series name") : out;
}

// The refusal for a topic name ExpandTopicName cannot resolve.
std::string UnresolvableTopicName(const std::string& where, const std::string& name) {
  return where + " '" + name + "' is not a topic name: '~' must be followed by '/', as in '~/" +
         name.substr(1) + "'";
}

// A camera, lidar or sensor name is a parameter path segment as well as a wire field, so a '.' or
// a '/' in one would silently address a different parameter than the operator wrote.
bool UsableSensorName(const std::string& name) {
  return !name.empty() && name.find('.') == std::string::npos &&
         name.find('/') == std::string::npos;
}

constexpr const char* kTopicConfigBlock = "topic_config";

// The refusal for a `topic_config` key under none of the keys the `topics` entries give. Nothing
// reads it: a group is declared and read only for an entry in `topics`.
std::string UnlistedTopicConfigKey(const std::string& key, const std::set<std::string>& listed) {
  std::string read;
  for (const std::string& name : listed) {
    read += (read.empty() ? "topic_config." : ", topic_config.") + name;
  }
  return key + " matches no entry in `topics`, and a topic's topic_config is read only when the "
         "topic is in `topics`: add the topic to `topics`, or remove the key. An entry's key is "
         "the entry as written, with each run of '/' and '*' collapsed to one '_' and a leading "
         "or trailing '_' removed: '/odom' is topic_config.odom, and '/robot1/*' is "
         "topic_config.robot1. " +
         (read.empty() ? std::string("`topics` is empty") : "The keys `topics` gives are " + read);
}

// The one cone_rangefinder name Session Replay draws a cone for.
constexpr const char* kConeDrawnFor = "ultrasonic";

// `camera.<name>.camera_info_topic`'s default: the image topic's sibling.
constexpr const char* kCameraInfoTopicAuto = "auto";
constexpr const char* kCameraInfoType = "sensor_msgs/msg/CameraInfo";

// The periods the node's timers are held to.
constexpr double kMinTimerPeriodS = 0.001;
constexpr double kMaxTimerPeriodS = 86400.0;

constexpr const char* kApplicationKeyMissing =
    "C3D_APPLICATION_API_KEY is not set in this process's environment; the node reads the "
    "application key from there and nowhere else";

// A ROS parameter is readable by anything on the graph and is echoed into launch logs and
// parameter dumps, so the application key is never one. The name is not declared, and any value of
// it, the empty one included, is refused: at configure from a params file without ever being
// readable as a parameter, and at once when set at runtime. The refusal never repeats the value.
constexpr const char* kApplicationKeyParameter = "application_key";
constexpr const char* kApplicationKeyParameterRefused =
    "application_key is not a parameter: the node reads the application key only from "
    "C3D_APPLICATION_API_KEY in its environment, because anything on the ROS graph can read a "
    "parameter. Remove application_key from the parameters and export C3D_APPLICATION_API_KEY.";

// Keys a 1.x params file sets that this release no longer reads: refused naming the key, rather
// than configuring with a setting that does nothing.
constexpr const char* kRemovedFlushKeys[] = {"camera_flush_s", "camera_flush_bytes",
                                             "lidar_flush_s", "lidar_flush_bytes"};
constexpr const char* kRemovedFlushKeyWhy =
    " was removed: a camera chunk, a scan block and a cloud object each close by their own rule. "
    "Delete the key";
constexpr const char* kRemovedCameraFields[] = {"format", "jpeg_quality"};
constexpr const char* kRemovedCameraFieldWhy =
    " was removed: a camera is encoded as H.264, its quality set by preset and crf. Delete the key";

// A scene id is a UUID, 8-4-4-4-12 hex digits in either case. The collector answers 200 for one
// that names no scene and discards the session, so a placeholder left in a params file is refused.
bool IsSceneId(const std::string& value) {
  if (value.size() != 36) return false;
  for (std::size_t i = 0; i < value.size(); ++i) {
    const bool dash = i == 8 || i == 13 || i == 18 || i == 23;
    if (dash ? value[i] != '-' : !std::isxdigit(static_cast<unsigned char>(value[i]))) {
      return false;
    }
  }
  return true;
}

std::string SceneIdRefusal(const std::string& value) {
  return "scene_id '" + value + "' is not a scene id: it must be the scene's UUID, 8-4-4-4-12 "
         "hexadecimal digits. Take it from the scene on the Cognitive3D dashboard, or from the "
         "sceneId the Upload Web App reports when you upload the scene (docs/scenes.md)";
}

constexpr const char* kPoseConventionMissing =
    "pose_convention is not set, and it has no default: 'gltf_authored' when the scene came from "
    "tools/export_world_gltf.sh or any ROS -> glTF exporter mapping gltf.xyz = (ros.x, ros.z, "
    "-ros.y); 'unity' when the scene was authored in Unity against ROS-TCP-Connector axes; "
    "'rep103' to put REP-103 poses on the wire unconverted, for debugging. The two scene "
    "conventions differ by a 90 degree yaw, and nothing in the payload shows the wrong one";

// `first` followed by every problem in `validated`, in Config::Validate's shape: one problem is
// its bare message, several are "N configuration problems: a | b". Validate joins only its
// mandatory-value block that way; anything else it returns is one message.
std::string JoinConfigurationProblems(std::vector<std::string> first,
                                      const std::string& validated) {
  if (!validated.empty()) {
    const std::string marker = " configuration problems: ";
    const std::string separator = " | ";
    const std::size_t marker_at = validated.find(marker);
    if (marker_at != std::string::npos && marker_at > 0 &&
        validated.find_first_not_of("0123456789") == marker_at) {
      const std::string joined = validated.substr(marker_at + marker.size());
      std::size_t from = 0;
      while (true) {
        const std::size_t at = joined.find(separator, from);
        first.push_back(joined.substr(from, at == std::string::npos ? std::string::npos
                                                                    : at - from));
        if (at == std::string::npos) break;
        from = at + separator.size();
      }
    } else {
      first.push_back(validated);
    }
  }
  if (first.empty()) return {};
  if (first.size() == 1) return first.front();
  std::string out = std::to_string(first.size()) + " configuration problems: " + first.front();
  for (std::size_t i = 1; i < first.size(); ++i) out += " | " + first[i];
  return out;
}

// The parameters a set changes while the node is configured: read again at every session start,
// or applied at once.
bool AppliesWhileConfigured(const std::string& name) {
  return name == "session_name" || name == "session_properties" || name == "uploads_paused";
}

// The service QoS argument, in whichever type this rclcpp accepts without complaint. Humble has
// only the `rmw_qos_profile_t` overload of `create_service`; rclcpp 21 added an `rclcpp::QoS` one
// and deprecated the profile it replaced. Naming either type unconditionally therefore costs a
// build: the new one does not compile on Humble, the old one warns on everything since.
#if RCLCPP_VERSION_GTE(21, 0, 0)
rclcpp::QoS ServiceQos() { return rclcpp::ServicesQoS(); }
#else
rmw_qos_profile_t ServiceQos() { return rclcpp::ServicesQoS().get_rmw_qos_profile(); }
#endif

}  // namespace

std::string DefaultSpoolDirectory(const std::function<const char*(const char*)>& lookup) {
  const char* state = lookup("XDG_STATE_HOME");
  if (state != nullptr && state[0] == '/') return std::string(state) + "/cognitive3d";
  const char* home = lookup("HOME");
  if (home != nullptr && home[0] == '/') return std::string(home) + "/.local/state/cognitive3d";
  return {};
}

std::string ExpandTopicName(const std::string& name, const std::string& node_namespace,
                            const std::string& node_fqn) {
  if (name.empty() || name.front() == '/') return name;
  if (name.front() == '~') {
    if (name.size() > 1 && name[1] != '/') return {};
    return node_fqn + name.substr(1);
  }
  return (node_namespace == "/" ? std::string("/") : node_namespace + "/") + name;
}

std::string DefaultDeviceId(const std::string& node_namespace) {
  std::string id = "ros";
  std::size_t from = 0;
  while (from <= node_namespace.size()) {
    const std::size_t slash = node_namespace.find('/', from);
    const std::size_t end = slash == std::string::npos ? node_namespace.size() : slash;
    if (end > from) id += "-" + node_namespace.substr(from, end - from);
    from = end + 1;
  }
  return id == "ros" ? "ros-unnamed" : id;
}

int ReaderTimeoutSeconds(double requested) {
  // A NaN is not a positive number, and every comparison below would answer false for it while
  // the cast at the end is undefined.
  if (std::isnan(requested)) return 0;
  if (requested <= 0.0) return 0;
  // Clamped in double space: `static_cast<int>` of a ceil that overflows int is undefined, and
  // a milliseconds value written into a seconds parameter overflows it easily.
  return static_cast<int>(std::clamp(std::ceil(requested), 1.0, 60.0));
}

Cognitive3DNode::Cognitive3DNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("cognitive3d", options) {
  DeclareParameters();
  // Applied inside the validation callback because it is the one both tier 1
  // distros have, and nothing else registers one that could still reject the
  // set afterwards. It runs on the default callback group, which is where every
  // callback of this node and every lifecycle transition run, so none of them
  // interleave with it.
  parameters_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& parameter : parameters) {
          if (parameter.get_name() == kApplicationKeyParameter) {
            result.successful = false;
            result.reason = kApplicationKeyParameterRefused;
            return result;
          }
        }
        // Every other parameter is read at configure, so a set while configured would leave
        // `ros2 param get` naming a value that is not in force.
        const std::uint8_t state = get_current_state().id();
        const bool configured = state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
                                state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE ||
                                state == lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING ||
                                state == lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING;
        for (const auto& parameter : parameters) {
          const std::string& name = parameter.get_name();
          if (!configured || AppliesWhileConfigured(name)) continue;
          result.successful = false;
          result.reason = name + " is read at configure, so a set now would apply only at the "
                          "next one. Clean the node up (`ros2 lifecycle set " +
                          std::string(get_node_base_interface()->get_fully_qualified_name()) +
                          " cleanup`), set it, and configure again; only session_name, "
                          "session_properties and uploads_paused apply to a configured node.";
          return result;
        }
        // Read again at every session start, so a set is refused here as configure refuses it.
        for (const auto& parameter : parameters) {
          if (parameter.get_name() != "session_properties" ||
              parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
            continue;
          }
          bool fetches_arms = get_parameter("remote_variables").as_bool();
          for (const auto& other : parameters) {
            if (other.get_name() == "remote_variables" &&
                other.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
              fetches_arms = other.as_bool();
            }
          }
          const std::string key =
              SdkKeyInSessionProperties(parameter.as_string_array(), fetches_arms);
          if (!key.empty()) {
            result.successful = false;
            result.reason = SdkKeyRefusal("session_properties", key);
            return result;
          }
        }
        for (const auto& parameter : parameters) {
          if (parameter.get_name() != "uploads_paused") continue;
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) continue;
          const bool paused = parameter.as_bool();
          c3d::Session* session = runner_ ? runner_->session() : nullptr;
          if (session != nullptr) {
            const std::size_t spooled = session->stats().spool_count;
            session->SetUploadsPaused(paused);
            RCLCPP_WARN(get_logger(), "uploads %s at runtime: %zu chunk(s) spooled",
                        paused ? "PAUSED" : "resumed", spooled);
          } else {
            RCLCPP_INFO(get_logger(),
                        "uploads_paused set to %s; applied at the next configure",
                        paused ? "true" : "false");
          }
        }
        return result;
      });
  // At construction, not at configure: whoever composes this node adds its own translators
  // between the two, and a configure that rebuilt the registry would be a second place for the
  // set to be decided.
  RegisterCoreTranslators(translators_);
  RCLCPP_INFO(get_logger(),
              "cognitive3d_ros %s (%s) constructed. Subscribe-only: this node publishes "
              "nothing except its own diagnostics.",
              c3d::kSdkVersion, RuntimeDescription().c_str());
}

Cognitive3DNode::~Cognitive3DNode() {
  StopTransformListener();
  // A stop signal shuts the context down before the process destroys the node, a container's
  // components included, so a session still open on a shut-down context is an orderly stop.
  if (runner_ && runner_->active() && !rclcpp::ok(get_node_base_interface()->get_context())) {
    StopSession("shutdown");
  }
}

bool Cognitive3DNode::uploads_paused() const {
  const c3d::Session* session = runner_ ? runner_->session() : nullptr;
  return session != nullptr && session->uploads_paused();
}

void Cognitive3DNode::AddTranslators(TranslatorRegistrar registrar) {
  if (registrar != nullptr) registrar(translators_);
}

void Cognitive3DNode::DeclareDouble(const std::string& name, double default_value) {
  if (has_parameter(name)) return;
  // A whole number in YAML or on the command line is an integer to rclcpp, which refuses it for a
  // double parameter by aborting the process at construction. Taken as the double it means.
  const auto& overrides = get_node_parameters_interface()->get_parameter_overrides();
  const auto given = overrides.find(name);
  if (given != overrides.end() &&
      given->second.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    declare_parameter<double>(name, static_cast<double>(given->second.get<std::int64_t>()),
                              rcl_interfaces::msg::ParameterDescriptor{},
                              /*ignore_override=*/true);
    return;
  }
  declare_parameter<double>(name, default_value);
}

void Cognitive3DNode::DeclareDoubleArray(const std::string& name) {
  if (has_parameter(name)) return;
  const auto& overrides = get_node_parameters_interface()->get_parameter_overrides();
  const auto given = overrides.find(name);
  if (given != overrides.end() &&
      given->second.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
    const auto whole = given->second.get<std::vector<std::int64_t>>();
    declare_parameter<std::vector<double>>(name, std::vector<double>(whole.begin(), whole.end()),
                                           rcl_interfaces::msg::ParameterDescriptor{},
                                           /*ignore_override=*/true);
    return;
  }
  declare_parameter<std::vector<double>>(name, std::vector<double>{});
}

void Cognitive3DNode::DeclareParameters() {
  declare_parameter<std::string>("scene_id", "");
  declare_parameter<std::string>("scene_version", "");
  declare_parameter<std::string>("device_id", "");
  // Empty means "the data host the process environment resolves". A literal default could not be
  // told apart from an operator deliberately naming the production host, so an explicit override
  // -- which `tools/stub_gateway.py` and any loopback capture need -- would be undetectable.
  declare_parameter<std::string>("gateway", "");
  // http sends the key unencrypted, so it is accepted only beside an overridden gateway: a
  // capture stub, never a Cognitive3D host.
  declare_parameter<std::string>("protocol", "https");
  // Empty resolves to a per-user state directory at configure: a shared, world-writable default
  // would let two instances, or two users, write into one spool.
  declare_parameter<std::string>("spool_dir", "");
  declare_parameter<int>("spool_max_mb", 1024);
  // How often a live session refreshes its spool marker, and how long a marker
  // must have gone unrefreshed before startup recovery may adopt it as an orphan.
  // Exposed because the only reason to change them is a deployment fact the SDK
  // cannot see -- several nodes sharing a spool directory, or a link slow enough
  // that the defaults are tight. Defaults match c3d::Config.
  DeclareDouble("marker_heartbeat_s", 5.0);
  DeclareDouble("orphan_stale_after_s", 30.0);
  // Uploads. `uploads_paused` records and spools as normal and sends nothing; it
  // is the one parameter here that is applied at runtime, by `ros2 param set`, as
  // well as at configure. The rest are read at configure: how an upload is judged
  // alive (connect timeout, a stall detector, a whole-request ceiling) and the
  // retry back-off. Defaults match c3d::Config, which says why they are what they are.
  declare_parameter<bool>("uploads_paused", c3d_config_.uploads_paused);
  declare_parameter<int>("http_connect_timeout_s", c3d_config_.http_connect_timeout_s);
  declare_parameter<int>("http_low_speed_limit_bytes_per_s",
                         c3d_config_.http_low_speed_limit_bytes_per_s);
  declare_parameter<int>("http_low_speed_time_s", c3d_config_.http_low_speed_time_s);
  declare_parameter<int>("http_timeout_s", c3d_config_.http_timeout_s);
  DeclareDouble("retry_min_s", c3d_config_.retry_min_s);
  DeclareDouble("retry_max_s", c3d_config_.retry_max_s);

  // No default: the right value depends on how the scene was authored, and a wrong one is
  // invisible in the payload.
  declare_parameter<std::string>("pose_convention", "");
  declare_parameter<bool>("dry_run", false);
  // Requires use_sim_time. Off, every wire timestamp is wall clock now.
  declare_parameter<bool>("sim_time_on_wire", false);
  DeclareDouble("sample_rate_hz", 30.0);
  declare_parameter<std::string>("reference_frame", "map");
  declare_parameter<std::string>("fallback_reference_frame", "odom");
  // Declared from the struct so the two defaults cannot drift apart.
  DeclareDouble("reference_frame_grace_s", sampler_config_.reference_frame_grace_s);
  // What counts as a relocalisation jump rather than as the robot moving, in the
  // preferred frame. Both are PER CHECK: poses about 0.1 s apart at a
  // `sample_rate_hz` of 10 or more, consecutive samples below it, where halving the
  // rate doubles what one honest sample covers.
  //
  // Exposed because the right value is a fact about the deployment that the SDK
  // cannot see. 0.5 m is coarse in a room the robot crosses in four steps and
  // trigger-happy in a warehouse where a localiser's ordinary correction is a
  // metre; a shipped constant is wrong at one end or the other for everyone.
  //
  // The rotational one exists because position alone cannot see a jump that is
  // almost pure rotation -- a localiser re-registering against an ambiguous room
  // re-estimates its yaw by a right angle while the robot stands still -- and
  // orientation is what the viewer turns the robot by, so missing one corrupts the
  // spatial timeline and not merely a travel metric. 45 degrees sits above the
  // 11 degrees a base spinning at 109 deg/s covers in 0.1 s, with room for a
  // check that came late, and below the right angle such a re-estimate is.
  //
  // Declared from the struct so the two sets of defaults cannot drift apart.
  DeclareDouble("relocalisation_jump_m", sampler_config_.relocalisation_jump_m);
  DeclareDouble("relocalisation_jump_deg", sampler_config_.relocalisation_jump_deg);
  declare_parameter<std::string>("robot_frame", "base_link");
  declare_parameter<std::string>("gaze_frame", "");
  declare_parameter<std::string>("gaze_frame_convention", "auto");
  // Off: the platform draws the robot from the pose timeline, so no object is registered, no
  // dynamics sample is recorded and no mesh has to exist on the dashboard. `robot_mesh`,
  // `robot_display_name` and `robot_object_id` matter only when it is on -- `robot_object_id` is
  // the id the dynamic object is registered and sampled under; with the switch off nothing uses
  // its value.
  declare_parameter<bool>("robot_dynamic_object", false);
  declare_parameter<std::string>("robot_display_name", "Robot");
  declare_parameter<std::string>("robot_mesh", "robot_placeholder");
  declare_parameter<std::string>("robot_object_id", "robot");
  DeclareDouble("mesh_yaw_offset_deg", 0.0);
  DeclareDouble("dynamic_up_offset_meters", 0.0);
  declare_parameter<std::string>("session_name", "");
  declare_parameter<bool>("auto_start_session", true);
  declare_parameter<std::vector<std::string>>("session_properties", std::vector<std::string>{});
  declare_parameter<bool>("remote_variables", true);
  DeclareDouble("remote_variables_timeout_s", 3.0);

  // Both empty by default, so a deployment that names no camera or lidar creates no subscription
  // for either, reads no parameter below, and sends nothing on either stream.
  declare_parameter<std::vector<std::string>>("cameras", std::vector<std::string>{});
  declare_parameter<std::vector<std::string>>("lidars", std::vector<std::string>{});
  // Empty by default too: a sensor description is a set of session properties and nothing else.
  declare_parameter<std::vector<std::string>>("sensors", std::vector<std::string>{});
  // The camera encoder, run as a child process: searched on PATH unless it contains a slash.
  declare_parameter<std::string>("ffmpeg_path", "ffmpeg");

  declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>{});

  // Actions and the two standard event types are enrolled without an allowlist
  // entry, because their topic names are a deployment detail the SDK cannot guess
  // while their TYPES are fixed. An action is matched by NAME, under the node's
  // namespace (anywhere from the root), so one entry covers every robot's -- and
  // setting `action_names` REPLACES the built-in list rather than adding to it.
  declare_parameter<bool>("action_discovery", true);
  declare_parameter<std::vector<std::string>>("action_names", DefaultActionNames());
  declare_parameter<bool>("auto_subscribe_standard_types", true);
  // Events per second, per source. The session model drops the NEWEST record past
  // its bound, so an unbounded source does not crowd the others out -- it deletes
  // whatever happens next. 0 removes the cap.
  DeclareDouble("event_rate_limit_hz", 5.0);

  DeclareDouble("discovery_period_s", 5.0);
  DeclareDouble("report_period_s", 10.0);
  declare_parameter<bool>("warn_on_missing_topics", true);
}

bool Cognitive3DNode::ReadInt(const std::string& name, std::int64_t min, std::int64_t max,
                              std::int64_t* out, std::string* error) {
  const std::int64_t value = get_parameter(name).as_int();
  if (value < min || value > max) {
    if (error) {
      *error = name + " must be a whole number from " + std::to_string(min) + " to " +
               std::to_string(max) + ", got " + std::to_string(value);
    }
    return false;
  }
  *out = value;
  return true;
}

bool Cognitive3DNode::ReadPositive(const std::string& name, double* out, std::string* error) {
  const double value = get_parameter(name).as_double();
  if (!std::isfinite(value) || value <= 0.0) {
    if (error) *error = name + " must be a positive number, got " + std::to_string(value);
    return false;
  }
  *out = value;
  return true;
}

bool Cognitive3DNode::ReadInRange(const std::string& name, double min, double max,
                                  const std::string& range, double* out, std::string* error) {
  const double value = get_parameter(name).as_double();
  if (!std::isfinite(value) || value < min || value > max) {
    if (error) {
      std::ostringstream got;
      got << value;
      *error = name + " must be " + range + ", got " + got.str();
    }
    return false;
  }
  *out = value;
  return true;
}

bool Cognitive3DNode::LoadConfig(std::string* error) {
  // Found among the overrides, since the name is never declared.
  if (get_node_parameters_interface()->get_parameter_overrides().count(
          kApplicationKeyParameter) != 0) {
    if (error) *error = kApplicationKeyParameterRefused;
    return false;
  }
  for (const char* removed : kRemovedFlushKeys) {
    if (get_node_parameters_interface()->get_parameter_overrides().count(removed) != 0) {
      if (error) *error = std::string(removed) + kRemovedFlushKeyWhy;
      return false;
    }
  }
  // Mandatory values the node resolves itself. They are reported together with
  // `Config::Validate`'s own, so one refusal names everything a fresh params file is missing.
  std::vector<std::string> missing;

  // The name follows the family the platform uses -- application, developer, organization --
  // which this repo's tooling reads under the same prefix.
  const char* key_from_env = std::getenv("C3D_APPLICATION_API_KEY");
  c3d_config_.application_key = key_from_env != nullptr ? key_from_env : "";
  if (c3d_config_.application_key.empty()) missing.push_back(kApplicationKeyMissing);
  c3d_config_.scene_id = get_parameter("scene_id").as_string();
  // An empty one is Config::Validate's to name.
  if (!c3d_config_.scene_id.empty() && !IsSceneId(c3d_config_.scene_id)) {
    missing.push_back(SceneIdRefusal(c3d_config_.scene_id));
  }
  c3d_config_.scene_version = get_parameter("scene_version").as_string();
  c3d_config_.device_id = get_parameter("device_id").as_string();
  // Both hosts are ONE decision, made from the process environment: unset is production, because
  // every client ships prod and must not need a variable at all, and anything unrecognised is
  // FATAL rather than falling through to production.
  if (!ResolveDestination([](const char* name) { return std::getenv(name); }, &destination_,
                          error)) {
    return false;
  }
  c3d_config_.environment = destination_.environment;
  const c3d::Hosts& hosts = destination_.hosts;
  // An explicit gateway still wins -- loopback capture depends on it -- but it moves ONLY the
  // data host. The api host always comes from the environment, so a params file, which is not a
  // trusted input, can never choose where an organisation key is sent.
  const std::string gateway_override = get_parameter("gateway").as_string();
  // An override naming ANOTHER environment's data host is never a capture stub -- it is the
  // cross-environment split itself, spelled as a params value. It is refused rather than warned
  // about because it reads as an ordinary value: `gateway: "data.cognitive3d.com"` looks like a
  // default, and beside a custom host pair it would silently split the run across two stacks.
  // Folded like the http check below: a port or capitals still reach that host.
  const std::string override_owner =
      c3d::EnvironmentOwningDataHost(GatewayHost(gateway_override));
  if (!override_owner.empty() && override_owner != c3d_config_.environment) {
    if (error) {
      *error = "gateway '" + gateway_override + "' is the " + override_owner +
               " data host, while this process's environment resolves " +
               DescribeDestination(destination_, "") +
               ". Telemetry would go to one environment while metadata is read from the other. "
               "Leave `gateway` empty to follow the environment, or override it only to a "
               "capture stub.";
    }
    return false;
  }
  c3d_config_.gateway = gateway_override.empty() ? hosts.data : gateway_override;
  c3d_config_.protocol = get_parameter("protocol").as_string();
  if (c3d_config_.protocol != "https" && c3d_config_.protocol != "http") {
    if (error) {
      *error = "protocol must be 'https', or 'http' beside a `gateway` naming a capture stub; "
               "got '" + c3d_config_.protocol + "'";
    }
    return false;
  }
  // Compared by host however it is spelled: a port, capitals or a trailing dot still reach the
  // same server.
  if (c3d_config_.protocol == "http" &&
      (gateway_override.empty() || NamesACognitive3DHost(gateway_override, hosts.data))) {
    if (error) {
      *error = "protocol 'http' sends the application key unencrypted, so it is accepted only with "
               "`gateway` set to a capture stub, never to a Cognitive3D host (gateway is '" +
               gateway_override + "'). Use 'https', or point `gateway` at the stub.";
    }
    return false;
  }
  c3d_config_.spool_dir = get_parameter("spool_dir").as_string();
  if (c3d_config_.spool_dir.empty()) {
    c3d_config_.spool_dir =
        DefaultSpoolDirectory([](const char* name) { return std::getenv(name); });
    if (c3d_config_.spool_dir.empty()) {
      if (error) {
        *error = "spool_dir is unset, and neither XDG_STATE_HOME nor HOME names an absolute "
                 "directory to put the default in; set spool_dir";
      }
      return false;
    }
  }
  // Range-checked before the multiplication: a negative value cast to unsigned is an unbounded
  // spool.
  std::int64_t spool_max_mb = 0;
  if (!ReadInt("spool_max_mb", 1, std::int64_t{1} << 30, &spool_max_mb, error)) return false;
  c3d_config_.spool_max_bytes = static_cast<std::uint64_t>(spool_max_mb) * 1024ull * 1024ull;
  c3d_config_.marker_heartbeat_s = get_parameter("marker_heartbeat_s").as_double();
  c3d_config_.orphan_stale_after_s = get_parameter("orphan_stale_after_s").as_double();
  c3d_config_.uploads_paused = get_parameter("uploads_paused").as_bool();
  // Range-checked here, before the narrowing: an int64 parameter wraps into an int
  // silently, and a wrapped value can land back inside the range Validate() accepts.
  for (const char* name : {"http_connect_timeout_s", "http_low_speed_limit_bytes_per_s",
                           "http_low_speed_time_s", "http_timeout_s"}) {
    const std::int64_t value = get_parameter(name).as_int();
    if (value <= 0 || value > std::numeric_limits<int>::max()) {
      if (error) {
        *error = std::string(name) + " must be a positive whole number, got " +
                 std::to_string(value) + "; 0 would mean no bound at all, not a strict one";
      }
      return false;
    }
  }
  c3d_config_.http_connect_timeout_s =
      static_cast<int>(get_parameter("http_connect_timeout_s").as_int());
  c3d_config_.http_low_speed_limit_bytes_per_s =
      static_cast<int>(get_parameter("http_low_speed_limit_bytes_per_s").as_int());
  c3d_config_.http_low_speed_time_s =
      static_cast<int>(get_parameter("http_low_speed_time_s").as_int());
  c3d_config_.http_timeout_s = static_cast<int>(get_parameter("http_timeout_s").as_int());
  c3d_config_.retry_min_s = get_parameter("retry_min_s").as_double();
  c3d_config_.retry_max_s = get_parameter("retry_max_s").as_double();
  if (c3d_config_.device_id.empty()) {
    // A stable id matters more than a pretty one; the operator should override.
    c3d_config_.device_id = DefaultDeviceId(get_namespace());
  }
  // The identifier the remote-variables draw is keyed on whenever no participant_id is supplied --
  // which is every auto-started session. The service refuses an over-long participant_id; this is
  // the same bound on the other way in, and it has to be at configure because nothing downstream
  // of here can refuse a parameter.
  if (c3d_config_.device_id.size() > c3d::kParticipantIdMax) {
    if (error) {
      *error = "device_id is longer than " + std::to_string(c3d::kParticipantIdMax) +
               " bytes: the platform would draw arms for the full string while the session "
               "records only the first " + std::to_string(c3d::kParticipantIdMax);
    }
    return false;
  }

  // Refused rather than defaulted, and refused whatever `robot_dynamic_object` says: an empty id
  // has no meaning under either value, and validity that moved with another parameter would be the
  // worse surface. A silent fallback is this repo's commonest defect shape: the parameter reads
  // empty, the wire carries a default, and the cross-system contract with the uploaded object
  // breaks with nothing said -- a templated params file rendered with the variable unset is the
  // realistic route in.
  robot_object_id_ = get_parameter("robot_object_id").as_string();
  if (robot_object_id_.empty()) {
    if (error) {
      *error =
          "robot_object_id must not be empty. It names the dynamic object -- the id every dynamics "
          "sample carries, and the id the uploaded dashboard object must be registered under -- and "
          "an empty name is refused whether or not robot_dynamic_object is on. Leave it unset for "
          "the default 'robot', or name the id you uploaded.";
    }
    return false;
  }

  const std::string convention_text = get_parameter("pose_convention").as_string();
  if (convention_text.empty()) {
    missing.push_back(kPoseConventionMissing);
  } else if (!ParsePoseConvention(convention_text, &pose_convention_)) {
    if (error) {
      *error = "pose_convention must be 'unity', 'gltf_authored' or 'rep103', got '" +
               convention_text + "'";
    }
    return false;
  }
  dry_run_ = get_parameter("dry_run").as_bool();
  use_sim_time_ = get_parameter("use_sim_time").as_bool();
  sim_time_on_wire_ = get_parameter("sim_time_on_wire").as_bool();
  if (sim_time_on_wire_ && !use_sim_time_) {
    if (error) {
      *error = "sim_time_on_wire is true but use_sim_time is false: the flag advances the wire "
               "with a simulator's /clock and means nothing on a real clock. Set use_sim_time "
               "too, or leave sim_time_on_wire unset.";
    }
    return false;
  }
  auto_start_session_ = get_parameter("auto_start_session").as_bool();
  remote_variables_ = get_parameter("remote_variables").as_bool();
  {
    const rclcpp::Parameter entries = get_parameter("session_properties");
    const std::string key = SdkKeyInSessionProperties(entries.as_string_array(), remote_variables_);
    if (!key.empty()) {
      if (error) *error = SdkKeyRefusal("session_properties", key);
      return false;
    }
  }
  remote_variables_timeout_s_ = get_parameter("remote_variables_timeout_s").as_double();
  // Refused whatever `remote_variables` says: validity that moved with another parameter would
  // let a file that is wrong configure until the day the fetch is switched on.
  if (!(remote_variables_timeout_s_ > 0.0)) {
    if (error) {
      *error = "remote_variables_timeout_s must be a positive number of seconds, got " +
               std::to_string(remote_variables_timeout_s_) +
               ". It is the fetch's timeout only; set remote_variables: false to skip the fetch.";
    }
    return false;
  }

  // Each timer's period between a millisecond and a day: rclcpp refuses one that does not fit in
  // nanoseconds, and a refusal at activation would come after the session had opened.
  if (!ReadInRange("sample_rate_hz", 1.0 / kMaxTimerPeriodS, 1.0 / kMinTimerPeriodS,
                   "a rate from one sample a day (1/86400) to 1000 Hz", &sampler_config_.rate_hz,
                   error)) {
    return false;
  }
  // Advertised in every gaze envelope as the time one sample stands for.
  c3d_config_.gaze_interval_s = 1.0 / sampler_config_.rate_hz;
  sampler_config_.stale_transform_after_s = StaleTransformAfterSeconds(sampler_config_.rate_hz);
  sampler_config_.preferred_reference_frame = get_parameter("reference_frame").as_string();
  sampler_config_.fallback_reference_frame =
      get_parameter("fallback_reference_frame").as_string();
  // 0 adopts the fallback as soon as it resolves; an infinite grace would record nothing for good.
  sampler_config_.reference_frame_grace_s =
      get_parameter("reference_frame_grace_s").as_double();
  if (!std::isfinite(sampler_config_.reference_frame_grace_s) ||
      sampler_config_.reference_frame_grace_s < 0.0) {
    if (error) {
      *error = "reference_frame_grace_s must be a number of seconds, 0 or more, got " +
               std::to_string(sampler_config_.reference_frame_grace_s);
    }
    return false;
  }
  // At or below 0, every sample that moves at all is a jump.
  if (!ReadPositive("relocalisation_jump_m", &sampler_config_.relocalisation_jump_m, error)) {
    return false;
  }
  if (!ReadPositive("relocalisation_jump_deg", &sampler_config_.relocalisation_jump_deg, error)) {
    return false;
  }
  sampler_config_.robot_frame = get_parameter("robot_frame").as_string();
  sampler_config_.gaze_frame = get_parameter("gaze_frame").as_string();
  const std::string gaze_convention_text = get_parameter("gaze_frame_convention").as_string();
  if (!ParseFrameConvention(gaze_convention_text, &sampler_config_.gaze_frame_convention)) {
    if (error) {
      *error = "gaze_frame_convention must be 'auto', 'body_forward_x' or "
               "'optical_forward_z', got '" + gaze_convention_text + "'";
    }
    return false;
  }
  sampler_config_.robot_display_name = get_parameter("robot_display_name").as_string();
  sampler_config_.robot_mesh = get_parameter("robot_mesh").as_string();
  sampler_config_.mesh_yaw_offset_deg = get_parameter("mesh_yaw_offset_deg").as_double();

  // A zero period is a timer that fires on every spin, and rclcpp refuses a negative one.
  const std::string period_range = "a number of seconds from 0.001 to 86400 (a day)";
  if (!ReadInRange("discovery_period_s", kMinTimerPeriodS, kMaxTimerPeriodS, period_range,
                   &discovery_period_s_, error) ||
      !ReadInRange("report_period_s", kMinTimerPeriodS, kMaxTimerPeriodS, period_range,
                   &report_period_s_, error)) {
    return false;
  }
  warn_on_missing_topics_ = get_parameter("warn_on_missing_topics").as_bool();
  action_discovery_ = get_parameter("action_discovery").as_bool();
  action_names_ = get_parameter("action_names").as_string_array();
  auto_subscribe_standard_types_ = get_parameter("auto_subscribe_standard_types").as_bool();
  const double event_rate_limit_hz = get_parameter("event_rate_limit_hz").as_double();
  if (!std::isfinite(event_rate_limit_hz) || event_rate_limit_hz < 0.0) {
    if (error) {
      *error = "event_rate_limit_hz must be 0 (no cap) or a positive number, got " +
               std::to_string(event_rate_limit_hz);
    }
    return false;
  }
  event_limiter_.set_limit_hz(event_rate_limit_hz);
  event_limiter_.set_clock(WallEpochSeconds);

  // The graph is compared by exact string, so a relative entry is resolved here the way
  // `ros2 topic` would resolve it for this node; unresolved, it would match nothing. Its
  // topic_config key stays the entry as written.
  const std::string node_fqn = get_node_base_interface()->get_fully_qualified_name();
  const std::vector<std::string> entries = get_parameter("topics").as_string_array();
  allowlist_.clear();
  plan_ = TopicPlan();
  // Every entry's group is declared before any value is read, so a key no group declares is named
  // ahead of a refusal of the default it left in force.
  std::set<std::string> keys_listed;
  for (const auto& entry : entries) {
    const std::string topic = ExpandTopicName(entry, get_namespace(), node_fqn);
    if (topic.empty() && !entry.empty()) {
      if (error) *error = UnresolvableTopicName("topics entry", entry);
      return false;
    }
    allowlist_.push_back(topic);
    // A '*' is a wildcard ONLY as the last character: TopicMatches compares
    // exactly when the pattern does not end in one. And the missing-topic check
    // skips any rule containing a '*' anywhere. So "/robot*/odom" subscribes to
    // nothing, is never reported missing, and STILL appears in the plan dump
    // with an ingest policy and a rate -- which reads as confirmation that it is
    // wired up. Refusing configure is the same trade this SDK makes for an empty
    // scene_id: a named failure now beats a silent one for the life of the run.
    const std::string::size_type star = topic.find('*');
    if (star != std::string::npos && star != topic.size() - 1) {
      if (error) {
        *error = "topic '" + topic +
                 "' has a '*' before the end; a wildcard is only supported as a "
                 "trailing suffix (e.g. '/robot1/*'), and this entry would match "
                 "nothing while appearing configured";
      }
      return false;
    }

    const std::string name = SanitizeForParameter(entry);
    keys_listed.insert(name);
    const std::string key = "topic_config." + name;
    // Per-topic overrides are declared lazily, because the set of topics is
    // configuration rather than something we can know at construction time.
    if (!has_parameter(key + ".ingest")) declare_parameter<std::string>(key + ".ingest", "");
    DeclareDouble(key + ".rate_hz", -1.0);
    if (!has_parameter(key + ".series")) declare_parameter<std::string>(key + ".series", "");
    if (!has_parameter(key + ".qos.reliability"))
      declare_parameter<std::string>(key + ".qos.reliability", "");
    if (!has_parameter(key + ".qos.durability"))
      declare_parameter<std::string>(key + ".qos.durability", "");
    if (!has_parameter(key + ".qos.history"))
      declare_parameter<std::string>(key + ".qos.history", "");
    if (!has_parameter(key + ".qos.depth")) declare_parameter<int>(key + ".qos.depth", -1);
  }
  const std::string unread = UnreadBlockKey(kTopicConfigBlock, "topics", keys_listed);
  if (!unread.empty()) {
    if (error) *error = unread;
    return false;
  }

  for (std::size_t i = 0; i < entries.size(); ++i) {
    const std::string& topic = allowlist_[i];
    const bool wildcard = topic.find('*') != std::string::npos;
    TopicRule rule;
    rule.topic = topic;
    const std::string key = "topic_config." + SanitizeForParameter(entries[i]);
    rule.config_key = key;

    const std::string ingest_text = get_parameter(key + ".ingest").as_string();
    if (!ingest_text.empty()) {
      if (!ParseIngest(ingest_text, &rule.ingest)) {
        if (error) *error = "topic " + topic + " has unknown ingest class '" + ingest_text + "'";
        return false;
      }
    } else {
      rule.ingest_from_type = true;
    }
    const double rate = get_parameter(key + ".rate_hz").as_double();
    if (rate > 0.0) {
      rule.rate_hz = rate;
    } else {
      rule.rate_from_type = true;
    }
    rule.series = get_parameter(key + ".series").as_string();
    // `Resolve` refuses to inherit a series name through a wildcard, because one
    // name for a whole namespace interleaves unrelated sensors into a single
    // channel. Say so here rather than dropping it silently: the operator wrote
    // a name and no topic will use it.
    if (wildcard && !rule.series.empty()) {
      RCLCPP_WARN(get_logger(),
                  "topic_config.%s.series ('%s') is set on the WILDCARD pattern '%s' and is "
                  "ignored: a series name names one channel, and every topic under the pattern "
                  "would record into it. Name the series on each topic's own entry.",
                  SanitizeForParameter(topic).c_str(), rule.series.c_str(), topic.c_str());
      rule.series.clear();
    }

    rule.qos.reliability = get_parameter(key + ".qos.reliability").as_string();
    rule.qos.durability = get_parameter(key + ".qos.durability").as_string();
    rule.qos.history = get_parameter(key + ".qos.history").as_string();
    std::int64_t depth = 0;
    if (!ReadInt(key + ".qos.depth", -1, std::numeric_limits<int>::max(), &depth, error)) {
      return false;
    }
    rule.qos.depth = static_cast<int>(depth);
    const std::string qos_problem = ValidateQosOverride(rule.qos, key);
    if (!qos_problem.empty()) {
      if (error) *error = qos_problem;
      return false;
    }

    plan_.AddRule(rule);
  }

  if (!LoadMediaConfig(error)) return false;
  if (!LoadSensorConfig(error)) return false;

  // Validated with a stand-in for a key that is missing, because that problem is already in
  // `missing` under the name of the variable that supplies it.
  c3d::Config validated = c3d_config_;
  if (validated.application_key.empty()) validated.application_key = "-";
  const std::string problem = JoinConfigurationProblems(missing, validated.Validate());
  if (!problem.empty()) {
    if (error) *error = problem;
    return false;
  }
  return true;
}

bool Cognitive3DNode::LoadMediaConfig(std::string* error) {
  ffmpeg_path_ = get_parameter("ffmpeg_path").as_string();
  camera_configs_.clear();
  lidar_configs_.clear();
  cameras_.clear();
  lidars_.clear();
  media_type_refusals_.clear();

  // Every listed block is declared before any value is read, so a key no block declares is named
  // ahead of a refusal of the default it left in force.
  const auto camera_names = get_parameter("cameras").as_string_array();
  std::set<std::string> cameras_listed;
  for (const std::string& name : camera_names) {
    if (!UsableSensorName(name)) {
      if (error) {
        *error = "cameras contains '" + name +
                 "': a camera name must be non-empty and carry no '.' or '/', because it is "
                 "both a parameter key (camera.<name>.topic) and the cameraId on the wire";
      }
      return false;
    }
    if (!cameras_listed.insert(name).second) {
      if (error) *error = "cameras names '" + name + "' twice";
      return false;
    }
    const std::string key = "camera." + name;
    if (!has_parameter(key + ".topic")) declare_parameter<std::string>(key + ".topic", "");
    DeclareDouble(key + ".rate_hz", 30.0);
    // Declared from the struct so the two sets of defaults cannot drift apart.
    const CameraConfig defaults;
    if (!has_parameter(key + ".preset"))
      declare_parameter<std::string>(key + ".preset", defaults.preset);
    if (!has_parameter(key + ".crf")) declare_parameter<int>(key + ".crf", defaults.crf);
    if (!has_parameter(key + ".max_bitrate_kbps"))
      declare_parameter<int>(key + ".max_bitrate_kbps", defaults.max_bitrate_kbps);
    if (!has_parameter(key + ".replay_max_mb"))
      declare_parameter<int>(key + ".replay_max_mb", defaults.replay_max_mb);
    if (!has_parameter(key + ".max_width_px")) declare_parameter<int>(key + ".max_width_px", 640);
    if (!has_parameter(key + ".frame")) declare_parameter<std::string>(key + ".frame", "");
    DeclareDouble(key + ".hfov_degrees", 0.0);
    DeclareDouble(key + ".vfov_degrees", 0.0);
    if (!has_parameter(key + ".camera_info_topic"))
      declare_parameter<std::string>(key + ".camera_info_topic", kCameraInfoTopicAuto);
    if (!has_parameter(key + ".qos.reliability"))
      declare_parameter<std::string>(key + ".qos.reliability", "");
    if (!has_parameter(key + ".qos.durability"))
      declare_parameter<std::string>(key + ".qos.durability", "");
    if (!has_parameter(key + ".qos.history"))
      declare_parameter<std::string>(key + ".qos.history", "");
    if (!has_parameter(key + ".qos.depth")) declare_parameter<int>(key + ".qos.depth", -1);
  }

  const auto lidar_names = get_parameter("lidars").as_string_array();
  std::set<std::string> lidars_listed;
  for (const std::string& name : lidar_names) {
    if (!UsableSensorName(name)) {
      if (error) {
        *error = "lidars contains '" + name +
                 "': a lidar name must be non-empty and carry no '.' or '/', because it is both "
                 "a parameter key (lidar.<name>.topic) and the sensorId on the wire";
      }
      return false;
    }
    if (!lidars_listed.insert(name).second) {
      if (error) *error = "lidars names '" + name + "' twice";
      return false;
    }
    const std::string key = "lidar." + name;
    const c3d::PointCloudOptions defaults;
    if (!has_parameter(key + ".topic")) declare_parameter<std::string>(key + ".topic", "");
    DeclareDouble(key + ".rate_hz", 30.0);
    if (!has_parameter(key + ".frame")) declare_parameter<std::string>(key + ".frame", "");
    if (!has_parameter(key + ".intensities")) declare_parameter<bool>(key + ".intensities", false);
    DeclareDouble(key + ".pointcloud.voxel_leaf_m", defaults.voxel_leaf_m);
    if (!has_parameter(key + ".pointcloud.position"))
      declare_parameter<std::string>(key + ".pointcloud.position",
                                     c3d::PointPositionName(defaults.position));
    if (!has_parameter(key + ".pointcloud.color"))
      declare_parameter<std::string>(key + ".pointcloud.color",
                                     c3d::PointColorName(defaults.color));
    if (!has_parameter(key + ".pointcloud.intensity"))
      declare_parameter<std::string>(key + ".pointcloud.intensity",
                                     c3d::PointIntensityName(defaults.intensity));
    if (!has_parameter(key + ".pointcloud.max_points"))
      declare_parameter<int>(key + ".pointcloud.max_points",
                             static_cast<int>(defaults.max_points));
    if (!has_parameter(key + ".pointcloud.zstd_level"))
      declare_parameter<int>(key + ".pointcloud.zstd_level", defaults.zstd_level);
  }

  std::string unread = UnreadBlockKey("camera", "cameras", cameras_listed);
  if (unread.empty()) unread = UnreadBlockKey("lidar", "lidars", lidars_listed);
  if (!unread.empty()) {
    if (error) *error = unread;
    return false;
  }

  for (const std::string& name : camera_names) {
    const std::string key = "camera." + name;
    CameraConfig camera;
    camera.name = name;
    const std::string camera_topic = get_parameter(key + ".topic").as_string();
    camera.topic = ExpandTopicName(camera_topic, get_namespace(),
                                   get_node_base_interface()->get_fully_qualified_name());
    if (camera.topic.empty() && !camera_topic.empty()) {
      if (error) *error = UnresolvableTopicName(key + ".topic", camera_topic);
      return false;
    }
    camera.rate_hz = get_parameter(key + ".rate_hz").as_double();
    camera.preset = get_parameter(key + ".preset").as_string();
    // Narrowed only once in range: an int64 wraps into an int silently, and a wrapped value can
    // land back inside the range ValidateCameraConfig accepts.
    std::int64_t narrowed = 0;
    if (!ReadInt(key + ".crf", kMinCameraCrf, kMaxCameraCrf, &narrowed, error)) return false;
    camera.crf = static_cast<int>(narrowed);
    if (!ReadInt(key + ".max_bitrate_kbps", 0, kMaxCameraBitrateKbps, &narrowed, error)) {
      return false;
    }
    camera.max_bitrate_kbps = static_cast<int>(narrowed);
    if (!ReadInt(key + ".replay_max_mb", 1, kMaxCameraReplayMb, &narrowed, error)) return false;
    camera.replay_max_mb = static_cast<int>(narrowed);
    if (!ReadInt(key + ".max_width_px", 0, std::numeric_limits<int>::max(), &narrowed, error)) {
      return false;
    }
    camera.max_width_px = static_cast<int>(narrowed);
    camera.frame = get_parameter(key + ".frame").as_string();
    camera.hfov_degrees = get_parameter(key + ".hfov_degrees").as_double();
    camera.vfov_degrees = get_parameter(key + ".vfov_degrees").as_double();
    // "auto" is the image topic's sibling, empty is none, anything else is a topic name,
    // expanded as the image topic is.
    const std::string info_topic = get_parameter(key + ".camera_info_topic").as_string();
    if (info_topic == kCameraInfoTopicAuto) {
      camera.camera_info_topic = camera.topic.empty() ? "" : SiblingCameraInfoTopic(camera.topic);
    } else if (!info_topic.empty()) {
      camera.camera_info_topic = ExpandTopicName(
          info_topic, get_namespace(), get_node_base_interface()->get_fully_qualified_name());
      if (camera.camera_info_topic.empty()) {
        if (error) *error = UnresolvableTopicName(key + ".camera_info_topic", info_topic);
        return false;
      }
    }
    camera.qos.reliability = get_parameter(key + ".qos.reliability").as_string();
    camera.qos.durability = get_parameter(key + ".qos.durability").as_string();
    camera.qos.history = get_parameter(key + ".qos.history").as_string();
    if (!ReadInt(key + ".qos.depth", -1, std::numeric_limits<int>::max(), &narrowed, error)) {
      return false;
    }
    camera.qos.depth = static_cast<int>(narrowed);

    const std::string problem = ValidateCameraConfig(camera);
    if (!problem.empty()) {
      if (error) *error = problem;
      return false;
    }
    camera_configs_.push_back(camera);
  }

  for (const std::string& name : lidar_names) {
    const std::string key = "lidar." + name;
    LidarConfig lidar;
    lidar.name = name;
    const std::string lidar_topic = get_parameter(key + ".topic").as_string();
    lidar.topic = ExpandTopicName(lidar_topic, get_namespace(),
                                  get_node_base_interface()->get_fully_qualified_name());
    if (lidar.topic.empty() && !lidar_topic.empty()) {
      if (error) *error = UnresolvableTopicName(key + ".topic", lidar_topic);
      return false;
    }
    lidar.rate_hz = get_parameter(key + ".rate_hz").as_double();
    lidar.frame = get_parameter(key + ".frame").as_string();
    lidar.intensities = get_parameter(key + ".intensities").as_bool();
    lidar.pointcloud.voxel_leaf_m = get_parameter(key + ".pointcloud.voxel_leaf_m").as_double();
    // A negative count cast to size_t is no cap at all.
    std::int64_t max_points = 0;
    if (!ReadInt(key + ".pointcloud.max_points", 1, std::numeric_limits<std::int64_t>::max(),
                 &max_points, error)) {
      return false;
    }
    lidar.pointcloud.max_points = static_cast<std::size_t>(max_points);
    std::int64_t zstd_level = 0;
    if (!ReadInt(key + ".pointcloud.zstd_level", 0, 22, &zstd_level, error)) return false;
    lidar.pointcloud.zstd_level = static_cast<int>(zstd_level);

    // The three enum-like values. Each is a lowercase constant the gateway
    // validates, so an unknown one here would be a 400 per part for the life of
    // the run -- and the run would look configured.
    const std::string position_text = get_parameter(key + ".pointcloud.position").as_string();
    if (!c3d::ParsePointPosition(position_text, &lidar.pointcloud.position)) {
      if (error) {
        *error = key + ".pointcloud.position must be 'int16_mm' or 'float32', got '" +
                 position_text + "'";
      }
      return false;
    }
    const std::string color_text = get_parameter(key + ".pointcloud.color").as_string();
    if (!c3d::ParsePointColor(color_text, &lidar.pointcloud.color)) {
      if (error) {
        *error = key + ".pointcloud.color must be 'none', 'rgb565', 'rgb332' or 'rgb888', got '" +
                 color_text + "'";
      }
      return false;
    }
    const std::string intensity_text = get_parameter(key + ".pointcloud.intensity").as_string();
    if (!c3d::ParsePointIntensity(intensity_text, &lidar.pointcloud.intensity)) {
      if (error) {
        *error = key + ".pointcloud.intensity must be 'none' or 'uint8', got '" +
                 intensity_text + "'";
      }
      return false;
    }

    const std::string problem = ValidateLidarConfig(lidar);
    if (!problem.empty()) {
      if (error) *error = problem;
      return false;
    }
    lidar_configs_.push_back(lidar);
  }

  // Asked once at configure rather than discovered at the first frame: without a usable encoder
  // every frame is dropped while the run looks configured.
  if (!camera_configs_.empty()) {
    std::string version;
    const std::string unusable = ProbeCameraEncoders(ffmpeg_path_, camera_configs_, &version);
    if (!unusable.empty()) {
      if (error) *error = FfmpegProblem(ffmpeg_path_, unusable) + ". " + kFfmpegRemedy;
      return false;
    }
  }

  for (const auto& camera : camera_configs_) {
    cameras_.push_back(std::make_unique<CameraCapture>(camera, ffmpeg_path_));
  }
  for (const auto& lidar : lidar_configs_) {
    lidars_.push_back(std::make_unique<LidarCapture>(lidar));
  }

  return true;
}

bool Cognitive3DNode::LoadSensorConfig(std::string* error) {
  sensor_configs_.clear();
  const auto names = get_parameter("sensors").as_string_array();
  std::set<std::string> seen;
  for (const std::string& name : names) {
    if (!UsableSensorName(name)) {
      if (error) {
        *error = "sensors contains '" + name +
                 "': a sensor name must be non-empty and carry no '.' or '/', because it is both "
                 "a parameter key (sensor.<name>.kind) and a segment of its session properties";
      }
      return false;
    }
    if (!seen.insert(name).second) {
      if (error) *error = "sensors names '" + name + "' twice";
      return false;
    }
    const std::string key = "sensor." + name;
    for (const char* field : {".kind", ".topic", ".frame", ".mount_parent", ".units",
                              ".meaning_0", ".meaning_1"}) {
      if (!has_parameter(key + field)) declare_parameter<std::string>(key + field, "");
    }
    if (!has_parameter(key + ".mount_measured"))
      declare_parameter<bool>(key + ".mount_measured", false);
    DeclareDouble(key + ".cone_half_angle_deg", 0.0);
    DeclareDouble(key + ".range_min_m", 0.0);
    DeclareDouble(key + ".range_max_m", 0.0);
    DeclareDoubleArray(key + ".mount_xyz_m");
    DeclareDouble(key + ".mount_yaw_deg", 0.0);
    if (!has_parameter(key + ".element_count")) declare_parameter<int>(key + ".element_count", 0);
  }
  const std::string unread = UnreadBlockKey("sensor", "sensors", seen);
  if (!unread.empty()) {
    if (error) *error = unread;
    return false;
  }
  for (const std::string& name : names) {
    const std::string key = "sensor." + name;
    SensorConfig sensor;
    sensor.name = name;
    sensor.kind = get_parameter(key + ".kind").as_string();
    const std::string topic = get_parameter(key + ".topic").as_string();
    sensor.topic = ExpandTopicName(topic, get_namespace(),
                                   get_node_base_interface()->get_fully_qualified_name());
    if (sensor.topic.empty() && !topic.empty()) {
      if (error) *error = UnresolvableTopicName(key + ".topic", topic);
      return false;
    }
    sensor.frame = get_parameter(key + ".frame").as_string();
    sensor.mount_parent = get_parameter(key + ".mount_parent").as_string();
    sensor.mount_measured = get_parameter(key + ".mount_measured").as_bool();
    sensor.cone_half_angle_deg = get_parameter(key + ".cone_half_angle_deg").as_double();
    sensor.units = get_parameter(key + ".units").as_string();
    sensor.range_min_m = get_parameter(key + ".range_min_m").as_double();
    sensor.range_max_m = get_parameter(key + ".range_max_m").as_double();
    sensor.mount_xyz_m = get_parameter(key + ".mount_xyz_m").as_double_array();
    sensor.mount_yaw_deg = get_parameter(key + ".mount_yaw_deg").as_double();
    // Past the translator's own cap, an element is never recorded, so its series would not exist.
    std::int64_t element_count = 0;
    if (!ReadInt(key + ".element_count", 0,
                 static_cast<std::int64_t>(Int32MultiArrayTranslator::kMaxElements),
                 &element_count, error)) {
      return false;
    }
    sensor.element_count = static_cast<int>(element_count);
    sensor.meaning_0 = get_parameter(key + ".meaning_0").as_string();
    sensor.meaning_1 = get_parameter(key + ".meaning_1").as_string();

    const std::string problem = ValidateSensorConfig(sensor);
    if (!problem.empty()) {
      if (error) *error = problem;
      return false;
    }
    // Only an allowlisted topic is recorded at all.
    if (!plan_.Allowed(sensor.topic)) {
      if (error) {
        *error = key + ".topic '" + sensor.topic + "' is not recorded: list it in `topics`, or "
                 "the description names a series that never reaches the session";
      }
      return false;
    }
    // Recorded either way; only the drawing depends on the name.
    if (sensor.kind == kConeRangefinder && name != kConeDrawnFor) {
      RCLCPP_INFO(get_logger(),
                  "sensor '%s' is a cone_rangefinder: its description is recorded, and Session "
                  "Replay draws a cone only for a cone_rangefinder named '%s', so it draws none "
                  "for this one. Name it %s in `sensors` and in its sensor.<name>.* keys to have "
                  "its cone drawn.",
                  name.c_str(), kConeDrawnFor, kConeDrawnFor);
    }
    // The series the topic's translator records, named by the translator's own rule: a cone
    // rangefinder is one Float32 value, a binary array one series per Int32MultiArray element.
    const bool cone = sensor.kind == kConeRangefinder;
    TranslationContext context;
    context.topic = sensor.topic;
    context.series = plan_.Resolve(sensor.topic, SensorMessageType(sensor.kind)).series;
    if (cone) {
      sensor.series.push_back(SeriesNameFor(context, "value"));
    } else {
      const std::string base = SeriesNameFor(context, "");
      for (int i = 0; i < sensor.element_count; ++i) {
        sensor.series.push_back(base + "." + std::to_string(i));
      }
    }
    sensor_configs_.push_back(sensor);
  }
  return true;
}

std::string Cognitive3DNode::UnreadBlockKey(const std::string& block, const std::string& list,
                                            const std::set<std::string>& listed) {
  const std::string prefix = block + ".";
  // The listed name a key is under: the longest the key continues past with a '.', because a
  // topic_config name is a topic made a key segment and may hold one. Empty for none.
  const auto owner = [&listed](const std::string& rest) {
    std::string found;
    for (const std::string& name : listed) {
      if (name.size() > found.size() && rest.compare(0, name.size(), name) == 0 &&
          (rest.size() == name.size() || rest[name.size()] == '.')) {
        found = name;
      }
    }
    return found;
  };
  const bool topic_config = block == kTopicConfigBlock;
  for (const auto& given : get_node_parameters_interface()->get_parameter_overrides()) {
    const std::string& key = given.first;
    if (key.rfind(prefix, 0) != 0) continue;
    const std::string rest = key.substr(prefix.size());
    const std::string name = owner(rest);
    if (name.empty()) {
      if (topic_config) return UnlistedTopicConfigKey(key, listed);
      const std::string unlisted = rest.substr(0, rest.find('.'));
      return key + " describes a " + block + " '" + unlisted + "' that `" + list +
             "` does not list: add '" + unlisted + "' to `" + list + "`, or remove every " +
             prefix + unlisted + ".* key";
    }
    if (has_parameter(key)) continue;
    if (block == "camera" && rest.size() > name.size()) {
      const std::string field = rest.substr(name.size() + 1);
      for (const char* removed : kRemovedCameraFields) {
        if (field == removed) return key + kRemovedCameraFieldWhy;
      }
    }
    const std::string own = prefix + name + ".";
    std::string fields;
    for (const std::string& declared : list_parameters({prefix + name}, 0).names) {
      if (declared.rfind(own, 0) != 0 || owner(declared.substr(prefix.size())) != name) continue;
      fields += (fields.empty() ? "" : ", ") + declared.substr(own.size());
    }
    return key + " is not a parameter: a " + (topic_config ? "topic_config entry" : block) +
           "'s fields are " + fields + ". Correct the spelling";
  }
  return {};
}

LifecycleNodeInterface::CallbackReturn Cognitive3DNode::on_configure(
    const rclcpp_lifecycle::State&) {
  std::string error;
  configure_error_.clear();
  if (!LoadConfig(&error)) {
    configure_error_ = error;
    // Failing configure is deliberate. A node that accepted an empty scene id
    // would warn once and then silently discard every payload for the life of
    // the process, so it refuses to leave `unconfigured` instead.
    RCLCPP_ERROR(get_logger(), "configuration rejected: %s", error.c_str());
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  // Opened before anything else this transition creates, so a spool that cannot be used -- held
  // by another process, a lock planted as a link, a directory this process cannot write -- fails
  // configure with nothing to undo, naming the directory and why. Otherwise every later session
  // start fails instead.
  std::shared_ptr<c3d::DiskSpool> spool;
  if (!dry_run_) {
    spool = std::make_shared<c3d::DiskSpool>(c3d_config_.spool_dir, c3d_config_.spool_max_bytes,
                                             c3d::DefaultLogger());
    const std::string problem = spool->error();
    if (!problem.empty()) {
      configure_error_ = problem;
      RCLCPP_ERROR(get_logger(), "configuration rejected: %s", problem.c_str());
      return LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
  }

  const bool sim_time = use_sim_time_;
  clock_policy_ = std::make_unique<ClockPolicy>(get_clock(), sim_time, sim_time_on_wire_);
  clock_policy_->OnBackwardJump([this](double delta) {
    RCLCPP_WARN(get_logger(),
                "ROS time jumped backwards by %.3fs (sim reset or bag loop). The "
                "current session must be closed and a new one opened.",
                -delta);
  });

  // ONE mutually exclusive group for every callback this node owns: the node's default group,
  // where rclcpp_lifecycle puts the transitions and rclcpp the parameter services. Under a
  // multi-threaded executor -- `component_container_mt` -- no two of them run at once, so the
  // state they share needs no lock. The tf listener's subscriptions keep a group and a thread of
  // their own, and the tf buffer locks itself.
  callback_group_ = get_node_base_interface()->get_default_callback_group();

  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "~/diagnostics", rclcpp::QoS(rclcpp::KeepLast(10)));

  // Created at configure, not at activate. An orchestrator that discovers the
  // service and calls too early gets a refusal that says what to do; a service
  // that does not exist yet gives it nothing to go on.
  start_session_srv_ = create_service<cognitive3d_ros_msgs::srv::StartSession>(
      "~/start_session",
      [this](const std::shared_ptr<cognitive3d_ros_msgs::srv::StartSession::Request> request,
             std::shared_ptr<cognitive3d_ros_msgs::srv::StartSession::Response> response) {
        OnStartSessionRequest(request, response);
      },
      ServiceQos(), callback_group_);
  end_session_srv_ = create_service<cognitive3d_ros_msgs::srv::EndSession>(
      "~/end_session",
      [this](const std::shared_ptr<cognitive3d_ros_msgs::srv::EndSession::Request> request,
             std::shared_ptr<cognitive3d_ros_msgs::srv::EndSession::Response> response) {
        OnEndSessionRequest(request, response);
      },
      ServiceQos(), callback_group_);

  // Named on every run, because a missing translator has no other symptom than a topic whose
  // report line reads `no-xlat` -- which is also what an allowlisted topic nobody wrote a
  // translator for looks like.
  {
    std::ostringstream types;
    const std::vector<std::string> known = translators_.known_types();
    for (std::size_t i = 0; i < known.size(); ++i) types << (i ? ", " : "") << known[i];
    RCLCPP_INFO(get_logger(), "translating %zu message type(s): %s", known.size(),
                types.str().c_str());
  }

  // tf2 owns the transform tree. The buffer lives from configure to cleanup; the listener
  // that fills it lives only while the node is active (see on_activate). It is filled on a thread
  // of its own, so this node's callbacks never starve tf of its transforms, and the buffer is
  // marked so, which Humble's tf2_ros requires before it answers the pose sampler's canTransform.
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,
                                             /*automatically_add_to_executor_with_node=*/false);
  pose_sampler_ = std::make_unique<PoseSampler>(tf_buffer_, sampler_config_);
  pose_sampler_->set_event_limiter(&event_limiter_);

  if (!sampler_config_.gaze_frame.empty()) {
    const FrameConvention resolved = pose_sampler_->resolved_gaze_frame_convention();
    // Say whether the convention was inferred, because an inference that is
    // wrong produces no error anywhere: the rotation is 90 degrees off, every
    // record is still well formed, and the warning below cannot fire. This log
    // line is the only signal.
    RCLCPP_INFO(get_logger(), "gaze pose is taken from '%s', %s convention (%s)",
                sampler_config_.gaze_frame.c_str(), FrameConventionName(resolved),
                sampler_config_.gaze_frame_convention == FrameConvention::kAuto
                    ? "inferred from the frame name; set gaze_frame_convention to override"
                    : "set explicitly");
    // Naming an optical frame and then declaring it body-style is a 90 degree
    // gaze error that the payload cannot report and the viewer renders without
    // complaint, so it is worth saying out loud even though it is a valid
    // configuration.
    if (FrameNameLooksOptical(sampler_config_.gaze_frame) &&
        resolved == FrameConvention::kBodyForwardX) {
      RCLCPP_WARN(get_logger(),
                  "gaze_frame '%s' is named like a REP-103 optical frame (+Z forward) "
                  "but gaze_frame_convention says body_forward_x; if that is wrong the "
                  "gaze rotation will be 90 degrees off with no error anywhere",
                  sampler_config_.gaze_frame.c_str());
    }
  }

  if (!dry_run_) {
    auto transport = std::make_shared<c3d::CurlTransport>(c3d::UploadTimeouts(c3d_config_));
    runner_ = std::make_unique<SessionRunner>(c3d_config_, pose_convention_, transport, spool,
                                              c3d::DefaultLogger());
    // Set right after construction: a later code path would let a dynamic
    // object's height silently default. test_session_control pins the wiring.
    runner_->set_dynamic_up_offset_meters(
        get_parameter("dynamic_up_offset_meters").as_double());
    runner_->set_robot_object_id(robot_object_id_);
    runner_->set_robot_dynamic_object(get_parameter("robot_dynamic_object").as_bool());
    const int recovered = runner_->RecoverOrphans();
    if (recovered > 0) {
      RCLCPP_WARN(get_logger(),
                  "recovered %d session(s) that ended without an orderly shutdown; "
                  "each has been closed with an abnormalEnd reason",
                  recovered);
    }
    // Said on every configure, because the two states look alike from outside: a
    // paused run and a run with a dead uplink both show a spool that only grows.
    if (c3d_config_.uploads_paused) {
      RCLCPP_WARN(get_logger(),
                  "uploads PAUSED (uploads_paused: true): recording and spooling to %s, "
                  "sending nothing. %zu chunk(s) already spooled. Drain later with a process "
                  "configured with uploads_paused: false, or `ros2 param set %s uploads_paused "
                  "false`.",
                  // Through the base interface: LifecycleNode, unlike Node, has no
                  // get_fully_qualified_name() of its own on Humble. A Jazzy build
                  // compiles either spelling, so only a Humble build catches this.
                  c3d_config_.spool_dir.c_str(), spool->Count(),
                  get_node_base_interface()->get_fully_qualified_name());
    } else {
      RCLCPP_INFO(get_logger(),
                  "uploads on: %zu chunk(s) spooled; connect timeout %ds, abandoned below %d "
                  "B/s for %ds, ceiling %ds per request, retry back-off %.0f-%.0fs",
                  spool->Count(), c3d_config_.http_connect_timeout_s,
                  c3d_config_.http_low_speed_limit_bytes_per_s,
                  c3d_config_.http_low_speed_time_s, c3d_config_.http_timeout_s,
                  c3d_config_.retry_min_s, c3d_config_.retry_max_s);
    }

    if (remote_variables_) {
      const int reader_timeout_s = ReaderTimeoutSeconds(remote_variables_timeout_s_);
      if (static_cast<double>(reader_timeout_s) != std::ceil(remote_variables_timeout_s_)) {
        RCLCPP_WARN(get_logger(),
                    "remote_variables_timeout_s %.1f is outside [1, 60] seconds and was clamped "
                    "to %d -- a value in the thousands is usually a milliseconds slip",
                    remote_variables_timeout_s_, reader_timeout_s);
      }
      auto reader = std::make_shared<c3d::CurlTransport>(reader_timeout_s);
      std::lock_guard<std::mutex> lock(request_state_mu_);
      remote_variables_reader_ = std::move(reader);
    } else {
      {
        std::lock_guard<std::mutex> lock(request_state_mu_);
        remote_variables_reader_.reset();
      }
      RCLCPP_INFO(get_logger(),
                  "remote_variables is false: remote variables will not be fetched and "
                  "sessions will record no c3d.remote_variable.*");
    }
  } else {
    RCLCPP_WARN(get_logger(),
                "dry_run is set: topics will be translated and counted but no session "
                "will be created and nothing will be uploaded.");
  }

  // Named on every run so an archived log answers "which environment was this, and did anything
  // override the gateway?" without inference. Both halves matter: an override moves the data host
  // only, so the two lines can legitimately disagree and a reader must be able to see that.
  RCLCPP_INFO(get_logger(), "environment %s",
              DescribeDestination(destination_, c3d_config_.gateway).c_str());

  RCLCPP_INFO(get_logger(),
              "configured. scene=%s version=%s device=%s spool=%s sim_time=%s "
              "robot_dynamic_object=%s",
              c3d_config_.scene_id.c_str(), c3d_config_.scene_version.c_str(),
              c3d_config_.device_id.c_str(), c3d_config_.spool_dir.c_str(),
              sim_time ? "true" : "false",
              get_parameter("robot_dynamic_object").as_bool() ? "true" : "false");
  if (sim_time_on_wire_) {
    RCLCPP_INFO(get_logger(),
                "use_sim_time and sim_time_on_wire are on. Cognitive3D timestamps start at the "
                "wall-clock session start and advance with ROS time, so the session lasts as "
                "long as the simulated run did.");
  } else if (sim_time) {
    RCLCPP_INFO(get_logger(),
                "use_sim_time is on. Cognitive3D timestamps are wall clock; ROS time only measures "
                "how old a message's stamp is, and never reaches the wire.");
  }
  RCLCPP_INFO(get_logger(),
              "pose convention on the wire: %s. %s",
              PoseConventionName(pose_convention_),
              pose_convention_ == PoseConvention::kRep103
                  ? "REP-103 native, matching what `ros2 topic echo` shows; the consumer "
                    "converts."
                  : pose_convention_ == PoseConvention::kGltfAuthored
                        ? "Converted for a glTF-authored scene: positions mirrored, and "
                          "the gaze and dynamics rotations use SEPARATE rules. Wrong here "
                          "is invisible in the payload -- check your exporter's axis map."
                        : "Converted to Unity handedness in this node.");
  LogPlanSummary();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void Cognitive3DNode::LogPlanSummary() const {
  if (allowlist_.empty()) {
    // An empty allowlist does not mean an empty session: a battery and a
    // diagnostics topic are taken by message type and an action server is
    // enrolled by name, neither needing an entry.
    // Cameras and lidars never needed one either -- they name their own topic.
    std::ostringstream still;
    if (auto_subscribe_standard_types_) still << ", battery and diagnostics by message type";
    if (action_discovery_) still << ", the action servers action_discovery enrols";
    if (!cameras_.empty() || !lidars_.empty()) {
      still << ", the cameras and lidars subscribed by name";
    }
    RCLCPP_WARN(get_logger(),
                "topic allowlist is empty: this session records the pose timeline, the node's "
                "own c3d.* series%s, and nothing else from this robot. Set the 'topics' "
                "parameter for its sensors.",
                still.str().c_str());
    return;
  }
  const std::string series_types = SeriesTypesText(translators_);
  std::ostringstream os;
  os << "allowlist (" << allowlist_.size() << " pattern(s)):";
  for (const auto& rule : plan_.rules()) {
    // The message type is not known until the topic is on the graph, so a value left to it is
    // named as such here and given at subscription.
    os << "\n  " << rule.topic << "  ingest=" << PlannedIngestText(rule) << "  rate=";
    if (rule.rate_from_type) {
      os << "(message type default)";
    } else {
      os << rule.rate_hz << "Hz (configured)";
    }
    // The resolved sensor-series name, printed WHETHER OR NOT one was set --
    // this is the only place a series written under another entry's key becomes
    // visible. That key is read for its own entry, this one's series silently
    // falls back to the topic-derived name, and nothing else in a run says so.
    // Samples still arrive, `doctor` still exits 0, and a consumer matching on
    // the declared name finds nothing.
    os << "  series=" << (rule.series.empty() ? "(from topic name)" : rule.series);
    // The type is not known yet either, and only some translators read the name.
    if (!rule.series.empty()) os << " (if the topic is " << series_types << ")";
  }
  RCLCPP_INFO(get_logger(), "%s", os.str().c_str());
}

LifecycleNodeInterface::CallbackReturn Cognitive3DNode::on_activate(
    const rclcpp_lifecycle::State&) {
  // Undone whole whenever a step refuses or throws, so the node is left inactive as it was, with
  // nothing subscribed and no session open. A throw left to rclcpp_lifecycle would be an error
  // transition to unconfigured with the subscriptions and the session still live.
  try {
    if (BringUp()) {
      if (!auto_start_session_) {
        RCLCPP_INFO(get_logger(),
                    "activated at %.1f Hz, with NO session open: auto_start_session is false, so "
                    "nothing is recorded until ~/start_session is called.",
                    sampler_config_.rate_hz);
      } else {
        RCLCPP_INFO(get_logger(), "activated. Sampling poses at %.1f Hz into session %s.",
                    sampler_config_.rate_hz,
                    runner_ && runner_->session() ? runner_->session()->session_id().c_str()
                                                  : "(dry run)");
      }
      active_ = true;
      return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
  } catch (const std::exception& thrown) {
    RCLCPP_ERROR(get_logger(), "activation failed: %s. Everything it started is undone, and the "
                 "node is inactive.", thrown.what());
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "activation failed on an unknown exception. Everything it started "
                 "is undone, and the node is inactive.");
  }
  BringDown("activation_failed");
  return LifecycleNodeInterface::CallbackReturn::FAILURE;
}

bool Cognitive3DNode::BringUp() {
  // Activation brings up ingest; whether it also OPENS a session is a
  // deployment choice. A standalone robot wants one session per activation and
  // gets it. An orchestrated one drives sessions through ~/start_session, and
  // for that a session opened here would be a session it did not name -- and
  // would make its own first call fail as "already running".
  if (auto_start_session_) {
    if (!StartSession()) return false;
    session_from_activation_ = true;
  }
  diagnostics_pub_->on_activate();
  // Created here and destroyed at deactivate, like every topic subscription, so an inactive node
  // receives no topic traffic at all. On a thread of its own, so tf ingestion cannot be starved by
  // this node's callbacks.
  StartTransformListener();
  // Reliable with a deep queue: a mission's events are low-rate and each one is a fact nothing
  // else records, so losing one to a full queue is not a trade-off worth making.
  event_sub_ = create_subscription<cognitive3d_ros_msgs::msg::Event>(
      "~/events", rclcpp::QoS(100).reliable(),
      [this](const cognitive3d_ros_msgs::msg::Event::SharedPtr message) { OnEvent(message); },
      SubscriptionOptionsInGroup());
  media_environment_ = MakeMediaEnvironment();
  DiscoverAndSubscribe();
  DiscoverAndSubscribeMedia();

  sample_timer_ = CreateActivationTimer(
      std::chrono::duration<double>(1.0 / sampler_config_.rate_hz), [this]() { OnSampleTimer(); });
  discovery_timer_ = CreateActivationTimer(std::chrono::duration<double>(discovery_period_s_),
                                           [this]() { OnDiscoveryTimer(); });
  report_timer_ = CreateActivationTimer(std::chrono::duration<double>(report_period_s_),
                                        [this]() { OnReportTimer(); });
  if (!cameras_.empty() || !lidars_.empty()) {
    media_hold_timer_ = CreateActivationTimer(std::chrono::milliseconds(10),
                                              [this]() { OnMediaHoldTimer(); });
  }
  return true;
}

rclcpp::TimerBase::SharedPtr Cognitive3DNode::CreateActivationTimer(
    std::chrono::duration<double> period, std::function<void()> callback) {
  return create_wall_timer(period, std::move(callback), callback_group_);
}

void Cognitive3DNode::BringDown(const std::string& reason) {
  sample_timer_.reset();
  discovery_timer_.reset();
  report_timer_.reset();
  media_hold_timer_.reset();
  TearDownSubscriptions();
  StopSession(reason);
  if (diagnostics_pub_) diagnostics_pub_->on_deactivate();
}

double Cognitive3DNode::WallClockNow() const { return WallEpochSeconds(); }

bool Cognitive3DNode::StartSession(const std::string& participant_id,
                                   bool reuse_remote_variables, std::string* refusal) {
  // Before the per-session resets, which would otherwise run on the live session's state.
  if (runner_ && runner_->active()) {
    RCLCPP_ERROR(get_logger(), "a session is already running (%s); not starting another",
                 runner_->session() ? runner_->session()->session_id().c_str() : "?");
    return false;
  }
  // Core's Begin refuses this too; asked here first so the caller is told why.
  if (runner_) {
    const std::string clock =
        c3d::WallClockProblem(WallClockNow(), c3d_config_.wall_clock_floor_epoch_s);
    if (!clock.empty()) {
      RCLCPP_ERROR(get_logger(), "no session started: %s", clock.c_str());
      if (refusal) *refusal = clock;
      return false;
    }
  }
  translators_.ResetAll();
  // Per session like the translators: the budget, the counters and the one
  // warning each source is allowed all belong to the session they describe.
  event_limiter_.Reset();
  warned_event_before_session_ = false;
  event_properties_dropped_ = 0;
  warned_event_property_keys_.clear();
  // Per session like every translator: the `seq` each sensor stamps its records
  // with counts from the session start, and a counter carried across a boundary
  // would put session N+1's first frame at seq 400.
  for (auto& capture : cameras_) capture->Reset();
  for (auto& capture : lidars_) capture->Reset();
  ResetPoseSamplingState();
  // Before Begin as well as after it: Begin makes the session active, and a record landing in
  // between would otherwise be placed against the previous session's anchor.
  if (clock_policy_) clock_policy_->Anchor(WallEpochSeconds(), clock_policy_->RosTime());
  if (!runner_) return true;  // dry run

  if (!runner_->Begin()) {
    RCLCPP_ERROR(get_logger(), "could not begin a Cognitive3D session");
    return false;
  }
  // From the timestamp the session id and `c3d.sessionStart` carry, so no record precedes them.
  // A reopen after a backward jump anchors at a later wall instant than the session it replaces,
  // and the policy never anchors below a wire time it has already issued. On the wall clock, a
  // start Begin moved past wall now to keep its id unique leads the whole session by as much.
  if (clock_policy_) {
    const double start = runner_->session()->session_timestamp();
    clock_policy_->Anchor(start, clock_policy_->RosTime());
    clock_policy_->LeadWallClock(start - WallEpochSeconds());
  }

  // Any jump BEFORE this session existed belongs to the gap, not to this session -- including one
  // nothing has observed yet. `OnSampleTimer` returns before polling while no session is active, so
  // between two sessions only the 5 s discovery tick advances the detector; without the Poll here,
  // a world reset in a gap shorter than that is first seen on the new session's first sample tick
  // and closes the session the caller was just handed the id of.
  if (clock_policy_) {
    clock_policy_->Poll();
    clock_policy_->ConsumeBackwardJump();
  }

  auto* session = runner_->session();
  SetHostIdentityProperties(*session);
  session->SetProperty("c3d.ros.rmw", rmw_get_implementation_identifier());
  session->SetProperty("c3d.ros.use_sim_time", use_sim_time_);
  session->SetProperty("c3d.ros.sim_time_on_wire", sim_time_on_wire_);
  // A session with no dynamics is the default rather than a fault, and nothing else in the
  // payload distinguishes the two: the stream is simply absent either way.
  session->SetProperty("c3d.ros.robot_dynamic_object",
                       get_parameter("robot_dynamic_object").as_bool());
  session->SetProperty("c3d.ros.namespace", std::string(get_namespace()));
  session->SetProperty("c3d.ros.node", std::string(get_name()));
  // Which frame the gaze channel actually reports, so a session can be read on
  // its own without the launch configuration beside it.
  if (!sampler_config_.gaze_frame.empty() && pose_sampler_) {
    session->SetProperty("c3d.ros.gaze_frame", sampler_config_.gaze_frame);
    session->SetProperty(
        "c3d.ros.gaze_frame_convention",
        std::string(FrameConventionName(pose_sampler_->resolved_gaze_frame_convention())));
  }
  // Which sensors this session carries, and where each came from, so a stored
  // session answers "what was recording?" without the launch configuration beside
  // it -- the same reason `c3d.ros.gaze_frame` is recorded.
  if (!cameras_.empty()) {
    std::string names;
    for (const auto& capture : cameras_) {
      if (!names.empty()) names += ",";
      const CameraConfig& camera = capture->config();
      names += camera.name;
      const std::string prefix = "c3d.ros.camera." + camera.name + ".";
      session->SetProperty(prefix + "topic", camera.topic);
      session->SetProperty(prefix + "rate_hertz", camera.rate_hz);
      session->SetProperty(prefix + "max_width_pixels", camera.max_width_px);
      session->SetProperty(prefix + "codec", std::string("h264"));
      session->SetProperty(prefix + "preset", camera.preset);
      session->SetProperty(prefix + "crf", camera.crf);
      session->SetProperty(prefix + "max_bitrate_kilobits_per_second", camera.max_bitrate_kbps);
      session->SetProperty(prefix + "chunk_seconds", c3d::kVideoChunkSeconds);
      // The parameters in force, 0 when unset, and where each chunk's field of view comes
      // from. The angles themselves travel with each chunk; these say how they were chosen.
      session->SetProperty(prefix + "hfov_degrees", camera.hfov_degrees);
      session->SetProperty(prefix + "vfov_degrees", camera.vfov_degrees);
      session->SetProperty(prefix + "fov_source", std::string(capture->fov_source()));
      session->SetProperty(prefix + "camera_info_topic", camera.camera_info_topic);
    }
    session->SetProperty("c3d.ros.cameras", names);
  }
  if (!lidars_.empty()) {
    std::string names;
    for (const auto& capture : lidars_) {
      if (!names.empty()) names += ",";
      const LidarConfig& lidar = capture->config();
      names += lidar.name;
      const std::string prefix = "c3d.ros.lidar." + lidar.name + ".";
      session->SetProperty(prefix + "topic", lidar.topic);
      session->SetProperty(prefix + "rate_hertz", lidar.rate_hz);
      // The type-specific half is known only once the topic is subscribed; a reopen already has it.
      const auto subscribed = media_subscriptions_.find("lidar:" + lidar.name);
      if (subscribed != media_subscriptions_.end()) {
        RecordLidarTypeProperties(lidar, subscribed->second.type);
      }
    }
    session->SetProperty("c3d.ros.lidars", names);
  }
  // Each described sensor, fixed for the session, so a reader can draw it from the session alone.
  if (!sensor_configs_.empty()) {
    std::string names;
    for (const auto& sensor : sensor_configs_) {
      names += (names.empty() ? "" : ",") + sensor.name;
      for (const auto& kv : SensorSessionProperties(sensor)) {
        session->SetProperty(kv.first, kv.second);
      }
    }
    session->SetProperty("c3d.ros.sensors", names);
  }
  const std::string identifier = participant_id.empty() ? c3d_config_.device_id : participant_id;
  session->SetParticipantId(identifier);
  ApplySessionIdentity("");
  // After Begin, always: End restores the properties to the Begin snapshot, so an arm set
  // before Begin would leak into the next session.
  if (reuse_remote_variables) ReapplyRemoteVariables(identifier);
  else FetchAndApplyRemoteVariables(identifier);
  return true;
}

void Cognitive3DNode::RecordRemoteVariablesUnavailable(const std::string& identifier) {
  if (!runner_ || !runner_->session()) return;
  c3d::RemoteVariableSet set;
  {
    std::lock_guard<std::mutex> lock(request_state_mu_);
    set = last_remote_variables_;
  }

  c3d::Properties properties;
  properties.emplace_back("identifier", identifier);
  properties.emplace_back("status", static_cast<std::int64_t>(set.status));
  properties.emplace_back("error", set.error);
  // Through the same capped door as every other event, at the last sampled body
  // position. This one fires inside StartSession, before the first sample has
  // landed, so in practice it takes the origin and says so rather than claiming it.
  c3d::Vec3 position;
  const bool position_known =
      pose_sampler_ != nullptr && pose_sampler_->last_body_position(&position);
  RecordCappedEvent(*runner_, &event_limiter_, kRemoteVariablesEventSource, WireNow(),
                    "c3d.remote_variables.unavailable", position, position_known,
                    std::move(properties));
  RCLCPP_WARN(get_logger(),
              "remote variables unavailable for '%s' (status %ld): %s -- this session records "
              "no c3d.remote_variable.* and carries a c3d.remote_variables.unavailable event",
              identifier.c_str(), set.status, set.error.c_str());
}

void Cognitive3DNode::FetchAndApplyRemoteVariables(const std::string& identifier) {
  // The handle is COPIED, not used through the member, so the reader outlives an on_cleanup called
  // directly from another thread while the GET below is still inside it.
  std::shared_ptr<c3d::IHttpGet> reader;
  {
    std::lock_guard<std::mutex> lock(request_state_mu_);
    last_remote_variables_ = c3d::RemoteVariableSet{};
    reader = remote_variables_reader_;
  }
  if (!reader || !runner_ || !runner_->session()) return;
  auto* session = runner_->session();

  c3d::RemoteVariableSet set;
  {
    std::lock_guard<std::mutex> lock(reader_mu_);
    set = c3d::FetchRemoteVariables(*reader, c3d_config_, identifier);
  }
  {
    std::lock_guard<std::mutex> lock(request_state_mu_);
    last_remote_variables_ = set;
  }
  if (set.fetched) {
    c3d::ApplyRemoteVariables(*session, set);
    for (const auto& skipped : set.skipped) {
      RCLCPP_WARN(get_logger(), "remote variable skipped: %s", skipped.c_str());
    }
    RCLCPP_INFO(get_logger(), "remote variables for '%s': %s", identifier.c_str(),
                set.ToJson().c_str());
    return;
  }

  RecordRemoteVariablesUnavailable(identifier);
}

void Cognitive3DNode::ReapplyRemoteVariables(const std::string& identifier) {
  if (!runner_ || !runner_->session()) return;
  auto* session = runner_->session();
  c3d::RemoteVariableSet set;
  std::shared_ptr<c3d::IHttpGet> reader;
  {
    std::lock_guard<std::mutex> lock(request_state_mu_);
    set = last_remote_variables_;
    reader = remote_variables_reader_;
  }
  if (set.fetched) {
    c3d::ApplyRemoteVariables(*session, set);
  } else if (reader) {
    RecordRemoteVariablesUnavailable(identifier);
  }
}

bool Cognitive3DNode::RemoteVariablesFetchEnabled() const {
  std::lock_guard<std::mutex> lock(request_state_mu_);
  return remote_variables_reader_ != nullptr;
}

std::string Cognitive3DNode::ApplySessionIdentity(const std::string& requested_name) {
  if (runner_ == nullptr || runner_->session() == nullptr) return "";
  auto* session = runner_->session();

  // ALWAYS set a name, and never leave it empty. `c3d.sessionname` is an ordinary
  // session property and `known_properties` deliberately survives a session
  // boundary, so a session started without a name would silently inherit the
  // previous one's -- which is worse than having none, because it is wrong
  // rather than absent. The session id is the last resort precisely because it
  // always exists and is never stale.
  std::string name = requested_name;
  if (name.empty()) name = get_parameter("session_name").as_string();
  if (name.empty()) name = session->session_id();
  session->SetSessionName(name);

  // Free-form operator metadata: site, fleet, policy version, whatever the
  // deployment wants to slice sessions by later.
  //
  // Bind the parameter to a named value first. `get_parameter(...)` returns a
  // temporary Parameter and `as_string_array()` returns a reference INTO it, so
  // iterating the call directly walks freed memory: range-for does not extend
  // the lifetime of a temporary reached through a reference-returning call
  // (fixed only in C++23, and we are C++17).
  const rclcpp::Parameter session_properties_param = get_parameter("session_properties");
  const bool fetch_enabled = RemoteVariablesFetchEnabled();
  for (const auto& entry : session_properties_param.as_string_array()) {
    const auto eq = entry.find('=');
    if (eq == std::string::npos || eq == 0) {
      RCLCPP_WARN(get_logger(), "ignoring malformed session_property '%s' (want key=value)",
                  entry.c_str());
      continue;
    }
    const std::string key = entry.substr(0, eq), value = entry.substr(eq + 1);
    // Configure and a parameter set both refuse these already.
    if (NamesAnSdkSessionProperty(key, fetch_enabled)) {
      RCLCPP_WARN(get_logger(), "ignoring session_property '%s': %s", key.c_str(),
                  SdkKeyRefusal("it", key).c_str());
      continue;
    }
    // Typed, not textual. The platform indexes numeric session properties
    // separately, so a number sent as a string lands where a numeric slice
    // cannot see it.
    session->SetProperty(key, c3d::ParseConfiguredProperty(key, value));
  }
  return name;
}

void Cognitive3DNode::OnStartSessionRequest(
    const std::shared_ptr<cognitive3d_ros_msgs::srv::StartSession::Request> request,
    std::shared_ptr<cognitive3d_ros_msgs::srv::StartSession::Response> response) {
  // Through the base interface: LifecycleNode has no get_fully_qualified_name() on Humble.
  const std::string node_name = get_node_base_interface()->get_fully_qualified_name();
  if (runner_ == nullptr) {
    response->ok = false;
    response->message = dry_run_ ? "dry_run is set: no session will be created"
                                 : "the node is not configured: configure and activate it first "
                                   "(`ros2 lifecycle set " + node_name + " configure`, then "
                                   "`activate`)";
    return;
  }
  // A session opened while inactive would have no pose, sensor or camera ingest behind it, and
  // the activation after it would then fail, so the session contract is: sessions exist only
  // while the node is active.
  if (!active_) {
    response->ok = false;
    response->message = "the node is inactive, so no session was started: activate it first "
                        "(`ros2 lifecycle set " + node_name + " activate`)";
    return;
  }
  if (runner_->active()) {
    // The id of the session that is BLOCKING the start, not an empty string.
    // Without it a caller cannot tell "already running as X" from a genuine
    // failure, and cannot name X in its own log or in a recovery command.
    response->ok = false;
    response->session_id = runner_->session() ? runner_->session()->session_id() : "";
    response->message =
        session_from_activation_
            ? "session " + response->session_id + " is already running: activation opened it "
              "because auto_start_session is true. End it with ~/end_session first, or set "
              "auto_start_session: false so that ~/start_session opens every session"
            : "a session is already running; end it first";
    return;
  }
  // Parsed BEFORE anything is opened, and that ordering is the contract:
  // `ok=false` with a non-empty session_id must mean exactly one thing -- "a
  // session is already running and it is not yours". A caller cannot act on a
  // response that also means "the session I just opened for you is running";
  // ending the id is right for one reading and destroys a stranger's session
  // under the other. ParsePropertiesJson is pure, so refusing here costs
  // nothing and leaves nothing behind.
  std::string error;
  c3d::Properties requested;
  if (!c3d::ParsePropertiesJson(request->properties_json, &requested, &error)) {
    response->ok = false;
    response->message = "properties_json was rejected, so no session was started: " + error;
    RCLCPP_WARN(get_logger(), "properties_json rejected, no session started: %s", error.c_str());
    return;
  }
  // Refused rather than truncated. The platform draws the arm for the whole string, so a caller
  // that passed a longer one would get a session keyed on an identity the session does not carry.
  if (request->participant_id.size() > c3d::kParticipantIdMax) {
    response->ok = false;
    response->message =
        "participant_id is longer than " + std::to_string(c3d::kParticipantIdMax) +
        " bytes, so no session was started: the platform draws for the full string but the "
        "session would record only the first " + std::to_string(c3d::kParticipantIdMax);
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }
  const bool fetches_arms = RemoteVariablesFetchEnabled();
  for (const auto& kv : requested) {
    if (!NamesAnSdkSessionProperty(kv.first, fetches_arms)) continue;
    response->ok = false;
    response->message = SdkKeyRefusal("properties_json", kv.first) + ", so no session was started";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  // Kept so a restart can reproduce this session rather than a differently-configured one, and
  // recorded BEFORE the session opens: the reopen runs on the ingest thread and can fire while
  // StartSession is still blocked on the fetch.
  {
    std::lock_guard<std::mutex> lock(request_state_mu_);
    requested_session_name_ = request->session_name;
    requested_session_properties_ = requested;
    requested_participant_id_ = request->participant_id;
  }

  std::string refusal;
  if (!StartSession(request->participant_id, false, &refusal)) {
    {
      std::lock_guard<std::mutex> lock(request_state_mu_);
      requested_session_name_.clear();
      requested_session_properties_.clear();
      requested_participant_id_.clear();
    }
    response->ok = false;
    response->message = refusal.empty() ? "the session could not be started; see the node log"
                                        : "no session was started: " + refusal;
    return;
  }

  session_from_activation_ = false;
  const std::string name = ApplySessionIdentity(request->session_name);
  for (const auto& kv : requested) runner_->session()->SetProperty(kv.first, kv.second);

  {
    std::lock_guard<std::mutex> lock(request_state_mu_);
    response->remote_variables_json =
        last_remote_variables_.fetched ? last_remote_variables_.ToJson() : "";
  }
  response->ok = true;
  response->session_id = runner_->session()->session_id();
  RCLCPP_INFO(get_logger(), "session %s started as '%s' with %zu requested propert%s",
              response->session_id.c_str(), name.c_str(), requested.size(),
              requested.size() == 1 ? "y" : "ies");
}

double Cognitive3DNode::WireNow() const {
  return clock_policy_ ? clock_policy_->WireTime() : WallEpochSeconds();
}

double Cognitive3DNode::GateNow() const {
  return sim_time_on_wire_ && clock_policy_ ? clock_policy_->WireTime() : SteadySeconds();
}

void Cognitive3DNode::OnEvent(const cognitive3d_ros_msgs::msg::Event::SharedPtr message) {
  const double received = WireNow();
  // A stamp is on the publisher's ROS clock, so it goes through the same placement a header stamp
  // does; ROS time itself never reaches the wire.
  const bool stamped = message->stamp.sec != 0 || message->stamp.nanosec != 0;
  double wire_time =
      stamped && clock_policy_
          ? clock_policy_->HeaderStampToWireTime(static_cast<double>(message->stamp.sec) +
                                                 static_cast<double>(message->stamp.nanosec) * 1e-9)
          : received;
  if (message->name.empty()) {
    ++events_dropped_;
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000,
                         "an event with an empty name was dropped");
    return;
  }
  if (runner_ == nullptr || !runner_->active()) {
    // Throttled, and COUNTED. A mission node that starts publishing before it
    // calls ~/start_session loses every event silently otherwise, and the
    // session it eventually opens looks merely uneventful.
    ++events_dropped_;
    const double steady_now = steady_clock_.now().seconds();
    if (steady_now - last_event_warn_ > 5.0) {
      last_event_warn_ = steady_now;
      RCLCPP_WARN(get_logger(),
                  "event '%s' dropped: no session is running (%llu dropped so far). Call "
                  "~/start_session before publishing.",
                  message->name.c_str(),
                  static_cast<unsigned long long>(events_dropped_));
    }
    return;
  }

  // Capped with the same budget a translator's topic gets, and for the same
  // reason: past the session's record bound the NEWEST record is dropped, so an
  // unbounded publisher does not merely crowd out the rest -- it deletes whatever
  // happens after it.
  // On arrival, not on the event's own time: the budget is spent when the event arrives.
  if (!event_limiter_.Allow(kUserEventSource, received)) {
    WarnOnSuppressedSources();
    return;
  }

  // Never before the session it is recorded into: no record precedes the session's start. Said
  // only of a stamp; arrival is placed at or after the start.
  const double session_start = runner_->session()->session_timestamp();
  if (wire_time < session_start) {
    if (stamped && !warned_event_before_session_) {
      warned_event_before_session_ = true;
      RCLCPP_WARN(get_logger(),
                  "event '%s' falls %.3fs before the session it is recorded into began; it is "
                  "placed at the session's start, as is any other this session receives. An "
                  "event's stamp says when something happened during the session.",
                  message->name.c_str(), session_start - wire_time);
    }
    wire_time = session_start;
  }

  c3d::Properties properties;
  std::string error;
  if (!c3d::ParsePropertiesJson(message->properties_json, &properties, &error)) {
    // The EVENT is still recorded. A malformed properties blob is a defect in the
    // publisher; dropping the event as well would delete the fact that something
    // happened, which is the part nothing else can reconstruct.
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000,
                         "event '%s': properties_json rejected (%s); recording it without them",
                         message->name.c_str(), error.c_str());
    properties.clear();
  }
  // The SDK's own event properties are `c3d.` keys, `c3d.position_unknown` below among them, so a
  // publisher's would contradict or duplicate them.
  for (auto it = properties.begin(); it != properties.end();) {
    if (!InSdkNamespace(it->first)) {
      ++it;
      continue;
    }
    ++event_properties_dropped_;
    if (warned_event_property_keys_.insert(it->first).second) {
      RCLCPP_WARN(get_logger(),
                  "event '%s': property '%s' dropped, because c3d.* event properties are the "
                  "SDK's. Every drop is counted in c3d.event_properties_dropped; this key is not "
                  "warned about again this session.",
                  message->name.c_str(), it->first.c_str());
    }
    it = properties.erase(it);
  }

  // Stamped where the robot was. An event with no position is a row in a table;
  // an event with one is a point on the map, which is most of what this platform
  // is for. Before the first pose there is nothing honest to stamp, so the origin
  // stands and a property says so rather than the point quietly lying. Where the
  // robot is on ARRIVAL, stamped or not: a pose looked up at the stamp would be a
  // tf wait inside the node's one callback group.
  c3d::Vec3 position;
  if (pose_sampler_ == nullptr || !pose_sampler_->last_body_position(&position)) {
    position = c3d::Vec3{};
    properties.emplace_back("c3d.position_unknown", true);
  }
  runner_->RecordEventAt(wire_time, message->name, position, properties,
                         message->dynamic_object_id);
  ++events_recorded_;
}

void Cognitive3DNode::OnEndSessionRequest(
    const std::shared_ptr<cognitive3d_ros_msgs::srv::EndSession::Request> request,
    std::shared_ptr<cognitive3d_ros_msgs::srv::EndSession::Response> response) {
  if (runner_ == nullptr || !runner_->active()) {
    response->ok = false;
    response->message = "no session is running";
    return;
  }
  response->session_id = runner_->session()->session_id();
  StopSession(request->reason.empty() ? "ended_by_service" : request->reason);
  response->ok = true;
  RCLCPP_INFO(get_logger(), "session %s ended (%s)", response->session_id.c_str(),
              request->reason.c_str());
}

std::optional<double> Cognitive3DNode::SessionEndOnWire() const {
  if (!clock_policy_) return std::nullopt;
  // Core's own end is wall now, which a session leading the wall clock may not have reached.
  if (!sim_time_on_wire_) {
    return clock_policy_->wall_clock_lead() > 0.0 ? std::optional<double>(WireNow())
                                                  : std::nullopt;
  }
  // The later of now and anything already issued: on the jump path now is already past the reset.
  if (!clock_policy_->anchored()) return std::nullopt;
  return std::max(WireNow(), clock_policy_->LatestWireTime());
}

void Cognitive3DNode::StopSession(const std::string& reason) {
  // Before End: the runner must still take the last access units each encoder drains.
  StopCameraEncoders(cameras_);
  if (runner_) runner_->End(reason, SessionEndOnWire());
  session_from_activation_ = false;
  // The remembered request dies with the session it described. Every deliberate end comes through
  // here -- the service, deactivate, cleanup, shutdown -- and the clock-jump restart deliberately
  // does not, because that is the one case that must reproduce the request. Without this, a session
  // auto-started at a later activate inherits the name and properties of an earlier service call
  // and is then stamped with them on its first restart.
  std::lock_guard<std::mutex> lock(request_state_mu_);
  requested_session_name_.clear();
  requested_session_properties_.clear();
  requested_participant_id_.clear();
}

LifecycleNodeInterface::CallbackReturn Cognitive3DNode::on_deactivate(
    const rclcpp_lifecycle::State&) {
  active_ = false;
  BringDown("deactivated");
  RCLCPP_INFO(get_logger(),
              "deactivated. Every subscription is destroyed -- topics, cameras and lidars, the tf "
              "listener and ~/events -- so the node receives no topic traffic until it is "
              "activated again, and an event published meanwhile is lost; the session services "
              "stay up and refuse.");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn Cognitive3DNode::on_cleanup(
    const rclcpp_lifecycle::State&) {
  active_ = false;
  TearDownSubscriptions();
  StopSession("cleanup");
  stats_.clear();
  action_topics_.clear();
  action_topics_logged_ = 0;
  pose_sampler_.reset();
  tf_buffer_.reset();
  tf_callback_group_.reset();
  runner_.reset();
  {
    std::lock_guard<std::mutex> lock(request_state_mu_);
    remote_variables_reader_.reset();
  }
  clock_policy_.reset();
  diagnostics_pub_.reset();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn Cognitive3DNode::on_shutdown(
    const rclcpp_lifecycle::State&) {
  active_ = false;
  // Shutdown can come straight from ACTIVE, without a deactivate, and a discovery tick left
  // running would subscribe everything again.
  sample_timer_.reset();
  discovery_timer_.reset();
  report_timer_.reset();
  media_hold_timer_.reset();
  TearDownSubscriptions();
  // An orderly end here is what stops the next process start from reporting
  // this session as having died abnormally.
  StopSession("shutdown");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp::SubscriptionOptions Cognitive3DNode::SubscriptionOptionsInGroup() const {
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  return options;
}

void Cognitive3DNode::StartTransformListener() {
  if (!tf_buffer_ || !tf_callback_group_) return;
  StopTransformListener();
  // No QoS overriding options, which would declare `qos_overrides./tf.*` parameters on this node.
  rclcpp::SubscriptionOptions in_group;
  in_group.callback_group = tf_callback_group_;
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_, this, /*spin_thread=*/false, tf2_ros::DynamicListenerQoS(),
      tf2_ros::StaticListenerQoS(), in_group, in_group);
  rclcpp::ExecutorOptions options;
  options.context = get_node_base_interface()->get_context();
  tf_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(options);
  tf_executor_->add_callback_group(tf_callback_group_, get_node_base_interface());
  tf_stop_ = false;
  // A loop on the flag with a bounded wait, never `spin()`: a cancel that lands before `spin()`
  // has started is lost, and the join below would then wait for good.
  tf_thread_ = std::thread([this, executor = tf_executor_, context = options.context]() {
    try {
      while (!tf_stop_ && rclcpp::ok(context)) executor->spin_once(std::chrono::milliseconds(100));
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "tf listener stopped: %s", e.what());
    }
  });
}

void Cognitive3DNode::StopTransformListener() {
  tf_stop_ = true;
  // Only to cut the current wait short; the flag is what ends the loop.
  if (tf_executor_) tf_executor_->cancel();
  if (tf_thread_.joinable()) tf_thread_.join();
  // The executor before the listener: it may still hold the listener's subscriptions, whose
  // callbacks point into the listener. Destroying it also frees the group for the next one.
  tf_executor_.reset();
  tf_listener_.reset();
}

void Cognitive3DNode::TearDownSubscriptions() {
  subscriptions_.clear();
  first_topic_of_type_.clear();
  media_subscriptions_.clear();
  for (auto& kv : stats_) kv.second.subscribed = false;
  event_sub_.reset();
  StopTransformListener();
  // Its dynamic transforms go too, so a later activation never samples a pose from before this
  // one. tf2 keeps static transforms through a clear, which is what they are.
  if (tf_buffer_) tf_buffer_->clear();
}

bool Cognitive3DNode::AutoSubscribesByType(const std::string& topic,
                                           const std::string& type) const {
  if (!auto_subscribe_standard_types_) return false;
  if (!InDiscoveryNamespace(topic, get_namespace())) return false;
  // Never this node's OWN health report, which is a DiagnosticArray like any
  // other. Taking it back in by type alone would turn every line the node says
  // about itself into an event about itself, once per report period, for the life
  // of the run. An explicit allowlist entry still wins, as it does everywhere.
  if (diagnostics_pub_ && topic == diagnostics_pub_->get_topic_name()) return false;
  return type == "sensor_msgs/msg/BatteryState" ||
         type == "diagnostic_msgs/msg/DiagnosticArray";
}

void Cognitive3DNode::LogActionEnrolment() {
  if (action_topics_.size() == action_topics_logged_) return;
  action_topics_logged_ = action_topics_.size();

  std::ostringstream recorded;
  std::ostringstream skipped;
  std::size_t n_recorded = 0;
  std::size_t n_skipped = 0;
  for (const auto& kv : action_topics_) {
    if (kv.second.enrolled) {
      recorded << (n_recorded++ ? ", " : "") << kv.first;
    } else {
      skipped << (n_skipped++ ? ", " : "") << kv.first << " (" << kv.second.reason << ")";
    }
  }
  RCLCPP_INFO(get_logger(),
              "action servers on the graph: %zu recorded [%s]; %zu NOT recorded [%s]. An action "
              "that is not recorded emits no events at all -- add its name to `action_names` or "
              "its topic to `topics`.",
              n_recorded, recorded.str().c_str(), n_skipped, skipped.str().c_str());
}

void Cognitive3DNode::DiscoverAndSubscribe() {
  const auto graph = get_topic_names_and_types();

  for (const auto& entry : graph) {
    const std::string& topic = entry.first;
    if (entry.second.empty()) continue;
    const std::string& type = entry.second.front();

    const bool allowlisted = plan_.Allowed(topic);
    // Resolved before anything decides the topic is taken: `refuse` creates no
    // subscription, and a verdict reached without it claims an action the run
    // never records.
    const TopicRule rule = plan_.Resolve(topic, type);

    if (type == kGoalStatusArrayType) {
      const ActionTopicVerdict verdict =
          ClassifyActionTopic(topic, allowlisted, rule.ingest == Ingest::kRefuse,
                              action_discovery_, action_names_, get_namespace());
      // Kept for every action on the graph, recorded or not. The ones that are not
      // are the reason this map exists: nothing else in a run reports them, and
      // after the run they look like actions that never fired.
      action_topics_[topic] = verdict;
      if (!verdict.enrolled) continue;
    } else if (!allowlisted && !AutoSubscribesByType(topic, type)) {
      continue;
    }
    if (subscriptions_.count(topic)) continue;

    TopicStat& stat = stats_[topic];
    stat.topic = topic;
    stat.type = type;
    stat.ingest = rule.ingest;
    stat.rate_hz = rule.rate_hz;
    stat.series = rule.series;
    // From the same rule the verdict came from. `Allowed` takes the FIRST matching
    // pattern and `Resolve` the LONGEST, so a topic configured by name under a
    // wildcard that refuses it resolves on its own rule while `Allowed` says no --
    // and the line below would then call a configured topic "discovered".
    stat.from_allowlist = plan_.Match(topic) != nullptr;

    if (rule.ingest == Ingest::kRefuse) {
      if (!stat.refused) {
        stat.refused = true;
        // Refusing means never creating the subscription. A best-effort
        // subscription that discards on arrival still copies every megabyte
        // through the RMW, which for a point cloud is tens of MB/s of pure
        // waste on the robot's compute box.
        //
        // Which of the two refusals this is, because they take different fixes:
        // a bulk type is refused by default and full_ingest overrides it, while
        // anything else got here because a `topic_config` entry asked for it.
        stat.refusal = IsBulkType(type) ? "bulk type" : "ingest is `refuse` in `topic_config`";
        RCLCPP_WARN(get_logger(),
                    "refusing %s (%s): %s, not subscribed at all. %s",
                    topic.c_str(), type.c_str(), stat.refusal.c_str(),
                    IsBulkType(type)
                        ? "Set ingest=full_ingest on this exact topic to override."
                        : "Remove that entry to record it.");
      }
      continue;
    }

    const QosResolution resolution = ResolveQos(this, topic, rule.qos);
    stat.qos_description = DescribeQos(resolution.qos);
    stat.qos_rationale = resolution.rationale;

    rclcpp::SubscriptionOptions options = SubscriptionOptionsInGroup();
    // Turns "there is no data" into a one-line answer.
    // A topic no rule configured has no override group, and only an allowlist entry gives it one.
    const TopicRule* configured = plan_.Match(topic);
    const std::string override_key = configured != nullptr ? configured->config_key : "";
    options.event_callbacks.incompatible_qos_callback =
        [this, topic, override_key](rclcpp::QOSRequestedIncompatibleQoSInfo& info) {
          auto it = stats_.find(topic);
          if (it != stats_.end()) it->second.incompatible_qos = true;
          RCLCPP_ERROR(get_logger(), "%s",
                       DescribeQosIncompatibility(topic, info.total_count, info.last_policy_kind,
                                                  override_key)
                           .c_str());
        };

    try {
      auto subscription = create_generic_subscription(
          topic, type, resolution.qos,
          [this, topic](std::shared_ptr<const rclcpp::SerializedMessage> message) {
            OnMessage(topic, *message);
          },
          options);
      subscriptions_[topic] = subscription;
      stat.subscribed = true;
      stat.has_translator = translators_.Handles(type);
      const auto [first, inserted] = first_topic_of_type_.emplace(type, topic);
      if (!inserted) {
        const std::string shared = SharedSeriesWarning(translators_, type, first->second, topic);
        if (!shared.empty()) RCLCPP_WARN(get_logger(), "%s", shared.c_str());
      }
      if (!stat.has_translator) {
        // Not an error. The topic is observed and counted, which is still useful
        // for characterising a robot, but nothing lands in the session model.
        RCLCPP_INFO(get_logger(),
                    "no translator for %s [%s]; it will be observed and counted only",
                    topic.c_str(), type.c_str());
      }
      // Said here, because the type is a fact of the graph and configure cannot know it.
      const ITranslator* translator = translators_.Find(type);
      if (!rule.series.empty() && (translator == nullptr || !translator->reads_series())) {
        RCLCPP_WARN(get_logger(),
                    "%s.series ('%s') is ignored: %s is %s, which records under series names of "
                    "its own. A series name is read only for %s.",
                    rule.config_key.c_str(), rule.series.c_str(), topic.c_str(), type.c_str(),
                    SeriesTypesText(translators_).c_str());
      }
      for (const auto& sensor : sensor_configs_) {
        if (sensor.topic != topic) continue;
        const std::string mismatch = SensorTypeMismatch(sensor, type);
        if (!mismatch.empty()) {
          RCLCPP_ERROR(get_logger(), "%s. %s.", mismatch.c_str(), SensorTypeRemedy(sensor).c_str());
        }
      }
      const std::string ingest = SubscribedIngestText(rule, type);
      const std::string ingest_shown = ingest.empty() ? "" : " ingest=" + ingest;
      RCLCPP_INFO(get_logger(), "subscribed %s%s [%s] qos=%s%s rate=%gHz%s (%s)",
                  topic.c_str(), stat.from_allowlist ? "" : " (discovered, not in `topics`)",
                  type.c_str(), stat.qos_description.c_str(), ingest_shown.c_str(), rule.rate_hz,
                  rule.rate_from_type ? " (type default)" : "", resolution.rationale.c_str());
    } catch (const std::exception& e) {
      // Almost always a missing message package on the robot: generic
      // subscriptions resolve typesupport at runtime via the ament prefix path.
      RCLCPP_ERROR(get_logger(),
                   "cannot subscribe %s [%s]: %s. Is the message package installed "
                   "on this machine?",
                   topic.c_str(), type.c_str(), e.what());
    }
  }

  if (warn_on_missing_topics_) {
    std::vector<std::string> missing;
    for (const auto& rule : plan_.rules()) {
      if (rule.topic.find('*') != std::string::npos) continue;
      if (subscriptions_.count(rule.topic)) continue;
      const bool present = std::any_of(
          graph.begin(), graph.end(),
          [&rule](const auto& e) { return e.first == rule.topic; });
      if (!present) missing.push_back(rule.topic);
    }
    if (!missing.empty()) {
      // ONE line naming every missing topic, because RCLCPP_WARN_THROTTLE
      // throttles per call site and not per message. Warning from inside the
      // loop would let the first missing topic consume the window and suppress
      // every other one for as long as the process runs -- and the case this
      // warning exists for, an allowlist that does not match the robot, always
      // has several missing at once.
      std::string joined;
      for (size_t i = 0; i < missing.size(); ++i) {
        if (i != 0) joined += ", ";
        joined += missing[i];
      }
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 30000,
                           "%zu allowlisted topic(s) not present on the graph: %s",
                           missing.size(), joined.c_str());
    }
  }

  LogActionEnrolment();
}

MediaEnvironment Cognitive3DNode::MakeMediaEnvironment() {
  MediaEnvironment environment;
  environment.lookup_pose = [this](const std::string& frame, const rclcpp::Time& stamp,
                                   c3d::Vec3* position, c3d::Quat* rotation) {
    return pose_sampler_ != nullptr &&
           pose_sampler_->LookupSensorPose(frame, stamp, position, rotation);
  };
  environment.wire_time = [this](const rclcpp::Time& stamp) {
    // The clock rule, at the one point these two streams touch it: the message's
    // own stamp mapped into wall clock by its age. Under use_sim_time the stamp
    // itself starts near zero, and a session whose record timestamps do that is
    // accepted with a 200 and never appears.
    return clock_policy_ ? clock_policy_->HeaderStampToWireTime(stamp.seconds())
                         : WallEpochSeconds();
  };
  environment.warn_once = [this](const std::string& text) {
    RCLCPP_WARN(get_logger(), "%s", text.c_str());
  };
  environment.arrival_time = [this]() { return GateNow(); };
  environment.pose_wait_s = HeldRecords::kPoseHoldSeconds;
  // The tf buffer's clock.
  environment.pose_clock = [this]() { return get_clock()->now().seconds(); };
  environment.publishers = [this](const std::string& topic) { return count_publishers(topic); };
  return environment;
}

void Cognitive3DNode::DiscoverAndSubscribeMedia() {
  if (cameras_.empty() && lidars_.empty()) return;
  const auto graph = get_topic_names_and_types();

  auto type_on_graph = [&graph](const std::string& topic) {
    for (const auto& entry : graph) {
      if (entry.first == topic && !entry.second.empty()) return entry.second.front();
    }
    return std::string();
  };

  auto subscribe = [&](const std::string& key, const std::string& topic,
                       const std::string& type, const QosOverride& qos_override,
                       const std::string& override_key) {
    const QosResolution resolution = ResolveQos(this, topic, qos_override);
    rclcpp::SubscriptionOptions options = SubscriptionOptionsInGroup();
    options.event_callbacks.incompatible_qos_callback =
        [this, key, topic, override_key](rclcpp::QOSRequestedIncompatibleQoSInfo& info) {
          RCLCPP_ERROR(get_logger(), "%s: %s", key.c_str(),
                       DescribeQosIncompatibility(topic, info.total_count, info.last_policy_kind,
                                                  override_key)
                           .c_str());
        };
    try {
      MediaSubscription entry;
      entry.type = type;
      entry.subscription = create_generic_subscription(
          topic, type, resolution.qos,
          [this, key](std::shared_ptr<const rclcpp::SerializedMessage> message) {
            OnMediaMessage(key, *message);
          },
          options);
      media_subscriptions_[key] = std::move(entry);
      RCLCPP_INFO(get_logger(), "subscribed %s for %s [%s] qos=%s (%s)", topic.c_str(),
                  key.c_str(), type.c_str(), DescribeQos(resolution.qos).c_str(),
                  resolution.rationale.c_str());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "cannot subscribe %s for %s [%s]: %s", topic.c_str(),
                   key.c_str(), type.c_str(), e.what());
    }
  };

  // The type is read from the GRAPH rather than configured, so it cannot be
  // checked at configure: on a robot that starts its telemetry before its camera
  // driver, the topic does not exist yet. The refusal therefore lands here, said
  // once per sensor, and `doctor` is what answers the question before a run.
  auto refuse_type = [this](const std::string& key, const std::string& topic,
                            const std::string& type, const char* expected) {
    if (!media_type_refusals_.insert(key).second) return;
    RCLCPP_ERROR(get_logger(),
                 "%s names %s, which the graph says is %s -- this SDK handles %s. Nothing from "
                 "it is recorded.",
                 key.c_str(), topic.c_str(), type.c_str(), expected);
  };

  for (const auto& capture : cameras_) {
    const std::string key = "camera:" + capture->config().name;
    if (media_subscriptions_.count(key)) continue;
    const std::string type = type_on_graph(capture->config().topic);
    if (type.empty()) continue;
    if (!IsCameraType(type)) {
      refuse_type(key, capture->config().topic, type,
                  "sensor_msgs/msg/Image and sensor_msgs/msg/CompressedImage");
      continue;
    }
    subscribe(key, capture->config().topic, type, capture->config().qos,
              "camera." + capture->config().name);
  }
  // A camera's CameraInfo, for its field of view. Not while the parameters are set, since they
  // win; with no QoS override, so the subscription takes whatever its publisher offers.
  for (const auto& capture : cameras_) {
    if (!capture->wants_camera_info()) continue;
    const std::string key = "camera_info:" + capture->config().name;
    if (media_subscriptions_.count(key)) continue;
    const std::string& topic = capture->config().camera_info_topic;
    const std::string type = type_on_graph(topic);
    if (type.empty()) continue;
    if (type != kCameraInfoType) {
      refuse_type(key, topic, type, kCameraInfoType);
      continue;
    }
    subscribe(key, topic, type, QosOverride{}, "");
  }
  for (const auto& capture : lidars_) {
    const std::string key = "lidar:" + capture->config().name;
    if (media_subscriptions_.count(key)) continue;
    const std::string type = type_on_graph(capture->config().topic);
    if (type.empty()) continue;
    if (!IsLidarType(type)) {
      refuse_type(key, capture->config().topic, type,
                  "sensor_msgs/msg/LaserScan and sensor_msgs/msg/PointCloud2");
      continue;
    }
    subscribe(key, capture->config().topic, type, QosOverride{}, "");
    if (media_subscriptions_.count(key)) RecordLidarTypeProperties(capture->config(), type);
  }
}

void Cognitive3DNode::RecordLidarTypeProperties(const LidarConfig& lidar,
                                                const std::string& type) {
  if (runner_ == nullptr || !runner_->active()) return;
  auto* session = runner_->session();
  const std::string prefix = "c3d.ros.lidar." + lidar.name + ".";
  session->SetProperty(prefix + "message_type", type);
  if (type == "sensor_msgs/msg/LaserScan") {
    session->SetProperty(prefix + "intensities", lidar.intensities);
    return;
  }
  session->SetProperty(prefix + "pointcloud.voxel_leaf_meters", lidar.pointcloud.voxel_leaf_m);
  session->SetProperty(prefix + "pointcloud.max_points",
                       static_cast<std::int64_t>(lidar.pointcloud.max_points));
  session->SetProperty(prefix + "pointcloud.position",
                       std::string(c3d::PointPositionName(lidar.pointcloud.position)));
  session->SetProperty(prefix + "pointcloud.color",
                       std::string(c3d::PointColorName(lidar.pointcloud.color)));
  session->SetProperty(prefix + "pointcloud.intensity",
                       std::string(c3d::PointIntensityName(lidar.pointcloud.intensity)));
  session->SetProperty(prefix + "pointcloud.zstd_level", lidar.pointcloud.zstd_level);
  // The encoding written, which float32 positions and a zstd_level of 0 overrule.
  session->SetProperty(prefix + "pointcloud.encoding",
                       std::string(c3d::PointCloudEncodingName(
                           c3d::pointcloud::EffectiveEncoding(lidar.pointcloud))));
}

void Cognitive3DNode::OnMediaMessage(const std::string& key,
                                     const rclcpp::SerializedMessage& message) {
  const std::string name = key.substr(key.find(':') + 1);
  // Before the session check: the latest CameraInfo describes the camera whether or not a
  // session is open, and the first frame of the next one should carry it.
  if (key.rfind("camera_info:", 0) == 0) {
    for (const auto& capture : cameras_) {
      if (capture->config().name == name) capture->HandleCameraInfo(message, media_environment_);
    }
    return;
  }
  if (runner_ == nullptr || !runner_->active()) return;
  const auto entry = media_subscriptions_.find(key);
  if (entry == media_subscriptions_.end()) return;

  if (key.rfind("camera:", 0) == 0) {
    for (const auto& capture : cameras_) {
      if (capture->config().name != name) continue;
      capture->Handle(entry->second.type, message, media_environment_, *runner_);
      return;
    }
    return;
  }
  for (const auto& capture : lidars_) {
    if (capture->config().name != name) continue;
    capture->Handle(entry->second.type, message, media_environment_, *runner_);
    return;
  }
}

void Cognitive3DNode::OnMediaHoldTimer() {
  if (runner_ == nullptr) return;
  for (const auto& capture : cameras_) capture->ServiceHeld(media_environment_, *runner_);
  for (const auto& capture : lidars_) capture->ServiceHeld(media_environment_, *runner_);
}

void Cognitive3DNode::OnMessage(const std::string& topic,
                                const rclcpp::SerializedMessage& message) {
  auto it = stats_.find(topic);
  if (it == stats_.end()) return;
  TopicStat& stat = it->second;

  const double wall_now = WallEpochSeconds();
  if (stat.messages == 0) stat.first_seen_wall = wall_now;
  stat.last_seen_wall = wall_now;
  ++stat.messages;
  stat.bytes += message.size();

  if (!stat.has_translator) return;
  ITranslator* translator = translators_.Find(stat.type);
  if (translator == nullptr) return;
  if (runner_ == nullptr || !runner_->active()) return;

  TranslationContext ctx;
  ctx.topic = topic;
  ctx.type = stat.type;
  ctx.ingest = stat.ingest;
  ctx.rate_hz = stat.rate_hz;
  ctx.series = stat.series;
  // Epoch seconds on the wire, always, and never ROS time. Under sim time the ROS
  // clock starts near zero, and the Cognitive3D session id embeds an epoch
  // timestamp, so this is not interchangeable.
  ctx.wire_time = WireNow();
  ctx.ros_time = clock_policy_ ? clock_policy_->RosTime() : 0.0;
  ctx.gate_time = GateNow();
  // Where the robot was when this arrived, on the same terms `~/events` gets: an
  // event with no position is a row in a table, and before the first pose sample
  // there is nothing honest to stamp.
  if (pose_sampler_ != nullptr) {
    ctx.position_known = pose_sampler_->last_body_position(&ctx.body_position);
  }
  ctx.event_limiter = &event_limiter_;

  translator->Translate(message, ctx, *runner_);
  ++stat.translated;
  WarnOnSuppressedSources();
}

void Cognitive3DNode::WarnOnSuppressedSources() {
  while (const std::optional<std::string> source = event_limiter_.TakeUnreportedSuppression()) {
    RCLCPP_WARN(get_logger(),
                "%s is producing events faster than event_rate_limit_hz (%.1f/s); the excess is "
                "counted as c3d.events_suppressed and discarded. Sensor series and the pose "
                "timeline are unaffected. Raise event_rate_limit_hz, or set it to 0 to remove "
                "the cap.",
                source->c_str(), event_limiter_.limit_hz());
  }
}

void Cognitive3DNode::ResetPoseSamplingState() {
  // Everything scoped to one session's pose sampling, reset in ONE place.
  //
  // Two reset sites would drift apart: one that reset the sampler but not both
  // pose counters would leave the counters disagreeing across a clock jump, which
  // is why every session start, the clock-jump restart included, comes through here.
  // (The camera warning's high-water bar lives on the sampler, beside the
  // counter it compares against, which is the only arrangement where one
  // Reset() cannot miss half of a decision.)
  if (pose_sampler_) pose_sampler_->Reset();
  pose_samples_ = 0;
  pose_failures_ = 0;
  last_gaze_frame_warn_ = 0.0;
  last_pose_warn_ = 0.0;
  last_self_telemetry_ = 0.0;
  next_sample_wire_ = 0.0;
}

void Cognitive3DNode::OnSampleTimer() {
  if (runner_ == nullptr || !runner_->active() || pose_sampler_ == nullptr) return;

  // A backward ROS time jump is a sim reset or a bag loop. Continuing into the
  // same session would produce negative durations and interleave two runs.
  // Poll to keep the clock current, then ask whether a jump is OUTSTANDING -- the discovery timer
  // polls too, and an edge belongs to whoever acts on it rather than to whoever saw it first.
  if (clock_policy_) clock_policy_->Poll();
  if (clock_policy_ && clock_policy_->ConsumeBackwardJump()) {
    RCLCPP_WARN(get_logger(), "closing session on backward clock jump and starting a new one");
    // Through StartSession, never through a bare End/Begin pair. `Session::End` restores the
    // properties to their pre-Begin snapshot, so a session reopened any other way carries core's
    // identity set plus the one property `SessionRunner::Begin` re-applies, `c3d.ros.pose_convention`
    // -- and nothing else: no `c3d.app.engine.version`, no `c3d.device.os`, no other `c3d.ros.*`,
    // no participant id, no session name, none of the operator's own `session_properties`.
    std::string participant_id;
    std::string session_name;
    c3d::Properties properties;
    {
      std::lock_guard<std::mutex> lock(request_state_mu_);
      participant_id = requested_participant_id_;
      session_name = requested_session_name_;
      properties = requested_session_properties_;
    }
    // Before End, as in StopSession: the runner must still take the last access units.
    StopCameraEncoders(cameras_);
    runner_->End("ros_time_jumped_backwards", SessionEndOnWire());
    if (!StartSession(participant_id, /*reuse_remote_variables=*/true)) {
      RCLCPP_ERROR(get_logger(), "could not reopen a session after the backward clock jump");
      return;
    }
    ApplySessionIdentity(session_name);
    for (const auto& kv : properties) {
      runner_->session()->SetProperty(kv.first, kv.second);
    }
    return;
  }

  const double wire_time = WireNow();
  // The timer is wall time. Under `sim_time_on_wire` the wire is not, so the rate is held on the
  // wire's own timeline: a grid of one period, taken a little early rather than skipping a tick
  // for jitter, and restarted rather than caught up after a pause.
  if (sim_time_on_wire_) {
    const double period = sampler_config_.rate_hz > 0.0 ? 1.0 / sampler_config_.rate_hz : 0.1;
    if (next_sample_wire_ > 0.0 && wire_time < next_sample_wire_ - 0.1 * period) return;
    next_sample_wire_ = next_sample_wire_ > 0.0 && wire_time < next_sample_wire_ + period
                            ? next_sample_wire_ + period
                            : wire_time + period;
  }
  const double steady_now = steady_clock_.now().seconds();
  std::string status;
  if (pose_sampler_->Sample(wire_time, *runner_, &status)) {
    ++pose_samples_;
  } else if (pose_sampler_->holding_for_preferred_frame()) {
    // Bounded by reference_frame_grace_s, and not a failure: a pose was there
    // and was deliberately not taken. Counting it would put a fabricated
    // `c3d.pose_failures` step at the start of every localised robot's session.
  } else if (pose_sampler_->last_sample_stale()) {
    ++pose_failures_;
    if (steady_now - last_pose_warn_ > 10.0) {
      last_pose_warn_ = steady_now;
      RCLCPP_WARN(get_logger(),
                  "pose transform is stale (%s): whatever publishes it may have stopped, or its "
                  "clock may have. Samples are skipped and counted as c3d.stale_poses until it "
                  "advances again.",
                  status.c_str());
    }
  } else {
    ++pose_failures_;
    // Throttled: a robot that has not started its localisation yet would
    // otherwise produce one of these per sample.
    if (steady_now - last_pose_warn_ > 10.0) {
      last_pose_warn_ = steady_now;
      RCLCPP_WARN(get_logger(),
                  "no pose yet (%s). Without a pose the session has no spatial "
                  "timeline; check reference_frame and robot_frame.",
                  status.c_str());
    }
  }
  // A gaze_frame that never resolves is the quietest misconfiguration this node
  // has: nothing fails, the contract passes, and the properties claim the frame.
  // Say so, throttled, and keep saying it.
  // Throttle FIRST, then consume: the other order spends the notice on a tick
  // the throttle rejects and the warning is lost rather than delayed.
  if (steady_now - last_gaze_frame_warn_ > 10.0 &&
      pose_sampler_->ConsumeGazeFrameFailureNotice()) {
    last_gaze_frame_warn_ = steady_now;
    RCLCPP_WARN(get_logger(),
                "gaze_frame '%s' could not be resolved (%llu samples so far); the gaze "
                "channel is falling back to '%s'. The session will look healthy and its "
                "replay viewpoint will be at the wrong height. Check the frame name against "
                "`ros2 run tf2_tools view_frames --wait-time 15`.",
                sampler_config_.gaze_frame.c_str(),
                static_cast<unsigned long long>(pose_sampler_->gaze_frame_lookup_failures()),
                sampler_config_.robot_frame.c_str());
  }

  WarnOnSuppressedSources();
  RecordSelfTelemetry(wire_time);
}

void Cognitive3DNode::RecordSelfTelemetry(double wire_time) {
  if (runner_ == nullptr || runner_->session() == nullptr) return;
  // A member, not a function-local static: this node ships as a composable
  // component, so two of them in one container would share the static and each
  // suppress the other's telemetry.
  if (wire_time - last_self_telemetry_ < 5.0) return;
  last_self_telemetry_ = wire_time;

  auto* session = runner_->session();
  const c3d::Stats stats = session->stats();
  // The node's own health, recorded into the session it produces. When someone
  // asks whether a gap in the data is real, this is the answer.
  session->RecordSensor("c3d.spool_chunks", wire_time, static_cast<double>(stats.spool_count));
  if (!sampler_config_.gaze_frame.empty() && pose_sampler_) {
    session->RecordSensor("c3d.gaze_frame_failures", wire_time,
                          static_cast<double>(pose_sampler_->gaze_frame_lookup_failures()));
  }
  session->RecordSensor("c3d.spool_mb", wire_time,
                        static_cast<double>(stats.spool_bytes) / (1024.0 * 1024.0));
  session->RecordSensor("c3d.records_dropped", wire_time,
                        static_cast<double>(stats.records_dropped));
  session->RecordSensor("c3d.upload_failures", wire_time,
                        static_cast<double>(stats.upload_failures));
  for (const auto& [name, value] : UploadSeries(stats)) {
    session->RecordSensor(name, wire_time, value);
  }
  session->RecordSensor("c3d.pose_failures", wire_time, static_cast<double>(pose_failures_));
  if (pose_sampler_) {
    session->RecordSensor("c3d.stale_poses", wire_time,
                          static_cast<double>(pose_sampler_->stale_transforms()));
  }
  session->RecordSensor("c3d.events_suppressed", wire_time,
                        static_cast<double>(event_limiter_.suppressed()));
  session->RecordSensor("c3d.event_properties_dropped", wire_time,
                        static_cast<double>(event_properties_dropped_));
  if (pose_sampler_) {
    session->RecordSensor("c3d.localisation_jumps", wire_time,
                          static_cast<double>(pose_sampler_->relocalisation_jumps()));
  }

  // Per sensor, because "the camera stream is healthy" is not a fact a fleet has:
  // one camera's pose failures and another's are different problems with the same
  // symptom in an aggregate.
  const std::map<std::string, std::uint64_t>& too_large = stats.camera_chunks_too_large;
  for (const auto& capture : cameras_) {
    const std::string prefix = "c3d.camera." + capture->config().name + ".";
    // A copy: the encoder's reader thread advances frames and bytes while this reads them.
    const CameraStats stats = capture->stats();
    session->RecordSensor(prefix + "frames", wire_time, static_cast<double>(stats.frames));
    session->RecordSensor(prefix + "bytes", wire_time, static_cast<double>(stats.bytes));
    session->RecordSensor(prefix + "pose_failures", wire_time,
                          static_cast<double>(stats.pose_failures));
    // The refusals are a SEPARATE series from the frame count, not the gap
    // between it and zero: a rate gate dropping frames and a codec refusing them
    // look identical in `frames` alone, and only one of them is a misconfiguration.
    session->RecordSensor(prefix + "refused", wire_time, static_cast<double>(stats.refused));
    session->RecordSensor(prefix + "dropped_backlog", wire_time,
                          static_cast<double>(stats.dropped_backlog));
    // These two count since configure: the stop that ends a session comes after its last
    // tick, so a drop there shows in the next session's series.
    session->RecordSensor(prefix + "dropped_at_stop", wire_time,
                          static_cast<double>(stats.dropped_at_stop));
    session->RecordSensor(prefix + "dropped_no_session", wire_time,
                          static_cast<double>(stats.dropped_no_session));
    session->RecordSensor(prefix + "encoder_restarts", wire_time,
                          static_cast<double>(stats.encoder_restarts));
    session->RecordSensor(prefix + "gap_restarts", wire_time,
                          static_cast<double>(stats.gap_restarts));
    session->RecordSensor(prefix + "size_restarts", wire_time,
                          static_cast<double>(stats.size_restarts));
    session->RecordSensor(prefix + "dropped_replay", wire_time,
                          static_cast<double>(stats.dropped_replay));
    session->RecordSensor(prefix + "backward_stamps", wire_time,
                          static_cast<double>(stats.backward_stamps));
    const auto rejected = too_large.find(capture->config().name);
    session->RecordSensor(prefix + "rejected_too_large", wire_time,
                          rejected == too_large.end() ? 0.0
                                                      : static_cast<double>(rejected->second));
  }
  for (const auto& capture : lidars_) {
    const std::string prefix = "c3d.lidar." + capture->config().name + ".";
    const LidarStats stats = capture->stats();
    session->RecordSensor(prefix + "scans", wire_time, static_cast<double>(stats.scans));
    session->RecordSensor(prefix + "bytes", wire_time, static_cast<double>(stats.bytes));
    session->RecordSensor(prefix + "dropped_oversize", wire_time,
                          static_cast<double>(stats.dropped_oversize));
    session->RecordSensor(prefix + "dropped_range", wire_time,
                          static_cast<double>(stats.dropped_range));
    session->RecordSensor(prefix + "pose_failures", wire_time,
                          static_cast<double>(stats.pose_failures));
    session->RecordSensor(prefix + "refused", wire_time, static_cast<double>(stats.refused));
    session->RecordSensor(prefix + "backward_stamps", wire_time,
                          static_cast<double>(stats.backward_stamps));
  }
}

void Cognitive3DNode::OnDiscoveryTimer() {
  if (clock_policy_) {
    clock_policy_->Poll();
    if (!clock_policy_->RosTimeReady()) {
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 10000,
                           "use_sim_time is set but ROS time is still zero: nothing "
                           "is publishing /clock yet.");
    }
  }
  // Topics appear late all the time, especially on a robot where drivers and
  // the navigation stack come up in stages.
  DiscoverAndSubscribe();
  DiscoverAndSubscribeMedia();
}

void Cognitive3DNode::OnReportTimer() {
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();

  std::size_t refused = 0;
  for (const auto& kv : stats_) {
    if (kv.second.refused) ++refused;
  }
  std::ostringstream summary;
  summary << "observing " << subscriptions_.size() << " topic(s)";
  if (refused > 0) summary << ", refusing " << refused;
  if (runner_ && runner_->session()) {
    const c3d::Stats st = runner_->session()->stats();
    const UploadHealth health = DescribeUploads(st, c3d_config_.retry_max_s);
    summary << "\n  session " << runner_->session()->session_id() << "  poses=" << pose_samples_
            << " frame=" << (pose_sampler_ ? pose_sampler_->active_reference_frame() : "-")
            << "  recorded=" << st.records_recorded << " dropped=" << st.records_dropped
            << "  uploaded=" << st.chunks_uploaded << " spooled=" << st.spool_count
            << health.report;
    // Reported SEPARATELY, and neither claims the other's cause. `chunks_discarded` is
    // every payload rejection (400, 413, 415, 422); a refused key discards nothing, and
    // is the credentials flag above. `unauthorized_retries` counts allowance retries
    // that were SCHEDULED, most of which then succeed, so it is not a subset of the
    // discards and must not be printed as one.
    if (st.chunks_discarded > 0) {
      summary << "  [" << st.chunks_discarded
              << " chunk(s) REJECTED and discarded; the log names the status of each]";
    }
    if (st.unauthorized_retries > 0) summary << "  401retries=" << st.unauthorized_retries;
    const std::uint64_t suppressed = event_limiter_.suppressed();
    if (events_recorded_ > 0 || events_dropped_ > 0 || suppressed > 0) {
      summary << "  events=" << events_recorded_;
      if (events_dropped_ > 0) summary << " DROPPED=" << events_dropped_;
      if (suppressed > 0) summary << " suppressed=" << suppressed;
    }
    diagnostic_msgs::msg::DiagnosticStatus uploads;
    uploads.name = "cognitive3d: uploads";
    uploads.hardware_id = c3d_config_.device_id;
    uploads.level = health.level;
    uploads.message = health.message;
    for (const auto& [key, value] : health.values) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = value;
      uploads.values.push_back(item);
    }
    array.status.push_back(std::move(uploads));
  } else if (dry_run_) {
    summary << "  [dry run: nothing is uploaded]";
  }

  for (const auto& kv : stats_) {
    const TopicStat& s = kv.second;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "cognitive3d: " + s.topic;
    status.hardware_id = c3d_config_.device_id;

    // A rate needs a meaningful span to be meaningful. Two latched messages
    // arriving microseconds apart otherwise report thousands of hertz, which
    // reads as a bug to anyone looking at the summary.
    const double span = s.last_seen_wall - s.first_seen_wall;
    const bool rate_meaningful = s.messages >= 3 && span >= 1.0;
    const double observed_hz =
        rate_meaningful ? static_cast<double>(s.messages - 1) / span : 0.0;

    if (s.refused) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "refused (" + s.refusal + ", not subscribed)";
    } else if (s.incompatible_qos) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "QoS incompatible with a publisher, no data arrives from it";
    } else if (!s.subscribed) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "not subscribed";
    } else if (s.messages == 0) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "subscribed but silent";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "receiving";
    }

    auto kv_add = [&status](const std::string& k, const std::string& v) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = k;
      item.value = v;
      status.values.push_back(item);
    };
    kv_add("type", s.type);
    // As the `subscribed` line says it: absent for a type default that does nothing.
    const std::string ingest =
        s.refused ? std::string(IngestName(Ingest::kRefuse))
                  : SubscribedIngestText(plan_.Resolve(s.topic, s.type), s.type);
    if (!ingest.empty()) kv_add("ingest", ingest);
    kv_add("qos", s.qos_description);
    kv_add("qos_rationale", s.qos_rationale);
    kv_add("messages", std::to_string(s.messages));
    kv_add("translated", std::to_string(s.translated));
    kv_add("has_translator", s.has_translator ? "true" : "false");
    kv_add("bytes", std::to_string(s.bytes));
    kv_add("observed_hz", rate_meaningful ? std::to_string(observed_hz) : "n/a");
    kv_add("configured_rate_hz", std::to_string(s.rate_hz));
    array.status.push_back(std::move(status));

    summary << "\n  " << s.topic << "  ";
    if (s.refused) {
      summary << "REFUSED (" << s.type << ", " << s.refusal << ", never subscribed)";
      continue;
    }
    summary << s.messages << " msg  ";
    if (s.has_translator) summary << s.translated << " xlat  ";
    else summary << "no-xlat  ";
    if (rate_meaningful) summary << static_cast<int>(observed_hz) << "Hz  ";
    else summary << "-Hz  ";
    summary << s.bytes / 1024 << "KiB  " << s.qos_description
            << (s.incompatible_qos ? "  [QOS INCOMPATIBLE]" : "");
  }

  if (diagnostics_pub_ && diagnostics_pub_->is_activated()) diagnostics_pub_->publish(array);
  RCLCPP_INFO(get_logger(), "%s", summary.str().c_str());
}

}  // namespace cognitive3d_ros

RCLCPP_COMPONENTS_REGISTER_NODE(cognitive3d_ros::Cognitive3DNode)
