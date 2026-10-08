// `doctor` -- says why a run will not work, before it does not work.
//
// The design decision worth knowing: this does NOT re-derive the configuration. It constructs the
// REAL node with dry_run set, calls configure(), and reports what the node resolved and what its
// own guards said. A preflight tool that works the configuration out for itself checks its own
// idea of it and drifts out of step with the node silently -- which is the failure class `doctor`
// exists to end, so building it that way would be self-defeating.
//
// dry_run means no session is opened, no transport is constructed and nothing is uploaded. A
// preflight that burns a production session to tell you your config is wrong is not a preflight,
// and a session can only be tagged, never deleted.
#include "cognitive3d_ros/run_doctor.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <tf2_ros/transform_listener.h>

#include <ament_index_cpp/get_package_prefix.hpp>

#include "c3d/environment.hpp"
#include "c3d/spool.hpp"
#include "c3d/transport.hpp"
#include "cognitive3d_ros/doctor.hpp"
#include "cognitive3d_ros/identity.hpp"
#include "cognitive3d_ros/media_capture.hpp"
#include "cognitive3d_ros/node.hpp"
#include "cognitive3d_ros/qos_resolver.hpp"
#include "cognitive3d_ros/topic_plan.hpp"
#include "cognitive3d_ros/translators.hpp"

namespace cognitive3d_ros {
namespace {

using cognitive3d_ros::doctor::Check;
using cognitive3d_ros::doctor::Severity;
namespace fs = std::filesystem;

// How long to wait for a transform before calling it absent. tf needs a moment to fill after the
// listener starts, so asking immediately would report a healthy robot as broken.
// Long enough for DDS discovery to surface a node that is genuinely already there, short
// enough not to be felt. Reading the graph once misses it; this is a real telemetry node
// running beside a preflight, which is the case the check exists for.
constexpr auto kDuplicateWait = std::chrono::seconds(2);
// How often the graph is re-read while doctor is alive with its own session services up.
constexpr auto kDuplicateScan = std::chrono::milliseconds(500);
constexpr auto kFrameWait = std::chrono::seconds(10);
// How long to watch every configured camera and lidar for one message each. Short:
// the slowest stream this SDK is meant for is 1 Hz, and a preflight that takes
// ten seconds is one nobody runs.
constexpr auto kMediaWatch = std::chrono::milliseconds(2500);

// How long to watch the pose channel once it resolves. Short on purpose: this can only ever
// prove the channel carries height or tilt, and a preflight runs on a robot that is parked.
constexpr auto kPoseChannelWindow = std::chrono::seconds(2);

bool Resolves(const std::shared_ptr<tf2_ros::Buffer>& buffer, const std::string& target,
              const std::string& source) {
  if (!buffer || target.empty() || source.empty()) return false;
  std::string ignored;
  return buffer->canTransform(target, source, tf2::TimePointZero, &ignored);
}

// A directory this creates is the one the node would have created: the node keeps a spool_dir it
// finds as it is.
//
// The probe is only ever a file this call creates. O_EXCL refuses a name that exists, a link
// included, so nothing already in the directory is written through, truncated or removed. A taken
// name says nothing about whether the directory is writable, so a fresh one is tried in its place.
bool DirectoryWritable(const std::string& directory) {
  if (directory.empty()) return false;
  c3d::MakeSpoolDirectory(directory);
  std::string probe = (fs::path(directory) / ".c3d-doctor-write-probe").string();
  int fd = ::open(probe.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600);
  if (fd < 0 && errno == EEXIST) {
    probe += ".XXXXXX";
    fd = ::mkostemp(probe.data(), O_CLOEXEC);
  }
  if (fd < 0) return false;
  const bool written = ::write(fd, "probe", 5) == 5;
  ::close(fd);
  ::unlink(probe.c_str());
  return written;
}

// Sessions on disk that have not ended in order: an orderly end removes its session's marker.
//
// Deliberately NOT called "orphaned": the node's own recovery reads only `.c3dsession` files and
// defers a marker whose `last_seen` is recent, because a session recording RIGHT NOW has a marker
// too. Only `.c3dsession` files count: a `.tmp` or an unrelated `notes.txt` in `sessions/` is not a
// session, and must not make a live session read as a crashed leftover about to be closed. Running
// `doctor` while the node records is the natural "why is my session empty?" move, so that message
// would land on exactly the wrong person.
int UnendedSessionMarkers(const std::string& spool_dir) {
  std::error_code ec;
  const auto sessions = fs::path(spool_dir) / "sessions";
  if (!fs::is_directory(sessions, ec)) return 0;
  int unended = 0;
  for (const auto& entry : fs::directory_iterator(sessions, ec)) {
    if (entry.is_regular_file() && entry.path().extension() == ".c3dsession") ++unended;
  }
  return unended;
}

Check ProbeGateway(const c3d::Config& config) {
  // A deliberately empty POST to the configured host. This asks exactly one question -- is there
  // something there speaking HTTP -- and nothing more. The collector answers 200 to a payload it is
  // about to discard and 200 for a scene id that does not exist, so no status from it is a receipt,
  // and `doctor` must not imply otherwise.
  //
  // NO AUTHORIZATION HEADER. A request the gateway does not route is answered 404 before any
  // authentication runs, so a probe carrying a key could never tell a good key from a bad one, and
  // a wrong, rotated or other-project key would pass as `[ok] gateway reachable`. Reachability is
  // all this check can honestly claim, and it needs no credential.
  c3d::CurlTransport transport(config.http_timeout_s);
  const std::string url = config.protocol + "://" + config.gateway + "/health";
  const c3d::HttpResponse response = transport.Post(url, "", {});
  return cognitive3d_ros::doctor::CheckGateway(config.protocol, config.gateway, response.status,
                                               response.saw_request_time_header, response.error);
}

// The mesh AND the id, from ONE request: the object list carries both, and the two questions are
// asked of the same rows. They are separate CHECKS because their consequences differ -- a missing
// mesh empties the replay, a mismatched id does not -- but they are one probe.
// `scene` is the scene document scene/project read, which this reads again for its version id.
std::vector<Check> ProbeRobotObjects(const c3d::Config& config, const std::string& api_host,
                                     bool robot_dynamic_object, const std::string& robot_mesh,
                                     const std::string& robot_object_id,
                                     const doctor::SceneProjectFacts& scene,
                                     const std::string& scene_body) {
  // Asked BEFORE any fact is gathered: with no object registered there is no question the API
  // could answer that would change the run, and a preflight must not spend three requests on one.
  if (!robot_dynamic_object) {
    return cognitive3d_ros::doctor::RobotObjectChecks(false, {}, {}, {});
  }
  cognitive3d_ros::doctor::RobotMeshFacts mesh_facts;
  cognitive3d_ros::doctor::RobotObjectIdFacts id_facts;
  mesh_facts.robot_mesh = robot_mesh;
  mesh_facts.scene_version = config.scene_version;
  id_facts.robot_object_id = robot_object_id;
  const char* project_env = std::getenv("C3D_PROJECT_ID");
  mesh_facts.project = project_env ? project_env : "";
  id_facts.project = mesh_facts.project;
  const char* org_key_env = std::getenv("C3D_ORG_API_KEY");
  const std::string org_key = org_key_env ? org_key_env : "";
  mesh_facts.org_key_present = !org_key.empty();
  id_facts.org_key_present = mesh_facts.org_key_present;

  // The decision itself is in the pure layer, where a test can hold it: the property is that
  // nothing in a params file can move this host, and that is only testable of a function which
  // receives the params.
  const cognitive3d_ros::doctor::ApiDestination destination =
      cognitive3d_ros::doctor::ApiDestinationFor(config, api_host);
  mesh_facts.destination_permitted = destination.permitted;
  id_facts.destination_permitted = destination.permitted;

  const bool nothing_to_ask = (robot_mesh.empty() && robot_object_id.empty()) ||
                              mesh_facts.project.empty() || !mesh_facts.org_key_present ||
                              !mesh_facts.destination_permitted;
  cognitive3d_ros::doctor::RobotMeshFilesFacts files_facts;
  files_facts.robot_mesh = robot_mesh;
  files_facts.scene_id = config.scene_id;
  files_facts.scene_version = config.scene_version;
  files_facts.org_key_present = mesh_facts.org_key_present;
  files_facts.destination_permitted = mesh_facts.destination_permitted;

  // The two probes have DIFFERENT preconditions and neither may gate the other. The
  // project-scoped listing needs C3D_PROJECT_ID; the version-scoped one addresses a scene and a
  // version and never names a project, so making it wait on a project id it does not use would
  // silently give up the only check that can see whether the mesh files exist.
  const bool can_ask_version_scoped = !robot_mesh.empty() && !config.scene_id.empty() &&
                                      !config.scene_version.empty() &&
                                      mesh_facts.org_key_present && destination.permitted;
  if (nothing_to_ask && !can_ask_version_scoped) {
    return cognitive3d_ros::doctor::RobotObjectChecks(true, mesh_facts, id_facts, files_facts);
  }

  c3d::CurlTransport transport(config.http_timeout_s);
  const std::vector<std::string> authorization = {
      "Authorization: " + cognitive3d_ros::doctor::OrganizationAuthorization(org_key)};

  if (!nothing_to_ask) {
    const std::string url =
        "https://" + destination.host + "/v0/projects/" + mesh_facts.project + "/objects";
    const c3d::HttpResponse response = transport.Get(url, authorization);
    mesh_facts.status = response.status;
    mesh_facts.transport_error = response.error;
    id_facts.status = response.status;
    id_facts.transport_error = response.error;

    // One parse answers both questions: whether the body was a readable list at all, and what is
    // in it. No heuristic on the body's characters -- "contains a bracket" would promote an error
    // object mentioning "[project 90001]" from unreadable to definitively-absent, advisory to
    // FATAL.
    mesh_facts.body_is_an_object_list =
        cognitive3d_ros::doctor::StringFieldIn(response.body, "meshName", &mesh_facts.available);
    id_facts.body_is_an_object_list =
        cognitive3d_ros::doctor::StringFieldIn(response.body, "sdkId", &id_facts.available);
    mesh_facts.mesh_listed = std::any_of(
        mesh_facts.available.begin(), mesh_facts.available.end(),
        [&mesh_facts](const std::string& name) { return name == mesh_facts.robot_mesh; });
    id_facts.id_listed = std::any_of(
        id_facts.available.begin(), id_facts.available.end(),
        [&id_facts](const std::string& id) { return id == id_facts.robot_object_id; });
  }

  // The project-scoped listing cannot see FILES -- it answers about the object ROW, which a
  // half-finished upload still has. The version-scoped route can, but it wants the NUMERIC version
  // id, which the node does not hold: `scene_version` is the version NUMBER. So the scene document
  // translates one into the other.
  if (can_ask_version_scoped) {
    files_facts.scene_status = scene.status;
    files_facts.transport_error = scene.transport_error;
    files_facts.scene_read_by_scene_project = scene.body_is_a_scene;
    if (scene.status == 200) {
      files_facts.scene_body_is_readable = cognitive3d_ros::doctor::VersionIdForNumber(
          scene_body, files_facts.scene_version, &files_facts.version_id);
    }
    if (!files_facts.version_id.empty()) {
      const std::string objects_url = "https://" + destination.host + "/v0/versions/" +
                                      files_facts.version_id + "/objects?includeFiles=true";
      const c3d::HttpResponse objects = transport.Get(objects_url, authorization);
      files_facts.status = objects.status;
      if (!objects.error.empty()) files_facts.transport_error = objects.error;
      files_facts.body_is_an_object_list = cognitive3d_ros::doctor::MeshFilesIn(
          objects.body, files_facts.robot_mesh, &files_facts.mesh_listed, &files_facts.files);
    }
  }

  return cognitive3d_ros::doctor::RobotObjectChecks(true, mesh_facts, id_facts, files_facts);
}

// One observation window for every configured camera and lidar at once, rather
// than one per sensor: a preflight that waited two seconds per sensor would take
// ten on a robot with five, and the window is the same window for all of them.
//
// Each camera's CameraInfo is watched in the same window, for its field of view check.
std::vector<cognitive3d_ros::doctor::MediaSensorFacts> ProbeMediaSensors(
    const std::shared_ptr<cognitive3d_ros::Cognitive3DNode>& node,
    rclcpp::executors::SingleThreadedExecutor& executor,
    const std::shared_ptr<tf2_ros::Buffer>& buffer, const std::string& reference_frame,
    std::vector<cognitive3d_ros::doctor::CameraFovFacts>* fov_facts) {
  using cognitive3d_ros::doctor::MediaSensorFacts;
  std::vector<MediaSensorFacts> facts;

  struct Probe {
    MediaSensorFacts facts;
    rclcpp::GenericSubscription::SharedPtr subscription;
    std::shared_ptr<const rclcpp::SerializedMessage> sample;
    std::size_t index = 0;
  };
  std::vector<std::shared_ptr<Probe>> probes;

  const auto& cameras = node->resolved_cameras();
  const auto& lidars = node->resolved_lidars();
  if (cameras.empty() && lidars.empty()) return facts;

  const auto graph = node->get_topic_names_and_types();
  auto type_on_graph = [&graph](const std::string& topic) {
    for (const auto& entry : graph) {
      if (entry.first == topic && !entry.second.empty()) return entry.second.front();
    }
    return std::string();
  };

  auto add = [&](const std::string& kind, const std::string& name, const std::string& topic,
                 double rate_hz, bool handled_type_check(const std::string&),
                 const cognitive3d_ros::QosOverride& qos_override, std::size_t index) {
    auto probe = std::make_shared<Probe>();
    probe->index = index;
    probe->facts.kind = kind;
    probe->facts.name = name;
    probe->facts.topic = topic;
    probe->facts.rate_hz = rate_hz;
    probe->facts.type = type_on_graph(topic);
    probe->facts.topic_present = !probe->facts.type.empty();
    probe->facts.type_handled =
        probe->facts.topic_present && handled_type_check(probe->facts.type);
    if (probe->facts.type_handled) {
      const auto resolution = cognitive3d_ros::ResolveQos(node.get(), topic, qos_override);
      probe->facts.qos = cognitive3d_ros::DescribeQos(resolution.qos);
      try {
        probe->subscription = node->create_generic_subscription(
            topic, probe->facts.type, resolution.qos,
            [probe](std::shared_ptr<const rclcpp::SerializedMessage> message) {
              if (!probe->sample) probe->sample = message;
            });
      } catch (const std::exception&) {
        // A missing typesupport package is already reported by the node's own
        // subscribe path; here it simply means no sample.
      }
    }
    probes.push_back(probe);
  };

  for (std::size_t i = 0; i < cameras.size(); ++i) {
    add("camera", cameras[i].name, cameras[i].topic, cameras[i].rate_hz,
        cognitive3d_ros::IsCameraType, cameras[i].qos, i);
  }
  for (std::size_t i = 0; i < lidars.size(); ++i) {
    add("lidar", lidars[i].name, lidars[i].topic, lidars[i].rate_hz,
        cognitive3d_ros::IsLidarType, cognitive3d_ros::QosOverride{}, i);
  }

  struct InfoProbe {
    cognitive3d_ros::doctor::CameraFovFacts facts;
    rclcpp::GenericSubscription::SharedPtr subscription;
    std::shared_ptr<const rclcpp::SerializedMessage> sample;
  };
  std::vector<std::shared_ptr<InfoProbe>> info_probes;
  for (const auto& camera : cameras) {
    auto probe = std::make_shared<InfoProbe>();
    probe->facts.name = camera.name;
    probe->facts.parameters_set = camera.fov_set();
    probe->facts.hfov_degrees = camera.hfov_degrees;
    probe->facts.vfov_degrees = camera.vfov_degrees;
    probe->facts.camera_info_topic = camera.camera_info_topic;
    if (!probe->facts.parameters_set && !camera.camera_info_topic.empty()) {
      probe->facts.type = type_on_graph(camera.camera_info_topic);
      probe->facts.topic_present = !probe->facts.type.empty();
      if (probe->facts.type == "sensor_msgs/msg/CameraInfo") {
        const auto resolution = cognitive3d_ros::ResolveQos(node.get(), camera.camera_info_topic,
                                                            cognitive3d_ros::QosOverride{});
        try {
          probe->subscription = node->create_generic_subscription(
              camera.camera_info_topic, probe->facts.type, resolution.qos,
              [probe](std::shared_ptr<const rclcpp::SerializedMessage> message) {
                if (!probe->sample) probe->sample = message;
              });
        } catch (const std::exception&) {
          // As for an image topic: no subscription is simply no sample.
        }
      }
    }
    info_probes.push_back(probe);
  }

  const auto deadline = std::chrono::steady_clock::now() + kMediaWatch;
  while (std::chrono::steady_clock::now() < deadline) {
    bool all_seen = true;
    for (const auto& probe : probes) {
      if (probe->subscription && !probe->sample) all_seen = false;
    }
    for (const auto& probe : info_probes) {
      if (probe->subscription && !probe->sample) all_seen = false;
    }
    if (all_seen) break;
    executor.spin_some(std::chrono::milliseconds(20));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  cognitive3d_ros::PoseConvention convention = cognitive3d_ros::PoseConvention::kUnity;
  cognitive3d_ros::ParsePoseConvention(node->get_parameter("pose_convention").as_string(),
                                       &convention);

  for (auto& probe : probes) {
    MediaSensorFacts& sensor = probe->facts;
    const bool is_camera = sensor.kind == "camera";
    const std::string configured_frame =
        is_camera ? cameras[probe->index].frame : lidars[probe->index].frame;
    sensor.frame = configured_frame;

    if (probe->sample) {
      sensor.sample_seen = true;
      std_msgs::msg::Header header;
      if (sensor.frame.empty() &&
          cognitive3d_ros::MediaMessageHeader(sensor.type, *probe->sample, &header)) {
        sensor.frame = header.frame_id;
      }
      std::string why;
      std::uint64_t bytes = 0;
      if (is_camera) {
        cognitive3d_ros::CameraCapture capture(cameras[probe->index]);
        std::uint16_t width = 0, height = 0;
        capture.MeasureEncodedFrame(sensor.type, *probe->sample, &bytes, &width, &height, &why);
      } else {
        cognitive3d_ros::LidarCapture capture(lidars[probe->index]);
        capture.MeasureEncodedScan(sensor.type, *probe->sample, convention, &bytes, &why);
        // The RAW points, before ConvertPosition: the question CheckOpticalCloud
        // asks is which ROS frame the publisher put them in, and the scene map
        // would hide it.
        cognitive3d_ros::PointCloudAxisSigns signs;
        if (cognitive3d_ros::MediaPointCloudAxisSigns(sensor.type, *probe->sample, &signs)) {
          sensor.finite_points = signs.finite;
          sensor.x_positive_points = signs.x_positive;
          sensor.z_positive_points = signs.z_positive;
        }
      }
      sensor.encoded_bytes = bytes;
      sensor.encode_error = why;
    }
    sensor.frame_resolves =
        !sensor.frame.empty() && Resolves(buffer, reference_frame, sensor.frame);
    probe->subscription.reset();
    facts.push_back(sensor);
  }
  for (auto& probe : info_probes) {
    if (probe->sample) {
      sensor_msgs::msg::CameraInfo info;
      if (cognitive3d_ros::Deserialize(*probe->sample, &info)) {
        probe->facts.sample_seen = true;
        const c3d::CameraFov fov = cognitive3d_ros::CameraFovFromCameraInfo(info);
        probe->facts.sample_hfov_degrees = fov.hfov_deg;
        probe->facts.sample_vfov_degrees = fov.vfov_deg;
        probe->facts.sample_fx = info.k[0];
      }
    }
    probe->subscription.reset();
    if (fov_facts) fov_facts->push_back(probe->facts);
  }
  return facts;
}

// Does this project's key list contain the configured application key?
//
// Containment on the raw body rather than a JSON parse: core ships no JSON reader, the response
// holds only that project's own keys, and a 32-character key appearing in it is decisive. The two
// ways containment could answer the wrong question are handled where they belong -- the pure layer
// refuses to run below `kShortestMaskable`, and requires the body to carry `"apiKey"` before it
// will read an absence as a FATAL rather than as an unreadable answer.
doctor::ProjectKeyFacts ProbeProjectKey(const c3d::Config& config, const std::string& api_host) {
  cognitive3d_ros::doctor::ProjectKeyFacts facts;
  facts.application_key = config.application_key;
  const char* expected_project_env = std::getenv("C3D_PROJECT_ID");
  facts.expected_project = expected_project_env ? expected_project_env : "";
  const char* org_key_env = std::getenv("C3D_ORG_API_KEY");
  const std::string org_key = org_key_env ? org_key_env : "";
  facts.org_key_present = !org_key.empty();

  // Same pure decision as the mesh probe. See ApiDestinationFor: a params file cannot name the
  // host an organisation key is sent to, and a test proves that of the function itself.
  const cognitive3d_ros::doctor::ApiDestination destination =
      cognitive3d_ros::doctor::ApiDestinationFor(config, api_host);
  facts.destination_permitted = destination.permitted;

  if (facts.expected_project.empty() || !facts.org_key_present || !facts.destination_permitted ||
      facts.application_key.size() < 16) {
    return facts;
  }

  c3d::CurlTransport transport(config.http_timeout_s);
  const std::string url =
      "https://" + api_host + "/v0/projects/" + facts.expected_project + "/apiKeys/data";
  const c3d::HttpResponse response = transport.Get(
      url, {"Authorization: " + cognitive3d_ros::doctor::OrganizationAuthorization(org_key)});
  facts.status = response.status;
  facts.transport_error = response.error;
  facts.body_is_a_key_list = response.body.find("\"apiKey\"") != std::string::npos;
  facts.key_listed = response.body.find(facts.application_key) != std::string::npos;
  return facts;
}

}  // namespace

doctor::SceneProjectFacts ProbeSceneProject(doctor::SceneProjectFacts facts,
                                            const std::string& api_base,
                                            const std::string& org_key, int timeout_s,
                                            std::string* body) {
  c3d::CurlTransport transport(timeout_s);
  const c3d::HttpResponse response =
      transport.Get(api_base + "/v0/scenes/" + facts.scene_id,
                    {"Authorization: " + doctor::OrganizationAuthorization(org_key)});
  facts.status = response.status;
  facts.transport_error = response.error;
  facts.body_is_a_scene = response.status == 200 &&
                          doctor::SceneDocumentIn(response.body, &facts.scene_project,
                                                  &facts.versions);
  if (body != nullptr) *body = response.body;
  return facts;
}

doctor::TopicQosFacts TopicQosFor(const std::string& topic,
                                  const std::vector<rclcpp::TopicEndpointInfo>& publishers,
                                  const TopicRule& rule) {
  doctor::TopicQosFacts facts;
  facts.topic = topic;
  facts.override_key = rule.config_key;
  facts.override_reliability = rule.qos.reliability;
  facts.override_durability = rule.qos.durability;
  for (const rclcpp::TopicEndpointInfo& publisher : publishers) {
    const rmw_qos_profile_t& offered = publisher.qos_profile().get_rmw_qos_profile();
    doctor::PublisherQos named;
    const std::string& ns = publisher.node_namespace();
    named.node = (ns.empty() || ns == "/" ? "/" : ns + "/") + publisher.node_name();
    named.reliable = offered.reliability != RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    named.transient_local = offered.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    facts.publishers.push_back(named);
  }
  const QosResolution resolution = ResolveQos(publishers, rule.qos);
  const rmw_qos_profile_t& requested = resolution.qos.get_rmw_qos_profile();
  facts.requests_reliable = requested.reliability != RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  facts.requests_transient_local =
      requested.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  facts.requested = DescribeQos(resolution.qos);
  return facts;
}

std::string LaunchedBuildDoctor(
    const std::function<std::string(const std::string&)>& package_prefix) {
  const std::string prefix = package_prefix("cognitive3d_ros_nav2");
  if (prefix.empty()) return "";
  return prefix + "/lib/cognitive3d_ros_nav2/doctor";
}

std::string InstalledPackagePrefix(const std::string& package, std::string* index_error) {
  try {
    return ament_index_cpp::get_package_prefix(package);
  } catch (const ament_index_cpp::PackageNotFoundError&) {
    return "";
  } catch (const std::exception& thrown) {
    // An unset AMENT_PREFIX_PATH throws a plain runtime_error, not PackageNotFoundError.
    if (index_error != nullptr) *index_error = thrown.what();
    return "";
  }
}

int RunDoctor(int argc, char** argv, const NodeFactory& make, const std::string& package,
              const std::string& handoff_error, const std::string& index_error) {
  doctor::BuildFacts build;
  build.package = package;
  build.handoff_error = handoff_error;
  build.index_error = index_error;

  try {
    rclcpp::init(argc, argv);
  } catch (const std::exception& thrown) {
    // A mistyped --params-file path throws RCLInvalidROSArgsError here, before any of this tool
    // runs. Uncaught, that is exit 250 and a C++ dump for the commonest CLI mistake there is.
    std::vector<Check> failed{doctor::CheckBuild(build),
                              {"command line", Severity::kFatal, thrown.what(),
                               "Check the arguments after --ros-args, especially the path given to "
                               "--params-file."}};
    std::cout << cognitive3d_ros::doctor::Render(failed, cognitive3d_ros::RuntimeDescription());
    return cognitive3d_ros::doctor::ExitCode(failed);
  }

  // dry_run is forced rather than defaulted, so no invocation of `doctor` can open a session even
  // if the operator's own parameter file says auto_start_session.
  rclcpp::NodeOptions options;
  options.append_parameter_override("dry_run", true);
  options.append_parameter_override("auto_start_session", false);

  // Construction and configure are wrapped because rclcpp throws on a mistyped parameter and on
  // an empty YAML list, and an uncaught throw here exits 250 with a C++ dump where a person needs
  // a sentence naming the parameter.
  std::shared_ptr<cognitive3d_ros::Cognitive3DNode> node;
  std::vector<Check> checks;
  try {
    node = make(options);
  } catch (const std::exception& thrown) {
    checks.push_back(doctor::CheckBuild(build));
    checks.push_back(cognitive3d_ros::doctor::CheckConstruction(thrown.what()));
    std::cout << cognitive3d_ros::doctor::Render(checks, cognitive3d_ros::RuntimeDescription());
    rclcpp::shutdown();
    return cognitive3d_ros::doctor::ExitCode(checks);
  }

  // A second node of the same name answers ~/start_session and ~/end_session for as long as this
  // runs, and rclcpp allows the duplicate. While `doctor` is up, a start_session call can be
  // answered by IT with "dry_run is set: no session will be created" -- so a mission runner
  // starting a session beside a `doctor` run can be told the start failed while the real node has
  // opened a session nobody will end. Renaming is not the fix: a node-name-keyed parameter file
  // would then not apply, and `doctor` would be reporting on a different configuration than the
  // node uses.
  //
  // The graph lists THIS node too, so the test is a count and not a membership: a duplicate shows
  // up as a second identical entry. Membership would refuse every invocation, because the name
  // doctor looks for is its own. And the count is polled rather than read once -- discovery is
  // asynchronous, so a duplicate that is genuinely there is not in the graph the instant this node
  // finishes constructing, and missing it is the failure this check exists to prevent.
  // Through the base interface, which has carried this name since Foxy. LifecycleNode is not a
  // Node and grew its own get_fully_qualified_name() only after Humble.
  const std::string own_name = node->get_node_base_interface()->get_fully_qualified_name();
  const auto duplicate_present = [&node, &own_name]() {
    const std::vector<std::string> graph = node->get_node_names();
    return std::count(graph.begin(), graph.end(), own_name) > 1;
  };
  const auto duplicate_check = [&own_name]() {
    return Check{"another node", Severity::kFatal,
                 "a second node called " + own_name + " is running",
                 "That is almost certainly the real telemetry node. Stop it before running "
                 "doctor: while this runs there are two servers for ~/start_session, either may "
                 "answer a caller, and a session opened by the other one could be left unended."};
  };

  build.constructed = true;
  build.translated_types = node->translators().known_types();
  checks.push_back(doctor::CheckBuild(build));

  const auto duplicate_deadline = std::chrono::steady_clock::now() + kDuplicateWait;
  bool duplicate_found = false;
  while (!duplicate_found) {
    duplicate_found = duplicate_present();
    if (duplicate_found || std::chrono::steady_clock::now() >= duplicate_deadline) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (duplicate_found) {
    checks.push_back(duplicate_check());
    std::cout << cognitive3d_ros::doctor::Render(checks, cognitive3d_ros::RuntimeDescription());
    rclcpp::shutdown();
    return cognitive3d_ros::doctor::ExitCode(checks);
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  rclcpp_lifecycle::State configured_state;
  try {
    configured_state = node->configure();
  } catch (const std::exception& thrown) {
    checks.push_back({"configuration", Severity::kFatal, thrown.what(),
                      "configure() threw rather than returning a refusal. Usually a parameter "
                      "type; see the message above."});
    std::cout << cognitive3d_ros::doctor::Render(checks, cognitive3d_ros::RuntimeDescription());
    rclcpp::shutdown();
    return cognitive3d_ros::doctor::ExitCode(checks);
  }
  const auto& configured = configured_state;
  if (configured.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    for (const auto& check :
         cognitive3d_ros::doctor::ChecksForConfigurationRefusal(node->configure_error())) {
      checks.push_back(check);
    }
    std::cout << cognitive3d_ros::doctor::Render(checks, cognitive3d_ros::RuntimeDescription());
    rclcpp::shutdown();
    return cognitive3d_ros::doctor::ExitCode(checks);
  }

  // configure() is where doctor's own ~/start_session and ~/end_session appear, and they live
  // until this process exits. The window the first poll closes ends before they exist, so the
  // graph is read again here and throughout the frame wait below: a real node arriving after a
  // single check would be served by two nodes for the rest of the run, which is the whole hazard.
  if (duplicate_present()) {
    checks.push_back(duplicate_check());
    std::cout << cognitive3d_ros::doctor::Render(checks, cognitive3d_ros::RuntimeDescription());
    rclcpp::shutdown();
    return cognitive3d_ros::doctor::ExitCode(checks);
  }

  const c3d::Config& config = node->resolved_config();
  const cognitive3d_ros::PoseSamplerConfig& sampler = node->resolved_sampler_config();
  // The node fills its buffer only while active, and doctor never activates it. A listener with
  // its own thread adds a node to this process's context and no DDS participant.
  const auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*node->tf_buffer(), true);

  checks.push_back({"configuration", Severity::kOk,
                    "accepted; scene " + config.scene_id + " version " + config.scene_version +
                        ", device " + config.device_id,
                    ""});
  checks.push_back(cognitive3d_ros::doctor::CheckApplicationKey(config.application_key));
  checks.push_back(cognitive3d_ros::doctor::CheckWallClock(
      std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count()));
  const cognitive3d_ros::Destination& destination = node->resolved_destination();
  checks.push_back(cognitive3d_ros::doctor::CheckEnvironment(
      cognitive3d_ros::DescribeDestination(destination, config.gateway)));
  checks.push_back(ProbeGateway(config));
  const doctor::ProjectKeyFacts project_key = ProbeProjectKey(config, destination.hosts.api);
  checks.push_back(doctor::CheckProjectKey(project_key));

  doctor::SceneProjectFacts scene;
  scene.scene_id = config.scene_id;
  scene.scene_version = config.scene_version;
  const char* expected_project_env = std::getenv("C3D_PROJECT_ID");
  scene.expected_project = expected_project_env ? expected_project_env : "";
  const char* org_key_env = std::getenv("C3D_ORG_API_KEY");
  const std::string org_key = org_key_env ? org_key_env : "";
  scene.org_key_present = !org_key.empty();
  scene.org_key_verified = doctor::OrganizationKeyProven(project_key);
  const std::string scene_base = doctor::SceneLookupBase(config, destination.hosts.api);
  scene.destination_permitted = !scene_base.empty();
  std::string scene_body;
  if (scene.org_key_present && scene.destination_permitted) {
    scene = ProbeSceneProject(scene, scene_base, org_key, config.http_timeout_s, &scene_body);
  }
  checks.push_back(doctor::CheckSceneProject(scene));
  checks.push_back(cognitive3d_ros::doctor::CheckSpoolDirectory(
      config.spool_dir, DirectoryWritable(config.spool_dir),
      UnendedSessionMarkers(config.spool_dir),
      cognitive3d_ros::doctor::ProbeSpoolLock(config.spool_dir)));

  // Poll rather than wait a fixed interval, and give it a generous ceiling: a robot whose
  // localisation publishes every 3.5 s must not read "neither frame resolves" FATAL, and a
  // preflight that fails a healthy robot gets muted within a day. Returns as soon as the frame question has
  // an answer -- which the fallback resolving is not, hence FrameWaitSettled and the node's own
  // grace, taken from the configuration the node resolved rather than from a second copy of it.
  const auto buffer = node->tf_buffer();
  const auto deadline = std::chrono::steady_clock::now() + kFrameWait;
  auto next_duplicate_scan = std::chrono::steady_clock::now() + kDuplicateScan;
  bool abandoned_to_duplicate = false;
  bool fallback_seen = false;
  std::chrono::steady_clock::time_point fallback_first_seen;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(20));
    const bool preferred_resolves =
        Resolves(buffer, sampler.preferred_reference_frame, sampler.robot_frame);
    const bool fallback_resolves =
        Resolves(buffer, sampler.fallback_reference_frame, sampler.robot_frame);
    const auto now = std::chrono::steady_clock::now();
    if (fallback_resolves && !fallback_seen) {
      fallback_seen = true;
      fallback_first_seen = now;
    }
    if (cognitive3d_ros::doctor::FrameWaitSettled(
            preferred_resolves, fallback_resolves, !sampler.preferred_reference_frame.empty(),
            fallback_seen ? std::chrono::duration<double>(now - fallback_first_seen).count() : 0.0,
            sampler.reference_frame_grace_s)) {
      break;
    }
    // Scanned on its own interval, not every iteration: this loop turns every 50 ms and reading
    // the whole graph that often to answer a question about a rare event is not worth it.
    if (std::chrono::steady_clock::now() >= next_duplicate_scan) {
      if (duplicate_present()) {
        abandoned_to_duplicate = true;
        break;
      }
      next_duplicate_scan = std::chrono::steady_clock::now() + kDuplicateScan;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (abandoned_to_duplicate) {
    checks.push_back(duplicate_check());
    std::cout << cognitive3d_ros::doctor::Render(checks, cognitive3d_ros::RuntimeDescription());
    rclcpp::shutdown();
    return cognitive3d_ros::doctor::ExitCode(checks);
  }

  checks.push_back(cognitive3d_ros::doctor::CheckFrames(
      sampler.preferred_reference_frame, sampler.fallback_reference_frame, sampler.robot_frame,
      Resolves(buffer, sampler.preferred_reference_frame, sampler.robot_frame),
      Resolves(buffer, sampler.fallback_reference_frame, sampler.robot_frame)));
  checks.push_back(cognitive3d_ros::doctor::CheckGazeFrame(
      sampler.gaze_frame, Resolves(buffer, sampler.gaze_frame, sampler.robot_frame)));

  // WHAT the pose channel contains, having established THAT it resolves. Sampled rather than
  // assumed so that a three-dimensional source can prove itself; the verdict for a source that
  // reads flat is structural and belongs to the check, not here.
  cognitive3d_ros::doctor::PoseChannelFacts channel;
  if (Resolves(buffer, sampler.preferred_reference_frame, sampler.robot_frame)) {
    channel.frame_in_force = sampler.preferred_reference_frame;
  } else if (Resolves(buffer, sampler.fallback_reference_frame, sampler.robot_frame)) {
    channel.frame_in_force = sampler.fallback_reference_frame;
  }
  if (!channel.frame_in_force.empty()) {
    double lowest = 0.0;
    double highest = 0.0;
    std::int64_t last_stamp = -1;
    const auto channel_deadline = std::chrono::steady_clock::now() + kPoseChannelWindow;
    while (std::chrono::steady_clock::now() < channel_deadline) {
      executor.spin_some(std::chrono::milliseconds(20));

      // BEFORE the lookup, not after it. This loop holds ~/start_session up for the whole window,
      // so it re-reads the graph for the same reason the frame wait does -- and below the two
      // `continue`s it would only have run on iterations that saw a fresh transform, which is
      // never at all for a channel that has stopped publishing.
      if (std::chrono::steady_clock::now() >= next_duplicate_scan) {
        if (duplicate_present()) {
          abandoned_to_duplicate = true;
          break;
        }
        next_duplicate_scan = std::chrono::steady_clock::now() + kDuplicateScan;
      }

      geometry_msgs::msg::TransformStamped latest;
      try {
        latest = buffer->lookupTransform(channel.frame_in_force, sampler.robot_frame,
                                         tf2::TimePointZero);
      } catch (const tf2::TransformException&) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }
      // TimePointZero returns the latest available, so a channel that has stopped publishing
      // would otherwise count one transform many times and report a sample count it never had.
      const std::int64_t stamp = static_cast<std::int64_t>(latest.header.stamp.sec) * 1000000000 +
                                 latest.header.stamp.nanosec;
      if (stamp == last_stamp) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }
      last_stamp = stamp;

      const double height = latest.transform.translation.z;
      lowest = channel.samples == 0 ? height : std::min(lowest, height);
      highest = channel.samples == 0 ? height : std::max(highest, height);
      // A planar producer builds roll and pitch from literal zeros and quaternion composition
      // carries them through exactly, so this counter stays at 0 however long the window runs;
      // anything that genuinely carries tilt trips it on the first sample. The magnitude is
      // reported but must not decide -- see CarriesTilt.
      if (cognitive3d_ros::doctor::CarriesTilt(latest.transform.rotation.x,
                                               latest.transform.rotation.y)) {
        ++channel.samples_with_tilt;
      }
      channel.max_tilt_degrees =
          std::max(channel.max_tilt_degrees,
                   cognitive3d_ros::doctor::TiltDegrees(
                       latest.transform.rotation.x, latest.transform.rotation.y,
                       latest.transform.rotation.z, latest.transform.rotation.w));
      ++channel.samples;

      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    channel.height_span_meters = highest - lowest;
  }
  if (abandoned_to_duplicate) {
    checks.push_back(duplicate_check());
    std::cout << cognitive3d_ros::doctor::Render(checks, cognitive3d_ros::RuntimeDescription());
    rclcpp::shutdown();
    return cognitive3d_ros::doctor::ExitCode(checks);
  }
  checks.push_back(cognitive3d_ros::doctor::CheckPoseChannel(channel));
  checks.push_back(cognitive3d_ros::doctor::CheckSimTime(
      node->get_parameter("use_sim_time").as_bool(),
      node->count_publishers("/clock") > 0, node->get_parameter("sim_time_on_wire").as_bool()));
  checks.push_back(cognitive3d_ros::doctor::CheckPoseConvention(
      node->get_parameter("pose_convention").as_string()));
  for (const Check& check :
       ProbeRobotObjects(config, node->resolved_destination().hosts.api,
                         node->get_parameter("robot_dynamic_object").as_bool(),
                         sampler.robot_mesh,
                         node->get_parameter("robot_object_id").as_string(), scene,
                         scene_body)) {
    checks.push_back(check);
  }

  // Presence is decided the way the NODE decides it: the live graph, matched with the node's own
  // TopicMatches. count_publishers() on each raw allowlist entry is wrong twice over -- the exact
  // class of drift this tool exists to end, in the tool itself:
  //
  //   * a trailing-`*` pattern (documented in config/cognitive3d.yaml for multi-robot setups, and
  //     supported by TopicMatches) throws InvalidTopicNameError and aborts with exit 250, printing
  //     nothing at all;
  //   * a RELATIVE name like `clock` is expanded against the node namespace by count_publishers
  //     and reported present, while the node -- which compares allowlist entries against the graph
  //     by exact string -- observes nothing. "Your sensors are fine" about a session that will have
  //     no sensors.
  const auto graph = node->get_topic_names_and_types();
  cognitive3d_ros::doctor::TopicFacts topics;
  topics.allowlist = node->resolved_allowlist();
  // The switches an empty allowlist is read against, so the verdict can say what the run still
  // records rather than claiming it records nothing.
  topics.standard_types_enabled = node->get_parameter("auto_subscribe_standard_types").as_bool();
  topics.action_discovery_enabled = node->get_parameter("action_discovery").as_bool();
  topics.media_count = node->resolved_cameras().size() + node->resolved_lidars().size();
  for (const auto& pattern : topics.allowlist) {
    const bool present = std::any_of(graph.begin(), graph.end(), [&pattern](const auto& entry) {
      return cognitive3d_ros::TopicMatches(pattern, entry.first);
    });
    if (!present) topics.missing.push_back(pattern);
  }
  checks.push_back(cognitive3d_ros::doctor::CheckTopics(topics));

  // Every publisher of every allowlisted topic, read now rather than when the node would subscribe:
  // the waits above give discovery the time the node does not have at startup.
  std::vector<doctor::TopicQosFacts> topic_qos;
  for (const auto& entry : graph) {
    if (entry.second.empty() || !node->resolved_plan().Allowed(entry.first)) continue;
    const TopicRule rule = node->resolved_plan().Resolve(entry.first, entry.second.front());
    if (rule.ingest == Ingest::kRefuse) continue;
    topic_qos.push_back(
        TopicQosFor(entry.first, node->get_publishers_info_by_topic(entry.first), rule));
  }
  checks.push_back(doctor::CheckTopicQos(topic_qos));

  // What a run subscribes to WITHOUT being told: the enrolled actions, and the two
  // types taken on their type alone. `resolved_plan()` rather than a second pass
  // over the patterns, so "is this allowlisted" has one answer.
  cognitive3d_ros::doctor::ActionEnrolmentFacts actions;
  actions.discovery_enabled = topics.action_discovery_enabled;
  actions.standard_types_enabled = topics.standard_types_enabled;
  const auto action_names = node->get_parameter("action_names").as_string_array();
  for (const auto& entry : graph) {
    if (entry.second.empty()) continue;
    const std::string& type = entry.second.front();
    const bool allowlisted = node->resolved_plan().Allowed(entry.first);
    // A refused topic is subscribed by nothing, and `Allowed` already reports it
    // as not allowlisted -- so without this it reads as discovered and recorded.
    const bool refused =
        node->resolved_plan().Resolve(entry.first, type).ingest == cognitive3d_ros::Ingest::kRefuse;
    if (type == cognitive3d_ros::kGoalStatusArrayType) {
      actions.actions.push_back(cognitive3d_ros::ClassifyActionTopic(
          entry.first, allowlisted, refused, actions.discovery_enabled, action_names,
          node->get_namespace()));
    } else if (!allowlisted && !refused && node->AutoSubscribesByType(entry.first, type)) {
      // The NODE's answer, not a second copy of the rule. `doctor` configures the
      // node, so `/cognitive3d/diagnostics` is on the graph while it looks, and a
      // reimplementation here would list the node's own health report as taken.
      actions.standard_topics.push_back(entry.first);
    }
  }
  checks.push_back(cognitive3d_ros::doctor::CheckActionEnrolment(actions));

  // Which topics the run would record together under one series, decided by the node's own
  // translators and allowlist answer.
  std::vector<std::string> shared_series;
  std::map<std::string, std::string> first_topic_of_type;
  for (const auto& entry : graph) {
    if (entry.second.empty()) continue;
    const std::string& type = entry.second.front();
    const auto& plan = node->resolved_plan();
    if (plan.Resolve(entry.first, type).ingest == cognitive3d_ros::Ingest::kRefuse) continue;
    if (!plan.Allowed(entry.first) && !node->AutoSubscribesByType(entry.first, type)) continue;
    const auto [first, inserted] = first_topic_of_type.emplace(type, entry.first);
    if (inserted) continue;
    const std::string warning =
        cognitive3d_ros::SharedSeriesWarning(node->translators(), type, first->second, entry.first);
    if (!warning.empty()) shared_series.push_back(warning);
  }
  if (!shared_series.empty()) {
    checks.push_back(cognitive3d_ros::doctor::CheckSharedSeries(shared_series));
  }

  // Each described sensor's topic against its kind, before a run, where the node can say it only
  // once it subscribes.
  for (const auto& sensor : node->resolved_sensors()) {
    doctor::SensorTopicFacts facts;
    facts.sensor = sensor;
    for (const auto& entry : graph) {
      if (entry.first == sensor.topic && !entry.second.empty()) facts.type = entry.second.front();
    }
    facts.publishers = node->count_publishers(sensor.topic);
    checks.push_back(doctor::CheckSensorTopic(facts));
  }

  // Last, because it is the only check that opens subscriptions of its own and
  // the frame question it asks depends on the reference frame settled above.
  std::vector<cognitive3d_ros::doctor::CameraFovFacts> fovs;
  const auto media = ProbeMediaSensors(node, executor, buffer, channel.frame_in_force, &fovs);
  for (const auto& sensor : media) {
    checks.push_back(cognitive3d_ros::doctor::CheckMediaSensor(sensor));
    // Only where there are points to read. A camera, a LaserScan or a topic that
    // published nothing gets no line, because the check would only repeat what the
    // one above it already said.
    if (sensor.kind == "lidar" && sensor.sample_seen &&
        sensor.type == "sensor_msgs/msg/PointCloud2") {
      checks.push_back(cognitive3d_ros::doctor::CheckOpticalCloud(sensor));
    }
  }
  for (const auto& fov : fovs) checks.push_back(cognitive3d_ros::doctor::CheckCameraFov(fov));
  checks.push_back(cognitive3d_ros::doctor::CheckMediaBandwidth(media));

  std::cout << cognitive3d_ros::doctor::Render(checks, cognitive3d_ros::RuntimeDescription());
  node->cleanup();
  executor.remove_node(node->get_node_base_interface());
  rclcpp::shutdown();
  return cognitive3d_ros::doctor::ExitCode(checks);
}

}  // namespace cognitive3d_ros
