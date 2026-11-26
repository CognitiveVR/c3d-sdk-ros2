#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "c3d/config.hpp"
#include "cognitive3d_ros/action_discovery.hpp"
#include "cognitive3d_ros/media_config.hpp"
#include "cognitive3d_ros/sensor_config.hpp"

// Preflight checks, as PURE functions.
//
// `doctor` exists because this platform's characteristic failure is silence: the data collector
// answers HTTP 200 and then discards a session whose property set is wrong, and it answers 200 for
// a scene id that does not exist. A check prints in a second what a misconfigured run otherwise
// reveals only after its session has uploaded and been found wrong.
//
// The I/O -- ROS parameters, tf lookups, the network probe -- lives in src/run_doctor.cpp. What
// is here is the DECISION each check makes, given facts already gathered, so that every verdict and
// every message a user will act on is unit-testable without a robot, a network or a clock.
namespace cognitive3d_ros::doctor {

enum class Severity { kOk, kAdvisory, kFatal };

// A check whose NAME ends in "(NOT CHECKED)" is a DECLINE, not a pass. It carries `kOk` on
// purpose -- an opt-in check that was never asked to run must not fail a run -- so severity alone
// cannot distinguish "verified" from "not verified". Anything automated reading these must match
// on the name, not only on `severity`.
struct Check {
  std::string name;
  Severity severity = Severity::kOk;
  // What was found. Never a credential: see MaskSecret.
  std::string detail;
  // What to do about it. REQUIRED for anything not kOk -- a check that reports a problem without
  // naming the action is a check that generates a support conversation instead of ending one.
  std::string remedy;
};

// `ending ...wxyz`: the last four characters, which doctor prints once, on the `application key`
// line, and on no other. An application key is not a secret -- it ships inside every client that
// talks to the platform, and it doubles as routing by naming the project that receives the data --
// but doctor's output is pasted whole into public support channels, so it carries no more of the
// key than tells two keys apart. A short value is reported by LENGTH ONLY, because four characters
// of a 10-character value is a large part of it.
std::string MaskSecret(const std::string& value);

// The worst severity present. kOk when empty -- an empty check list is a programming error the
// caller must not turn into a pass, so ExitCode treats it separately.
Severity Aggregate(const std::vector<Check>& checks);

// 0 when nothing is fatal, 1 otherwise. Advisories do NOT fail: "no map yet" is normal on a robot
// that has not localised, and a tool that exits non-zero for it would be muted within a day.
int ExitCode(const std::vector<Check>& checks);

// Which build of the node this report checked. The plain node and cognitive3d_ros_nav2's translate
// different message types, and the launch file runs the second whenever its package is installed,
// so a report on the other one says nothing about the Nav2 translators the run will have.
struct BuildFacts {
  std::string package;
  // False when its parameters did not construct the node, so its types are unknown.
  bool constructed = false;
  std::vector<std::string> translated_types;
  // Why the doctor of the build the launch file runs did not start, when this report is the
  // fallback to this package's own.
  std::string handoff_error;
  // Why the ament index could not say which build the launch file runs.
  std::string index_error;
};
Check CheckBuild(const BuildFacts& facts);

// The checks a `configure()` refusal maps to. `Config::Validate` reports EVERY missing mandatory
// value in one string, joined with " | " behind an "N configuration problems: " count, so one
// refusal can name several at once. Each problem this recognises becomes the check carrying its own
// remedy; the rest are reported together. Matching the joined string as a whole names only the
// first and drops the others, bringing back the round trip per unset field that joining avoids.
std::vector<Check> ChecksForConfigurationRefusal(const std::string& configure_error);

// The verdict for an exception thrown while the node is CONSTRUCTED: rclcpp reads the parameter
// overrides there, before any of the node's own code runs. The two shapes it throws -- an empty
// YAML list, and a value of the wrong type -- are recognised and get a remedy that names the
// parameter; anything else is reported as it was thrown.
Check CheckConstruction(const std::string& what);

Check CheckApplicationKey(const std::string& key);

// Whether the node could take `<spool_dir>/.lock`, core's DiskSpool lock, asked without taking it
// from a node that may be running on the directory: a non-blocking lock released at once. Creates
// nothing, and never constructs a DiskSpool, whose construction takes the lock and prunes the
// directory.
struct SpoolLockFacts {
  // Why the node would refuse the lock file itself -- a symbolic or hard link -- or empty.
  std::string refused;
  bool held = false;
  // The pid the holder wrote into the lock file, or empty.
  std::string holder;
};
SpoolLockFacts ProbeSpoolLock(const std::string& spool_dir);
Check CheckSpoolDirectory(const std::string& directory, bool writable, int orphaned_sessions,
                          const SpoolLockFacts& lock = {});
// Whether the wall clock reads late enough for a session to begin: the node refuses every start
// before c3d::kWallClockFloorEpochSeconds, a clock that has not been set.
Check CheckWallClock(double now_epoch_s);
// `use_sim_time` against whether anything publishes `/clock`. The node warns about this too, but
// only once it is activated and recording: by then the run that is capturing a robot which is not
// there has already started. A verdict before the session opens is the whole point.
// Which pose convention is in force. It has no default, so a configured node always names one
// the operator chose; empty is the refusal a node gives when it is not set.
Check CheckPoseConvention(const std::string& convention);

Check CheckSimTime(bool use_sim_time, bool clock_publisher_present,
                   bool sim_time_on_wire = false);
Check CheckFrames(const std::string& preferred, const std::string& fallback,
                  const std::string& robot, bool preferred_resolved, bool fallback_resolved);
// Whether the wait for a reference frame may stop, given what resolves right now.
//
// The fallback resolving is NOT the answer. `odom -> base_link` is published from the first wheel
// tick while nav2_amcl is still waiting for a scan, so stopping there reports a healthy, fully
// localised robot as one with no map -- while `tf2_echo map base_link` beside it resolves fine.
// The preferred frame gets the same grace the node gives it, read from the same resolved
// PoseSamplerConfig, so the preflight and the run cannot disagree about what they waited for.
//
// `seconds_since_fallback_resolved` is measured from the first moment the fallback answered, not
// from the start of the wait: a robot whose tf tree appears late must still get the full grace.
bool FrameWaitSettled(bool preferred_resolved, bool fallback_resolved, bool preferred_configured,
                      double seconds_since_fallback_resolved, double grace_seconds);
Check CheckGazeFrame(const std::string& gaze_frame, bool resolved);
// The `topics` allowlist against what is actually on the graph, naming every missing one. Never
// fatal -- a robot mid-bring-up legitimately has none of them yet, and a preflight that fails
// during bring-up gets muted. The case it exists for is a shipped default allowlist that does not
// match the robot in front of you, which is a decision to make before a run rather than a warning
// to read during one.
struct TopicFacts {
  std::vector<std::string> allowlist;
  // Allowlist entries nothing on the graph matches.
  std::vector<std::string> missing;
  // What an EMPTY allowlist still records, which is not nothing and must say the same thing here
  // as the node says at configure: an operator who is told "nothing will be observed" goes looking
  // for the wrong fault when the session arrives carrying battery, diagnostics and objectives.
  bool standard_types_enabled = true;
  bool action_discovery_enabled = true;
  // Cameras and lidars carry their own topic in their own parameter, so they are recorded with no
  // allowlist entry at all.
  std::size_t media_count = 0;
};
Check CheckTopics(const TopicFacts& facts);

// One allowlisted topic's publishers, as the graph reports them, and the subscription the node
// would request for it.
struct PublisherQos {
  std::string node;
  bool reliable = true;
  bool transient_local = false;
};
struct TopicQosFacts {
  std::string topic;
  // The `topic_config.<key>` group the topic's rule was read from, and what its `qos.reliability`
  // and `qos.durability` say: empty when unset.
  std::string override_key;
  std::string override_reliability;
  std::string override_durability;
  std::vector<PublisherQos> publishers;
  // The profile ResolveQos chooses from every publisher, with the overrides applied.
  bool requests_reliable = false;
  bool requests_transient_local = false;
  std::string requested;
};
// Every allowlisted topic's publishers, against each other and against that subscription.
//
// The node chooses a topic's profile from the publishers it has discovered at the moment it
// subscribes, and keeps it. Publishers that disagree are all reachable only by the weaker request,
// which the node makes only if it has discovered the weaker publisher by then: with ros2_control's
// joint_state_broadcaster (transient_local) and joint_state_publisher (volatile) both on
// /joint_states, a run is incompatible on one start and fine on the next. doctor reads the graph
// after its waits, so it sees every publisher, and the override that pins the weaker value is the
// fix. The remedy is the node's own incompatibility advice, word for word.
Check CheckTopicQos(const std::vector<TopicQosFacts>& topics);
// REACHABILITY ONLY. This deliberately says nothing about whether the credential is valid, and the
// probe sends no Authorization header at all: the collector answers an unrouted request 404 before
// any authentication runs, so a wrong key cannot be detected here, and a probe that sent one would
// report it as `[ok]`. See the note in run_doctor.cpp.
//
// `status` is the HTTP status, or 0 when the transport never got one; `transport_error` is curl's
// own message in that case. `saw_request_time_header` is the `cvr-request-time` response header:
// a 200 WITHOUT it is a captive portal or an intercepting proxy answering on the collector's
// behalf, not a success -- the core transport already treats that as retryable, and an operator
// behind hotel or corporate wifi needs to be told which of the two they are looking at.
Check CheckGateway(const std::string& protocol, const std::string& gateway, long status,
                   bool saw_request_time_header, const std::string& transport_error);

// The action servers on the graph against what this configuration will record.
//
// An action nobody enrolled is subscribed by nothing and emits no events, and the
// session that results is not wrong -- it is missing an objective the operator
// assumed was there, which after the run is indistinguishable from an action that
// never fired. This is the only moment that difference is visible, so the check
// reads as "these exist and will not emit" rather than as a failure. Never fatal:
// most robots run actions nobody wants recorded.
struct ActionEnrolmentFacts {
  bool discovery_enabled = true;
  bool standard_types_enabled = true;
  std::vector<ActionTopicVerdict> actions;
  // Battery and diagnostics topics no allowlist entry names, which are therefore
  // taken on their type alone. Reported here because they are the other half of
  // what a run subscribes to without being told.
  std::vector<std::string> standard_topics;
};
Check CheckActionEnrolment(const ActionEnrolmentFacts& facts);

// Topics the run records whose translator writes the same series for every topic of their type,
// each SharedSeriesWarning's sentence. Advisory: the run records, under one series per type.
Check CheckSharedSeries(const std::vector<std::string>& warnings);

// One described sensor's topic as doctor found it on the graph, against the type its kind reads.
//
// The node compares the two only once it subscribes, with the session already open. A topic of
// another type records under other series names, or none, so the description names a series that
// never exists while the session looks complete: FATAL. A topic with no publisher is a note,
// since a driver that starts after the node is picked up then.
struct SensorTopicFacts {
  SensorConfig sensor;
  std::string type;  // as the graph reports it; empty when the topic is absent
  std::size_t publishers = 0;
};
Check CheckSensorTopic(const SensorTopicFacts& facts);

// Authorization header value for an organisation key. TWO FORMATS, and the newer one carries its
// own prefix: keys issued after May 2025 look like `orgkey-...` and are sent verbatim, older ones
// take `APIKEY:ORGANIZATION `. Prefixing an already-prefixed key produces `orgkey-orgkey-...` and a
// 401 that reads as a bad credential, on a key that is fine.
std::string OrganizationAuthorization(const std::string& organization_key);

// Where an organisation key may be sent for this configuration, and whether it may be sent at all.
//
// PURE, and taking the WHOLE config beside the api host the node resolved from the process
// environment. The property that matters here is a NEGATIVE one -- no value in a params file may
// move this host -- and a function that never receives the gateway cannot be tested for it,
// however hostile the values a test passes alongside. Keeping the decision here rather than inline
// in each probe means one test covers every caller, and a probe that went back to reading
// `gateway` fails it.
struct ApiDestination {
  std::string host;        // empty when the environment is not one this build knows
  bool permitted = false;  // whether an organisation key may be sent there at all
};
ApiDestination ApiDestinationFor(const c3d::Config& config, const std::string& api_host);

// The scheme and host scene/project reads the scene from: ApiDestinationFor's host, over https.
// Empty when no organisation key may be sent for this configuration.
std::string SceneLookupBase(const c3d::Config& config, const std::string& api_host);

// Which environment the run is in and where each half of it goes: production, or a custom pair
// named by C3D_DATA_HOST / C3D_API_HOST, and a gateway override when there is one.
Check CheckEnvironment(const std::string& description);

// Whether the application key belongs to the project the operator expected.
//
// This is the only check for the failure with no symptom: an application key from another project
// ingests COMPLETELY into that project -- no error, all four data types present -- because the key
// is routing as well as authentication, and the data path auto-creates a scene record for the
// unknown id there. Nothing downstream notices.
//
// STRICTLY OPT-IN, and the reason is the credential. The only route that answers this needs an
// ORGANISATION key, which unlocks every project in the org, and its response carries that
// project's application keys in plaintext. A developer key is refused with a 401. So doctor
// never requires one, and a check that did not run NEVER renders as one that passed.
struct ProjectKeyFacts {
  std::string application_key;
  std::string expected_project;         // empty: the operator did not ask for this check
  bool org_key_present = false;         // they asked, and C3D_ORG_API_KEY is set
  bool destination_permitted = false;   // the api host is on the allowlist AND the scheme is https
  long status = 0;                      // HTTP status; 0 means no response at all
  bool body_is_a_key_list = false;      // the 200 body actually looked like the expected shape
  bool key_listed = false;              // meaningful only when status 200 and body_is_a_key_list
  std::string transport_error;
};
Check CheckProjectKey(const ProjectKeyFacts& facts);

// Whether key/project's answer proves the organisation key: the API listed the keys of
// C3D_PROJECT_ID's project for it, whether or not the application key is among them.
bool OrganizationKeyProven(const ProjectKeyFacts& facts);

// Whether the configured scene is in the operator's project and has the configured version.
//
// Neither is checked when a session is accepted. A scene id from another project makes the
// platform create an empty shadow scene with that id in the application key's project, and the
// session records against it; Session Replay then draws the robot in an empty world. A version
// the scene does not have is accepted too, and the numeric version id the platform lists beside
// each version number is the usual way to configure one. Asked whenever C3D_ORG_API_KEY is set,
// because it is one read of the scene; the project is compared with C3D_PROJECT_ID.
struct SceneVersion {
  std::string number;
  std::string id;  // empty when the entry carries none
};
struct SceneProjectFacts {
  std::string scene_id;
  std::string scene_version;     // the version NUMBER, as configured
  std::string expected_project;  // C3D_PROJECT_ID, empty when unset
  bool org_key_present = false;
  // OrganizationKeyProven for the same key in the same run. A scene the API then does not show
  // that key is not in the organisation that owns expected_project.
  bool org_key_verified = false;
  bool destination_permitted = false;  // api host allowlisted AND scheme is https
  long status = 0;                     // 0 means no response at all
  std::string transport_error;
  bool body_is_a_scene = false;
  std::string scene_project;
  std::vector<SceneVersion> versions;
};

// `projectId` and every `versions[]` entry of a scene document. FALSE when the body is not one: not
// an object, no `projectId`, no `versions` array, or an entry with no `versionNumber`. A number and
// a string of digits read alike.
bool SceneDocumentIn(const std::string& body, std::string* project,
                     std::vector<SceneVersion>* versions);

Check CheckSceneProject(const SceneProjectFacts& facts);
// Whether the dynamic-object mesh this run will register actually exists on the platform.
//
// `robot_mesh` is a NAME resolved on the dashboard, not an asset in this repository, and the
// upload that puts it there answers 200 for a format it discards. So a params file can name a mesh
// that is not present, the session records perfectly, and the replay has no robot in it. Nothing
// downstream reports that.
struct RobotMeshFacts {
  std::string robot_mesh;
  std::string project;                 // empty: the operator did not ask for this check
  std::string scene_version;           // reported, not checked against -- see CheckRobotMesh
  bool org_key_present = false;
  bool destination_permitted = false;  // api host allowlisted AND scheme is https
  long status = 0;                     // 0 means no response at all
  bool body_is_an_object_list = false;
  bool mesh_listed = false;
  // Mesh names the project does have, for the remedy. A check that says "not found" without
  // saying what IS there starts the support conversation it was meant to end.
  std::vector<std::string> available;
  std::string transport_error;
};

// Collects one string field from every object in a JSON array. Returns FALSE when `body` is not a
// JSON array at all: "this project has no objects" and "I could not read the answer" are different
// facts, and only the first may be reported as a proven absence.
//
// In the PURE layer on purpose -- the I/O layer carries no tests, so parsing does not live there.
bool StringFieldIn(const std::string& body, const std::string& field,
                   std::vector<std::string>* values);

Check CheckRobotMesh(const RobotMeshFacts& facts);

// Whether the dynamic-object id this run will EMIT is registered on the platform.
//
// Separate from the mesh check, and ADVISORY where that one is fatal, because the consequences
// differ. A missing mesh empties the replay. A mismatched id does not: the viewer reads the mesh
// name from the runtime manifest and fabricates a registry entry when the id is unknown, so the
// robot still renders -- it simply renders without the uploaded row's initial transform, scale,
// thumbnail or group, and aggregate gaze, which keys on the uploaded id, reports zero against a
// full timeline. That failure looks like a missing mesh and is not one.
struct RobotObjectIdFacts {
  std::string robot_object_id;
  std::string project;
  bool org_key_present = false;
  bool destination_permitted = false;
  long status = 0;
  bool body_is_an_object_list = false;
  bool id_listed = false;
  std::vector<std::string> available;  // the ids the project does have, for the remedy
  std::string transport_error;
};
Check CheckRobotObjectId(const RobotObjectIdFacts& facts);

// The object ROW and its mesh FILES are created by two different HTTP calls, and the second
// answers 200 for a post that stores nothing -- so "the object is registered" and "the replay has
// a mesh to draw" are different facts. Only a version-scoped read with `includeFiles=true`
// separates them: the row is what call 1 makes, the files are what call 2 makes.
struct RobotMeshFilesFacts {
  std::string robot_mesh;
  std::string scene_id;
  std::string scene_version;  // the version NUMBER, as configured
  bool org_key_present = false;
  bool destination_permitted = false;
  long scene_status = 0;       // 0 means no response at all
  // Readable-and-absent is not the same fact as unreadable. A 200 whose body is malformed must
  // stay UNVERIFIED; a 200 that genuinely does not list the configured version is an answer.
  bool scene_body_is_readable = false;
  // scene/project read the same document as a scene, so it reports a version the scene lacks.
  bool scene_read_by_scene_project = false;
  std::string version_id;      // numeric; empty with a readable body means no such version
  long status = 0;             // the object listing
  bool body_is_an_object_list = false;
  bool mesh_listed = false;
  std::vector<std::string> files;  // this mesh's file names, for the remedy
  std::string transport_error;
};

// The numeric version id for a version NUMBER, read out of a scene document. The two values are
// different and not interchangeable: the objects route answers `400 Cannot parse parameter
// sceneVersionId as Long` for a scene UUID.
//
// Returns FALSE when the body is not a readable scene document -- including the case where the
// matching version entry carries no usable `id`, which proves nothing about whether the version
// exists. Returns TRUE with an empty `*version_id` when the document is readable and simply does
// not list that number, which is a fact the caller may act on.
bool VersionIdForNumber(const std::string& scene_body, const std::string& version_number,
                        std::string* version_id);

// Collects one mesh row's file names. Returns FALSE when `body` is not a JSON array at all, so
// "this version has no such mesh" and "I could not read the answer" stay different facts. Sets
// `found` when the row exists -- a row with an empty `files` array is found with no files, which
// is precisely the half-finished upload this check exists to catch.
// Returns FALSE when the answer cannot be read as a definite one: not a JSON array, a row whose
// `meshName` is not a string (so absence cannot be proven -- the unreadable row might be the one),
// or a MATCHING row whose `files` is not an array (so "it has no files" would be invented). Only a
// true return licenses `*found` and `*files` as facts.
bool MeshFilesIn(const std::string& body, const std::string& mesh_name, bool* found,
                 std::vector<std::string>* files);

Check CheckRobotMeshFiles(const RobotMeshFilesFacts& facts);

// The dynamic-object verdicts a run gets, which is where `robot_dynamic_object` is answered.
//
// All three checks above ask about an upload only a run that REGISTERS the object needs. With the
// switch off the robot is drawn from the pose timeline, nothing is registered and no dynamics
// sample is sent, so a missing mesh, an unregistered id and a row with no files are not faults --
// they are answers about an object this run never names. One decline stands in for the three, and
// it is a decline rather than a pass because nothing was verified.
//
// The gate lives here, in the pure layer, rather than at the call site: it decides what a run is
// told, so it is a decision a test has to be able to hold.
std::vector<Check> RobotObjectChecks(bool robot_dynamic_object, const RobotMeshFacts& mesh,
                                     const RobotObjectIdFacts& object_id,
                                     const RobotMeshFilesFacts& files);

// The angle in degrees between a rotation's own +Z and the reference +Z. Zero for every pure yaw,
// which is what makes it the test for "this rotation channel carries no tilt at all". A non-unit
// quaternion is normalised rather than refused: tf interpolation does not produce exact units.
double TiltDegrees(double qx, double qy, double qz, double qw);

// Whether a rotation carries ANY tilt: an exact test on the two components a pure yaw leaves at
// literal zero.
//
// Deliberately NOT `TiltDegrees(...) > 0`. That expression is exact only if the compiler does not
// contract it into fused multiply-adds -- with contraction the numerator and denominator round
// differently and an exact pure yaw can report a tilt of about 1e-6 degrees, which would promote a
// planar source and silence the check that depends on this count. The comparison below has no
// arithmetic in it to contract.
bool CarriesTilt(double qx, double qy);

// What the pose channel CONTAINS, as opposed to whether it resolves.
//
// A transform that resolves can still be structurally two-dimensional, and both frames this SDK
// ships default to are. `odom` on a differential-drive base is wheel-encoder odometry: z, roll and
// pitch are zero by construction, permanently. `map` is no better on a standard stack, because
// nav2_amcl builds map -> odom from a 3-DOF filter state -- `q.setRPY(0, 0, yaw)` with the literal
// `0.0` for z, in `calculateMaptoOdomTransform` -- and composing two planar transforms stays
// planar. A robot that climbs a ramp is therefore recorded flat, and every verifier still passes,
// because each compares a session against itself.
//
// The frame NAME is a hint about the producer, never a proof: `odom` is also what
// `robot_localization` publishes with `two_d_mode: false`, and `map` is what a 6-DOF SLAM
// publishes. So this reports what it OBSERVED and uses the name only to decide how firmly to word
// the advisory -- and the exact-zero count above is what keeps that honest, because a source that
// really does carry tilt is visible even parked.
struct PoseChannelFacts {
  std::string frame_in_force;  // the reference frame this run will actually sample against
  int samples = 0;             // transforms read during the observation window
  // Rotations that were not EXACTLY a pure yaw. Counted rather than thresholded, because that is
  // what separates the two cases: a planar producer builds its quaternion with roll and pitch of
  // literal zero, and quaternion composition carries those exact zeros through the whole chain, so
  // it reads 0 of N. A genuine three-dimensional source jitters at around 1e-6 even parked, so it
  // reads N of N. This distinction survives on flat ground, where the magnitudes do not.
  int samples_with_tilt = 0;
  double height_span_meters = 0.0;
  double max_tilt_degrees = 0.0;
};
Check CheckPoseChannel(const PoseChannelFacts& facts);

// One configured camera or lidar, as doctor found it on the graph.
//
// A lidar's size is MEASURED rather than assumed: doctor encodes one real message
// through the capture's own encoder, so the figure carries the operator's voxel
// leaf and zstd level instead of a table of guesses. A camera's cannot be: one
// frame says nothing about a video stream whose bytes depend on what moves, so
// doctor prepares one real frame the way the run does, for its size, and
// estimates the rate from that size and `crf`. `sample_seen` false means nothing
// arrived in the observation window, which is a fact about the topic, not about
// the size.
struct MediaSensorFacts {
  std::string kind;  // "camera" or "lidar"
  std::string name;
  std::string topic;
  double rate_hz = 0.0;
  bool topic_present = false;
  std::string type;  // as the graph reports it; empty when the topic is absent
  bool type_handled = false;
  std::string qos;    // the profile that was resolved, empty when it was not
  std::string frame;  // the frame a pose will be looked up for, empty when unknown
  bool frame_resolves = false;
  bool sample_seen = false;
  // A lidar's: one sample's encoded size.
  std::uint64_t encoded_bytes = 0;
  // A camera's: what one sample prepares to, 0 by 0 when it could not be, and the
  // crf in force.
  std::uint16_t prepared_width = 0;
  std::uint16_t prepared_height = 0;
  int crf = 0;
  int max_bitrate_kbps = 0;
  int replay_max_mb = 0;
  // Why the sample could not be prepared or encoded, when one arrived and was
  // refused. This is the check that catches a camera pointed at a depth topic.
  std::string encode_error;

  // A PointCloud2's own points, counted in the SENSOR's axes before any
  // conversion: how many were finite, and how many of those were on the positive
  // side of x and of z. All zero for a camera, for a LaserScan and for a cloud
  // that never arrived. Read by CheckOpticalCloud and by nothing else.
  std::uint64_t finite_points = 0;
  std::uint64_t x_positive_points = 0;
  std::uint64_t z_positive_points = 0;
};
// Where one configured camera's frames take their field of view from, as doctor
// found it. Without one, every frame carries 0 by 0 and Session Replay draws no
// frustum for the camera -- which nothing else reports, since the frames are fine.
struct CameraFovFacts {
  std::string name;
  // The parameters, which win when set; the CameraInfo topic is then not looked at.
  bool parameters_set = false;
  double hfov_degrees = 0.0;
  double vfov_degrees = 0.0;
  // Resolved; empty when `camera_info_topic` switches the automatic source off.
  std::string camera_info_topic;
  bool topic_present = false;
  std::string type;  // as the graph reports it
  bool sample_seen = false;
  // From the sample, by CameraFovFromCameraInfo: 0 by 0 when its intrinsics are unusable.
  float sample_hfov_degrees = 0.0f;
  float sample_vfov_degrees = 0.0f;
  double sample_fx = 0.0;
};
Check CheckCameraFov(const CameraFovFacts& facts);

Check CheckMediaSensor(const MediaSensorFacts& facts);

// What the configured streams will cost, added up: a lidar's measured sample and a
// camera's estimate, each at its `rate_hz`. WARN above this, because the only
// limit is the configuration: nothing on the platform rate-limits a session's
// streams.
constexpr double kMediaBandwidthWarnBytesPerSecond = 2.0 * 1024 * 1024;
Check CheckMediaBandwidth(const std::vector<MediaSensorFacts>& facts);

// Whether the binary `ffmpeg_path` names can encode the configured cameras.
// FATAL only with a camera configured: without one nothing runs it, and a robot
// that records no video needs no ffmpeg.
struct CameraEncoderFacts {
  std::size_t cameras = 0;
  std::string ffmpeg_path;
  // c3d::ProbeFfmpeg's answer: empty when it runs and lists libx264.
  std::string problem;
  // Its first `-version` line, when it ran.
  std::string version;
};
Check CheckCameraEncoder(const CameraEncoderFacts& facts);

// A cloud's frame NAME against the geometry of its own points.
//
// Gazebo's `rgbd_camera` writes its `<optical_frame_id>` into the PointCloud2
// header and leaves the point data in the frame the sensor is ATTACHED to. The SDK
// then poses the cloud against the optical frame -- correctly, on what the header
// said -- and the stored cloud lies on its side. Nothing downstream reports it:
// the session uploads, the points are all there, and every number in the record is
// self-consistent with every other. `lidar.<name>.frame` is the fix, and this
// check is the only thing that makes the mislabel visible.
//
// The discriminator is which axis carries depth. In an optical frame (z forward,
// x right, y down) a valid return has z > 0 by definition, and x, being the
// horizontal axis of a roughly centred field of view, is mixed. In the body frame
// (x forward, y left, z up) it is the other way round. So the test is BOTH
// fractions: a cloud whose frame name says optical while its points say body frame
// is mislabelled, and either fraction alone would misread a sensor that is merely
// aimed unusually.
constexpr double kOpticalDepthFraction = 0.95;
constexpr double kBodyForwardFraction = 0.9;
// Fewer finite points than this and the fractions are an accident of what the
// sensor happened to be looking at. An organised depth cloud carries tens of
// thousands, so this only excludes a sample that arrived nearly empty.
constexpr std::uint64_t kOpticalCloudMinPoints = 300;
Check CheckOpticalCloud(const MediaSensorFacts& facts);

// The report, under a header naming the SDK version and, when given, `runtime` -- the ROS distro
// and RMW -- so a pasted report says which build produced it.
std::string Render(const std::vector<Check>& checks, const std::string& runtime = "");

}  // namespace cognitive3d_ros::doctor
