#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "c3d/config.hpp"
#include "c3d/properties.hpp"
#include "c3d/remote_variables.hpp"
#include "c3d/transport.hpp"
#include "cognitive3d_ros_msgs/msg/event.hpp"
#include "cognitive3d_ros_msgs/srv/end_session.hpp"
#include "cognitive3d_ros_msgs/srv/start_session.hpp"

#include "cognitive3d_ros/action_discovery.hpp"
#include "cognitive3d_ros/clock_policy.hpp"
#include "cognitive3d_ros/destination.hpp"
#include "cognitive3d_ros/event_rate_limiter.hpp"
#include "cognitive3d_ros/media_capture.hpp"
#include "cognitive3d_ros/media_config.hpp"
#include "cognitive3d_ros/pose_sampler.hpp"
#include "cognitive3d_ros/qos_resolver.hpp"
#include "cognitive3d_ros/sensor_config.hpp"
#include "cognitive3d_ros/tf_rig_sampler.hpp"
#include "cognitive3d_ros/session_runner.hpp"
#include "cognitive3d_ros/topic_plan.hpp"
#include "cognitive3d_ros/translators.hpp"

namespace cognitive3d_ros {

// Per-topic observation state: that the node found the topic, the QoS it
// negotiated with it and how the topic behaves, all of it independent of what
// is sent to Cognitive3D.
struct TopicStat {
  std::string topic;
  std::string type;
  Ingest ingest = Ingest::kDownsample;
  double rate_hz = 1.0;
  // Operator-supplied sensor-series name; empty means topic-derived.
  std::string series;
  std::string qos_description;
  std::string qos_rationale;
  bool subscribed = false;
  bool refused = false;
  // Why it was refused, as the report and `~/diagnostics` say it.
  std::string refusal;
  bool incompatible_qos = false;
  std::uint64_t messages = 0;
  std::uint64_t bytes = 0;
  std::uint64_t translated = 0;
  bool has_translator = false;
  // False when nothing in `topics` named this one: an action enrolled by name, or
  // a battery or diagnostics topic taken on its type alone.
  bool from_allowlist = true;
  // Mean age of messages according to their own header stamp. This is a
  // diagnostic in its own right: it measures publisher backlog and, across
  // machines, clock skew.
  double header_age_sum = 0.0;
  std::uint64_t header_age_count = 0;
  double first_seen_wall = 0.0;
  double last_seen_wall = 0.0;
};

using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

// The spool directory an empty `spool_dir` resolves to: `$XDG_STATE_HOME/cognitive3d`, else
// `$HOME/.local/state/cognitive3d`, or empty when neither is usable. A relative XDG_STATE_HOME is
// ignored, as the XDG base directory specification requires. `lookup` returns a variable's value
// or null.
std::string DefaultSpoolDirectory(const std::function<const char*(const char*)>& lookup);

// A topic name resolved the way `ros2 topic` resolves one for a node: absolute names as they are,
// `~` under the node's fully qualified name, anything else under its namespace. Empty for a `~`
// followed by anything but `/`, which ROS 2 refuses as a name.
std::string ExpandTopicName(const std::string& name, const std::string& node_namespace,
                            const std::string& node_fqn);

// The `device_id` a node with none configured records: `ros-` and the namespace's segments joined
// by `-`, so `/fleet/robot1` is `ros-fleet-robot1`, and `ros-unnamed` at the root.
std::string DefaultDeviceId(const std::string& node_namespace);

// Whole seconds, because that is the resolution the reader's transport takes: a positive value is
// rounded up and clamped into [1, 60], and anything else is 0.
int ReaderTimeoutSeconds(double requested);

// Lifecycle node so telemetry can be deactivated without killing the process,
// and so subscriptions -- every topic, the tf listener and `~/events` -- are
// genuinely destroyed on deactivate rather than merely ignored. That is a real
// kill switch for topic traffic, and it is the single thing a safety reviewer
// most wants to see.
//
// This node is SUBSCRIBE-ONLY. It creates no publisher on any topic it does not
// own, so it structurally cannot write to a control stack.
class Cognitive3DNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit Cognitive3DNode(const rclcpp::NodeOptions& options);
  ~Cognitive3DNode() override;

  LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
  LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

  const std::map<std::string, TopicStat>& topic_stats() const { return stats_; }

  // Adds an optional package's translators, e.g. cognitive3d_ros_nav2's.
  //
  // Call it BEFORE configure. The alternative -- this package reaching for the optional ones --
  // would make every robot install `nav2_msgs` to run the node, which is the whole reason those
  // translators are a separate package. So a robot that runs Nav2 runs the entry point that
  // composes both, and one that does not runs this one; `launch/cognitive3d.launch.py` picks.
  void AddTranslators(TranslatorRegistrar registrar);
  // What this node can translate. A topic whose type is absent here is subscribed, counted and
  // discarded -- which the periodic report names `no-xlat`.
  const TranslatorRegistry& translators() const { return translators_; }

  // For `doctor`, which configures this node in dry_run and then reports what it RESOLVED. Read
  // only, and deliberately not a re-derivation: a preflight tool that works out the configuration
  // for itself is checking its own idea of it, and drifts out of step with the node silently.
  const c3d::Config& resolved_config() const { return c3d_config_; }
  // The environment and the host pair it resolved, the api host included, which no params file
  // can move.
  const Destination& resolved_destination() const { return destination_; }
  const PoseSamplerConfig& resolved_sampler_config() const { return sampler_config_; }
  // The allowlist as the node RESOLVED it, so `doctor` decides "is this topic
  // allowlisted" with the node's own answer rather than a second implementation
  // of the matching rules.
  const TopicPlan& resolved_plan() const { return plan_; }
  // The `topics` entries with relative names resolved against the node's namespace.
  const std::vector<std::string>& resolved_allowlist() const { return allowlist_; }
  // Empty unless on_configure rejected the configuration.
  const std::string& configure_error() const { return configure_error_; }
  // Whether the configured session is holding its uploads; false with none.
  bool uploads_paused() const;
  // The node's tf buffer, from configure to cleanup. Its listener runs only while the node is
  // active, so `doctor`, which only configures, attaches a listener of its own to it.
  std::shared_ptr<tf2_ros::Buffer> tf_buffer() const { return tf_buffer_; }

  // The camera and lidar configuration the node RESOLVED, for `doctor` -- read
  // back rather than re-derived, for the same reason `resolved_config` is.
  const std::vector<CameraConfig>& resolved_cameras() const { return camera_configs_; }
  const std::vector<LidarConfig>& resolved_lidars() const { return lidar_configs_; }
  // The described sensors, series included, as the node RESOLVED them.
  const std::vector<SensorConfig>& resolved_sensors() const { return sensor_configs_; }
  // The pose stream as the node RESOLVED it: whether it is on, its rate, and the rig sampler's
  // configuration -- the frame set's rules and the manifest's id, mesh and kinds.
  bool resolved_pose_enabled() const { return pose_enabled_; }
  double resolved_pose_rate_hz() const { return pose_rate_hz_; }
  const TfRigSamplerConfig& resolved_rig_config() const { return rig_config_; }
  bool resolved_pose_read_robot_description() const { return pose_read_robot_description_; }
  // The robot description the rig is decorated from, or null: none read, the stream off, or the
  // node not active.
  std::shared_ptr<const UrdfDescription> robot_description() const {
    return rig_sampler_ ? rig_sampler_->description() : nullptr;
  }

  // Types taken without an allowlist entry. Both carry a robot's most useful
  // events and neither has a topic name this SDK could guess -- `/battery_state`,
  // `/battery_status` and a namespaced `/robot1/battery_state` are all in use.
  // Only under this node's namespace, so that one node per robot, each in its
  // robot's namespace, records only its own robot.
  //
  // Public because `doctor` reports this set and must report the node's own
  // answer: it configures this node, so its own `~/diagnostics` is on the graph
  // while it looks, and a second implementation of the rule would list it.
  bool AutoSubscribesByType(const std::string& topic, const std::string& type) const;

  // Every action status topic on the graph and whether this run records it. The
  // NOT half is what the startup line exists for, and the only place a run says
  // it at all.
  const std::map<std::string, ActionTopicVerdict>& action_enrolment() const {
    return action_topics_;
  }

 protected:
  // The wall clock a session start is checked against. Virtual so a test can stand in a clock
  // that has not been set.
  virtual double WallClockNow() const;
  // Each of activation's timers, in the node's callback group. Virtual so a test can stand in a
  // timer rclcpp refuses to create.
  virtual rclcpp::TimerBase::SharedPtr CreateActivationTimer(std::chrono::duration<double> period,
                                                             std::function<void()> callback);

 private:
  void DeclareParameters();
  // A double parameter that also accepts an integer, once; a later call for a declared name is a
  // no-op, which is what the per-topic and per-sensor groups need at every configure.
  void DeclareDouble(const std::string& name, double default_value);
  // DeclareDouble, for a list of doubles: `[0, 0, 0]` is an integer list to rclcpp.
  void DeclareDoubleArray(const std::string& name);
  bool LoadConfig(std::string* error);
  // An integer parameter, refused by name outside [min, max] before anything narrows it.
  bool ReadInt(const std::string& name, std::int64_t min, std::int64_t max, std::int64_t* out,
               std::string* error);
  // A double parameter that must be finite and above zero, refused by name otherwise.
  bool ReadPositive(const std::string& name, double* out, std::string* error);
  // A double parameter that must be finite and in [min, max], refused by name otherwise; `range`
  // says the bounds in the refusal.
  bool ReadInRange(const std::string& name, double min, double max, const std::string& range,
                   double* out, std::string* error);
  // Activation's steps, in order. False when the session start is refused, and any step may
  // throw; on_activate undoes what either leaves behind.
  bool BringUp();
  // Destroys the timers and every subscription, ends the session with `reason` and silences the
  // diagnostics: what deactivate does, and what a failed activation is undone with.
  void BringDown(const std::string& reason);
  // The camera and lidar groups, whose per-sensor keys are declared lazily for the
  // same reason `topic_config`'s are: the set of sensors is configuration, and a
  // node cannot know it at construction.
  bool LoadMediaConfig(std::string* error);
  // The `sensors` group, declared lazily as the camera group is. Runs after the allowlist is
  // read, because a sensor's series is the one its topic's rule names.
  bool LoadSensorConfig(std::string* error);
  // The `pose.*` group. Runs after the cameras, lidars and described sensors, whose frames the
  // rig tags with `c3d.kind`, and after `device_id`, which names the rig.
  bool LoadPoseConfig(std::string* error);
  // The refusal for the first `<block>.<name>.<field>` override nothing reads, or empty: a name
  // `list` leaves out, or a field `<block>.<name>` does not declare. Asked once every listed
  // name's block is declared, so the declarations are the one list of a block's fields. For
  // `topic_config`, `listed` holds the keys the `topics` entries give.
  std::string UnreadBlockKey(const std::string& block, const std::string& list,
                             const std::set<std::string>& listed);
  void DiscoverAndSubscribe();
  // One line naming every action server on the graph and whether it is recorded.
  // Said again only when the set changes, because a discovery tick that repeated
  // it every five seconds would be ignored by the second minute.
  void LogActionEnrolment();
  // Separate from DiscoverAndSubscribe's loop on purpose. The allowlist path
  // refuses every bulk type outright and must keep doing so; these two subscribe
  // to exactly the topics an operator named in `cameras`/`lidars` and nothing
  // else, which is the only bypass of that refusal in the node.
  void DiscoverAndSubscribeMedia();
  // The half of a lidar's configuration its message type uses, and the type, into the open
  // session. Nothing without one.
  void RecordLidarTypeProperties(const LidarConfig& lidar, const std::string& type);
  void OnMediaMessage(const std::string& key, const rclcpp::SerializedMessage& message);
  void OnMediaHoldTimer();
  MediaEnvironment MakeMediaEnvironment();
  void OnMessage(const std::string& topic, const rclcpp::SerializedMessage& message);
  // One line for each source the cap has taken something from, named by the source
  // that was actually CHARGED. Drained rather than asked about the caller: the
  // vanished-goal sweep spends the goal's own action budget while running on
  // whichever action published last, and the pose channel spends a budget that is
  // no topic at all.
  void WarnOnSuppressedSources();
  void OnSampleTimer();
  // The pose stream's own tick, at `pose.rate_hz`, apart from the gaze timeline's.
  void OnPoseTimer();
  // One message on `/robot_description`: parsed, and the rig decorated from it, or from nothing
  // when it cannot be read.
  void OnRobotDescription(const std::string& xml);
  // The pose stream's session totals, into the session about to end: every path to
  // `SessionRunner::End` comes through here first.
  void RecordPoseStreamTotals();
  // `participant_id` is the identity the platform draws A/B arms for and the session's
  // participant; empty means the node's device_id. A refusal this node can explain goes to
  // `refusal`.
  bool StartSession(const std::string& participant_id = "", bool reuse_remote_variables = false,
                    std::string* refusal = nullptr);
  // One GET on the caller's thread, then either typed `c3d.remote_variable.*` properties or a
  // `c3d.remote_variables.unavailable` event. Never both, never neither.
  void FetchAndApplyRemoteVariables(const std::string& identifier);
  // The reopen path: no network. The arm is a hash of the identifier and cannot differ, and this
  // runs on the sample timer, which must never wait on a socket.
  void ReapplyRemoteVariables(const std::string& identifier);
  bool RemoteVariablesFetchEnabled() const;
  void RecordRemoteVariablesUnavailable(const std::string& identifier);
  void StopSession(const std::string& reason);

  // Session control for a process that is not this node -- a mission or task
  // orchestrator, which is normally a separate node because driving a robot has
  // nothing to do with telemetry.
  void OnStartSessionRequest(
      const std::shared_ptr<cognitive3d_ros_msgs::srv::StartSession::Request> request,
      std::shared_ptr<cognitive3d_ros_msgs::srv::StartSession::Response> response);
  void OnEvent(const cognitive3d_ros_msgs::msg::Event::SharedPtr message);
  void OnEndSessionRequest(
      const std::shared_ptr<cognitive3d_ros_msgs::srv::EndSession::Request> request,
      std::shared_ptr<cognitive3d_ros_msgs::srv::EndSession::Response> response);
  // Applies `session_name` (falling back through the parameter to the session id,
  // so a session can never inherit the previous one's name) and the
  // `session_properties` parameter. Returns the name it settled on.
  std::string ApplySessionIdentity(const std::string& requested_name);
  // Register the node's own health as sensors, so the analytics record says
  // whether to trust itself.
  void ResetPoseSamplingState();
  void RecordSelfTelemetry(double wire_time);
  // Epoch seconds for a record, through the clock policy once there is one.
  double WireNow() const;
  // Seconds on the clock every rate gate runs on, read when a message arrives: steady time, or
  // under `sim_time_on_wire` the wire's own clock, so that `rate_hz` is per second of the
  // session's timeline either way. Neither runs backwards.
  double GateNow() const;
  // Where a session ends on the wire when the node keeps the session's timeline itself: under
  // `sim_time_on_wire`, or while the session leads the wall clock. nullopt leaves it to core.
  std::optional<double> SessionEndOnWire() const;
  rclcpp::SubscriptionOptions SubscriptionOptionsInGroup() const;
  // The tf listener and the thread that spins it, both started at activate. Stopping returns
  // within one spin timeout however soon after the start it is asked for.
  void StartTransformListener();
  void StopTransformListener();
  void TearDownSubscriptions();
  void OnDiscoveryTimer();
  void OnReportTimer();
  void LogPlanSummary() const;

  c3d::Config c3d_config_;
  Destination destination_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_;
  PoseConvention pose_convention_ = PoseConvention::kRep103;
  PoseSamplerConfig sampler_config_;
  TopicPlan plan_;
  std::vector<std::string> allowlist_;
  std::unique_ptr<ClockPolicy> clock_policy_;
  std::unique_ptr<SessionRunner> runner_;
  TranslatorRegistry translators_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  // tf is spun here rather than on a thread tf2_ros owns: that one's destructor can miss its own
  // stop request and block for good, and it runs inside deactivate. The listener subscribes on this
  // node's own interfaces, so /tf and /tf_static resolve under its namespace and remaps however it
  // was started, in a group no executor of the node's takes. The group lives from configure to
  // cleanup; the listener, the executor and the thread only while active.
  rclcpp::CallbackGroup::SharedPtr tf_callback_group_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> tf_executor_;
  std::thread tf_thread_;
  std::atomic<bool> tf_stop_{false};
  std::unique_ptr<PoseSampler> pose_sampler_;
  // The pose stream. Built at configure when `pose.enabled`, over the same buffer, and null
  // otherwise, which is the whole of "off": no timer, no manifest, no property.
  bool pose_enabled_ = false;
  double pose_rate_hz_ = 10.0;
  TfRigSamplerConfig rig_config_;
  std::unique_ptr<TfRigSampler> rig_sampler_;
  // `pose.read_robot_description`: whether the rig is decorated from `/robot_description`.
  bool pose_read_robot_description_ = true;

  std::map<std::string, rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  // The first topic subscribed for each message type, for SharedSeriesWarning.
  std::map<std::string, std::string> first_topic_of_type_;
  std::map<std::string, TopicStat> stats_;

  // Every action status topic seen on the graph, recorded or not. The NOT half is
  // the point: an action nobody enrolled emits no events, and after the run a
  // session with no docking objectives is indistinguishable from a robot that
  // never docked.
  std::map<std::string, ActionTopicVerdict> action_topics_;
  std::size_t action_topics_logged_ = 0;

  // Keyed `camera:<name>` / `lidar:<name>` rather than by topic: two sensors may
  // legitimately name the same topic, and the per-sensor counters and the wire's
  // `cameraId` are keyed on the NAME.
  struct MediaSubscription {
    std::string type;
    rclcpp::GenericSubscription::SharedPtr subscription;
  };
  std::vector<CameraConfig> camera_configs_;
  std::vector<LidarConfig> lidar_configs_;
  std::vector<SensorConfig> sensor_configs_;
  std::vector<std::unique_ptr<CameraCapture>> cameras_;
  std::vector<std::unique_ptr<LidarCapture>> lidars_;
  std::map<std::string, MediaSubscription> media_subscriptions_;
  // Built once at activate: the three callbacks a capture needs, all closing over
  // this node. Rebuilding them per message would allocate three std::functions on
  // the ingest thread for every frame.
  MediaEnvironment media_environment_;
  // Sensors whose topic is on the graph with a type this SDK does not handle.
  // Recorded so the refusal is said once rather than on every discovery tick.
  std::set<std::string> media_type_refusals_;

  // Every callback this node creates, in one mutually exclusive group; see on_configure.
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr discovery_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;
  rclcpp::TimerBase::SharedPtr sample_timer_;
  rclcpp::TimerBase::SharedPtr pose_timer_;
  // Retries camera frames and lidar scans held for their pose. Only while a sensor is configured.
  rclcpp::TimerBase::SharedPtr media_hold_timer_;

  rclcpp::Service<cognitive3d_ros_msgs::srv::StartSession>::SharedPtr start_session_srv_;
  rclcpp::Service<cognitive3d_ros_msgs::srv::EndSession>::SharedPtr end_session_srv_;
  rclcpp::Subscription<cognitive3d_ros_msgs::msg::Event>::SharedPtr event_sub_;
  // Only while active, and only with the pose stream on and `pose.read_robot_description`.
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  double last_event_warn_ = 0.0;
  // Per session: a stamp from before the session is said once, not once per event.
  bool warned_event_before_session_ = false;
  std::uint64_t events_recorded_ = 0;
  std::uint64_t events_dropped_ = 0;
  // Per session: `c3d.*` keys taken out of published events, and the keys already warned about.
  std::uint64_t event_properties_dropped_ = 0;
  std::set<std::string> warned_event_property_keys_;
  // Per source, so a translator in a loop spends its own budget. Keyed by topic
  // for a translator event and by `~/events` for an externally published one.
  EventRateLimiter event_limiter_;

  // Diagnostic throttling runs on STEADY time, never on the node clock.
  // rcutils initialises a throttle's last-logged stamp to 0 and fires when
  // `now >= last + period`, so a ROS clock under use_sim_time -- which starts
  // near zero and may never advance -- suppresses the first message until sim
  // time passes the period, and the "ROS time is still zero" warning becomes
  // unsatisfiable outright: it is gated on the very clock it reports as dead.
  // That holds for every throttled site here, two of which are reached from a
  // subscription callback rather than from a timer.
  //
  // Not a complete escape from the same shape: CLOCK_MONOTONIC counts from
  // boot, so a node started before uptime reaches the period still holds its
  // first message that long. Bounded by the period, once per boot.
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // The node's own health, on its own topic. This is the only thing it ever
  // publishes.
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_pub_;

  double discovery_period_s_ = 5.0;
  double report_period_s_ = 10.0;
  bool warn_on_missing_topics_ = true;
  bool action_discovery_ = true;
  std::vector<std::string> action_names_;
  bool auto_subscribe_standard_types_ = true;
  bool dry_run_ = false;
  bool use_sim_time_ = false;
  bool sim_time_on_wire_ = false;
  // Resolved at configure, which is where an empty value is refused.
  std::string robot_object_id_ = "robot";
  std::string configure_error_;
  // What the last `~/start_session` asked for. A session reopened after a backward clock jump has
  // to be the SAME session the caller configured, not a default one wearing its place.
  std::string requested_session_name_;
  c3d::Properties requested_session_properties_;
  std::string requested_participant_id_;
  c3d::RemoteVariableSet last_remote_variables_;
  // Reads run here, never through the runner's transport: that one belongs to the worker.
  std::shared_ptr<c3d::IHttpGet> remote_variables_reader_;
  // Guards the four members above and `remote_variables_reader_`. The node's own callbacks are
  // serialised in one group; this covers a lifecycle transition called directly from another
  // thread. Never held across the network.
  mutable std::mutex request_state_mu_;
  // One easy handle: two concurrent GETs on it are undefined, so reads are serialized here.
  std::mutex reader_mu_;
  bool auto_start_session_ = true;
  // The node's own fact, set when activation has finished and cleared before a deactivation
  // starts: `~/start_session` is refused outside it.
  std::atomic<bool> active_{false};
  // Whether the open session is the one activation opened under `auto_start_session`, which is
  // what a refused `~/start_session` names as the way out.
  bool session_from_activation_ = false;
  bool remote_variables_ = true;
  double remote_variables_timeout_s_ = 3.0;
  std::uint64_t pose_samples_ = 0;
  std::uint64_t pose_failures_ = 0;
  double last_pose_warn_ = 0.0;
  double last_gaze_frame_warn_ = 0.0;
  double last_self_telemetry_ = 0.0;
  // The wire time the next pose sample is due at under `sim_time_on_wire`; 0 is "now".
  double next_sample_wire_ = 0.0;
  // The same, for the pose stream's tick.
  double next_pose_wire_ = 0.0;
};

}  // namespace cognitive3d_ros
