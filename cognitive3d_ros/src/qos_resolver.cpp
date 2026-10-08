#include "cognitive3d_ros/qos_resolver.hpp"

#include <algorithm>
#include <sstream>

namespace cognitive3d_ros {
namespace {

std::size_t OfferedDepth(const rclcpp::QoS& qos) {
  return qos.get_rmw_qos_profile().depth;
}

}  // namespace

std::string QosRelaxation(const std::string& override_key, rmw_qos_policy_kind_t policy) {
  // A requested profile is incompatible when it asks for MORE than a publisher offers, so the
  // fix is always to ask for less.
  if (override_key.empty()) return "";
  if (policy == RMW_QOS_POLICY_RELIABILITY) {
    return "Set " + override_key + ".qos.reliability: best_effort to match a best-effort "
           "publisher.";
  }
  if (policy == RMW_QOS_POLICY_DURABILITY) {
    return "Set " + override_key + ".qos.durability: volatile to match a volatile publisher.";
  }
  return "";
}

std::string DescribeQosIncompatibility(const std::string& topic, int total_count,
                                       rmw_qos_policy_kind_t policy,
                                       const std::string& override_key) {
  std::string message = "QoS incompatibility on " + topic + ": " + std::to_string(total_count) +
                        " so far, last on the " + rclcpp::qos_policy_name_from_kind(policy) +
                        ". No data arrives from an incompatible publisher until the profiles "
                        "match; a compatible publisher on the same topic still delivers. ";
  const std::string relaxation = QosRelaxation(override_key, policy);
  if (!relaxation.empty()) {
    message += relaxation;
  } else if (override_key.empty()) {
    message += "Compare the publisher's profile, from `ros2 topic info --verbose " + topic +
               "`, with what this subscription requests.";
  } else {
    message += "Compare the publisher's profile, from `ros2 topic info --verbose " + topic +
               "`, with " + override_key + ".qos.*.";
  }
  return message;
}

std::string DescribeQos(const rclcpp::QoS& qos) {
  const auto& p = qos.get_rmw_qos_profile();
  std::ostringstream os;
  os << (p.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT ? "best_effort" : "reliable")
     << "/"
     << (p.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL ? "transient_local"
                                                                   : "volatile")
     << "/"
     << (p.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL ? "keep_all" : "keep_last")
     << "(" << p.depth << ")";
  return os.str();
}

QosResolution ResolveQos(const std::vector<rclcpp::TopicEndpointInfo>& publishers,
                         const QosOverride& override_cfg) {
  QosResolution result;
  result.publisher_count = publishers.size();
  result.publishers_found = !publishers.empty();

  bool all_reliable = true;
  bool all_transient_local = true;
  bool any_reliable = false;
  bool any_transient_local = false;
  std::size_t depth = 10;

  for (const auto& p : publishers) {
    const auto& profile = p.qos_profile().get_rmw_qos_profile();
    const bool reliable = profile.reliability != RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    const bool transient = profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    all_reliable = all_reliable && reliable;
    all_transient_local = all_transient_local && transient;
    any_reliable = any_reliable || reliable;
    any_transient_local = any_transient_local || transient;
    depth = std::max(depth, OfferedDepth(p.qos_profile()));
  }

  result.publishers_disagreed =
      (any_reliable != all_reliable) || (any_transient_local != all_transient_local);

  std::ostringstream why;
  if (publishers.empty()) {
    // Nothing is publishing yet. Request the weakest profile that connects to
    // almost anything, and let rediscovery upgrade it later.
    result.qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    why << "no publishers yet, requesting weakest compatible profile";
  } else {
    rclcpp::QoS qos{rclcpp::KeepLast(depth)};
    if (all_reliable) {
      qos.reliable();
      why << "all " << publishers.size() << " publisher(s) offer reliable";
    } else {
      qos.best_effort();
      why << "at least one publisher is best_effort, so requesting best_effort "
             "(requesting reliable would silently connect to nothing)";
    }
    if (all_transient_local) {
      qos.transient_local();
      why << "; all offer transient_local, requesting it so latched data is not missed";
    } else {
      qos.durability_volatile();
      if (any_transient_local) {
        why << "; publishers disagree on durability, requesting volatile to stay compatible";
      }
    }
    result.qos = qos;
  }

  if (override_cfg.any()) {
    result.overridden = true;
    rclcpp::QoS qos = result.qos;
    if (override_cfg.depth >= 0) {
      qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(override_cfg.depth)));
      // Re-apply whatever we already decided, since the constructor reset it.
      const auto& prev = result.qos.get_rmw_qos_profile();
      if (prev.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT) qos.best_effort();
      else qos.reliable();
      if (prev.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) qos.transient_local();
      else qos.durability_volatile();
    }
    if (override_cfg.history == "keep_all") qos.keep_all();
    if (override_cfg.reliability == "reliable") qos.reliable();
    if (override_cfg.reliability == "best_effort") qos.best_effort();
    if (override_cfg.durability == "transient_local") qos.transient_local();
    if (override_cfg.durability == "volatile") qos.durability_volatile();
    result.qos = qos;
    why << "; overridden by configuration";
  }

  result.rationale = why.str();
  return result;
}

}  // namespace cognitive3d_ros
