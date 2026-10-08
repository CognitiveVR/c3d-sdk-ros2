#pragma once

#include <functional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "cognitive3d_ros/doctor.hpp"
#include "cognitive3d_ros/run_node.hpp"
#include "cognitive3d_ros/topic_plan.hpp"

namespace cognitive3d_ros {

// The body of every shipped `doctor` executable: it builds the node `make` builds, in dry_run, and
// reports on it. `package` is the package whose node that is, named in the report's `build` line
// beside the message types it translates. `handoff_error` is why the doctor of the build the launch
// file runs could not be started, when this is a fallback from it, and `index_error` why the ament
// index could not say which build that is.
int RunDoctor(int argc, char** argv, const NodeFactory& make, const std::string& package,
              const std::string& handoff_error = "", const std::string& index_error = "");

// The doctor executable of the build `launch/cognitive3d.launch.py` runs, when that is not this
// package's: cognitive3d_ros_nav2's whenever that package is installed, decided the way the launch
// file decides it. Empty when it is not installed. `package_prefix` answers a package's install
// prefix, or empty when the package is not installed.
std::string LaunchedBuildDoctor(
    const std::function<std::string(const std::string&)>& package_prefix);

// The ament index's answer to `package_prefix` above. Empty as well when the index cannot be read,
// with why in `index_error`.
std::string InstalledPackagePrefix(const std::string& package, std::string* index_error);

// scene/project's facts, from one GET of `<api_base>/v0/scenes/<scene_id>` with the organisation
// key: `api_base` is a scheme and a host. `facts` arrives with what the configuration and the
// environment say; `body` receives the scene document for mesh/files, which reads it too.
doctor::SceneProjectFacts ProbeSceneProject(doctor::SceneProjectFacts facts,
                                            const std::string& api_base,
                                            const std::string& org_key, int timeout_s,
                                            std::string* body);

// topics/qos's facts for one allowlisted topic: every publisher the graph reports, and the profile
// ResolveQos chooses from all of them under `rule`, the rule the node resolves for the topic.
doctor::TopicQosFacts TopicQosFor(const std::string& topic,
                                  const std::vector<rclcpp::TopicEndpointInfo>& publishers,
                                  const TopicRule& rule);

}  // namespace cognitive3d_ros
