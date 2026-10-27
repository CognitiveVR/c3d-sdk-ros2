// Every parameter the node declares is documented twice: as a row in docs/configuration.md, and as
// a key in config/cognitive3d.yaml, commented out or not. The names come from the real node, a
// constructed and configured Cognitive3DNode asked for its parameter list, so a parameter declared
// by a helper, in a loop or only at configure is held to the same rule as the rest.
#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <memory>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "cognitive3d_ros/node.hpp"

namespace fs = std::filesystem;

namespace {

// Configure refuses a scene_id that is not a UUID.
constexpr const char kSceneId[] = "a1b2c3d4-0000-4000-8000-000000000001";

// Passed in by CMake rather than derived from the working directory, which is the build tree.
const fs::path kRepoRoot{C3D_REPO_ROOT};
const fs::path kConfigurationPage = kRepoRoot / "docs" / "configuration.md";
const fs::path kReferenceFile = kRepoRoot / "cognitive3d_ros" / "config" / "cognitive3d.yaml";

std::string ReadWhole(const fs::path& path) {
  std::ifstream in(path);
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

// The documented spelling of a name declared per entry: a reader picks the second segment.
std::string WithPlaceholder(const std::string& name) {
  static const std::map<std::string, std::string> kBlocks = {
      {"topic_config", "<key>"}, {"camera", "<name>"}, {"lidar", "<name>"},
      {"sensor", "<name>"}};
  const std::size_t first = name.find('.');
  if (first == std::string::npos) return name;
  const auto block = kBlocks.find(name.substr(0, first));
  const std::size_t second = name.find('.', first + 1);
  if (block == kBlocks.end() || second == std::string::npos) return name;
  return name.substr(0, first + 1) + block->second + name.substr(second);
}

std::set<std::string> ListedBy(const rclcpp_lifecycle::LifecycleNode& node) {
  const std::vector<std::string> names = node.list_parameters({}, 0).names;
  return {names.begin(), names.end()};
}

// What a node with one allowlist entry, one camera and one lidar declares, which is every name a
// configuration can reach, minus rclcpp's own: every name the page and the reference file must
// document. use_sim_time is rclcpp's and is read by the node.
const std::set<std::string>& DeclaredByTheNode() {
  static const std::set<std::string> declared = [] {
    const std::set<std::string> framework =
        ListedBy(*std::make_shared<rclcpp_lifecycle::LifecycleNode>("parameter_reference_probe"));

    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"scene_id", std::string(kSceneId)},
        {"scene_version", std::string("1")},
        {"pose_convention", std::string("unity")},
        {"spool_dir", std::string("/tmp/c3d-parameter-reference-unused")},
        {"dry_run", true},
        {"auto_start_session", false},
        {"topics", std::vector<std::string>{"/drift"}},
        {"cameras", std::vector<std::string>{"drift"}},
        {"camera.drift.topic", std::string("/drift/image")},
        {"lidars", std::vector<std::string>{"drift"}},
        {"lidar.drift.topic", std::string("/drift/points")},
        {"sensors", std::vector<std::string>{"drift"}},
        {"sensor.drift.kind", std::string("binary_array")},
        {"sensor.drift.topic", std::string("/drift")},
        {"sensor.drift.element_count", 1},
    });
    auto node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        << "the per-entry parameters are declared at configure: " << node->configure_error();

    std::set<std::string> result;
    for (const std::string& name : ListedBy(*node)) {
      if (framework.count(name) != 0 && name != "use_sim_time") continue;
      result.insert(WithPlaceholder(name));
    }
    node->cleanup();
    return result;
  }();
  return declared;
}

std::string FirstCell(const std::string& row) {
  const std::size_t open = row.find('|');
  const std::size_t close = row.find('|', open + 1);
  if (open == std::string::npos || close == std::string::npos) return {};
  std::string cell = row.substr(open + 1, close - open - 1);
  cell.erase(0, cell.find_first_not_of(' '));
  cell.erase(cell.find_last_not_of(' ') + 1);
  if (cell.size() >= 2 && cell.front() == '`' && cell.back() == '`') {
    cell = cell.substr(1, cell.size() - 2);
  }
  return cell;
}

// The first cell of every row of every `| parameter |` table on the page.
std::set<std::string> ReadConfigurationPage() {
  std::set<std::string> rows;
  std::istringstream in(ReadWhole(kConfigurationPage));
  std::string line;
  enum class Table { kNone, kParameters, kOther } table = Table::kNone;
  while (std::getline(in, line)) {
    if (line.rfind("|", 0) != 0) {
      table = Table::kNone;
      continue;
    }
    const std::string cell = FirstCell(line);
    if (table == Table::kNone) {
      table = cell == "parameter" ? Table::kParameters : Table::kOther;
    } else if (table == Table::kParameters && cell.find_first_not_of('-') != std::string::npos) {
      rows.insert(cell);
    }
  }
  return rows;
}

// Every key path below `ros__parameters`. With `commented`, a comment whose text after "# " is a
// YAML key counts too, at the indentation it would have uncommented; without it, only a key that
// sets a value on its own line.
std::set<std::string> ReferenceFileKeys(bool commented) {
  static const std::regex kKey(R"(^([A-Za-z0-9_/*.<>~-]+):(\s.*)?$)");
  static const std::string kRoot = "/**.ros__parameters.";
  std::set<std::string> keys;
  std::vector<std::pair<std::size_t, std::string>> path;
  std::istringstream in(ReadWhole(kReferenceFile));
  std::string line;
  while (std::getline(in, line)) {
    std::size_t indent = line.find_first_not_of(' ');
    if (indent == std::string::npos) continue;
    std::string text = line.substr(indent);
    if (text.front() == '#') {
      if (!commented) continue;
      text.erase(0, text.size() > 1 && text[1] == ' ' ? 2 : 1);
      const std::size_t inner = text.find_first_not_of(' ');
      if (inner == std::string::npos) continue;
      indent += inner;
      text.erase(0, inner);
    }
    std::smatch key;
    if (!std::regex_match(text, key, kKey)) continue;
    while (!path.empty() && path.back().first >= indent) path.pop_back();
    path.emplace_back(indent, key[1].str());
    std::string joined;
    for (const auto& segment : path) joined += (joined.empty() ? "" : ".") + segment.second;
    // A key with nothing after it on its line only opens a nested block (`pose:`), and sets
    // nothing. Counted where documentation is asked for, where a list's key opens its items too.
    const std::string rest = key[2].str();
    const std::size_t value = rest.find_first_not_of(" \t");
    const bool sets_a_value = value != std::string::npos && rest[value] != '#';
    if (joined.rfind(kRoot, 0) == 0 && (commented || sets_a_value)) {
      keys.insert(WithPlaceholder(joined.substr(kRoot.size())));
    }
  }
  return keys;
}

void RequireTheFile(const fs::path& path) {
  ASSERT_TRUE(fs::exists(path)) << path << " is not where this test was told the repo is ("
                                << kRepoRoot << "), so nothing below would be checked.";
}

}  // namespace

TEST(ParameterReference, EveryParameterTheNodeDeclaresHasARowOnTheConfigurationPage) {
  RequireTheFile(kConfigurationPage);
  const std::set<std::string> rows = ReadConfigurationPage();
  for (const std::string& name : DeclaredByTheNode()) {
    EXPECT_EQ(rows.count(name), 1u)
        << "docs/configuration.md has no `| `" << name << "` |` row. Add one to the table of its "
        << "group, with its type, default, unit and what it does.";
  }
}

TEST(ParameterReference, EveryRowOnTheConfigurationPageIsAParameterTheNodeDeclares) {
  RequireTheFile(kConfigurationPage);
  const std::set<std::string>& declared = DeclaredByTheNode();
  for (const std::string& row : ReadConfigurationPage()) {
    EXPECT_EQ(declared.count(row), 1u)
        << "docs/configuration.md documents `" << row << "`, which the node does not declare. "
        << "Remove the row.";
  }
}

TEST(ParameterReference, EveryParameterTheNodeDeclaresIsAKeyInTheReferenceFile) {
  RequireTheFile(kReferenceFile);
  const std::set<std::string> keys = ReferenceFileKeys(/*commented=*/true);
  for (const std::string& name : DeclaredByTheNode()) {
    EXPECT_EQ(keys.count(name), 1u)
        << "config/cognitive3d.yaml has no key for " << name
        << ". Add it at its default, commented out when it is a list or belongs to a block.";
  }
}

TEST(ParameterReference, EveryValueTheReferenceFileSetsIsTheDefaultTheNodeDeclares) {
  RequireTheFile(kReferenceFile);
  std::map<std::string, rclcpp::ParameterValue> defaults;
  {
    auto bare = std::make_shared<cognitive3d_ros::Cognitive3DNode>(rclcpp::NodeOptions());
    for (const std::string& name : ListedBy(*bare)) {
      defaults[name] = bare->get_parameter(name).get_parameter_value();
    }
  }
  rclcpp::NodeOptions from_file;
  from_file.arguments({"--ros-args", "--params-file", kReferenceFile.string()});
  auto loaded = std::make_shared<cognitive3d_ros::Cognitive3DNode>(from_file);
  for (const auto& [name, value] : defaults) {
    const rclcpp::ParameterValue given = loaded->get_parameter(name).get_parameter_value();
    EXPECT_TRUE(given == value) << "config/cognitive3d.yaml sets " << name << " to "
                                << rclcpp::to_string(given) << ", and its default is "
                                << rclcpp::to_string(value);
  }
  for (const std::string& key : ReferenceFileKeys(/*commented=*/false)) {
    EXPECT_EQ(defaults.count(key), 1u)
        << "config/cognitive3d.yaml sets " << key << ", which the node does not declare, so "
        << "rclcpp ignores it without a word.";
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ::setenv("C3D_APPLICATION_API_KEY", "K", 1);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
