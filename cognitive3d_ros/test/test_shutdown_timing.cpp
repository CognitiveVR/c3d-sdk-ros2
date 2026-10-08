// Ending a session, deactivating, shutting down and SIGINT all return promptly while an upload is
// stuck: a gateway that accepts the connection and never answers. Ending a session spools its end
// and closes its marker; it never waits for an upload, and the destructor gives the tail a bounded
// chance. A node that waited would sit out curl's low-speed limit, minutes here.
//
// Its own binary and ROS domain, because its failure is a hang: the watchdog that turns one into
// a failed run ends the process.

#include <fcntl.h>
#include <gtest/gtest.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "cognitive3d_ros/node.hpp"
#include "cognitive3d_ros_msgs/srv/end_session.hpp"
#include "cognitive3d_ros_msgs/srv/start_session.hpp"
#include "throttled_http_server.hpp"

extern char** environ;

namespace fs = std::filesystem;
using cognitive3d_ros_msgs::srv::EndSession;
using cognitive3d_ros_msgs::srv::StartSession;
using lifecycle_msgs::msg::State;

namespace {

// Configure refuses a scene_id that is not a UUID.
constexpr const char kSceneId[] = "a1b2c3d4-0000-4000-8000-000000000001";

// How long curl may go without moving a byte before it gives up on the stalled upload: long
// enough that only a node that waits for the upload can be slow.
constexpr int kLowSpeedTimeS = 120;
// Far short of kLowSpeedTimeS, and far past what a prompt return takes.
constexpr double kPrompt = 2.0;
// The destructor's bound: Config::shutdown_drain_s, then the interrupt curl polls about once a
// second.
constexpr double kDestroyed = 2.0 + 3.0;
// launch escalates SIGINT to SIGTERM after five seconds.
constexpr double kSigintGrace = 5.0;

double Now() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool WaitUntil(const std::function<bool()>& done, double seconds) {
  const double deadline = Now() + seconds;
  while (Now() < deadline) {
    if (done()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return done();
}

// Ends the process when a step outlives its deadline: a step that never returns is the failure
// under test, and it cannot be joined.
class Watchdog {
 public:
  Watchdog() : thread_([this] { Run(); }) {}
  ~Watchdog() {
    stop_.store(true);
    thread_.join();
  }
  // Runs `step`, failing the run if it has not returned within `limit` seconds. Returns how long
  // it took.
  double Time(const std::string& name, double limit, const std::function<void()>& step) {
    {
      std::lock_guard<std::mutex> lock(mu_);
      name_ = name;
      deadline_ = Now() + limit;
    }
    const double start = Now();
    step();
    const double took = Now() - start;
    std::lock_guard<std::mutex> lock(mu_);
    deadline_ = 0.0;
    return took;
  }

 private:
  void Run() {
    while (!stop_.load()) {
      {
        std::lock_guard<std::mutex> lock(mu_);
        if (deadline_ > 0.0 && Now() > deadline_) {
          std::fprintf(stderr, "FAILED: '%s' did not return; the node waited for the upload\n",
                       name_.c_str());
          std::fflush(stderr);
          std::_Exit(1);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
  std::mutex mu_;
  std::string name_;
  double deadline_ = 0.0;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

std::vector<rclcpp::Parameter> Overrides(const fs::path& dir, const std::string& gateway) {
  return {
      {"scene_id", std::string(kSceneId)},
      {"scene_version", std::string("1")},
      {"device_id", std::string("robot")},
      {"spool_dir", dir.string()},
      {"protocol", std::string("http")},
      {"gateway", gateway},
      {"pose_convention", std::string("unity")},
      {"remote_variables", false},
      {"topics", std::vector<std::string>{}},
      {"http_low_speed_time_s", kLowSpeedTimeS},
      {"http_timeout_s", 600},
  };
}

std::shared_ptr<cognitive3d_ros::Cognitive3DNode> MakeNode(const fs::path& dir,
                                                           const std::string& gateway,
                                                           std::vector<rclcpp::Parameter> extra) {
  auto overrides = Overrides(dir, gateway);
  for (auto& parameter : extra) overrides.push_back(std::move(parameter));
  rclcpp::NodeOptions options;
  options.parameter_overrides(overrides);
  return std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
}

// A paused run's spool, recorded for `gateway`, for the node under test to start uploading the
// moment it is configured.
void RecordBacklog(const fs::path& dir, const std::string& gateway) {
  auto recorder = MakeNode(dir, gateway, {{"uploads_paused", true}});
  ASSERT_EQ(recorder->configure().id(), State::PRIMARY_STATE_INACTIVE)
      << recorder->configure_error();
  ASSERT_EQ(recorder->activate().id(), State::PRIMARY_STATE_ACTIVE);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  recorder->deactivate();
  recorder->cleanup();
}

std::size_t OpenMarkers(const fs::path& dir) {
  std::size_t n = 0;
  std::error_code ec;
  for (const auto& entry : fs::directory_iterator(dir / "sessions", ec)) {
    if (entry.path().extension() == ".c3dsession") ++n;
  }
  return n;
}

c3d::testing_support::ThrottledHttpServer::Options NeverAnswers() {
  c3d::testing_support::ThrottledHttpServer::Options options;
  options.stall_after_bytes = 0;
  return options;
}

fs::path FreshDirectory(const std::string& name) {
  const fs::path dir = fs::temp_directory_path() / (name + "_" + std::to_string(::getpid()));
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

TEST(ShutdownTiming, EndSessionDeactivateShutdownAndDestructionReturnPromptlyBehindAStalledUpload) {
  const fs::path dir = FreshDirectory("c3d_stalled");
  c3d::testing_support::ThrottledHttpServer server(NeverAnswers());
  RecordBacklog(dir, server.host());

  Watchdog watchdog;
  auto node = MakeNode(dir, server.host(), {});
  auto client = std::make_shared<rclcpp::Node>("shutdown_timing_client");
  auto start = client->create_client<StartSession>("/cognitive3d/start_session");
  auto end = client->create_client<EndSession>("/cognitive3d/end_session");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(client);

  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE) << node->configure_error();
  ASSERT_TRUE(WaitUntil([&] { return server.stalled() >= 1; }, 15.0))
      << "the backlog never reached the gateway";
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(end->wait_for_service(std::chrono::seconds(5)));

  bool ended = false;
  const double end_took = watchdog.Time("~/end_session", 30.0, [&] {
    auto future = end->async_send_request(std::make_shared<EndSession::Request>());
    ended = executor.spin_until_future_complete(future, std::chrono::seconds(25)) ==
                rclcpp::FutureReturnCode::SUCCESS &&
            future.get()->ok;
  });
  EXPECT_TRUE(ended);
  EXPECT_LT(end_took, kPrompt) << "~/end_session waited behind the stalled upload";
  EXPECT_EQ(OpenMarkers(dir), 0u) << "the ended session's marker is still open";
  EXPECT_GE(server.stalled(), 1u) << "the upload under test was no longer stalled";

  auto future = start->async_send_request(std::make_shared<StartSession::Request>());
  ASSERT_EQ(executor.spin_until_future_complete(future, std::chrono::seconds(10)),
            rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(future.get()->ok);
  const double deactivate_took = watchdog.Time("deactivate", 30.0, [&] {
    EXPECT_EQ(node->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  });
  EXPECT_LT(deactivate_took, kPrompt) << "deactivate waited behind the stalled upload";

  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);
  const double shutdown_took = watchdog.Time("shutdown", 30.0, [&] {
    EXPECT_EQ(node->shutdown().id(), State::PRIMARY_STATE_FINALIZED);
  });
  EXPECT_LT(shutdown_took, kPrompt) << "shutdown from active waited behind the stalled upload";
  EXPECT_EQ(OpenMarkers(dir), 0u);
  EXPECT_GE(server.stalled(), 1u) << "the upload under test was no longer stalled";

  executor.remove_node(client);
  executor.remove_node(node->get_node_base_interface());
  const double destroy_took = watchdog.Time("destroying the node", 30.0, [&] { node.reset(); });
  EXPECT_LT(destroy_took, kDestroyed) << "destruction outwaited the drain and the interrupt";
  std::error_code ec;
  fs::remove_all(dir, ec);
}

// SIGINT reaches the shipped executable's own handler, which shuts the context down; the node is
// then destroyed and the process exits. Behind a stalled upload that must still beat launch's
// escalation to SIGTERM, and end the session in order, as a shutdown: no marker is left for
// recovery.
TEST(ShutdownTiming, SigintEndsTheShippedExecutableInOrderBehindAStalledUpload) {
  const fs::path dir = FreshDirectory("c3d_sigint");
  c3d::testing_support::ThrottledHttpServer server(NeverAnswers());
  RecordBacklog(dir, server.host());

  const std::string name = "c3d_sigint_" + std::to_string(::getpid());
  const fs::path params = dir / "params.yaml";
  {
    std::ofstream out(params);
    out << "/**:\n  ros__parameters:\n"
        << "    scene_id: \"" << kSceneId << "\"\n"
        << "    scene_version: \"1\"\n    device_id: \"robot\"\n"
        << "    spool_dir: \"" << dir.string() << "\"\n"
        << "    protocol: \"http\"\n    gateway: \"" << server.host() << "\"\n"
        << "    pose_convention: \"unity\"\n    remote_variables: false\n"
        << "    http_low_speed_time_s: " << kLowSpeedTimeS << "\n    http_timeout_s: 600\n";
  }
  const fs::path log = dir / "node.log";
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_addopen(&actions, 1, log.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  posix_spawn_file_actions_adddup2(&actions, 1, 2);
  std::vector<std::string> args = {C3D_NODE_EXECUTABLE, "--ros-args", "--params-file",
                                   params.string(),     "-r",         "__node:=" + name};
  std::vector<char*> argv;
  for (auto& arg : args) argv.push_back(arg.data());
  argv.push_back(nullptr);
  pid_t child = -1;
  ASSERT_EQ(posix_spawn(&child, C3D_NODE_EXECUTABLE, &actions, nullptr, argv.data(), environ), 0);
  posix_spawn_file_actions_destroy(&actions);
  const auto log_text = [&log] {
    std::ifstream in(log);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  };
  const auto stop_child = [&child] {
    if (child > 0) {
      ::kill(child, SIGKILL);
      ::waitpid(child, nullptr, 0);
      child = -1;
    }
  };

  auto client = std::make_shared<rclcpp::Node>("sigint_timing_client");
  auto change = client->create_client<lifecycle_msgs::srv::ChangeState>("/" + name +
                                                                       "/change_state");
  const auto transition = [&](std::uint8_t id) {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = id;
    auto future = change->async_send_request(request);
    return rclcpp::spin_until_future_complete(client, future, std::chrono::seconds(15)) ==
               rclcpp::FutureReturnCode::SUCCESS &&
           future.get()->success;
  };
  if (!change->wait_for_service(std::chrono::seconds(20))) {
    stop_child();
    FAIL() << "the node never offered its lifecycle services:\n" << log_text();
  }
  if (!transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE) ||
      !transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
    stop_child();
    FAIL() << "the node did not configure and activate:\n" << log_text();
  }
  if (!WaitUntil([&] { return server.stalled() >= 1; }, 15.0)) {
    stop_child();
    FAIL() << "the backlog never reached the gateway:\n" << log_text();
  }

  ASSERT_EQ(::kill(child, SIGINT), 0);
  const double signalled = Now();
  int status = 0;
  pid_t reaped = 0;
  while (Now() - signalled < 30.0) {
    reaped = ::waitpid(child, &status, WNOHANG);
    if (reaped != 0) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  const double took = Now() - signalled;
  if (reaped == 0) {
    stop_child();
    FAIL() << "SIGINT did not end the node within 30 s:\n" << log_text();
  }
  child = -1;
  EXPECT_LT(took, kSigintGrace) << "launch would have escalated to SIGTERM:\n" << log_text();
  EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0)
      << "status " << status << ":\n" << log_text();
  EXPECT_EQ(OpenMarkers(dir), 0u) << "the session was not ended in order:\n" << log_text();
  EXPECT_NE(log_text().find(" ended (shutdown)"), std::string::npos)
      << "a stop by signal is an orderly shutdown, and the session's end says so:\n" << log_text();
  EXPECT_EQ(log_text().find(" ended (destructor)"), std::string::npos) << log_text();
  std::error_code ec;
  fs::remove_all(dir, ec);
}

}  // namespace

int main(int argc, char** argv) {
  ::setenv("C3D_APPLICATION_API_KEY", "SHUTDOWNTIMINGKEY", 1);
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
