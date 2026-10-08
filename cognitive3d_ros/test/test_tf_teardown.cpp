// Deactivate is the kill switch, so every transition that tears the tf listener down must return,
// whatever state the listener's thread is in. A listener destroyed before its thread has started
// spinning must not block its destructor, or the transition blocks with it. These loops run the
// transitions back to back with every core busy, where that window is widest.
//
// A hang is the failure here, so a watchdog turns one into a failed run rather than a test that
// never finishes: it exits the binary when no step has completed for kStallSeconds.

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "cognitive3d_ros/node.hpp"

namespace fs = std::filesystem;
using lifecycle_msgs::msg::State;

namespace {

// Configure refuses a scene_id that is not a UUID.
constexpr const char kSceneId[] = "a1b2c3d4-0000-4000-8000-000000000001";

constexpr int kStallSeconds = 20;

std::atomic<long> g_progress{0};
std::atomic<const char*> g_step{"start"};
std::atomic<int> g_cycle{0};

void Step(const char* name, int cycle) {
  g_step = name;
  g_cycle = cycle;
  ++g_progress;
}

void Watchdog() {
  long seen = -1;
  auto since = std::chrono::steady_clock::now();
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    const long now = g_progress.load();
    if (now != seen) {
      seen = now;
      since = std::chrono::steady_clock::now();
    } else if (std::chrono::steady_clock::now() - since > std::chrono::seconds(kStallSeconds)) {
      std::fprintf(stderr,
                   "FAILED: no transition returned for %d s; stuck in '%s' at cycle %d. A "
                   "teardown that blocks is the defect this binary exists to catch.\n",
                   kStallSeconds, g_step.load(), g_cycle.load());
      std::fflush(stderr);
      std::_Exit(1);
    }
  }
}

// Twice as many spinning threads as cores, for as long as it lives.
class CpuContention {
 public:
  CpuContention() {
    const unsigned n = std::max(2u, std::thread::hardware_concurrency()) * 2;
    for (unsigned i = 0; i < n; ++i) {
      threads_.emplace_back([this]() {
        volatile unsigned long spins = 0;
        while (run_) ++spins;
      });
    }
  }
  ~CpuContention() {
    run_ = false;
    for (auto& thread : threads_) thread.join();
  }

 private:
  std::atomic<bool> run_{true};
  std::vector<std::thread> threads_;
};

class TfTeardownTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() /
           ("c3d_tfteardown_" + std::to_string(::getpid()) + "_" + std::to_string(counter_++));
    fs::create_directories(dir_);
  }
  void TearDown() override {
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }
  std::shared_ptr<cognitive3d_ros::Cognitive3DNode> MakeNode() {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"scene_id", std::string(kSceneId)},
        {"scene_version", std::string("1")},
        {"spool_dir", dir_.string()},
        {"protocol", std::string("http")},
        {"gateway", std::string("127.0.0.1:1")},
        {"pose_convention", std::string("unity")},
        {"remote_variables", false},
        {"auto_start_session", false},
    });
    return std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  }

  fs::path dir_;
  static int counter_;
};
int TfTeardownTest::counter_ = 0;

}  // namespace

// What an operator's deactivate right after an activate does, and a lifecycle manager rolling back
// a failed bring-up.
TEST_F(TfTeardownTest, ActivateThenDeactivateAtOnceAlwaysReturns) {
  CpuContention contention;
  auto node = MakeNode();
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE) << node->configure_error();
  for (int cycle = 0; cycle < 300; ++cycle) {
    Step("activate", cycle);
    ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE) << "cycle " << cycle;
    Step("deactivate", cycle);
    ASSERT_EQ(node->deactivate().id(), State::PRIMARY_STATE_INACTIVE) << "cycle " << cycle;
  }
  Step("activate before shutdown", 300);
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);
  Step("shutdown from active", 300);
  ASSERT_EQ(node->shutdown().id(), State::PRIMARY_STATE_FINALIZED);
}

TEST_F(TfTeardownTest, ConfigureThenCleanupAtOnceAlwaysReturns) {
  CpuContention contention;
  auto node = MakeNode();
  for (int cycle = 0; cycle < 300; ++cycle) {
    Step("configure", cycle);
    ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE) << node->configure_error();
    Step("cleanup", cycle);
    ASSERT_EQ(node->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED) << "cycle " << cycle;
  }
}

// A node destroyed while active, with no transition at all, as a container unloading it does.
TEST_F(TfTeardownTest, ANodeDestroyedWhileActiveAlwaysReturns) {
  CpuContention contention;
  for (int cycle = 0; cycle < 100; ++cycle) {
    Step("construct, configure and activate", cycle);
    auto node = MakeNode();
    ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE) << node->configure_error();
    ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE) << "cycle " << cycle;
    Step("destroy while active", cycle);
    node.reset();
  }
  Step("done", 100);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ::setenv("C3D_APPLICATION_API_KEY", "K", 1);
  rclcpp::init(argc, argv);
  std::thread(Watchdog).detach();
  const int result = RUN_ALL_TESTS();
  Step("rclcpp shutdown", -1);
  rclcpp::shutdown();
  return result;
}
