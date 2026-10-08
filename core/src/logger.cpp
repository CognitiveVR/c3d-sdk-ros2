#include "c3d/logger.hpp"
#include <iostream>
#include <mutex>

namespace c3d {
namespace {
std::mutex g_log_mutex;
const char* LevelName(LogLevel l) {
  switch (l) {
    case LogLevel::kDebug: return "DEBUG";
    case LogLevel::kInfo: return "INFO";
    case LogLevel::kWarn: return "WARN";
    case LogLevel::kError: return "ERROR";
  }
  return "INFO";
}
}  // namespace

void StderrLogger::Log(LogLevel level, const std::string& message) {
  if (static_cast<int>(level) < static_cast<int>(min_)) return;
  std::lock_guard<std::mutex> lock(g_log_mutex);
  std::cerr << "[c3d:" << LevelName(level) << "] " << message << std::endl;
}

std::shared_ptr<ILogger> DefaultLogger() {
  static std::shared_ptr<ILogger> instance = std::make_shared<StderrLogger>();
  return instance;
}

}  // namespace c3d
