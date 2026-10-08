#pragma once

#include <memory>
#include <string>

namespace c3d {

enum class LogLevel { kDebug, kInfo, kWarn, kError };

// Every log line goes through this seam, never straight to std::cout or
// std::cerr, so a host such as a ROS node can route it into its own logging.
class ILogger {
 public:
  virtual ~ILogger() = default;
  virtual void Log(LogLevel level, const std::string& message) = 0;

  void Debug(const std::string& m) { Log(LogLevel::kDebug, m); }
  void Info(const std::string& m) { Log(LogLevel::kInfo, m); }
  void Warn(const std::string& m) { Log(LogLevel::kWarn, m); }
  void Error(const std::string& m) { Log(LogLevel::kError, m); }
};

// Default implementation: stderr, level-filtered.
class StderrLogger : public ILogger {
 public:
  explicit StderrLogger(LogLevel min_level = LogLevel::kInfo) : min_(min_level) {}
  void Log(LogLevel level, const std::string& message) override;

 private:
  LogLevel min_;
};

class NullLogger : public ILogger {
 public:
  void Log(LogLevel, const std::string&) override {}
};

std::shared_ptr<ILogger> DefaultLogger();

}  // namespace c3d
