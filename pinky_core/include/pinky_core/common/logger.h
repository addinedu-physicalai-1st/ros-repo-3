#pragma once

#include <string>
#include <memory>

// Forward-declare spdlog to keep the header lightweight.
// Users who only call the Log*() free functions never need spdlog headers.
namespace spdlog { class logger; }

namespace pinky {

// ---------------------------------------------------------------------------
// Logger — thin wrapper around spdlog
// ---------------------------------------------------------------------------
class Logger {
 public:
  enum class Level { kTrace, kDebug, kInfo, kWarn, kError };

  // Initialize the global logger. Call once at startup.
  static void Init(const std::string& name = "pinky",
                   Level level = Level::kInfo);

  // Access the underlying spdlog logger (for advanced use).
  static std::shared_ptr<spdlog::logger> Get();

  // Convenience free functions
  static void Trace(const std::string& msg);
  static void Debug(const std::string& msg);
  static void Info(const std::string& msg);
  static void Warn(const std::string& msg);
  static void Error(const std::string& msg);
};

}  // namespace pinky
