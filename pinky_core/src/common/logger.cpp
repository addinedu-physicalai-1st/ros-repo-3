#include "pinky_core/common/logger.h"

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace pinky {

namespace {
std::shared_ptr<spdlog::logger> g_logger;
}

void Logger::Init(const std::string& name, Level level) {
  g_logger = spdlog::stdout_color_mt(name);

  switch (level) {
    case Level::kTrace: g_logger->set_level(spdlog::level::trace); break;
    case Level::kDebug: g_logger->set_level(spdlog::level::debug); break;
    case Level::kInfo:  g_logger->set_level(spdlog::level::info);  break;
    case Level::kWarn:  g_logger->set_level(spdlog::level::warn);  break;
    case Level::kError: g_logger->set_level(spdlog::level::err);   break;
  }

  g_logger->set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");
}

std::shared_ptr<spdlog::logger> Logger::Get() {
  if (!g_logger) {
    Init();
  }
  return g_logger;
}

void Logger::Trace(const std::string& msg) { Get()->trace(msg); }
void Logger::Debug(const std::string& msg) { Get()->debug(msg); }
void Logger::Info(const std::string& msg)  { Get()->info(msg); }
void Logger::Warn(const std::string& msg)  { Get()->warn(msg); }
void Logger::Error(const std::string& msg) { Get()->error(msg); }

}  // namespace pinky
