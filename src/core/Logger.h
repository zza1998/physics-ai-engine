#pragma once

#include <iostream>
#include <string>

namespace leo {

// M1 用最简单的日志, 后续可换 spdlog
enum class LogLevel { Trace, Info, Warn, Error };

inline const char* levelStr(LogLevel l) {
    switch (l) {
        case LogLevel::Trace: return "[TRACE]";
        case LogLevel::Info:  return "[INFO] ";
        case LogLevel::Warn:  return "[WARN] ";
        case LogLevel::Error: return "[ERROR]";
    }
    return "[?????]";
}

inline void log(LogLevel level, const std::string& msg) {
    std::cout << levelStr(level) << ' ' << msg << '\n';
}

#define LEO_LOG_INFO(msg)  ::leo::log(::leo::LogLevel::Info,  msg)
#define LEO_LOG_WARN(msg)  ::leo::log(::leo::LogLevel::Warn,  msg)
#define LEO_LOG_ERROR(msg) ::leo::log(::leo::LogLevel::Error, msg)

} // namespace leo
