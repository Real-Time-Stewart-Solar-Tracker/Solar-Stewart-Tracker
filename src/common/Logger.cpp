#include "common/Logger.hpp"

#include <iostream>

namespace solar {

// Single-line console logger with severity prefixing.

// The logger stays intentionally tiny so it can be used from every layer.

void Logger::info(const std::string& msg) const {
    log_("INFO", msg);
}

void Logger::warn(const std::string& msg) const {
    log_("WARN", msg);
}

void Logger::error(const std::string& msg) const {
    log_("ERROR", msg);
}

void Logger::log_(const char* level, const std::string& msg) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::cerr << "[" << level << "] " << msg << '\n';
}

} // namespace solar