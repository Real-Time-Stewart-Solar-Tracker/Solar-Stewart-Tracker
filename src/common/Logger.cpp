#include "common/Logger.hpp"

#include <chrono>
#include <iostream>
#include <sstream>

namespace solar {

Logger::Logger(const std::string& logFilePath) {
    std::lock_guard<std::mutex> lock(m_);
    file_.open(logFilePath, std::ios::out | std::ios::app);
    fileEnabled_ = file_.is_open();
}

Logger::~Logger() {
    std::lock_guard<std::mutex> lock(m_);
    if (file_.is_open()) {
        file_.flush();
        file_.close();
    }
}

void Logger::info(const std::string& msg)  { logLine_("INFO",  msg, false); }
void Logger::warn(const std::string& msg)  { logLine_("WARN",  msg, false); }
void Logger::error(const std::string& msg) { logLine_("ERROR", msg, true ); }

void Logger::mark(const std::string& stage, std::chrono::steady_clock::time_point t) {
    const auto us = std::chrono::duration_cast<std::chrono::microseconds>(t.time_since_epoch()).count();
    std::ostringstream oss;
    oss << stage << " t_us=" << us;
    logLine_("MARK", oss.str(), false);
}

void Logger::logLine_(const std::string& level, const std::string& msg, bool flush_now) {
    std::lock_guard<std::mutex> lock(m_);

    const auto now = std::chrono::steady_clock::now();
    const auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    std::ostringstream line;
    line << "[" << ms << "ms] " << level << ": " << msg << "\n";

    // Console (still potentially slow; avoid calling logger too frequently in hot path)
    std::cout << line.str();

    if (fileEnabled_) {
        file_ << line.str();

        // Flush only on ERROR, or if explicitly requested
        if (flush_now) {
            file_.flush();
        } else {
            // Optional: periodic flush (every 200 lines) to reduce data loss risk
            if (++lineCount_ % 200u == 0u) {
                file_.flush();
            }
        }
    }
}

} // namespace solar