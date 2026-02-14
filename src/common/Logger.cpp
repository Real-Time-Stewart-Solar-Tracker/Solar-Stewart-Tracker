#include "common/Logger.hpp"

#include <iostream>
#include <sstream>

namespace solar {

static long long to_us(std::chrono::steady_clock::time_point t) {
    return std::chrono::duration_cast<std::chrono::microseconds>(t.time_since_epoch()).count();
}

Logger::Logger(const std::string& logFilePath) {
    file_.open(logFilePath, std::ios::out | std::ios::app);
    fileEnabled_ = file_.is_open();
    if (fileEnabled_) {
        logLine_("INFO", "Logger started (file enabled): " + logFilePath);
    } else {
        logLine_("WARN", "Logger started (file open failed): " + logFilePath);
    }
}

Logger::~Logger() {
    // Ensure file is flushed/closed safely
    std::lock_guard<std::mutex> lock(m_);
    if (fileEnabled_) {
        file_.flush();
        file_.close();
    }
}

void Logger::logLine_(const std::string& level, const std::string& msg) {
    // Assumes mutex is held by caller
    std::cout << "[" << level << "] " << msg << "\n";
    if (fileEnabled_) {
        file_ << "[" << level << "] " << msg << "\n";
        file_.flush();
    }
}

void Logger::info(const std::string& msg) {
    std::lock_guard<std::mutex> lock(m_);
    logLine_("INFO", msg);
}

void Logger::warn(const std::string& msg) {
    std::lock_guard<std::mutex> lock(m_);
    logLine_("WARN", msg);
}

void Logger::error(const std::string& msg) {
    std::lock_guard<std::mutex> lock(m_);
    logLine_("ERROR", msg);
}

void Logger::mark(const std::string& stage,
                  std::chrono::steady_clock::time_point t) {
    std::lock_guard<std::mutex> lock(m_);

    std::ostringstream oss;
    oss << "stage=" << stage << " t_us=" << to_us(t);

    logLine_("MARK", oss.str());
}

} // namespace solar