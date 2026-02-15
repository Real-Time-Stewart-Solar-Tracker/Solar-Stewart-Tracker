#pragma once

#include <chrono>
#include <fstream>
#include <mutex>
#include <string>

namespace solar {

/**
 * Logger
 *
 * Thread-safe logger for:
 * - human-readable info/warn/error
 * - timestamped stage markers for latency analysis
 *
 * Notes:
 * - Uses steady_clock for latency (monotonic).
 * - If a log file is opened, logs are written to both console and file.
 */
class Logger {
public:
    Logger() = default;
    explicit Logger(const std::string& logFilePath);

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    ~Logger();

    void info(const std::string& msg);
    void warn(const std::string& msg);
    void error(const std::string& msg);

    // Mark a pipeline stage with a timepoint (for latency analysis).
    void mark(const std::string& stage,
              std::chrono::steady_clock::time_point t);

private:
    void logLine_(const std::string& level, const std::string& msg);

    std::mutex m_;
    std::ofstream file_;
    bool fileEnabled_{false};
};

} // namespace solar