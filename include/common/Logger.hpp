#pragma once

#include <chrono>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>

namespace solar {

/**
 * @brief Thread-safe logging utility for runtime diagnostics and latency tracing.
 *
 * Provides:
 * - Human-readable info/warn/error logging
 * - Timestamped stage markers for latency analysis
 *
 * Uses std::chrono::steady_clock for monotonic timing.
 * If a file path is provided, logs are written to both console and file.
 */
class Logger {
public:
    /// @brief Construct a console-only logger.
    Logger() = default;

    /// @brief Construct a logger that writes to console and a log file.
    /// @param logFilePath Path to output log file.
    explicit Logger(const std::string& logFilePath);

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    /// @brief Destructor closes the log file if open.
    ~Logger();

    /// @brief Log an informational message.
    /// @param msg Message text.
    void info(const std::string& msg);

    /// @brief Log a warning message.
    /// @param msg Message text.
    void warn(const std::string& msg);

    /// @brief Log an error message (forces immediate flush).
    /// @param msg Message text.
    void error(const std::string& msg);

    /**
     * @brief Log a timestamped stage marker.
     * @param stage Logical stage name (e.g., "capture", "control").
     * @param t Monotonic time point associated with the stage.
     */
    void mark(const std::string& stage,
              std::chrono::steady_clock::time_point t);

private:
    /// @brief Internal helper for formatting and output.
    void logLine_(const std::string& level,
                  const std::string& msg,
                  bool flush_now);

    std::mutex m_;
    std::ofstream file_;
    bool fileEnabled_{false};

    /// @brief Line counter used for periodic flushing.
    std::uint32_t lineCount_{0};
};

} // namespace solar