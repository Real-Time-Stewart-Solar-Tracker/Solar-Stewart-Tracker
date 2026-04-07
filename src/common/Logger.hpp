#pragma once

/**
 * @file Logger.hpp
 * @brief Minimal thread-safe logger used across the runtime.
 *
 * This logger is intentionally small:
 * - thread-safe line output
 * - severity-labelled messages
 * - no asynchronous buffering
 * - no file rotation or external dependency
 */

#include <mutex>
#include <string>

namespace solar {

/**
 * @brief Minimal thread-safe logger.
 */
class Logger {
public:
    /**
     * @brief Log an informational message.
     *
     * @param msg Message text.
     */
    void info(const std::string& msg) const;

    /**
     * @brief Log a warning message.
     *
     * @param msg Message text.
     */
    void warn(const std::string& msg) const;

    /**
     * @brief Log an error message.
     *
     * @param msg Message text.
     */
    void error(const std::string& msg) const;

private:
    void log_(const char* level, const std::string& msg) const;

private:
    mutable std::mutex mutex_;
};

} // namespace solar
