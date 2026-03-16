#pragma once

#include "common/Logger.hpp"
#include "system/SystemManager.hpp"

#include <string>

namespace solar::app {

/**
 * @brief Parses simple terminal commands and drives SystemManager setters.
 *
 * Kept separate from LinuxEventLoop for SRP and testability.
 */
class CliController {
public:
    CliController(Logger& log, SystemManager& sys);

    /**
     * @brief Handle a single line. Returns true if caller should quit.
     */
    bool handleLine(const std::string& line);

private:
    Logger& log_;
    SystemManager& sys_;
};

} // namespace solar::app
