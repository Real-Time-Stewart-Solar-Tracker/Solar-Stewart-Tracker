#include "app/CliController.hpp"

#include <cstdio>

namespace solar::app {

CliController::CliController(Logger& log, SystemManager& sys)
    : log_(log), sys_(sys) {}

bool CliController::handleLine(const std::string& line) {
    if (line == "q") {
        return true;
    }
    if (line == "m") {
        sys_.enterManual();
        log_.info("MANUAL: use 'sp <tilt_rad> <pan_rad>' or 'a' for AUTO");
        return false;
    }
    if (line == "a") {
        sys_.exitManual();
        log_.info("AUTO mode");
        return false;
    }
    if (line.rfind("sp ", 0) == 0) {
        float tilt = 0.f, pan = 0.f;
        if (std::sscanf(line.c_str(), "sp %f %f", &tilt, &pan) == 2) {
            sys_.setManualSetpoint(tilt, pan);
            log_.info("Manual setpoint sent");
        } else {
            log_.warn("Format: sp <tilt_rad> <pan_rad>");
        }
        return false;
    }

    log_.info("Commands: m (manual), a (auto), sp <tilt> <pan>, q (quit)");
    return false;
}

} // namespace solar::app
