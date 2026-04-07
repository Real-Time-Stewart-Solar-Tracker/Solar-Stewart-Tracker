#include "app/CliController.hpp"

#include "system/SystemManager.hpp"
#include "system/TrackerState.hpp"

#include <charconv>
#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>

#include <unistd.h>

namespace solar::app {
namespace {

bool parseTwoFloats(const std::string_view text, float& a, float& b) {
    const std::string s(text);
    const auto pos = s.find_first_of(" \t");
    if (pos == std::string::npos) {
        return false;
    }

    const std::string first = s.substr(0, pos);
    const std::string rest = s.substr(pos + 1);

    char* end1 = nullptr;
    char* end2 = nullptr;

    const float v1 = std::strtof(first.c_str(), &end1);
    const float v2 = std::strtof(rest.c_str(), &end2);

    if (end1 == first.c_str() || end2 == rest.c_str()) {
        return false;
    }

    a = v1;
    b = v2;
    return true;
}

} // namespace

CliController::CliController(SystemManager& system)
    : system_(system) {
}

void CliController::attachInputFd(const int fd) {
    input_fd_ = fd;
}

void CliController::detachInputFd() {
    input_fd_ = -1;
    pending_.clear();
}

bool CliController::onTick() {
    ++tick_count_;

    // Print a one-line runtime status at the configured interval so the
    // operator can confirm that the headless process is alive and in the
    // expected state without querying it explicitly.
    if (tick_count_ % kStatusIntervalTicks == 0U) {
        const TrackerState s = system_.state();
        std::printf("[solar] state=%s\n", solar::toString(s));
        std::fflush(stdout);
    }

    return true;
}

bool CliController::onInputReady() {
    if (input_fd_ < 0) {
        return true;
    }

    char buffer[256];
    const ssize_t n = ::read(input_fd_, buffer, sizeof(buffer));
    if (n <= 0) {
        return false;
    }

    pending_.append(buffer, buffer + n);

    std::size_t newline = std::string::npos;
    while ((newline = pending_.find('\n')) != std::string::npos) {
        std::string line = pending_.substr(0, newline);
        pending_.erase(0, newline + 1);

        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }

        if (!executeLine_(line)) {
            return false;
        }
    }

    return true;
}

bool CliController::executeLine_(const std::string& line) {
    if (line.empty()) {
        return true;
    }

    if (line == "quit" || line == "exit") {
        return false;
    }

    if (line == "help") {
        printHelp_();
        return true;
    }

    if (line == "manual") {
        system_.enterManual();
        std::puts("CLI: entered manual mode");
        return true;
    }

    if (line == "auto") {
        system_.exitManual();
        std::puts("CLI: returned to automatic mode");
        return true;
    }

    if (line.rfind("threshold ", 0) == 0) {
        const std::string arg = line.substr(std::strlen("threshold "));
        char* end = nullptr;
        const long value = std::strtol(arg.c_str(), &end, 10);
        if (end == arg.c_str() || value < 0 || value > 255) {
            std::puts("CLI: invalid threshold");
            return true;
        }

        system_.setTrackerThreshold(static_cast<std::uint8_t>(value));
        std::puts("CLI: threshold updated");
        return true;
    }

    if (line.rfind("set ", 0) == 0) {
        const std::string args = line.substr(std::strlen("set "));
        float tilt_rad = 0.0F;
        float pan_rad = 0.0F;
        if (!parseTwoFloats(args, tilt_rad, pan_rad)) {
            std::puts("CLI: usage: set <tilt_rad> <pan_rad>");
            return true;
        }

        system_.setManualSetpoint(tilt_rad, pan_rad);
        std::puts("CLI: manual setpoint sent");
        return true;
    }

    std::puts("CLI: unknown command");
    printHelp_();
    return true;
}

void CliController::printHelp_() const {
    std::puts("Commands:");
    std::puts("  help");
    std::puts("  manual");
    std::puts("  auto");
    std::puts("  threshold <0..255>");
    std::puts("  set <tilt_rad> <pan_rad>");
    std::puts("  quit");
}

} // namespace solar::app
