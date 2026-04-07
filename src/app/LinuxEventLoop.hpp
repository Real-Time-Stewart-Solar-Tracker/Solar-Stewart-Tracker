#pragma once

/**
 * @file LinuxEventLoop.hpp
 * @brief Headless Linux application event loop for CLI/runtime control.
 *
 * This loop is intentionally narrow in scope:
 * - wait for user/OS control events
 * - drive clean shutdown
 * - keep `main()` tiny
 *
 * It is NOT the realtime sensor/control loop.
 */

#include <cstdint>

namespace solar {
class SystemManager;
}

namespace solar::app {

class CliController;

/**
 * @brief Small Linux userspace event loop for the headless application.
 */
class LinuxEventLoop {
public:
    /**
     * @brief Construct the event loop.
     *
     * @param system Runtime system manager.
     * @param cli CLI controller.
     * @param tick_hz Tick frequency used for timer-driven CLI servicing.
     */
    LinuxEventLoop(SystemManager& system, CliController& cli, std::uint32_t tick_hz);

    LinuxEventLoop(const LinuxEventLoop&) = delete;
    LinuxEventLoop& operator=(const LinuxEventLoop&) = delete;

    /**
     * @brief Run the event loop until shutdown.
     *
     * @return Process exit code.
     */
    int run();

private:
    SystemManager& system_;
    CliController& cli_;
    std::uint32_t tick_hz_{30U};
};

} // namespace solar::app
