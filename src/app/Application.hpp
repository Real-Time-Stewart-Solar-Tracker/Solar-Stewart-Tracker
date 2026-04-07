#pragma once

/**
 * @file Application.hpp
 * @brief Top-level headless application object.
 *
 * The application owns:
 * - runtime configuration
 * - logger
 * - top-level system manager
 * - CLI controller
 * - Linux event loop
 */

#include "app/AppConfig.hpp"
#include "common/Logger.hpp"

#include <memory>

namespace solar {
class SystemManager;
}

namespace solar::app {

class CliController;
class LinuxEventLoop;

/**
 * @brief Headless application object.
 */
class Application {
public:
    /**
     * @brief Construct the application using default runtime configuration.
     */
    Application();

    /**
     * @brief Destructor.
     */
    ~Application();

    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    /**
     * @brief Run the application.
     *
     * This method constructs and starts the runtime, enters the headless Linux
     * event loop, and returns the final process exit code.
     *
     * @return Process exit code.
     */
    int run();

private:
    Logger logger_{};
    AppConfig config_{};
    std::unique_ptr<solar::SystemManager> system_{};
    std::unique_ptr<CliController> cli_{};
    std::unique_ptr<LinuxEventLoop> loop_{};
};

} // namespace solar::app
