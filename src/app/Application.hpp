#pragma once

#include "app/AppConfig.hpp"
#include "common/Logger.hpp"

#include <memory>
#include <string>

namespace solar {

class SystemManager;  // Forward declaration

namespace app {

/**
 * @brief Headless (CLI) application entry point.
 *
 * Owns the SystemManager lifecycle and runs the platform event loop.
 * The tracking pipeline itself is implemented inside SystemManager.
 */
class Application final {
public:
    Application();
    ~Application();

    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    /**
     * @brief Run the application until shutdown.
     * @return Exit code (0 on clean exit).
     */
    int run();

private:
    void buildSystem_();
    int runLinux_();
    int runNonLinux_();
    void handleCommandLine_(const std::string& line);

private:
    Logger log_;
    AppConfig cfg_;
    std::unique_ptr<SystemManager> sys_;
};

} // namespace app
} // namespace solar