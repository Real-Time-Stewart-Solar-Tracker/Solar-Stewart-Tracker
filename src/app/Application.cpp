#include "app/Application.hpp"

#include "app/CliController.hpp"
#include "app/LinuxEventLoop.hpp"
#include "app/SystemFactory.hpp"
#include "system/SystemManager.hpp"

namespace solar::app {

Application::Application()
    : logger_{},
      config_(defaultConfig()) {
    system_ = SystemFactory::makeSystem(logger_, config_);
    cli_    = std::make_unique<CliController>(*system_);
    loop_   = std::make_unique<LinuxEventLoop>(*system_, *cli_, config_.tick_hz);
}

Application::~Application() = default;

int Application::run() {
    if (!system_ || !cli_ || !loop_) {
        logger_.error("Application: failed to construct runtime");
        return 1;
    }

    return loop_->run();
}

} // namespace solar::app
