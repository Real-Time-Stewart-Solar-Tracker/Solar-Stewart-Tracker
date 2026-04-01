#pragma once

#include "app/AppConfig.hpp"
#include "common/Logger.hpp"
#include "system/SystemManager.hpp"

#include <memory>

namespace solar::app {

/**
 * @brief Creates the application graph (Camera + SystemManager) from AppConfig.
 */
class SystemFactory {
public:
    static std::unique_ptr<SystemManager> makeSystem(Logger& log, const AppConfig& cfg);

private:
    static std::unique_ptr<ICamera> makeCamera_(Logger& log, const AppConfig& cfg);
};

} // namespace solar::app
