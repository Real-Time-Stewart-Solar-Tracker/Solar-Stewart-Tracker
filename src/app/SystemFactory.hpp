#pragma once

/**
 * @file SystemFactory.hpp
 * @brief Composition root for constructing the application runtime graph.
 *
 * This factory is responsible for:
 * - selecting the camera backend
 * - constructing the SystemManager
 * - keeping runtime assembly out of `main()`
 */

#include "app/AppConfig.hpp"

#include <memory>

namespace solar {
class ICamera;
class Logger;
class SystemManager;
}

namespace solar::app {

/**
 * @brief Factory for building the final runtime graph.
 */
class SystemFactory {
public:
    /**
     * @brief Build the top-level system manager.
     *
     * @param log Shared logger.
     * @param cfg Runtime configuration.
     * @return Fully constructed SystemManager or nullptr on failure.
     */
    static std::unique_ptr<SystemManager> makeSystem(Logger& log, const AppConfig& cfg);

private:
    /**
     * @brief Build the selected camera backend.
     *
     * @param log Shared logger.
     * @param cfg Runtime configuration.
     * @return Owned camera backend or nullptr on failure.
     */
    static std::unique_ptr<ICamera> makeCamera_(Logger& log, const AppConfig& cfg);
};

} // namespace solar::app