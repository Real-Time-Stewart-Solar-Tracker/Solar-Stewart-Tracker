#pragma once

#include "app/AppConfig.hpp"
#include "app/CliController.hpp"
#include "common/Logger.hpp"
#include "system/SystemManager.hpp"

#if SOLAR_HAVE_OPENCV
#include "ui/UiViewer.hpp"
#endif

#include <cstdint>

namespace solar::app {

/**
 * @brief Linux-only event loop (poll on signalfd/timerfd/stdin).
 *
 * Owns no business logic; delegates command parsing to CliController.
 */
class LinuxEventLoop {
public:
    LinuxEventLoop(Logger& log,
                  SystemManager& sys,
                  const AppConfig& cfg
#if SOLAR_HAVE_OPENCV
                  , UiViewer* ui
#endif
                  );

    /**
     * @return process return code
     */
    int run();

private:
    Logger& log_;
    SystemManager& sys_;
    const AppConfig& cfg_;
    CliController cli_;

#if SOLAR_HAVE_OPENCV
    UiViewer* ui_{nullptr};
#endif
};

} // namespace solar::app
