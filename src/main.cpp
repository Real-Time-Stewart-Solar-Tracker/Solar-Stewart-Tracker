#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>

#include "actuators/ServoDriver.hpp"
#include "common/Logger.hpp"
#include "control/Controller.hpp"
#include "control/Kinematics3RRS.hpp"
#include "sensors/ICamera.hpp"
#include "system/SystemManager.hpp"
#include "vision/SunTracker.hpp"

#if defined(__linux__) && defined(HAVE_LIBCAMERA)
#include "sensors/LibcameraPublisher.hpp"
#endif

#include "sensors/SimulatedPublisher.hpp"

using namespace solar;

static bool g_running = true;

void signalHandler(int) {
    g_running = false;
}

int main() {
    Logger log;

    std::signal(SIGINT, signalHandler);

    log.info("Solar Stewart Tracker starting...");

    // -------------------------------
    // Select camera backend
    // -------------------------------
    std::unique_ptr<ICamera> camera;

#if defined(__linux__) && defined(HAVE_LIBCAMERA)
    {
        LibcameraPublisher::Config camCfg;
        camCfg.width = 640;
        camCfg.height = 480;
        camCfg.fps = 30;

        camera = std::make_unique<LibcameraPublisher>(log, camCfg);
        log.info("Using LibcameraPublisher");
    }
#else
    {
        SimulatedPublisher::Config camCfg;
        camCfg.width = 640;
        camCfg.height = 480;
        camCfg.fps = 30;
        camCfg.moving_spot = true;

        camera = std::make_unique<SimulatedPublisher>(log, camCfg);
        log.info("Using SimulatedPublisher");
    }
#endif

    // -------------------------------
    // Module configurations
    // -------------------------------
    SunTracker::Config trackerCfg;
    trackerCfg.threshold = 50;

    Controller::Config controllerCfg;
    controllerCfg.width = 640;
    controllerCfg.height = 480;

    Kinematics3RRS::Config kinCfg;
    ActuatorManager::Config actCfg;
    ServoDriver::Config drvCfg;

    // -------------------------------
    // Build system
    // -------------------------------
    SystemManager system(
        log,
        std::move(camera),
        trackerCfg,
        controllerCfg,
        kinCfg,
        actCfg,
        drvCfg
    );

    if (!system.start()) {
        log.error("System failed to start");
        return 1;
    }

    log.info("System running. Press Ctrl+C to stop.");

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    system.stop();

    log.info("Shutdown complete.");
    return 0;
}