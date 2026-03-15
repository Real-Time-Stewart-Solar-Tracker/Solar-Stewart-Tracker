#include "app/SystemFactory.hpp"

#if SOLAR_HAVE_LIBCAMERA
#include "sensors/LibcameraPublisher.hpp"
#endif
#include "sensors/SimulatedPublisher.hpp"

namespace solar::app {

std::unique_ptr<ICamera> SystemFactory::makeCamera_(Logger& log, const AppConfig& cfg) {
#if SOLAR_HAVE_LIBCAMERA
    log.info("Using LibcameraPublisher (Pi Camera)");
    return std::make_unique<LibcameraPublisher>(log, cfg.camera);
#else
    log.info("Using SimulatedPublisher (Desktop)");
    return std::make_unique<SimulatedPublisher>(log, cfg.camera);
#endif
}

std::unique_ptr<SystemManager> SystemFactory::makeSystem(Logger& log, const AppConfig& cfg) {
    auto cam = makeCamera_(log, cfg);

    return std::make_unique<SystemManager>(
        log,
        std::move(cam),
        cfg.tracker,
        cfg.controller,
        cfg.kinematics,
        cfg.actuator,
        cfg.servo
    );
}

} // namespace solar::app
