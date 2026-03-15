#include "app/Application.hpp"
#include "system/SystemManager.hpp"
#include "app/LinuxEventLoop.hpp"
#include "app/SystemFactory.hpp"

#if SOLAR_HAVE_OPENCV
#include "ui/UiViewer.hpp"
#endif

#include <iostream>

namespace solar {
namespace app {

Application::Application()
    : log_()
    ,cfg_(defaultConfig()) 
    , sys_(nullptr) {}

Application::~Application() = default;
int Application::run() {
    sys_ = SystemFactory::makeSystem(log_, cfg_);

    if (!sys_ || !sys_->start()) {
        log_.error("Failed to start SystemManager");
        return 1;
    }

#if SOLAR_HAVE_OPENCV
    UiViewer::Config uiCfg{};
    uiCfg.width        = cfg_.controller.width;
    uiCfg.height       = cfg_.controller.height;
    uiCfg.threshold    = cfg_.tracker.threshold;
    uiCfg.plot_height  = 0;
    uiCfg.plot_stride_px = 2;
    uiCfg.plot_history = 600;
    UiViewer ui(log_, uiCfg);

    sys_->registerFrameObserver([&ui](const FrameEvent& fe) { ui.onFrame(fe); });
    sys_->registerEstimateObserver([&ui](const SunEstimate& est) { ui.onEstimate(est); });
    sys_->registerSetpointObserver([&ui](const PlatformSetpoint& sp) { ui.onSetpoint(sp); });
    sys_->registerCommandObserver([&ui](const ActuatorCommand& cmd) { ui.onCommand(cmd); });

    sys_->registerLatencyObserver([&ui](uint64_t id, float a, float b, float c) {
        UiViewer::LatencySample ls{};
        ls.frame_id = id;
        ls.cap_to_est_ms = a;
        ls.est_to_ctrl_ms = b;
        ls.ctrl_to_act_ms = c;
        ui.onLatency(ls);
    });
#endif

    std::cout << "\n=== Solar Stewart Tracker ===\n";
#if SOLAR_HAVE_OPENCV
    std::cout << "UI: left click increases threshold, right click decreases. ESC/q closes window.\n";
#endif
    std::cout << "Running event-driven. Press Ctrl+C to stop.\n\n";

#ifdef __linux__
#if SOLAR_HAVE_OPENCV
    LinuxEventLoop loop(log_, *sys_, cfg_, &ui);
#else
    LinuxEventLoop loop(log_, *sys_, cfg_);
#endif
    const int rc = loop.run();
#else
#if SOLAR_HAVE_OPENCV
    // Non-Linux: just tick UI
    while (ui.tick()) {
        sys_->setTrackerThreshold(static_cast<std::uint8_t>(ui.threshold()));
    }
    const int rc = 0;
#else
    std::cout << "Non-Linux build: press Enter to stop...\n";
    std::cin.get();
    const int rc = 0;
#endif
#endif

    sys_->stop();
    std::cout << "Shutdown complete.\n";
    return rc;
}

}} // namespace solar::app
