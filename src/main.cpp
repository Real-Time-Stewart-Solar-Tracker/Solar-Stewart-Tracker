#include "common/Logger.hpp"
#include "system/SystemManager.hpp"
#include "sensors/SimulatedPublisher.hpp"

#if SOLAR_HAVE_LIBCAMERA
#include "sensors/LibcameraPublisher.hpp"
#endif

#if SOLAR_HAVE_OPENCV
#include "ui/UiViewer.hpp"
#endif

#include <iostream>
#include <memory>

#ifdef __linux__
#include <errno.h>
#include <poll.h>
#include <signal.h>
#include <string.h>
#include <sys/signalfd.h>
#include <sys/timerfd.h>
#include <unistd.h>
#endif

using namespace solar;

int main() {
    Logger log;

    // -----------------------------
    // Configs
    // -----------------------------
    SunTracker::Config trackerCfg{};
    trackerCfg.threshold = 200;
    trackerCfg.min_pixels = 10;
    trackerCfg.confidence_scale = 1.0f;

    Controller::Config controllerCfg{};
    controllerCfg.width = 640;
    controllerCfg.height = 480;
    controllerCfg.min_confidence = 0.4f;
    controllerCfg.deadband = 0.0f;
    controllerCfg.k_pan = 0.8f;
    controllerCfg.k_tilt = 0.8f;
    controllerCfg.max_pan_rad = 0.35f;
    controllerCfg.max_tilt_rad = 0.35f;

    Kinematics3RRS::Config kinCfg{};
    ActuatorManager::Config actCfg{};
    ServoDriver::Config drvCfg{};

    // -----------------------------
    // Camera selection (compile-time)
    // -----------------------------
    std::unique_ptr<ICamera> camera;

#if SOLAR_HAVE_LIBCAMERA
    log.info("Using LibcameraPublisher (Pi Camera)");
    LibcameraPublisher::Config camCfg{};
    camCfg.width = controllerCfg.width;
    camCfg.height = controllerCfg.height;
    camCfg.fps = 30;
    camera = std::make_unique<LibcameraPublisher>(log, camCfg);
#else
    log.info("Using SimulatedPublisher (Desktop)");
    SimulatedPublisher::Config camCfg{};
    camCfg.width = controllerCfg.width;
    camCfg.height = controllerCfg.height;
    camCfg.fps = 30;
    camCfg.moving_spot = true;
    camera = std::make_unique<SimulatedPublisher>(log, camCfg);
#endif

    // -----------------------------
    // System
    // -----------------------------
    SystemManager sys(log,
                      std::move(camera),
                      trackerCfg,
                      controllerCfg,
                      kinCfg,
                      actCfg,
                      drvCfg);

    if (!sys.start()) {
        log.error("Failed to start SystemManager");
        return 1;
    }

#if SOLAR_HAVE_OPENCV
    UiViewer::Config uiCfg{};
    uiCfg.width = controllerCfg.width;
    uiCfg.height = controllerCfg.height;
    uiCfg.threshold = trackerCfg.threshold;
    uiCfg.plot_height = 260;    // makes the plot readable
    uiCfg.plot_stride_px = 2;   // smoother lines
    uiCfg.plot_history = 600;   // more history on screen
    UiViewer ui(log, uiCfg);

    // Feed UI from the event-driven pipeline
    sys.registerFrameObserver(
        [&ui](const FrameEvent& fe) { ui.onFrame(fe); });

    sys.registerEstimateObserver(
        [&ui](const SunEstimate& est) { ui.onEstimate(est); });

    sys.registerSetpointObserver(
        [&ui](const PlatformSetpoint& sp) { ui.onSetpoint(sp); });

    sys.registerCommandObserver(
        [&ui](const ActuatorCommand& cmd) { ui.onCommand(cmd); });

    // 🔥 NEW — live latency plot
    sys.registerLatencyObserver(
        [&ui](uint64_t id, float a, float b, float c) {
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
    // ---------------------------------------------------------
    // ZERO-RISK EVENT LOOP (Linux/Pi):
    // - No sleeps.
    // - poll() blocks until kernel event occurs.
    // - timerfd drives UI ticks (and any periodic housekeeping).
    // ---------------------------------------------------------

    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGINT);
    sigaddset(&mask, SIGTERM);

    if (sigprocmask(SIG_BLOCK, &mask, nullptr) < 0) {
        log.error(std::string("sigprocmask failed: ") + strerror(errno));
        sys.stop();
        return 1;
    }

    const int sfd = signalfd(-1, &mask, SFD_CLOEXEC);
    if (sfd < 0) {
        log.error(std::string("signalfd failed: ") + strerror(errno));
        sys.stop();
        return 1;
    }

    // Tick rate for UI + housekeeping (30 Hz recommended)
    const int tfd = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC);
    if (tfd < 0) {
        log.error(std::string("timerfd_create failed: ") + strerror(errno));
        close(sfd);
        sys.stop();
        return 1;
    }

    itimerspec its{};
    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = 33333333; // ~30 Hz
    its.it_value = its.it_interval;

    if (timerfd_settime(tfd, 0, &its, nullptr) < 0) {
        log.error(std::string("timerfd_settime failed: ") + strerror(errno));
        close(tfd);
        close(sfd);
        sys.stop();
        return 1;
    }

    bool quit = false;

    while (!quit) {
        pollfd fds[2]{};
        fds[0].fd = sfd;
        fds[0].events = POLLIN;
        fds[1].fd = tfd;
        fds[1].events = POLLIN;

        const int r = poll(fds, 2, -1);
        if (r < 0) {
            if (errno == EINTR) continue;
            log.error(std::string("poll failed: ") + strerror(errno));
            break;
        }

        // Ctrl+C / SIGTERM
        if (fds[0].revents & POLLIN) {
            signalfd_siginfo si{};
            const ssize_t n = read(sfd, &si, sizeof(si));
            if (n == sizeof(si)) {
                if (si.ssi_signo == SIGINT || si.ssi_signo == SIGTERM) {
                    log.info("Stop requested via signal");
                    quit = true;
                }
            }
        }

        // Periodic tick
        if (fds[1].revents & POLLIN) {
            uint64_t expirations = 0;
            (void)read(tfd, &expirations, sizeof(expirations));

#if SOLAR_HAVE_OPENCV
            // UI tick pumps window events + draws plot
            if (!ui.tick()) {
                quit = true; // user closed window
            } else {
                // Apply UI threshold to tracker (mouse interaction -> meaningful output change)
                sys.setTrackerThreshold(static_cast<uint8_t>(ui.threshold()));
            }
#endif
        }
    }

    close(tfd);
    close(sfd);

#else
    // ---------------------------------------------------------
    // Non-Linux fallback:
    // Keep simple for laptop tests.
    // ---------------------------------------------------------
#if SOLAR_HAVE_OPENCV
    // Run UI until user quits (OpenCV handles event pump)
    while (ui.tick()) {
        sys.setTrackerThreshold(static_cast<uint8_t>(ui.threshold()));
    }
#else
    std::cout << "Non-Linux build: press Enter to stop...\n";
    std::cin.get();
#endif
#endif

    sys.stop(); // prints latency summary
    std::cout << "Shutdown complete.\n";
    return 0;
}
