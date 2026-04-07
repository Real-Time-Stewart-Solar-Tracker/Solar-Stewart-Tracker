/**
 * @file main.cpp
 * @brief Pipeline demo — runs the full automatic tracking pipeline without hardware.
 *
 * This demo wires together the same stages used in production:
 *
 *   SimulatedPublisher → SunTracker → Controller → Kinematics3RRS
 *
 * No camera, no servos, no I²C bus required.
 * The SimulatedPublisher generates synthetic frames with a moving bright spot
 * that orbits the centre of the image. Each stage processes events and prints
 * its output so you can see the pipeline in action.
 *
 * The demo runs until interrupted by SIGINT (Ctrl+C) or SIGTERM. Shutdown is
 * handled via signalfd so the process always completes cleanly.
 *
 * Expected output (one line per frame at 30 fps):
 *
 *   [frame   1] sun=(320.0, 240.0) conf=1.00 | tilt= 0.000 pan= 0.000 | ...
 *   [frame   2] sun=(322.3, 240.0) conf=1.00 | tilt=-0.003 pan= 0.003 | ...
 *   ...
 *   ^C
 *   Demo stopped after 152 frames.
 *
 * Build with CMake (see README.md in this directory).
 */

#include "common/Logger.hpp"
#include "common/Types.hpp"
#include "control/Controller.hpp"
#include "control/Kinematics3RRS.hpp"
#include "sensors/SimulatedPublisher.hpp"
#include "vision/SunTracker.hpp"

#include <atomic>
#include <cstdio>

#include <signal.h>
#include <sys/signalfd.h>
#include <unistd.h>

int main() {
    using namespace solar;

    // -----------------------------------------------------------------------
    // 1. Logger
    // -----------------------------------------------------------------------
    Logger log;

    // -----------------------------------------------------------------------
    // 2. SimulatedPublisher
    //    Generates 640x480 frames at 30 fps with a moving bright spot.
    // -----------------------------------------------------------------------
    SimulatedPublisher::Config camCfg{};
    camCfg.width        = 640;
    camCfg.height       = 480;
    camCfg.fps          = 30;
    camCfg.moving_spot  = true;
    camCfg.noise_std    = 3.0F;
    camCfg.background   = 20U;
    camCfg.spot_value   = 240U;
    camCfg.spot_radius  = 15;
    SimulatedPublisher camera(log, camCfg);

    // -----------------------------------------------------------------------
    // 3. SunTracker
    // -----------------------------------------------------------------------
    SunTracker::Config trackerCfg{};
    trackerCfg.threshold        = 200U;
    trackerCfg.min_pixels       = 10U;
    trackerCfg.confidence_scale = 1.0F;
    SunTracker tracker(log, trackerCfg);

    // -----------------------------------------------------------------------
    // 4. Controller
    // -----------------------------------------------------------------------
    Controller::Config ctrlCfg{};
    ctrlCfg.image_width    = camCfg.width;
    ctrlCfg.image_height   = camCfg.height;
    ctrlCfg.k_pan          = 0.8F;
    ctrlCfg.k_tilt         = 0.8F;
    ctrlCfg.max_pan_rad    = 0.35F;
    ctrlCfg.max_tilt_rad   = 0.35F;
    ctrlCfg.deadband       = 0.0F;
    ctrlCfg.min_confidence = 0.4F;
    Controller controller(log, ctrlCfg);

    // -----------------------------------------------------------------------
    // 5. Kinematics3RRS
    // -----------------------------------------------------------------------
    Kinematics3RRS::Config kinCfg{};
    kinCfg.base_radius_m     = 0.20F;
    kinCfg.platform_radius_m = 0.12F;
    kinCfg.home_height_m     = 0.18F;
    kinCfg.horn_length_m     = 0.10F;
    kinCfg.rod_length_m      = 0.18F;
    kinCfg.base_theta_deg    = {120.0F, 240.0F, 0.0F};
    kinCfg.plat_theta_deg    = {120.0F, 240.0F, 0.0F};
    kinCfg.servo_neutral_deg = {90.0F, 90.0F, 90.0F};
    kinCfg.servo_dir         = {-1, -1, -1};
    Kinematics3RRS kinematics(log, kinCfg);

    // -----------------------------------------------------------------------
    // 6. Wire the pipeline with callbacks.
    //
    //    camera --> tracker --> controller --> kinematics --> print
    // -----------------------------------------------------------------------
    std::atomic<std::uint64_t> frameCount{0};

    kinematics.registerCommandCallback([&](const ActuatorCommand& cmd) {
        const auto& t = cmd.actuator_targets;
        std::printf("  servos=[%5.1f, %5.1f, %5.1f]  status=%s\n",
                    t[0], t[1], t[2],
                    cmd.status == CommandStatus::Ok ? "OK" : "DEGRADED");
    });

    controller.registerSetpointCallback([&](const PlatformSetpoint& sp) {
        std::printf("  tilt=%+6.3f rad  pan=%+6.3f rad  ->",
                    sp.tilt_rad, sp.pan_rad);
        kinematics.onSetpoint(sp);
    });

    tracker.registerEstimateCallback([&](const SunEstimate& est) {
        std::printf("[frame %4llu] sun=(%6.1f, %6.1f) conf=%.2f  ->",
                    static_cast<unsigned long long>(est.frame_id),
                    est.cx, est.cy, est.confidence);
        controller.onEstimate(est);
    });

    camera.registerFrameCallback([&](const FrameEvent& frame) {
        frameCount.fetch_add(1, std::memory_order_relaxed);
        tracker.onFrame(frame);
    });

    // -----------------------------------------------------------------------
    // 7. Establish a signalfd shutdown path.
    //    The process blocks in read() below until SIGINT or SIGTERM arrives,
    //    then stops the camera and exits cleanly. No sleep or busy-wait is used.
    // -----------------------------------------------------------------------
    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGINT);
    sigaddset(&mask, SIGQUIT);
    sigaddset(&mask, SIGHUP);
    sigaddset(&mask, SIGTERM);
    sigprocmask(SIG_BLOCK, &mask, nullptr);

    const int sfd = signalfd(-1, &mask, SFD_CLOEXEC);
    if (sfd < 0) {
        std::puts("ERROR: signalfd failed.");
        return 1;
    }

    // -----------------------------------------------------------------------
    // 8. Start and run until a termination signal is received.
    // -----------------------------------------------------------------------
    std::puts("=== Solar Stewart Tracker - Pipeline Demo ===");
    std::puts("Press Ctrl+C to stop.\n");

    if (!camera.start()) {
        std::puts("ERROR: failed to start simulated camera.");
        ::close(sfd);
        return 1;
    }

    // Block here until SIGINT, SIGQUIT, SIGHUP, or SIGTERM.
    signalfd_siginfo fdsi{};
    const ssize_t n = ::read(sfd, &fdsi, sizeof(fdsi));
    (void)n;

    camera.stop();
    ::close(sfd);

    std::puts("\n=== Demo stopped ===");
    std::printf("Total frames processed: %llu\n",
                static_cast<unsigned long long>(frameCount.load()));

    return 0;
}
