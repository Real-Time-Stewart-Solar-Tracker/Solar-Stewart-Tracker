#include "app/AppConfig.hpp"
#include <array>

namespace {
constexpr std::array<float, 3> kRigServoParkDeg = {41.f, 41.f, 41.f};
// Measured park angles for the current physical rig after servo-horn alignment.
// These are hardware calibration values, not the kinematic neutral angles.
}

namespace solar::app {

AppConfig defaultConfig() {
    AppConfig cfg{};

    // -----------------------------
    // Vision
    // -----------------------------
    cfg.tracker.threshold        = 200;
    cfg.tracker.min_pixels       = 10;
    cfg.tracker.confidence_scale = 1.0f;

    // -----------------------------
    // Controller
    // -----------------------------
    cfg.controller.width          = 640;
    cfg.controller.height         = 480;
    cfg.controller.min_confidence = 0.4f;
    cfg.controller.deadband       = 0.0f;
    cfg.controller.k_pan          = 0.8f;
    cfg.controller.k_tilt         = 0.8f;
    cfg.controller.max_pan_rad    = 0.35f;
    cfg.controller.max_tilt_rad   = 0.35f;

    // -----------------------------
    // Kinematics (meters)
    // -----------------------------
    cfg.kinematics.base_radius_m     = 0.20f;
    cfg.kinematics.platform_radius_m = 0.12f;
    cfg.kinematics.home_height_m     = 0.18f;
    cfg.kinematics.horn_length_m     = 0.10f;
    cfg.kinematics.rod_length_m      = 0.18f;

    cfg.kinematics.base_theta_deg = {120.f, 240.f, 0.f};
    cfg.kinematics.plat_theta_deg = {120.f, 240.f, 0.f};

    cfg.kinematics.servo_neutral_deg = {90.f, 90.f, 90.f};
    cfg.kinematics.servo_dir         = {-1, -1, -1};

    // -----------------------------
    // Actuator limits
    // -----------------------------
    cfg.actuator.min_out  = {0.f, 0.f, 0.f};
    cfg.actuator.max_out  = {180.f, 180.f, 180.f};
    cfg.actuator.max_step = {999.f, 999.f, 999.f};

    // -----------------------------
    // Servo driver mapping
    // -----------------------------
    cfg.servo.log_every_n = 10;

#if SOLAR_HAVE_LIBCAMERA
    cfg.servo.startup_policy = solar::ServoDriver::StartupPolicy::RequireHardware;
#else
    cfg.servo.startup_policy = solar::ServoDriver::StartupPolicy::LogOnly;
#endif

    cfg.servo.ch[0].channel = 2; // front
    cfg.servo.ch[1].channel = 0; // left
    cfg.servo.ch[2].channel = 1; // right

    cfg.servo.ch[0].min_pulse_us = 500.f;  cfg.servo.ch[0].max_pulse_us = 2500.f;
    cfg.servo.ch[1].min_pulse_us = 500.f;  cfg.servo.ch[1].max_pulse_us = 2500.f;
    cfg.servo.ch[2].min_pulse_us = 500.f;  cfg.servo.ch[2].max_pulse_us = 2500.f;

    cfg.servo.ch[0].min_deg = 0.f;  cfg.servo.ch[0].max_deg = 180.f;
    cfg.servo.ch[1].min_deg = 0.f;  cfg.servo.ch[1].max_deg = 180.f;
    cfg.servo.ch[2].min_deg = 0.f;  cfg.servo.ch[2].max_deg = 180.f;

    // Calibrated neutral / park angle validated on the current rig.
    cfg.servo.ch[0].neutral_deg = kRigServoParkDeg[0];
    cfg.servo.ch[1].neutral_deg = kRigServoParkDeg[1];
    cfg.servo.ch[2].neutral_deg = kRigServoParkDeg[2];

    cfg.servo.ch[0].invert = false;
    cfg.servo.ch[1].invert = false;
    cfg.servo.ch[2].invert = false;

    cfg.servo.pca9685_addr  = 0x40;
    cfg.servo.pwm_hz        = 50.0f;
    cfg.servo.park_on_start = true;
    cfg.servo.park_on_stop  = true;

    // -----------------------------
    // Camera defaults
    // -----------------------------
#if SOLAR_HAVE_LIBCAMERA
    cfg.camera.width  = cfg.controller.width;
    cfg.camera.height = cfg.controller.height;
    cfg.camera.fps    = 30;
#else
    cfg.camera.width       = cfg.controller.width;
    cfg.camera.height      = cfg.controller.height;
    cfg.camera.fps         = 30;
    cfg.camera.moving_spot = true;
#endif

    cfg.tick_hz = 30;
    return cfg;
}

} // namespace solar::app