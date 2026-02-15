#pragma once

#include <functional>

#include "common/Logger.hpp"
#include "common/Types.hpp"

namespace solar {

/**
 * Controller
 *
 * Converts SunEstimate (centroid error) into a PlatformSetpoint (tilt/pan).
 *
 * Input:
 * - SunEstimate: (cx, cy, confidence)
 *
 * Output:
 * - PlatformSetpoint: (tilt_rad, pan_rad)
 *
 * Notes:
 * - Pure logic (no threads inside)
 * - Called event-driven when a new SunEstimate arrives
 */
class Controller {
public:
    using SetpointCallback = std::function<void(const PlatformSetpoint&)>;

    struct Config {
        // Camera image size for normalization (must match camera config)
        int width{640};
        int height{480};

        // Deadband in normalized coordinates (0..1)
        float deadband{0.02f};

        // Proportional gain from normalized error to radians
        float k_pan{0.8f};
        float k_tilt{0.8f};

        // Output limits (radians)
        float max_pan_rad{0.6f};
        float max_tilt_rad{0.6f};

        // Minimum confidence to act; below this output goes to 0 (safe)
        float min_confidence{0.2f};
    };

    Controller(Logger& log, Config cfg);

    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;

    void registerSetpointCallback(SetpointCallback cb);

    // Process one estimate (event-driven call)
    void onEstimate(const SunEstimate& est);

    Config config() const;

private:
    Logger& log_;
    Config cfg_;
    SetpointCallback setpointCb_{};
};

} // namespace solar