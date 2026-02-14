#pragma once

#include <array>
#include <functional>

#include "common/Logger.hpp"
#include "common/Types.hpp"

namespace solar {

/**
 * ActuatorManager
 *
 * Safety and smoothing layer between kinematics and hardware driver.
 *
 * Responsibilities:
 * - Clamp commands to safe bounds
 * - Rate-limit actuator changes (prevents sudden jumps)
 * - Output the final safe command to ServoDriver (via callback)
 */
class ActuatorManager {
public:
    using SafeCommandCallback = std::function<void(const ActuatorCommand&)>;

    struct Config {
        std::array<float, 3> min_out{-1.0f, -1.0f, -1.0f};
        std::array<float, 3> max_out{ 1.0f,  1.0f,  1.0f};

        // Maximum change per update (rate limit)
        std::array<float, 3> max_step{0.02f, 0.02f, 0.02f};
    };

    ActuatorManager(Logger& log, Config cfg);

    ActuatorManager(const ActuatorManager&) = delete;
    ActuatorManager& operator=(const ActuatorManager&) = delete;

    void registerSafeCommandCallback(SafeCommandCallback cb);

    // Process a command (event-driven call)
    void onCommand(const ActuatorCommand& cmd);

    Config config() const;

private:
    Logger& log_;
    Config cfg_;
    SafeCommandCallback safeCb_{};

    // For rate limiting
    std::array<float, 3> lastOut_{0.0f, 0.0f, 0.0f};
    bool hasLast_{false};
};

} // namespace solar