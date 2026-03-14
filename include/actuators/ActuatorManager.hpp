#pragma once

#include <array>
#include <functional>
#include <mutex>

#include "common/Logger.hpp"
#include "common/Types.hpp"

namespace solar {

/**
 * @brief Safety wrapper for actuator commands.
 *
 * Applies per-channel saturation and slew-rate limiting, then forwards the safe command
 * via a registered callback.
 */
class ActuatorManager {
public:
    /// @brief Callback delivering a safety-limited actuator command to downstream/hardware layers.
    using SafeCommandCallback = std::function<void(const ActuatorCommand&)>;

    /// @brief Safety limits for saturation and per-update step limiting.
    struct Config {
        /// @brief Minimum allowed output per channel.
        std::array<float, 3> min_out{-1.0f, -1.0f, -1.0f};
        /// @brief Maximum allowed output per channel.
        std::array<float, 3> max_out{ 1.0f,  1.0f,  1.0f};
        /// @brief Maximum allowed change per update step per channel.
        std::array<float, 3> max_step{0.02f, 0.02f, 0.02f};
    };

    /// @brief Construct with logger and configuration.
    ActuatorManager(Logger& log, Config cfg);

    ActuatorManager(const ActuatorManager&) = delete;
    ActuatorManager& operator=(const ActuatorManager&) = delete;

    /// @brief Register callback for forwarding the safe command.
    void registerSafeCommandCallback(SafeCommandCallback cb);

    /// @brief Process an incoming command (saturate + rate-limit) and forward it if a callback is set.
    void onCommand(const ActuatorCommand& cmd);

    /// @brief Get current configuration.
    Config config() const;

private:
    Logger& log_;
    Config cfg_;

    mutable std::mutex cbMtx_;
    SafeCommandCallback safeCb_{};

    mutable std::mutex mtx_;
    std::array<float, 3> lastOut_{0.0f, 0.0f, 0.0f};
    bool hasLast_{false};
};

} // namespace solar