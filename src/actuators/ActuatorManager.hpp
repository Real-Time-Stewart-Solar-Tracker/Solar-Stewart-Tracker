#pragma once

/**
 * @file ActuatorManager.hpp
 * @brief Safety conditioning stage for actuator commands.
 *
 * This class receives actuator commands from kinematics and applies:
 * - per-channel output clamping
 * - per-channel slew-rate limiting
 *
 * It then emits a safe command via callback.
 *
 * It does NOT:
 * - own hardware
 * - own worker threads
 * - perform waiting or timing
 */

#include "common/Logger.hpp"
#include "common/Types.hpp"

#include <array>
#include <functional>
#include <mutex>

namespace solar {

/**
 * @brief Safety conditioning for actuator commands.
 */
class ActuatorManager {
public:
    /**
     * @brief Callback for safe actuator commands.
     */
    using SafeCommandCallback = std::function<void(const ActuatorCommand&)>;

    /**
     * @brief Runtime safety configuration.
     */
    struct Config {
        std::array<float, 3> min_out{0.0F, 0.0F, 0.0F};   ///< Minimum per-channel output.
        std::array<float, 3> max_out{180.0F, 180.0F, 180.0F}; ///< Maximum per-channel output.
        std::array<float, 3> max_step{8.0F, 8.0F, 8.0F};  ///< Maximum per-update change per channel.
    };

    /**
     * @brief Construct the actuator manager.
     *
     * @param log Shared logger.
     * @param cfg Safety configuration.
     */
    ActuatorManager(Logger& log, Config cfg);

    ActuatorManager(const ActuatorManager&) = delete;
    ActuatorManager& operator=(const ActuatorManager&) = delete;

    /**
     * @brief Register safe-command callback.
     *
     * @param cb Callback receiving safe actuator commands.
     */
    void registerSafeCommandCallback(SafeCommandCallback cb);

    /**
     * @brief Process one actuator command.
     *
     * @param cmd Input actuator command.
     */
    void onCommand(const ActuatorCommand& cmd);

    /**
     * @brief Return the current configuration.
     *
     * @return Current safety configuration.
     */
    Config config() const;

    /**
     * @brief Reset internal slew-history state.
     *
     * After reset, the next command is clamped but not history-limited relative
     * to a previous command.
     */
    void resetHistory();

private:
    float clamp_(float v, float lo, float hi) const;
    float limitStep_(float target, float prev, float max_step) const;

private:
    Logger& log_;
    Config cfg_{};

    mutable std::mutex cfg_mtx_;
    mutable std::mutex cb_mtx_;
    mutable std::mutex hist_mtx_;

    SafeCommandCallback cb_{};

    bool have_last_{false};
    std::array<float, 3> last_targets_{0.0F, 0.0F, 0.0F};
};

} // namespace solar