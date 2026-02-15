#pragma once

#include <array>
#include <functional>

#include "common/Logger.hpp"
#include "common/Types.hpp"

namespace solar {

class ActuatorManager {
public:
    using SafeCommandCallback = std::function<void(const ActuatorCommand&)>;

    struct Config {
        std::array<float, 3> min_out{-1.0f, -1.0f, -1.0f};
        std::array<float, 3> max_out{ 1.0f,  1.0f,  1.0f};
        std::array<float, 3> max_step{0.02f, 0.02f, 0.02f};
    };

    ActuatorManager(Logger& log, Config cfg);

    ActuatorManager(const ActuatorManager&) = delete;
    ActuatorManager& operator=(const ActuatorManager&) = delete;

    void registerSafeCommandCallback(SafeCommandCallback cb);
    void onCommand(const ActuatorCommand& cmd);
    Config config() const;

private:
    Logger& log_;
    Config cfg_;
    SafeCommandCallback safeCb_{};

    std::array<float, 3> lastOut_{0.0f, 0.0f, 0.0f};
    bool hasLast_{false};
};

} // namespace solar