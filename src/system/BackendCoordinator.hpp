#pragma once

/**
 * @file BackendCoordinator.hpp
 * @brief Optional hardware backend bring-up and teardown.
 *
 * This class encapsulates the lifecycle of the two optional hardware backends:
 * - ADS1115 manual potentiometer input
 * - MPU-6050/ICM-20600 IMU
 *
 * Extracting backend lifecycle from @c SystemManager gives each class a
 * single responsibility:
 * - @c BackendCoordinator — hardware backend bring-up, teardown, and I2C/GPIO
 *   resource ownership
 * - @c SystemManager — runtime pipeline orchestration, thread lifecycle, and
 *   state machine
 *
 * The coordinator is constructed with two callbacks: one for manual pot
 * samples and one for IMU samples. Both are wired by the caller (SystemManager)
 * so the coordinator itself has no knowledge of the pipeline stages.
 */

#include "app/AppConfig.hpp"
#include "common/Logger.hpp"
#include "common/ManualInputTypes.hpp"
#include "control/ImuTiltEstimator.hpp"
#include "sensors/imu/IIMU.hpp"

#include <functional>
#include <memory>

namespace solar {

class ADS1115ManualInput;
class LinuxI2CDevice;
class IIMU;
class Mpu6050Publisher;

namespace control {
struct ImuSample;
}

/**
 * @brief Optional hardware backend lifecycle manager.
 *
 * Owns the ADS1115 and MPU-6050/ICM-20600 hardware objects and their
 * associated I2C/GPIO resources. Delivers samples upstream via callbacks
 * registered at construction time.
 */
class BackendCoordinator {
public:
    /// Callback type for manual potentiometer samples.
    using ManualCallback = std::function<void(const ManualPotSample&)>;

    /// Callback type for IMU samples.
    using ImuCallback = std::function<void(const control::ImuSample&)>;

    /**
     * @brief Result of a start() attempt.
     */
    struct StartResult {
        bool manual_ok{true}; ///< True if manual backend started or was not requested.
        bool imu_ok{true};    ///< True if IMU backend started or was not requested.
    };

    /**
     * @brief Construct the coordinator.
     *
     * @param cfg         Full application configuration.
     * @param manual_cb   Callback for incoming potentiometer samples.
     * @param imu_cb      Callback for incoming IMU samples.
     */
    BackendCoordinator(const app::AppConfig& cfg,
                       ManualCallback manual_cb,
                       ImuCallback imu_cb);

    ~BackendCoordinator();

    BackendCoordinator(const BackendCoordinator&) = delete;
    BackendCoordinator& operator=(const BackendCoordinator&) = delete;

    /**
     * @brief Start all configured hardware backends.
     *
     * @param log Logger for startup messages.
     * @return StartResult indicating which backends started successfully.
     */
    StartResult start(Logger& log);

    /**
     * @brief Stop all running hardware backends and release resources.
     */
    void stop();

private:
    /**
     * @brief Adapts the IIMU::CallbackInterface to a plain std::function.
     *
     * This avoids exposing the IIMU callback interface outside of the
     * BackendCoordinator implementation.
     */
    class ImuCallbackForwarder final : public IIMU::CallbackInterface {
    public:
        explicit ImuCallbackForwarder(ImuCallback callback)
            : callback_(std::move(callback)) {
        }

        void hasSample(const IImuSample& sample) override;

    private:
        ImuCallback callback_{};
    };

    bool startManual_(Logger& log);
    void stopManual_();

    bool startImu_(Logger& log);
    void stopImu_();

    app::AppConfig config_{};
    ManualCallback manual_callback_{};
    ImuCallback    imu_callback_{};

    bool manual_backend_requested_{false};
    bool imu_backend_requested_{false};

    std::unique_ptr<ADS1115ManualInput>   manual_input_{};
    std::unique_ptr<LinuxI2CDevice>       imu_i2c_{};
    std::unique_ptr<IIMU>                 imu_input_{};
    std::unique_ptr<ImuCallbackForwarder> imu_callback_forwarder_{};
};

} // namespace solar
