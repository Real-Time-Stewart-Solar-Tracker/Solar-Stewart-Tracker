#pragma once

/**
 * @file IIMU.hpp
 * @brief Event-driven IMU publisher interface.
 *
 * This interface is intentionally small:
 * - register one callback target
 * - start acquisition
 * - stop acquisition
 *
 * The IMU publishes samples in physical units through a callback.
 * It does not expose a polling getter API for the runtime pipeline.
 */

#include <cstdint>

namespace solar {

/**
 * @brief One IMU sample in physical units.
 *
 * Units:
 * - acceleration in m/s^2
 * - angular rate in rad/s
 */
struct IImuSample {
    float ax_mps2{0.0F};   ///< Acceleration X.
    float ay_mps2{0.0F};   ///< Acceleration Y.
    float az_mps2{0.0F};   ///< Acceleration Z.

    float gx_rps{0.0F};    ///< Angular rate X.
    float gy_rps{0.0F};    ///< Angular rate Y.
    float gz_rps{0.0F};    ///< Angular rate Z.

    std::uint64_t timestamp_us{0U}; ///< Monotonic timestamp in microseconds.
    bool valid{false};              ///< Validity flag.
};

/**
 * @brief Abstract IMU publisher interface.
 */
class IIMU {
public:
    /**
     * @brief Callback sink for published IMU samples.
     */
    struct CallbackInterface {
        /**
         * @brief Receive one IMU sample.
         *
         * @param sample IMU sample.
         */
        virtual void hasSample(const IImuSample& sample) = 0;

        /**
         * @brief Virtual destructor.
         */
        virtual ~CallbackInterface() = default;
    };

    /**
     * @brief Virtual destructor.
     */
    virtual ~IIMU() = default;

    /**
     * @brief Register the output callback sink.
     *
     * @param cb Callback sink.
     */
    virtual void registerEventCallback(CallbackInterface* cb) = 0;

    /**
     * @brief Start acquisition.
     *
     * @return True on success.
     */
    virtual bool start() = 0;

    /**
     * @brief Stop acquisition.
     */
    virtual void stop() = 0;
};

} // namespace solar