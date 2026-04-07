#pragma once

/**
 * @file ImuFeedbackMapper.hpp
 * @brief Pure mapping logic for IMU-based tilt correction.
 */

namespace solar::control {

/**
 * @brief Maps a measured IMU tilt into a bounded control correction.
 *
 * This class is intentionally pure logic. It does not own hardware, threads,
 * callbacks, or runtime state.
 */
class ImuFeedbackMapper {
public:
    /**
     * @brief Configuration for IMU feedback correction.
     */
    struct Config {
        float gain;               ///< Proportional gain applied to measured tilt.
        float deadband_rad;       ///< Symmetric deadband around zero tilt error.
        float max_correction_rad; ///< Maximum absolute correction magnitude.

        /**
         * @brief Construct the correction configuration.
         *
         * @param gain_ Proportional gain.
         * @param deadband_rad_ Deadband in radians.
         * @param max_correction_rad_ Maximum correction magnitude in radians.
         */
        Config(float gain_ = 0.12f,
               float deadband_rad_ = 0.01745329252f,
               float max_correction_rad_ = 0.01745329252f)
            : gain(gain_),
              deadband_rad(deadband_rad_),
              max_correction_rad(max_correction_rad_) {
        }
    };

    /**
     * @brief Construct the mapper.
     *
     * @param cfg Correction configuration.
     */
    explicit ImuFeedbackMapper(Config cfg = Config())
        : cfg_(cfg) {
    }

    /**
     * @brief Replace the current configuration.
     *
     * @param cfg New correction configuration.
     */
    void setConfig(Config cfg) { cfg_ = cfg; }

    /**
     * @brief Return the current configuration.
     *
     * @return Current correction configuration.
     */
    Config config() const { return cfg_; }

    /**
     * @brief Update only the proportional gain.
     *
     * @param gain New proportional gain.
     */
    void setGain(float gain) { cfg_.gain = gain; }

    /**
     * @brief Return the current proportional gain.
     *
     * @return Proportional gain.
     */
    float gain() const { return cfg_.gain; }

    /**
     * @brief Apply IMU correction to a commanded tilt.
     *
     * @param commanded_tilt_rad Input commanded tilt in radians.
     * @param measured_tilt_rad Measured tilt in radians.
     * @param valid True if the measured tilt is valid.
     * @return Corrected commanded tilt in radians.
     */
    float apply(float commanded_tilt_rad, float measured_tilt_rad, bool valid) const;

    /**
     * @brief Convert a measured tilt into a bounded correction term.
     *
     * @param tilt_rad Measured tilt in radians.
     * @return Correction term in radians.
     */
    float correctionFromTilt(float tilt_rad) const;

private:
    static float applyDeadband_(float value, float deadband);
    static float clampSymmetric_(float value, float limit);

    Config cfg_;
};

} // namespace solar::control
