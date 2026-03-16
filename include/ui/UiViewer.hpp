#pragma once

#if SOLAR_HAVE_OPENCV

#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "common/Logger.hpp"
#include "common/Types.hpp"

namespace solar {

/**
 * @brief OpenCV-based UI viewer for live debugging and visualization.
 *
 * Displays the latest camera frame with overlays (centroid, confidence, setpoint)
 * and live plots (centroid history, actuator targets, latency).
 *
 * Threading model:
 * - Data is fed via callback-style methods (onFrame/onEstimate/...)
 * - tick() renders one UI iteration and handles user input
 * - Internal state is protected by a mutex where required
 *
 * Only compiled when SOLAR_HAVE_OPENCV is enabled.
 */
class UiViewer {
public:
    /// @brief UI configuration parameters.
    struct Config {
        /// @brief Window title.
        std::string window_name{"Solar Tracker"};

        /// @brief Expected frame width (pixels).
        int width{640};

        /// @brief Expected frame height (pixels).
        int height{480};

        /// @brief Initial threshold value controlled via UI.
        int threshold{200};

        /// @brief Step size for threshold adjustments.
        int thr_step{10};

        /// @brief Height of the plot panel (pixels).
        int plot_height{140};

        /// @brief Number of samples stored in history buffers.
        int plot_history{240};

        /// @brief Horizontal pixels per sample in plots (newest at right).
        int plot_stride_px{3};

        /// @brief If true, show tilt/pan setpoint overlay.
        bool show_setpoint_overlay{true};

        /// @brief If true, show latency overlay.
        bool show_latency_overlay{true};
    };

    /// @brief Latency sample (milliseconds) used for live plotting.
    struct LatencySample {
        /// @brief Associated frame identifier.
        uint64_t frame_id{0};

        /// @brief Capture → estimate latency (ms).
        float cap_to_est_ms{0.0f};

        /// @brief Estimate → control latency (ms).
        float est_to_ctrl_ms{0.0f};

        /// @brief Control → actuation latency (ms).
        float ctrl_to_act_ms{0.0f};
    };

    /// @brief Construct UI viewer with logger and configuration.
    UiViewer(Logger& log, Config cfg);

    // ------------------------------------------------------------------
    // Data feed (thread-safe)
    // ------------------------------------------------------------------

    /// @brief Update the latest frame.
    void onFrame(const FrameEvent& fe);

    /// @brief Update the latest vision estimate.
    void onEstimate(const SunEstimate& est);

    /// @brief Update the latest platform setpoint (used for overlay).
    void onSetpoint(const PlatformSetpoint& sp);

    /// @brief Update the latest actuator command (used for plotting).
    void onCommand(const ActuatorCommand& cmd);

    /// @brief Update the latest latency sample (used for plotting).
    void onLatency(const LatencySample& ls);

    /**
     * @brief Run one UI tick: render, process input, and update plots.
     * @return false if the user requested quit (ESC or 'q'), true otherwise.
     */
    bool tick();

    /// @brief Get current UI threshold.
    int threshold() const noexcept;

private:
    /// @brief Static mouse callback trampoline for OpenCV.
    /// @details OpenCV C-style callback API requires a void* userdata pointer.
    ///          We pass `this` and immediately cast back to UiViewer* inside the callback.
    ///          The pointer is non-owning and never stored beyond the callback.
    static void onMouse_(int event, int x, int y, int flags, void* userdata);

    /// @brief Handle mouse events (implementation-specific).
    void handleMouse_(int event);

    /// @brief Push actuator sample values to history buffers.
    void pushSample_(float v0, float v1, float v2);

    /// @brief Push centroid values to history buffers.
    void pushCentroid_(float cx, float cy);

    /// @brief Push latency values to history buffers.
    void pushLatency_(float a, float b, float c);

    /// @brief Draw basic overlay elements (centroid/confidence/threshold).
    void drawOverlay_(cv::Mat& top, float conf, int thr);

    /// @brief Draw setpoint overlay information.
    void drawSetpointOverlay_(cv::Mat& top);

    /// @brief Draw plot panels (centroid/actuator/latency history).
    void drawPlots_(cv::Mat& vis);

private:
    Logger& log_;
    Config cfg_;

    mutable std::mutex m_;

    /// @brief Latest grayscale frame (CV_8UC1).
    cv::Mat gray_;

    SunEstimate lastEst_{};
    PlatformSetpoint lastSp_{};
    ActuatorCommand lastCmd_{};
    LatencySample lastLat_{};

    /// @brief UI-controlled threshold (atomic for cross-thread reads).
    std::atomic<int> threshold_{200};

    /// @brief Set when user requests quit.
    bool quit_{false};

    // ------------------------------------------------------------------
    // History buffers (ring)
    // ------------------------------------------------------------------

    std::vector<float> hist_cx_;
    std::vector<float> hist_cy_;
    std::array<std::vector<float>, 3> hist_u_;
    std::array<std::vector<float>, 3> hist_lat_;

    /// @brief Next write index into ring buffers.
    int hist_head_{0};

    /// @brief Number of valid samples currently stored.
    int hist_count_{0};
};

} // namespace solar

#endif