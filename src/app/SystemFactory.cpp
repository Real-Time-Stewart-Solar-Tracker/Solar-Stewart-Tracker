#include "app/SystemFactory.hpp"

#include "common/Logger.hpp"
#include "common/Types.hpp"
#include "sensors/ICamera.hpp"
#include "sensors/SimulatedPublisher.hpp"
#include "system/SystemManager.hpp"

#if SOLAR_HAVE_LIBCAMERA
#include "external/libcamera2opencv/libcam2opencv.h"
#include <opencv2/imgproc.hpp>
#endif

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <utility>

namespace solar::app {
namespace {

#if SOLAR_HAVE_LIBCAMERA
void copyMatIntoFrameEvent(const cv::Mat& src,
                           FrameEvent& dst,
                           const PixelFormat fmt) {
    dst.width = src.cols;
    dst.height = src.rows;
    dst.format = fmt;
    dst.stride_bytes = static_cast<int>(src.step[0]);

    const std::size_t total_bytes =
        static_cast<std::size_t>(dst.stride_bytes) *
        static_cast<std::size_t>(dst.height);

    dst.data.resize(total_bytes);

    if (src.isContinuous()) {
        std::memcpy(dst.data.data(), src.data, total_bytes);
        return;
    }

    for (int r = 0; r < dst.height; ++r) {
        const auto* row_src = src.ptr<std::uint8_t>(r);
        auto* row_dst = dst.data.data() +
                        static_cast<std::size_t>(r) * static_cast<std::size_t>(dst.stride_bytes);
        std::memcpy(row_dst, row_src, static_cast<std::size_t>(dst.stride_bytes));
    }
}

class Libcamera2OpenCvCameraAdapter final
    : public solar::ICamera {
public:
    Libcamera2OpenCvCameraAdapter(Logger& log, const LibcameraConfig& cfg)
        : log_(log) {
        settings_.width = static_cast<unsigned int>(cfg.width);
        settings_.height = static_cast<unsigned int>(cfg.height);
        settings_.framerate = static_cast<unsigned int>(cfg.fps);
    }

    void registerFrameCallback(FrameCallback cb) override {
        frame_cb_ = std::move(cb);
    }

    bool start() override {
        if (running_) {
            return true;
        }

        if (!frame_cb_) {
            log_.error("Libcamera2OpenCvCameraAdapter: frame callback not registered");
            return false;
        }

        try {
            camera_.registerCallback([this](const cv::Mat& frame,
                                            const libcamera::ControlList& controls) {
                onFrame_(frame, controls);
            });
            camera_.start(settings_);
            running_ = true;
            log_.info("SystemFactory: using libcamera2opencv camera backend");
            return true;
        } catch (...) {
            log_.error("Libcamera2OpenCvCameraAdapter: failed to start libcamera2opencv");
            running_ = false;
            return false;
        }
    }

    void stop() override {
        if (!running_) {
            return;
        }
        camera_.stop();
        running_ = false;
    }

    bool isRunning() const noexcept override {
        return running_;
    }

private:
    /**
     * @brief Detect whether the received Mat has a stride mismatch.
     *
     * The FormatConverter NATIVE path creates a cv::Mat without accounting
     * for ISP stride padding. On Raspberry Pi 5, RGB888 buffers often use
     * a stride of width*4 bytes despite only 3 bytes per pixel being valid.
     * This manifests as a tripled/interlaced image.
     *
     * Detection: if the image is tripled, row 0 and row rows/3 in the
     * corrupted Mat will contain near-identical pixel data because the
     * stride offset wraps by exactly one Mat row width every 3 rows.
     *
     * @return The detected actual stride in bytes, or 0 if no mismatch.
     */
    std::size_t detectStrideMismatch_(const cv::Mat& frame) const {
        if (frame.channels() != 3 || frame.rows < 6 || frame.cols < 4) {
            return 0;
        }

        const std::size_t nominal_step = static_cast<std::size_t>(frame.cols) * 3;
        if (frame.step[0] != nominal_step) {
            return 0;
        }

        const int third = frame.rows / 3;
        int matches = 0;
        const int samples = std::min(frame.cols, 32);

        for (int x = 0; x < samples; ++x) {
            const auto* row_a = frame.ptr<std::uint8_t>(0) + x * 3;
            const auto* row_b = frame.ptr<std::uint8_t>(third) + x * 3;

            if (std::abs(static_cast<int>(row_a[0]) - static_cast<int>(row_b[0])) < 4 &&
                std::abs(static_cast<int>(row_a[1]) - static_cast<int>(row_b[1])) < 4 &&
                std::abs(static_cast<int>(row_a[2]) - static_cast<int>(row_b[2])) < 4) {
                ++matches;
            }
        }

        if (matches > samples * 3 / 4) {
            return static_cast<std::size_t>(frame.cols) * 4;
        }

        return 0;
    }

    void onFrame_(const cv::Mat& frame, const libcamera::ControlList&) {
        if (!frame_cb_ || frame.empty()) {
            return;
        }

        FrameEvent fe{};
        fe.frame_id = next_frame_id_++;
        fe.t_capture = Clock::now();

        if (frame.channels() == 1) {
            copyMatIntoFrameEvent(frame, fe, PixelFormat::Gray8);
            frame_cb_(fe);
            return;
        }

        // On the first frame, detect whether the FormatConverter NATIVE
        // path delivered a Mat with incorrect stride.
        if (!stride_checked_) {
            stride_checked_ = true;
            detected_stride_ = detectStrideMismatch_(frame);
            if (detected_stride_ > 0) {
                std::fprintf(stderr,
                    "Libcamera2OpenCvCameraAdapter: detected ISP stride "
                    "mismatch (nominal=%zu, actual=%zu); compensating\n",
                    static_cast<std::size_t>(frame.step[0]),
                    detected_stride_);
            }
        }

        if (detected_stride_ > 0) {
            cv::Mat fixed(frame.rows, frame.cols, frame.type(),
                          const_cast<std::uint8_t*>(frame.data),
                          detected_stride_);
            copyMatIntoFrameEvent(fixed, fe, PixelFormat::BGR888);
        } else {
            copyMatIntoFrameEvent(frame, fe, PixelFormat::BGR888);
        }

        frame_cb_(fe);
    }

private:
    Logger& log_;
    Libcam2OpenCVSettings settings_{};
    Libcam2OpenCV camera_{};
    FrameCallback frame_cb_{};
    bool running_{false};
    std::uint64_t next_frame_id_{1};

    bool stride_checked_{false};
    std::size_t detected_stride_{0};
};
#endif

} // namespace

std::unique_ptr<solar::ICamera> SystemFactory::makeCamera_(Logger& log, const AppConfig& cfg) {
    switch (cfg.camera_backend) {
        case CameraBackend::Simulated: {
            solar::SimulatedPublisher::Config c{};
            c.width = cfg.simulated_camera.width;
            c.height = cfg.simulated_camera.height;
            c.fps = cfg.simulated_camera.fps;
            c.moving_spot = cfg.simulated_camera.moving_spot;
            c.noise_std = cfg.simulated_camera.noise_std;
            c.background = cfg.simulated_camera.background;
            c.spot_value = cfg.simulated_camera.spot_value;
            c.spot_radius = cfg.simulated_camera.spot_radius;

            log.info("SystemFactory: using SimulatedPublisher");
            return std::make_unique<solar::SimulatedPublisher>(log, c);
        }

        case CameraBackend::Libcamera: {
#if SOLAR_HAVE_LIBCAMERA
            return std::make_unique<Libcamera2OpenCvCameraAdapter>(log, cfg.libcamera);
#else
            log.error("SystemFactory: Libcamera requested but SOLAR_HAVE_LIBCAMERA is OFF");
            return nullptr;
#endif
        }
    }

    log.error("SystemFactory: unknown camera backend");
    return nullptr;
}

std::unique_ptr<solar::SystemManager> SystemFactory::makeSystem(Logger& log, const AppConfig& cfg) {
    auto camera = makeCamera_(log, cfg);
    if (!camera) {
        log.error("SystemFactory: failed to construct camera backend");
        return nullptr;
    }

    return std::make_unique<solar::SystemManager>(log, std::move(camera), cfg);
}

} // namespace solar::app