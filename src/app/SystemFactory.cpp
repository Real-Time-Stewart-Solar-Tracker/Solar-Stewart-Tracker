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
    : public solar::ICamera
    , private Libcam2OpenCV::Callback {
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
            camera_.registerCallback(this);
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
    void hasFrame(const cv::Mat& frame, const libcamera::ControlList&) override {
        if (!frame_cb_ || frame.empty()) {
            return;
        }

        FrameEvent fe{};
        fe.frame_id = next_frame_id_++;
        fe.t_capture = Clock::now();

        if (frame.type() == CV_8UC1) {
            copyMatIntoFrameEvent(frame, fe, PixelFormat::Gray8);
            frame_cb_(fe);
            return;
        }

        cv::Mat converted;
        PixelFormat out_fmt = PixelFormat::Gray8;

        if (frame.type() == CV_8UC3) {
            converted = frame;
            out_fmt = PixelFormat::BGR888;
        } else if (frame.type() == CV_8UC4) {
            cv::cvtColor(frame, converted, cv::COLOR_BGRA2BGR);
            out_fmt = PixelFormat::BGR888;
        } else {
            cv::cvtColor(frame, converted, cv::COLOR_BGR2GRAY);
            out_fmt = PixelFormat::Gray8;
        }

        copyMatIntoFrameEvent(converted, fe, out_fmt);
        frame_cb_(fe);
    }

private:
    Logger& log_;
    Libcam2OpenCVSettings settings_{};
    Libcam2OpenCV camera_{};
    FrameCallback frame_cb_{};
    bool running_{false};
    std::uint64_t next_frame_id_{1};
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