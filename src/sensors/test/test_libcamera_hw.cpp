#include "common/Logger.hpp"
#include "common/Types.hpp"

#if !SOLAR_HAVE_LIBCAMERA

int main() {
    return 77; // skipped
}

#else

#include "sensors/LibcameraPublisher.hpp"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>

int main() {
    const char* run_env = std::getenv("SOLAR_RUN_CAMERA_HW_TESTS");
    if (!run_env || std::string(run_env) != "1") {
        return 77; // skipped unless explicitly enabled
    }

    solar::Logger log;

    solar::LibcameraPublisher::Config cfg{};
    cfg.width = 640;
    cfg.height = 480;
    cfg.fps = 30;

    solar::LibcameraPublisher cam(log, cfg);

    std::atomic<int> frames{0};
    std::atomic<bool> contract_ok{true};

    cam.registerFrameCallback([&](const solar::FrameEvent& fe) {
        if (fe.width != cfg.width) contract_ok = false;
        if (fe.height != cfg.height) contract_ok = false;
        if (fe.format != solar::PixelFormat::Gray8) contract_ok = false;
        if (fe.stride_bytes != fe.width) contract_ok = false;
        if (fe.data.size() != static_cast<std::size_t>(fe.width * fe.height)) contract_ok = false;
        ++frames;
    });

    if (!cam.start()) {
        return 1;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline && frames.load() < 3) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    cam.stop();

    if (frames.load() < 3) return 1;
    if (!contract_ok.load()) return 1;

    return 0;
}

#endif