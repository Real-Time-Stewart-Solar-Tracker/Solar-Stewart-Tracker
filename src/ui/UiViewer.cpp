#if SOLAR_HAVE_OPENCV

#include "ui/UiViewer.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

namespace solar {
namespace {

// -------------------- constants --------------------
constexpr float kPi = 3.14159265358979323846f;

// -------------------- small helpers --------------------
static inline int clampi(int v, int lo, int hi) {
    return std::max(lo, std::min(v, hi));
}

static inline std::string fmt(float v, int prec = 3) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", prec, v);
    return std::string(buf);
}

static inline std::string fmt(double v, int prec = 3) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", prec, v);
    return std::string(buf);
}

static inline void drawTextBox(cv::Mat& img,
                               const std::string& text,
                               cv::Point org,
                               int fontFace,
                               double fontScale,
                               const cv::Scalar& textColor,
                               int thickness,
                               const cv::Scalar& boxColor,
                               double boxAlpha = 0.55) {
    if (img.empty()) return;

    int base = 0;
    const cv::Size sz = cv::getTextSize(text, fontFace, fontScale, thickness, &base);

    const int padX = 10;
    const int padY = 7;

    cv::Rect r(org.x - padX,
               org.y - sz.height - padY,
               sz.width + 2 * padX,
               sz.height + base + 2 * padY);

    const cv::Rect imgRect(0, 0, img.cols, img.rows);
    r = r & imgRect;

    if (r.width <= 0 || r.height <= 0) {
        if (org.x >= 0 && org.x < img.cols && org.y >= 0 && org.y < img.rows) {
            cv::putText(img, text, org, fontFace, fontScale, textColor, thickness, cv::LINE_AA);
        }
        return;
    }

    cv::Mat roi = img(r);
    cv::Mat overlay;
    roi.copyTo(overlay);
    cv::rectangle(overlay, cv::Rect(0, 0, r.width, r.height), boxColor, cv::FILLED);
    cv::addWeighted(overlay, boxAlpha, roi, 1.0 - boxAlpha, 0.0, roi);

    cv::putText(img, text, org, fontFace, fontScale, textColor, thickness, cv::LINE_AA);
}

static bool frameReadableByUi(const FrameEvent& fe) {
    if (fe.width <= 0 || fe.height <= 0 || fe.stride_bytes <= 0) {
        return false;
    }

    const std::size_t bpp = bytesPerPixel(fe.format);
    const std::size_t minStride = static_cast<std::size_t>(fe.width) * bpp;
    if (static_cast<std::size_t>(fe.stride_bytes) < minStride) {
        return false;
    }

    const std::size_t required =
        static_cast<std::size_t>(fe.stride_bytes) *
        static_cast<std::size_t>(fe.height);
    return fe.data.size() >= required;
}

} // namespace

// -------------------- ctor --------------------
UiViewer::UiViewer(Logger& log, Config cfg)
    : log_(log), cfg_(std::move(cfg)), threshold_(cfg_.threshold) {

    cv::namedWindow(cfg_.window_name, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(cfg_.window_name, &UiViewer::onMouse_, this);

    const int N = std::max(32, cfg_.plot_history);

    hist_cx_.assign(N, 0.0f);
    hist_cy_.assign(N, 0.0f);

    for (auto& v : hist_u_)   v.assign(N, 0.0f);
    for (auto& v : hist_lat_) v.assign(N, 0.0f);

    hist_head_  = 0;
    hist_count_ = 0;
}

// -------------------- callbacks (thread-safe) --------------------
void UiViewer::onFrame(const FrameEvent& fe) {
    if (!frameReadableByUi(fe)) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_);

    switch (fe.format) {
        case PixelFormat::Gray8: {
            cv::Mat tmp(fe.height,
                        fe.width,
                        CV_8UC1,
                        const_cast<uint8_t*>(fe.data.data()),
                        static_cast<std::size_t>(fe.stride_bytes));
            tmp.copyTo(gray_);
            break;
        }

        case PixelFormat::RGB888: {
            cv::Mat rgb(fe.height,
                        fe.width,
                        CV_8UC3,
                        const_cast<uint8_t*>(fe.data.data()),
                        static_cast<std::size_t>(fe.stride_bytes));
            cv::cvtColor(rgb, gray_, cv::COLOR_RGB2GRAY);
            break;
        }

        case PixelFormat::BGR888: {
            cv::Mat bgr(fe.height,
                        fe.width,
                        CV_8UC3,
                        const_cast<uint8_t*>(fe.data.data()),
                        static_cast<std::size_t>(fe.stride_bytes));
            cv::cvtColor(bgr, gray_, cv::COLOR_BGR2GRAY);
            break;
        }
    }
}

void UiViewer::onEstimate(const SunEstimate& est) {
    std::lock_guard<std::mutex> lock(m_);
    lastEst_ = est;

    if (hist_count_ > 0 && !hist_cx_.empty()) {
        const int N = static_cast<int>(hist_cx_.size());
        const int idx = (hist_head_ - 1 + N) % N;
        hist_cx_[idx] = est.cx;
        hist_cy_[idx] = est.cy;
    }
}

void UiViewer::onSetpoint(const PlatformSetpoint& sp) {
    std::lock_guard<std::mutex> lock(m_);
    lastSp_ = sp;
}

void UiViewer::onCommand(const ActuatorCommand& cmd) {
    std::lock_guard<std::mutex> lock(m_);
    lastCmd_ = cmd;

    pushSample_(cmd.actuator_targets[0],
                cmd.actuator_targets[1],
                cmd.actuator_targets[2]);

    if (!hist_cx_.empty()) {
        const int N = static_cast<int>(hist_cx_.size());
        const int idx = (hist_head_ - 1 + N) % N;
        hist_cx_[idx] = lastEst_.cx;
        hist_cy_[idx] = lastEst_.cy;
    }

    if (!hist_lat_[0].empty()) {
        const int N = static_cast<int>(hist_lat_[0].size());
        const int idx = (hist_head_ - 1 + N) % N;
        hist_lat_[0][idx] = lastLat_.cap_to_est_ms;
        hist_lat_[1][idx] = lastLat_.est_to_ctrl_ms;
        hist_lat_[2][idx] = lastLat_.ctrl_to_act_ms;
    }
}

void UiViewer::onLatency(const LatencySample& ls) {
    std::lock_guard<std::mutex> lock(m_);
    lastLat_ = ls;

    if (hist_count_ > 0) {
        pushLatency_(ls.cap_to_est_ms, ls.est_to_ctrl_ms, ls.ctrl_to_act_ms);
    }
}

int UiViewer::threshold() const noexcept {
    return threshold_.load();
}

// -------------------- mouse --------------------
void UiViewer::onMouse_(int event, int, int, int, void* userdata) {
    auto* self = static_cast<UiViewer*>(userdata);
    if (!self) return;
    self->handleMouse_(event);
}

void UiViewer::handleMouse_(int event) {
    int thr = threshold_.load();

    if (event == cv::EVENT_LBUTTONDOWN) {
        thr += cfg_.thr_step;
    } else if (event == cv::EVENT_RBUTTONDOWN) {
        thr -= cfg_.thr_step;
    } else {
        return;
    }

    thr = clampi(thr, 0, 255);
    threshold_.store(thr);
    log_.info("UI threshold -> " + std::to_string(thr));
}

// -------------------- ring buffer helpers --------------------
void UiViewer::pushSample_(float v0, float v1, float v2) {
    const int N = static_cast<int>(hist_u_[0].size());
    if (N <= 0) return;

    hist_u_[0][hist_head_] = v0;
    hist_u_[1][hist_head_] = v1;
    hist_u_[2][hist_head_] = v2;

    hist_head_ = (hist_head_ + 1) % N;
    hist_count_ = std::min(hist_count_ + 1, N);
}

void UiViewer::pushLatency_(float a, float b, float c) {
    const int N = static_cast<int>(hist_lat_[0].size());
    if (N <= 0) return;

    const int idx = (hist_head_ - 1 + N) % N;
    hist_lat_[0][idx] = a;
    hist_lat_[1][idx] = b;
    hist_lat_[2][idx] = c;
}

void UiViewer::pushCentroid_(float cx, float cy) {
    const int N = static_cast<int>(hist_cx_.size());
    if (N <= 0) return;

    const int idx = (hist_head_ - 1 + N) % N;
    hist_cx_[idx] = cx;
    hist_cy_[idx] = cy;
}

// -------------------- overlay (text + arrows) --------------------
void UiViewer::drawOverlay_(cv::Mat& top, float conf, int thr) {
    const int font = cv::FONT_HERSHEY_SIMPLEX;

    const float cx0 = 0.5f * static_cast<float>(top.cols);
    const float cy0 = 0.5f * static_cast<float>(top.rows);
    const float ex  = lastEst_.cx - cx0;
    const float ey  = lastEst_.cy - cy0;

    drawTextBox(top,
                "thr=" + std::to_string(thr) + "   conf=" + fmt(conf, 6),
                {20, 36}, font, 0.85, {255,255,255}, 2, {0,0,0}, 0.55);

    if (conf > 0.0f) {
        drawTextBox(top,
                    "er-x=" + fmt(ex, 1) + " px   er-y=" + fmt(ey, 1) + " px",
                    {20, 76}, font, 0.85, {0,255,255}, 2, {0,0,0}, 0.55);
    } else {
        drawTextBox(top,
                    "er-x=--  er-y=--",
                    {20, 76}, font, 0.85, {0,255,255}, 2, {0,0,0}, 0.55);
    }

    const double tilt_deg = static_cast<double>(lastSp_.tilt_rad) * (180.0 / static_cast<double>(kPi));
    const double pan_deg  = static_cast<double>(lastSp_.pan_rad)  * (180.0 / static_cast<double>(kPi));

    drawTextBox(top,
                "tilt=" + fmt(tilt_deg, 3) + " deg   pan=" + fmt(pan_deg, 3) + " deg",
                {20, 116}, font, 0.85, {0,255,0}, 2, {0,0,0}, 0.55);

    const cv::Point c(top.cols / 2, top.rows / 2);

    if (conf > 0.0f) {
        const cv::Point s(
            static_cast<int>(std::lround(lastEst_.cx)),
            static_cast<int>(std::lround(lastEst_.cy))
        );

        cv::arrowedLine(top, c, s, cv::Scalar(0,255,255), 2, cv::LINE_AA, 0, 0.18);
        cv::circle(top, s, 7, cv::Scalar(0,255,255), 2, cv::LINE_AA);

        drawTextBox(top, "ERR",
                    {s.x + 14, s.y - 12},
                    cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(0,255,255), 2,
                    cv::Scalar(0,0,0), 0.55);

        if (cfg_.show_setpoint_overlay) {
            cv::arrowedLine(top, s, c, cv::Scalar(0,255,0), 3, cv::LINE_AA, 0, 0.22);
            drawTextBox(top, "SP",
                        {c.x + 14, c.y - 12},
                        cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        cv::Scalar(0,255,0), 2,
                        cv::Scalar(0,0,0), 0.55);
        }
    }
}

void UiViewer::drawSetpointOverlay_(cv::Mat&) {
    // intentionally unused now (we draw everything inside drawOverlay_ to avoid duplicates)
}

// -------------------- plots --------------------
void UiViewer::drawPlots_(cv::Mat& vis) {
    const int H = std::max(180, cfg_.plot_height);
    const int W = vis.cols;

    cv::Rect plotRect(0, vis.rows - H, W, H);
    cv::rectangle(vis, plotRect, cv::Scalar(255,255,255), cv::FILLED);

    const int L = 62, R = 14, T = 26, B = 14;
    cv::Rect ax(plotRect.x + L,
                plotRect.y + T,
                plotRect.width - L - R,
                plotRect.height - T - B);

    cv::rectangle(vis, ax, cv::Scalar(0,0,0), 1);

    if (hist_count_ < 2) {
        cv::putText(vis, "Waiting for actuator data...",
                    {ax.x + 10, ax.y + 30},
                    cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0,0,0), 1, cv::LINE_AA);
        return;
    }

    const int stride = std::max(1, cfg_.plot_stride_px);
    const int maxSamples = std::min(hist_count_, ax.width / stride);

    auto idxAt = [&](int back) {
        const int N = static_cast<int>(hist_u_[0].size());
        int idx = hist_head_ - 1 - back;
        while (idx < 0) idx += N;
        return idx % N;
    };

    float mn = std::numeric_limits<float>::infinity();
    float mx = -std::numeric_limits<float>::infinity();
    for (int i = 0; i < maxSamples; ++i) {
        const int idx = idxAt(i);
        for (int k = 0; k < 3; ++k) {
            mn = std::min(mn, hist_u_[k][idx]);
            mx = std::max(mx, hist_u_[k][idx]);
        }
    }
    if (!std::isfinite(mn) || !std::isfinite(mx) || mn == mx) {
        mn = 0.f;
        mx = 1.f;
    }
    const float r = (mx - mn);
    mn -= 0.08f * r;
    mx += 0.08f * r;

    auto xOf = [&](int back) -> int {
        const int xi = ax.x + (ax.width - 1) - back * stride;
        return std::clamp(xi, ax.x, ax.x + ax.width - 1);
    };

    auto yOf = [&](float v) -> int {
        const float t = (v - mn) / (mx - mn);
        const float yf = static_cast<float>(ax.y) + (1.0f - t) * static_cast<float>(ax.height - 1);
        const int yi = static_cast<int>(std::lround(yf));
        return std::clamp(yi, ax.y, ax.y + ax.height - 1);
    };

    for (int i = 1; i <= 4; ++i) {
        const int y = ax.y + (i * ax.height) / 5;
        cv::line(vis, {ax.x, y}, {ax.x + ax.width, y}, cv::Scalar(230,230,230), 1);
    }
    for (int i = 1; i <= 6; ++i) {
        const int x = ax.x + (i * ax.width) / 7;
        cv::line(vis, {x, ax.y}, {x, ax.y + ax.height}, cv::Scalar(242,242,242), 1);
    }

    for (int i = 0; i <= 4; ++i) {
        const float a = static_cast<float>(i) / 4.0f;
        const float val = mx - a * (mx - mn);
        const int y = ax.y + static_cast<int>(a * (ax.height - 1));
        cv::line(vis, {ax.x - 5, y}, {ax.x, y}, cv::Scalar(0,0,0), 1);

        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.3f", val);
        cv::putText(vis, buf, {plotRect.x + 6, y + 4},
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0,0,0), 1, cv::LINE_AA);
    }

    cv::putText(vis, "Actuator Targets (u0, u1, u2)",
                {ax.x, ax.y - 8},
                cv::FONT_HERSHEY_SIMPLEX, 0.60, cv::Scalar(0,0,0), 1, cv::LINE_AA);

    auto putLegendRight = [&](int y) {
        const int font = cv::FONT_HERSHEY_SIMPLEX;
        const double fs = 0.60;
        const int th = 2;
        const int gap = 18;

        struct Item { const char* s; cv::Scalar col; };
        Item items[3] = {
            {"u0", cv::Scalar(0,255,255)},
            {"u1", cv::Scalar(255,255,0)},
            {"u2", cv::Scalar(255,0,255)}
        };

        int totalW = 0;
        for (auto& it : items) {
            int base = 0;
            totalW += cv::getTextSize(it.s, font, fs, th, &base).width + gap;
        }
        totalW -= gap;

        int x = ax.x + ax.width - totalW;
        x = std::max(x, ax.x + 10);

        for (auto& it : items) {
            cv::putText(vis, it.s, {x, y}, font, fs, it.col, th, cv::LINE_AA);
            int base = 0;
            x += cv::getTextSize(it.s, font, fs, th, &base).width + gap;
        }
    };
    putLegendRight(ax.y - 8);

    const cv::Scalar colU[3] = { {0,255,255}, {255,255,0}, {255,0,255} };
    for (int k = 0; k < 3; ++k) {
        cv::Point prev;
        bool hasPrev = false;
        for (int i = 0; i < maxSamples; ++i) {
            const int idx = idxAt(i);
            const cv::Point p(xOf(i), yOf(hist_u_[k][idx]));
            if (hasPrev) cv::line(vis, prev, p, colU[k], 2, cv::LINE_AA);
            prev = p;
            hasPrev = true;
        }
    }
}

// -------------------- tick --------------------
bool UiViewer::tick() {
    cv::Mat grayCopy;
    SunEstimate estCopy;

    {
        std::lock_guard<std::mutex> lock(m_);
        if (!gray_.empty()) gray_.copyTo(grayCopy);
        estCopy = lastEst_;
    }

    if (grayCopy.empty()) {
        cv::Mat blank(cfg_.height, cfg_.width, CV_8UC3, cv::Scalar(0,0,0));
        cv::putText(blank, "Waiting for frames...",
                    {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(200,200,200), 2, cv::LINE_AA);
        cv::imshow(cfg_.window_name, blank);
    } else {
        const int plotH = std::max(140, cfg_.plot_height);
        cv::Mat vis(cfg_.height + plotH, cfg_.width, CV_8UC3, cv::Scalar(0,0,0));

        cv::Mat top = vis(cv::Rect(0, 0, cfg_.width, cfg_.height));
        cv::cvtColor(grayCopy, top, cv::COLOR_GRAY2BGR);

        if (estCopy.confidence > 0.0f) {
            cv::circle(top,
                       cv::Point(static_cast<int>(std::lround(estCopy.cx)),
                                 static_cast<int>(std::lround(estCopy.cy))),
                       8, cv::Scalar(0,255,255), 2, cv::LINE_AA);
        }

        drawOverlay_(top, estCopy.confidence, threshold_.load());
        drawPlots_(vis);

        cv::imshow(cfg_.window_name, vis);
    }

    const int key = cv::waitKey(1);
    if (key == 27 || key == 'q' || key == 'Q') {
        quit_ = true;
    }
    return !quit_;
}

} // namespace solar

#endif