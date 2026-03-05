#include "ui/OverlayDraw.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include <opencv2/imgproc.hpp>

namespace solar::ui {

static inline float rad2deg(float r) {
    return r * 180.0f / 3.14159265358979323846f;
}

// Simple filled textbox with alpha (so text stays readable)
static inline void drawTextBox(cv::Mat& img,
                               const std::string& text,
                               cv::Point org,                 // baseline start
                               int fontFace,
                               double fontScale,
                               const cv::Scalar& textColor,
                               int thickness,
                               const cv::Scalar& boxColor,
                               double boxAlpha = 0.55) {
    int base = 0;
    const cv::Size sz = cv::getTextSize(text, fontFace, fontScale, thickness, &base);

    const int padX = 10;
    const int padY = 7;

    cv::Rect r(org.x - padX,
               org.y - sz.height - padY,
               sz.width + 2 * padX,
               sz.height + base + 2 * padY);

    // clamp inside image
    r.x = std::max(0, r.x);
    r.y = std::max(0, r.y);
    r.width  = std::min(r.width,  img.cols - r.x);
    r.height = std::min(r.height, img.rows - r.y);

    cv::Mat roi = img(r);
    cv::Mat overlay;
    roi.copyTo(overlay);

    cv::rectangle(overlay, cv::Rect(0, 0, r.width, r.height), boxColor, cv::FILLED);
    cv::addWeighted(overlay, boxAlpha, roi, 1.0 - boxAlpha, 0.0, roi);

    cv::putText(img, text, org, fontFace, fontScale, textColor, thickness, cv::LINE_AA);
}

// Draw arrow with a small label box at the end
static inline void drawArrowWithLabel(cv::Mat& img,
                                      cv::Point from,
                                      cv::Point to,
                                      const cv::Scalar& color,
                                      const std::string& label) {
    cv::arrowedLine(img, from, to, color, 2, cv::LINE_AA, 0, 0.18);

    // label near arrow tip
    const int font = cv::FONT_HERSHEY_SIMPLEX;
    const double scale = 0.6;
    const int thick = 2;

    // Slight offset so it doesn't sit exactly on the tip
    cv::Point labOrg(to.x + 10, to.y - 10);
    labOrg.x = std::clamp(labOrg.x, 10, img.cols - 10);
    labOrg.y = std::clamp(labOrg.y, 30, img.rows - 10);

    drawTextBox(img, label, labOrg, font, scale,
                cv::Scalar(255, 255, 255), thick,
                cv::Scalar(40, 40, 40), 0.65);
}

void drawOverlay(cv::Mat& img,
                 const SunEstimate& est,
                 const PlatformSetpoint& sp,
                 const ActuatorCommand& cmd,
                 uint8_t thr) {
    if (img.empty()) return;

    // Ensure we draw on 3-channel BGR for colored overlay
    if (img.type() == CV_8UC1) {
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    } else if (img.type() != CV_8UC3) {
        // Best-effort convert to 8UC3
        cv::Mat tmp;
        img.convertTo(tmp, CV_8U);
        if (tmp.channels() == 1) cv::cvtColor(tmp, img, cv::COLOR_GRAY2BGR);
        else if (tmp.channels() == 3) img = tmp;
        else return;
    }

    const int W = img.cols;
    const int H = img.rows;

    // We assume the target setpoint pixel is the image center
    const cv::Point spPix(W / 2, H / 2);

    // Estimate pixel (clamp to image)
    const int ex = static_cast<int>(std::lround(est.cx));
    const int ey = static_cast<int>(std::lround(est.cy));
    const cv::Point estPix(std::clamp(ex, 0, W - 1), std::clamp(ey, 0, H - 1));

    // Error in pixels (estimate - setpoint)
    const float err_x = static_cast<float>(estPix.x - spPix.x);
    const float err_y = static_cast<float>(estPix.y - spPix.y);

    // ---- Top-left text HUD (similar to your screenshot) ----
    const int font = cv::FONT_HERSHEY_SIMPLEX;
    const double scale1 = 0.8;
    const double scale2 = 0.85;
    const int thick = 2;

    // line 1: thr + conf
    {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "thr=%u   conf=%.6f", static_cast<unsigned>(thr), est.confidence);
        drawTextBox(img, buf, cv::Point(20, 45), font, scale1,
                    cv::Scalar(255, 255, 255), thick,
                    cv::Scalar(0, 0, 0), 0.55);
    }

    // line 2: pixel error (yellow text)
    {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "er-x=%.1f px   er-y=%.1f px", err_x, err_y);
        drawTextBox(img, buf, cv::Point(20, 90), font, scale2,
                    cv::Scalar(0, 255, 255), thick,
                    cv::Scalar(0, 0, 0), 0.55);
    }

    // line 3: tilt/pan in degrees (green text)
    {
        const float tilt_deg = rad2deg(sp.tilt_rad);
        const float pan_deg  = rad2deg(sp.pan_rad);

        char buf[128];
        std::snprintf(buf, sizeof(buf), "tilt=%.3f deg   pan=%.3f deg", tilt_deg, pan_deg);
        drawTextBox(img, buf, cv::Point(20, 135), font, scale2,
                    cv::Scalar(0, 255, 0), thick,
                    cv::Scalar(0, 0, 0), 0.55);
    }

    // ---- SP + ERR arrows (like your screenshot) ----
    // Green arrow: from estimate to setpoint (where we want it to go)
    drawArrowWithLabel(img, estPix, spPix, cv::Scalar(0, 255, 0), "SP");

    // Yellow arrow: from setpoint to estimate (error direction)
    drawArrowWithLabel(img, spPix, estPix, cv::Scalar(0, 255, 255), "ERR");

    // Mark points
    cv::circle(img, spPix, 6, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    cv::circle(img, estPix, 6, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

    // ---- Optional: show u0/u1/u2 degrees in a small box (bottom-left) ----
    {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "u0=%.1f  u1=%.1f  u2=%.1f deg",
                      cmd.actuator_targets[0], cmd.actuator_targets[1], cmd.actuator_targets[2]);
        drawTextBox(img, buf, cv::Point(20, H - 20), font, 0.7,
                    cv::Scalar(255, 255, 255), 2,
                    cv::Scalar(0, 0, 0), 0.55);
    }
}

} // namespace solar::ui