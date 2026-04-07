#pragma once

/**
 * @file MainWindow.hpp
 * @brief Qt main window for the Solar Stewart Tracker runtime.
 */

#include "common/Types.hpp"

#include <QImage>
#include <QMainWindow>
#include <QPointF>
#include <QString>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

QT_BEGIN_NAMESPACE
class QLabel;
class QPushButton;
class QScrollArea;
class QSlider;
class QTimer;
QT_END_NAMESPACE

namespace QtCharts {
class QChart;
class QChartView;
class QLineSeries;
class QValueAxis;
}

namespace solar {

class SystemManager;

class MainWindow final : public QMainWindow {
public:
    explicit MainWindow(SystemManager& sys, QWidget* parent = nullptr);
    ~MainWindow() override;

protected:
    void closeEvent(QCloseEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    // UI helpers and the converter thread keep preview handling separate from user interaction.
    SystemManager& sys_;
    bool manualMode_{false};
    bool closing_{false};

    // Top-level mode controls.
    QPushButton* btnAuto_{nullptr};
    QPushButton* btnManual_{nullptr};
    QPushButton* btnStop_{nullptr};
        
    QLabel* statusLabel_{nullptr};
    QLabel* modeLabel_{nullptr};
    QLabel* imuLabel_{nullptr};
    QLabel* manualLabel_{nullptr};

    QPushButton* btnThrMinus_{nullptr};
    QPushButton* btnThrPlus_{nullptr};
    QLabel* thrLabel_{nullptr};
    std::atomic<std::uint8_t> thr_{200U};

    QSlider* pan_{nullptr};
    QSlider* tilt_{nullptr};
    QLabel* panVal_{nullptr};
    QLabel* tiltVal_{nullptr};
    QPushButton* btnSend_{nullptr};
    QPushButton* btnZero_{nullptr};


    QLabel* camView_{nullptr};
    QTimer* camTimer_{nullptr};
    QLabel* manualSourceLabel_{nullptr};
    QPushButton* btnUsePot_{nullptr};
    QPushButton* btnUseGui_{nullptr};

    bool guiManualOwner_{false};

    void refreshManualSourceUi_();
    void updateCamera_();

    void startConverter_();
    void stopConverter_();
    void converterLoop_();

    std::thread convThread_;
    std::atomic<bool> convRunning_{false};

    std::mutex convMutex_;
    std::condition_variable convCv_;
    bool convHasFrame_{false};

    std::vector<std::uint8_t> pendingFrameBytes_{};
    int pendingFrameW_{0};
    int pendingFrameH_{0};
    int pendingFrameStride_{0};
    PixelFormat pendingFrameFormat_{PixelFormat::Gray8};

    std::mutex imgMutex_;
    // Latest converted preview image ready for paint/update.
    QImage latestImg_;
    std::atomic<bool> imgReady_{false};

    QtCharts::QChartView* outputChartView_{nullptr};
    QtCharts::QChart* outputChart_{nullptr};
    QtCharts::QLineSeries* u0Series_{nullptr};
    QtCharts::QLineSeries* u1Series_{nullptr};
    QtCharts::QLineSeries* u2Series_{nullptr};
    QtCharts::QValueAxis* outputAxX_{nullptr};
    QtCharts::QValueAxis* outputAxY_{nullptr};

    QtCharts::QChartView* manualChartView_{nullptr};
    QtCharts::QChart* manualChart_{nullptr};
    QtCharts::QLineSeries* vTiltSeries_{nullptr};
    QtCharts::QLineSeries* vPanSeries_{nullptr};
    QtCharts::QValueAxis* manualAxX_{nullptr};
    QtCharts::QValueAxis* manualAxY_{nullptr};

    QtCharts::QChartView* imuChartView_{nullptr};
    QtCharts::QChart* imuChart_{nullptr};
    QtCharts::QLineSeries* imuAxSeries_{nullptr};
    QtCharts::QLineSeries* imuAySeries_{nullptr};
    QtCharts::QLineSeries* imuAzSeries_{nullptr};
    QtCharts::QLineSeries* imuTiltSeries_{nullptr};
    QtCharts::QValueAxis* imuAxX_{nullptr};
    QtCharts::QValueAxis* imuAccelY_{nullptr};
    QtCharts::QValueAxis* imuTiltY_{nullptr};

    QTimer* plotTimer_{nullptr};

    int outputX_{0};
    int manualX_{0};
    int imuX_{0};
    const int windowN_{300};

    void setupOutputPlot_();
    void setupManualVoltagePlot_();
    void setupImuPlot_();
    void onPlotTick_();

    void appendOutputPlot_(float u0, float u1, float u2);
    void appendManualVoltagePlot_(float tilt_v, float pan_v);
    void appendImuPlot_(float ax, float ay, float az, float tilt_deg);

    std::atomic<float> u0_{0.0F};
    std::atomic<float> u1_{0.0F};
    std::atomic<float> u2_{0.0F};

    std::atomic<float> manualTiltVoltageV_{0.0F};
    std::atomic<float> manualPanVoltageV_{0.0F};
    std::atomic<std::uint64_t> manualSequence_{0U};

    std::atomic<float> imuAx_{0.0F};
    std::atomic<float> imuAy_{0.0F};
    std::atomic<float> imuAz_{0.0F};
    std::atomic<float> imuTiltDeg_{0.0F};
    std::atomic<bool> imuValid_{false};

    std::atomic<std::uint64_t> outputSequence_{0U};
    std::atomic<std::uint64_t> imuSequence_{0U};

    std::uint64_t lastPlottedOutputSequence_{0U};
    std::uint64_t lastPlottedManualSequence_{0U};
    std::uint64_t lastPlottedImuSequence_{0U};

    std::mutex ovMtx_;
    SunEstimate lastEst_{};
    PlatformSetpoint lastSp_{};
    ActuatorCommand lastCmd_{};
    bool haveEst_{false};
    bool haveSp_{false};
    bool haveCmd_{false};

    void drawOverlay_(QImage& img);
    static QPointF clampToImage_(QPointF p, int w, int h);

    void setStatus_(const QString& s);
    void refreshModeUi_();
    void refreshStatusText_();
    void setManualEnabled_(bool enabled);
    void sendManual_();

    static float deg2rad_(int deg) {
        return static_cast<float>(deg) * 3.14159265358979323846F / 180.0F;
    }
};

} // namespace solar