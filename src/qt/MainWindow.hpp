#pragma once

#include <QMainWindow>
#include <QPointF>
#include <QString>
#include <QImage>

#include <atomic>
#include <cstdint>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <QSplitter>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <QtCharts/QChartView>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

// Types used in UI callbacks / overlays.
#include "common/Types.hpp"
#include "system/SystemManager.hpp"

namespace solar {

class MainWindow final : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(SystemManager& sys, QWidget* parent = nullptr);
    ~MainWindow() override;

protected:
    void closeEvent(QCloseEvent* e) override;

private:
    // =============================
    // Core
    // =============================
    SystemManager& sys_;

    // =============================
    // UI controls
    // =============================
    QPushButton* btnAuto_{nullptr};
    QPushButton* btnManual_{nullptr};
    QPushButton* btnStop_{nullptr};

    QLabel* status_{nullptr};

    // Threshold controls
    QPushButton* btnThrMinus_{nullptr};
    QPushButton* btnThrPlus_{nullptr};
    QLabel*      thrLabel_{nullptr};
    std::atomic<uint8_t> thr_{200};

    // Manual controls
    QSlider* pan_{nullptr};
    QSlider* tilt_{nullptr};
    QLabel*  panVal_{nullptr};
    QLabel*  tiltVal_{nullptr};
    QPushButton* btnSend_{nullptr};
    QPushButton* btnZero_{nullptr};
    bool manualMode_{false};

    // =============================
    // Camera
    // =============================
    QLabel* camView_{nullptr};
    QTimer* camTimer_{nullptr};

    void updateCamera_();

    // --- Converter worker (keeps UI thread light) ---
    void startConverter_();
    void stopConverter_();
    void converterLoop_();

    std::thread convThread_;
    std::atomic<bool> convRunning_{false};

    std::mutex convMutex_;
    std::condition_variable convCv_;
    bool convHasFrame_{false};

    // Latest frame copied from the system callback.
    // The public FrameEvent contract now uses explicit format + stride.
    std::shared_ptr<std::vector<uint8_t>> latestBuf_;
    int latestW_{0};
    int latestH_{0};
    int latestStride_{0};
    PixelFormat latestFormat_{PixelFormat::Gray8};

    // Latest converted image for display.
    std::mutex imgMutex_;
    QImage latestImg_;
    std::atomic<bool> imgReady_{false};

    // =============================
    // Plot
    // =============================
    QtCharts::QChartView*  chartView_{nullptr};
    QtCharts::QChart*      chart_{nullptr};
    QtCharts::QLineSeries* s0_{nullptr};
    QtCharts::QLineSeries* s1_{nullptr};
    QtCharts::QLineSeries* s2_{nullptr};
    QtCharts::QValueAxis*  axX_{nullptr};
    QtCharts::QValueAxis*  axY_{nullptr};
    QTimer* plotTimer_{nullptr};

    std::atomic<float> u0_{0.f};
    std::atomic<float> u1_{0.f};
    std::atomic<float> u2_{0.f};

    int x_{0};
    const int windowN_{300};

    void setupPlot_();
    void onPlotTick_();

    // =============================
    // Overlay
    // =============================
    std::mutex ovMtx_;
    SunEstimate      lastEst_{};
    PlatformSetpoint lastSp_{};
    ActuatorCommand  lastCmd_{};
    bool haveEst_{false};
    bool haveSp_{false};
    bool haveCmd_{false};

    void drawOverlay_(QImage& img);
    static QPointF clampToImage_(QPointF p, int w, int h);

    // =============================
    // Helpers
    // =============================
    void setStatus_(const QString& s);
    void setManualEnabled_(bool en);
    void sendManual_();

    static float deg2rad_(int d) {
        return static_cast<float>(d) * 3.14159265358979323846f / 180.f;
    }
};

} // namespace solar