#include "qt/MainWindow.hpp"

#include "system/SystemManager.hpp"

#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

#include <QCloseEvent>
#include <QFrame>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QPainter>
#include <QPen>
#include <QPixmap>
#include <QPushButton>
#include <QResizeEvent>
#include <QScrollArea>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <utility>

using namespace QtCharts;

namespace solar {
namespace {

// Basic frame-shape validation before any Qt-side conversion.
bool frameReadableForQt(const std::vector<std::uint8_t>& data,
                        const int width,
                        const int height,
                        const int strideBytes,
                        const PixelFormat format) {
    if (width <= 0 || height <= 0 || strideBytes <= 0) {
        return false;
    }

    const std::size_t bpp = bytesPerPixel(format);
    const std::size_t minStride = static_cast<std::size_t>(width) * bpp;

    if (static_cast<std::size_t>(strideBytes) < minStride) {
        return false;
    }

    const std::size_t required =
        static_cast<std::size_t>(strideBytes) *
        static_cast<std::size_t>(height);

    return data.size() >= required;
}

QString yesNo(const bool v) {
    return v ? "yes" : "no";
}

void setSeriesWidth(QLineSeries* s, int width) {
    if (!s) return;
    QPen p = s->pen();
    p.setWidth(width);
    s->setPen(p);
}

} // namespace

void MainWindow::startConverter_() {
    // Keep colour conversion off the GUI thread so repaint stays responsive.
    if (convRunning_.load()) {
        return;
    }

    convRunning_.store(true);
    convThread_ = std::thread(&MainWindow::converterLoop_, this);
}

// Converter-thread stop path and wake-up sequence.
void MainWindow::stopConverter_() {
    if (!convRunning_.load()) {
        return;
    }

    convRunning_.store(false);
    {
        std::lock_guard<std::mutex> lk(convMutex_);
        convHasFrame_ = true;
    }
    convCv_.notify_all();

    if (convThread_.joinable()) {
        convThread_.join();
    }
}

void MainWindow::converterLoop_() {
    // The converter thread always works on the latest complete preview frame.
    while (convRunning_.load()) {
        std::vector<std::uint8_t> buf;
        int w = 0;
        int h = 0;
        int stride = 0;
        PixelFormat format = PixelFormat::Gray8;

        {
            std::unique_lock<std::mutex> lk(convMutex_);
            // Sleep until a fresh preview frame arrives for conversion.
            convCv_.wait(lk, [this] {
                return !convRunning_.load() || convHasFrame_;
            });

            if (!convRunning_.load()) {
                break;
            }

            buf = std::move(pendingFrameBytes_);
            w = pendingFrameW_;
            h = pendingFrameH_;
            stride = pendingFrameStride_;
            format = pendingFrameFormat_;
            convHasFrame_ = false;
        }

        if (!frameReadableForQt(buf, w, h, stride, format)) {
            continue;
        }

        QImage img;

        // Format-specific conversion into a Qt-owned image.
        switch (format) {
            case PixelFormat::Gray8:
                // Copy into a Qt-owned image so the worker buffer can be reused immediately.
                img = QImage(buf.data(),
                             w,
                             h,
                             stride,
                             QImage::Format_Grayscale8).copy();
                break;

            case PixelFormat::RGB888:
                img = QImage(buf.data(),
                             w,
                             h,
                             stride,
                             QImage::Format_RGB888).copy();
                break;

            case PixelFormat::BGR888:
                img = QImage(buf.data(),
                             w,
                             h,
                             stride,
                             QImage::Format_RGB888).rgbSwapped().copy();
                break;
        }

        if (img.isNull()) {
            continue;
        }

        {
            std::lock_guard<std::mutex> lk(imgMutex_);
            latestImg_ = std::move(img);
            imgReady_.store(true, std::memory_order_release);
        }
    }
}

MainWindow::MainWindow(SystemManager& sys, QWidget* parent)
    : QMainWindow(parent),
      sys_(sys) {
    auto* content = new QWidget(this);
    auto* root = new QVBoxLayout(content);

    auto* topRow = new QHBoxLayout();
    btnAuto_ = new QPushButton("AUTO", content);
    btnManual_ = new QPushButton("MANUAL", content);
    btnStop_ = new QPushButton("STOP", content);
    btnStop_->setMinimumHeight(46);

    topRow->addWidget(btnAuto_);
    topRow->addWidget(btnManual_);
    topRow->addWidget(btnStop_);
    root->addLayout(topRow);

    modeLabel_ = new QLabel("Mode: AUTO", content);
    statusLabel_ = new QLabel("Status: starting", content);
    manualLabel_ = new QLabel("Manual input: tilt_v=0.000  pan_v=0.000  seq=0", content);
    imuLabel_ = new QLabel("IMU: valid=no  ax=0.00  ay=0.00  az=0.00  tilt=0.00 deg", content);
    manualSourceLabel_ = new QLabel("Manual source: Pot", content);

    root->addWidget(modeLabel_);
    root->addWidget(statusLabel_);
    root->addWidget(manualLabel_);
    root->addWidget(imuLabel_);
    root->addWidget(manualSourceLabel_);

    auto* thrRow = new QHBoxLayout();
    thrRow->addWidget(new QLabel("Threshold:", content));

    btnThrMinus_ = new QPushButton("-", content);
    btnThrPlus_ = new QPushButton("+", content);
    thrLabel_ = new QLabel(QString::number(int(thr_.load())), content);
    thrLabel_->setMinimumWidth(50);

    thrRow->addWidget(btnThrMinus_);
    thrRow->addWidget(thrLabel_);
    thrRow->addWidget(btnThrPlus_);
    thrRow->addStretch(1);
    root->addLayout(thrRow);

    root->addWidget(new QLabel("Live preview:", content));
    camView_ = new QLabel(content);
    camView_->setAlignment(Qt::AlignCenter);
    camView_->setMinimumSize(1000, 700);
    camView_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    camView_->setFrameShape(QFrame::Box);
    root->addWidget(camView_);

    root->addWidget(new QLabel("Manual setpoint (degrees):", content));

    auto* sourceRow = new QHBoxLayout();
    btnUsePot_ = new QPushButton("Use Pots", content);
    btnUseGui_ = new QPushButton("Use GUI", content);
    sourceRow->addWidget(btnUsePot_);
    sourceRow->addWidget(btnUseGui_);
    sourceRow->addStretch(1);
    root->addLayout(sourceRow);

    auto* panRow = new QHBoxLayout();
    panRow->addWidget(new QLabel("Pan", content));
    pan_ = new QSlider(Qt::Horizontal, content);
    pan_->setRange(-20, 20);
    pan_->setValue(0);
    pan_->setSingleStep(1);
    pan_->setPageStep(5);
    pan_->setTracking(true);
    panVal_ = new QLabel("0", content);
    panVal_->setMinimumWidth(40);
    panRow->addWidget(pan_);
    panRow->addWidget(panVal_);
    root->addLayout(panRow);

    auto* tiltRow = new QHBoxLayout();
    tiltRow->addWidget(new QLabel("Tilt", content));
    tilt_ = new QSlider(Qt::Horizontal, content);
    tilt_->setRange(-20, 20);
    tilt_->setValue(0);
    tilt_->setSingleStep(1);
    tilt_->setPageStep(5);
    tilt_->setTracking(true);
    tiltVal_ = new QLabel("0", content);
    tiltVal_->setMinimumWidth(40);
    tiltRow->addWidget(tilt_);
    tiltRow->addWidget(tiltVal_);
    root->addLayout(tiltRow);

    auto* sendRow = new QHBoxLayout();
    btnSend_ = new QPushButton("Send manual setpoint", content);
    btnZero_ = new QPushButton("Zero", content);
    sendRow->addWidget(btnSend_);
    sendRow->addWidget(btnZero_);
    root->addLayout(sendRow);

    root->addSpacing(12);
    root->addWidget(new QLabel("Servo commands:", content));
    setupOutputPlot_();
    outputChartView_->setMinimumHeight(360);
    root->addWidget(outputChartView_);

    root->addWidget(new QLabel("ADS1115 manual voltages:", content));
    setupManualVoltagePlot_();
    manualChartView_->setMinimumHeight(300);
    root->addWidget(manualChartView_);

    root->addWidget(new QLabel("MPU6050 readings:", content));
    setupImuPlot_();
    imuChartView_->setMinimumHeight(360);
    root->addWidget(imuChartView_);

    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    scroll->setWidget(content);
    setCentralWidget(scroll);

    resize(1500, 950);
    setWindowTitle("Solar Stewart Tracker Qt Viewer");

    setManualEnabled_(false);
    refreshModeUi_();
    refreshManualSourceUi_();
    refreshStatusText_();

    // Keep the GUI callback short: copy once, wake the converter thread, return.
    sys_.registerFrameObserver([this](const FrameEvent& fe) {
        if (!frameReadableForQt(fe.data, fe.width, fe.height, fe.stride_bytes, fe.format)) {
            return;
        }

        {
            std::lock_guard<std::mutex> lk(convMutex_);
            pendingFrameBytes_.resize(fe.data.size());
            std::memcpy(pendingFrameBytes_.data(), fe.data.data(), fe.data.size());
            pendingFrameW_ = fe.width;
            pendingFrameH_ = fe.height;
            pendingFrameStride_ = fe.stride_bytes;
            pendingFrameFormat_ = fe.format;
            convHasFrame_ = true;
        }
        convCv_.notify_one();
    });

    sys_.registerEstimateObserver([this](const SunEstimate& est) {
        std::lock_guard<std::mutex> lk(ovMtx_);
        lastEst_ = est;
        haveEst_ = true;
    });

    sys_.registerSetpointObserver([this](const PlatformSetpoint& sp) {
        std::lock_guard<std::mutex> lk(ovMtx_);
        lastSp_ = sp;
        haveSp_ = true;
    });

    sys_.registerCommandObserver([this](const ActuatorCommand& cmd) {
        u0_.store(cmd.actuator_targets[0], std::memory_order_relaxed);
        u1_.store(cmd.actuator_targets[1], std::memory_order_relaxed);
        u2_.store(cmd.actuator_targets[2], std::memory_order_relaxed);
        outputSequence_.fetch_add(1U, std::memory_order_relaxed);

        std::lock_guard<std::mutex> lk(ovMtx_);
        lastCmd_ = cmd;
        haveCmd_ = true;
    });

    sys_.registerManualObserver([this](const ManualPotSample& sample) {
        manualTiltVoltageV_.store(sample.tilt_voltage_v, std::memory_order_relaxed);
        manualPanVoltageV_.store(sample.pan_voltage_v, std::memory_order_relaxed);
        manualSequence_.store(sample.sequence, std::memory_order_relaxed);
    });

    sys_.registerImuObserver([this](const control::ImuSample& sample, const float tilt_rad, const bool valid) {
        imuAx_.store(sample.ax, std::memory_order_relaxed);
        imuAy_.store(sample.ay, std::memory_order_relaxed);
        imuAz_.store(sample.az, std::memory_order_relaxed);
        imuTiltDeg_.store(tilt_rad * 180.0F / 3.14159265358979323846F, std::memory_order_relaxed);
        imuValid_.store(valid, std::memory_order_relaxed);
        imuSequence_.fetch_add(1U, std::memory_order_relaxed);
    });

    startConverter_();

    camTimer_ = new QTimer(this);
    connect(camTimer_, &QTimer::timeout, this, [this]() {
        updateCamera_();
    });
    camTimer_->start(33);

    plotTimer_ = new QTimer(this);
    connect(plotTimer_, &QTimer::timeout, this, [this]() {
        onPlotTick_();
    });
    plotTimer_->start(33);

    connect(btnAuto_, &QPushButton::clicked, this, [this]() {
        sys_.exitManual();
        manualMode_ = false;
        guiManualOwner_ = false;
        setManualEnabled_(false);
        refreshModeUi_();
        refreshManualSourceUi_();
        refreshStatusText_();
    });

    connect(btnManual_, &QPushButton::clicked, this, [this]() {
        sys_.enterManual();
        sys_.setManualCommandSource(SystemManager::ManualCommandSource::Pot);
        manualMode_ = true;
        guiManualOwner_ = false;
        setManualEnabled_(true);
        refreshModeUi_();
        refreshManualSourceUi_();
        refreshStatusText_();
    });

    connect(btnUsePot_, &QPushButton::clicked, this, [this]() {
        if (!manualMode_ || closing_) {
            return;
        }

        sys_.setManualCommandSource(SystemManager::ManualCommandSource::Pot);
        guiManualOwner_ = false;
        refreshManualSourceUi_();
        refreshStatusText_();
    });

    connect(btnUseGui_, &QPushButton::clicked, this, [this]() {
        if (!manualMode_ || closing_) {
            return;
        }

        sys_.setManualCommandSource(SystemManager::ManualCommandSource::Gui);
        guiManualOwner_ = true;
        refreshManualSourceUi_();
        refreshStatusText_();
    });

    connect(btnStop_, &QPushButton::clicked, this, [this]() {
        if (closing_) {
            return;
        }

        closing_ = true;

        if (camTimer_) camTimer_->stop();
        if (plotTimer_) plotTimer_->stop();

        btnAuto_->setEnabled(false);
        btnManual_->setEnabled(false);
        btnStop_->setEnabled(false);
        btnThrMinus_->setEnabled(false);
        btnThrPlus_->setEnabled(false);
        setManualEnabled_(false);

        setStatus_("Status: stopping...");
        stopConverter_();
        sys_.stop();
        close();
    });

    connect(btnThrMinus_, &QPushButton::clicked, this, [this]() {
        int t = int(thr_.load());
        t = std::max(0, t - 5);
        thr_.store(static_cast<std::uint8_t>(t));
        thrLabel_->setText(QString::number(t));
        sys_.setTrackerThreshold(static_cast<std::uint8_t>(t));
        refreshStatusText_();
    });

    connect(btnThrPlus_, &QPushButton::clicked, this, [this]() {
        int t = int(thr_.load());
        t = std::min(255, t + 5);
        thr_.store(static_cast<std::uint8_t>(t));
        thrLabel_->setText(QString::number(t));
        sys_.setTrackerThreshold(static_cast<std::uint8_t>(t));
        refreshStatusText_();
    });

    connect(pan_, &QSlider::valueChanged, this, [this](const int v) {
        panVal_->setText(QString::number(v));
        sendManual_();
    });

    connect(tilt_, &QSlider::valueChanged, this, [this](const int v) {
        tiltVal_->setText(QString::number(v));
        sendManual_();
    });

    connect(btnSend_, &QPushButton::clicked, this, [this]() {
        sendManual_();
    });

    connect(btnZero_, &QPushButton::clicked, this, [this]() {
        pan_->setValue(0);
        tilt_->setValue(0);
        sendManual_();
    });
}

MainWindow::~MainWindow() {
    stopConverter_();
}

void MainWindow::closeEvent(QCloseEvent* event) {
    if (!closing_) {
        closing_ = true;

        if (camTimer_) camTimer_->stop();
        if (plotTimer_) plotTimer_->stop();

        stopConverter_();
        sys_.stop();
    }

    QMainWindow::closeEvent(event);
}

void MainWindow::resizeEvent(QResizeEvent* event) {
    QMainWindow::resizeEvent(event);
    updateCamera_();
}

// Latest converted frame plus lightweight overlay drawing.
void MainWindow::updateCamera_() {
    if (!imgReady_.load()) {
        return;
    }

    QImage img;
    {
        std::lock_guard<std::mutex> lk(imgMutex_);
        img = latestImg_;
    }

    if (img.isNull()) {
        return;
    }

    drawOverlay_(img);

    const QPixmap pix = QPixmap::fromImage(img);
    camView_->setPixmap(
        pix.scaled(
            camView_->size(),
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation));
}

void MainWindow::setupOutputPlot_() {
    u0Series_ = new QLineSeries();
    u1Series_ = new QLineSeries();
    u2Series_ = new QLineSeries();

    u0Series_->setName("u0");
    u1Series_->setName("u1");
    u2Series_->setName("u2");
    u0Series_->setColor(QColor(220, 50, 47));
    u1Series_->setColor(QColor(38, 139, 210));
    u2Series_->setColor(QColor(133, 153, 0));
    setSeriesWidth(u0Series_, 3);
    setSeriesWidth(u1Series_, 3);
    setSeriesWidth(u2Series_, 3);

    outputChart_ = new QChart();
    outputChart_->addSeries(u0Series_);
    outputChart_->addSeries(u1Series_);
    outputChart_->addSeries(u2Series_);
    outputChart_->legend()->setVisible(true);
    outputChart_->setTitle("Servo Commands (degrees)");

    outputAxX_ = new QValueAxis();
    outputAxY_ = new QValueAxis();
    outputAxX_->setTitleText("samples");
    outputAxY_->setTitleText("degrees");
    outputAxY_->setRange(0.0, 90.0);

    outputChart_->addAxis(outputAxX_, Qt::AlignBottom);
    outputChart_->addAxis(outputAxY_, Qt::AlignLeft);

    u0Series_->attachAxis(outputAxX_);
    u0Series_->attachAxis(outputAxY_);
    u1Series_->attachAxis(outputAxX_);
    u1Series_->attachAxis(outputAxY_);
    u2Series_->attachAxis(outputAxX_);
    u2Series_->attachAxis(outputAxY_);

    outputChartView_ = new QChartView(outputChart_);
    outputChartView_->setRenderHint(QPainter::Antialiasing, true);
}

void MainWindow::setupManualVoltagePlot_() {
    vTiltSeries_ = new QLineSeries();
    vPanSeries_ = new QLineSeries();

    vTiltSeries_->setName("Tilt voltage");
    vPanSeries_->setName("Pan voltage");

    setSeriesWidth(vTiltSeries_, 3);
    setSeriesWidth(vPanSeries_, 3);
    vTiltSeries_->setColor(QColor(203, 75, 22));
    vPanSeries_->setColor(QColor(108, 113, 196));

    manualChart_ = new QChart();
    manualChart_->addSeries(vTiltSeries_);
    manualChart_->addSeries(vPanSeries_);
    manualChart_->legend()->setVisible(true);
    manualChart_->setTitle("ADS1115 Pot Voltages");

    manualAxX_ = new QValueAxis();
    manualAxY_ = new QValueAxis();
    manualAxX_->setTitleText("manual samples");
    manualAxY_->setTitleText("volts");
    manualAxY_->setRange(0.0, 4.2);

    manualChart_->addAxis(manualAxX_, Qt::AlignBottom);
    manualChart_->addAxis(manualAxY_, Qt::AlignLeft);

    vTiltSeries_->attachAxis(manualAxX_);
    vTiltSeries_->attachAxis(manualAxY_);
    vPanSeries_->attachAxis(manualAxX_);
    vPanSeries_->attachAxis(manualAxY_);

    manualChartView_ = new QChartView(manualChart_);
    manualChartView_->setRenderHint(QPainter::Antialiasing, true);
}

void MainWindow::setupImuPlot_() {
    imuAxSeries_ = new QLineSeries();
    imuAySeries_ = new QLineSeries();
    imuAzSeries_ = new QLineSeries();
    imuTiltSeries_ = new QLineSeries();

    imuAxSeries_->setName("ax (m/s^2)");
    imuAySeries_->setName("ay (m/s^2)");
    imuAzSeries_->setName("az (m/s^2)");
    imuTiltSeries_->setName("tilt (deg)");
    imuAxSeries_->setColor(QColor(220, 50, 47));
    imuAySeries_->setColor(QColor(38, 139, 210));
    imuAzSeries_->setColor(QColor(133, 153, 0));
    imuTiltSeries_->setColor(QColor(211, 54, 130));
    setSeriesWidth(imuAxSeries_, 3);
    setSeriesWidth(imuAySeries_, 3);
    setSeriesWidth(imuAzSeries_, 3);
    setSeriesWidth(imuTiltSeries_, 3);

    imuChart_ = new QChart();
    imuChart_->addSeries(imuAxSeries_);
    imuChart_->addSeries(imuAySeries_);
    imuChart_->addSeries(imuAzSeries_);
    imuChart_->addSeries(imuTiltSeries_);
    imuChart_->legend()->setVisible(true);
    imuChart_->setTitle("MPU6050 Readings");

    imuAxX_ = new QValueAxis();
    imuAccelY_ = new QValueAxis();
    imuTiltY_ = new QValueAxis();

    imuAxX_->setTitleText("samples");
    imuAccelY_->setTitleText("accel (m/s^2)");
    imuTiltY_->setTitleText("tilt (deg)");

    imuAccelY_->setRange(-15.0, 15.0);
    imuTiltY_->setRange(-45.0, 45.0);

    imuChart_->addAxis(imuAxX_, Qt::AlignBottom);
    imuChart_->addAxis(imuAccelY_, Qt::AlignLeft);
    imuChart_->addAxis(imuTiltY_, Qt::AlignRight);

    imuAxSeries_->attachAxis(imuAxX_);
    imuAxSeries_->attachAxis(imuAccelY_);

    imuAySeries_->attachAxis(imuAxX_);
    imuAySeries_->attachAxis(imuAccelY_);

    imuAzSeries_->attachAxis(imuAxX_);
    imuAzSeries_->attachAxis(imuAccelY_);

    imuTiltSeries_->attachAxis(imuAxX_);
    imuTiltSeries_->attachAxis(imuTiltY_);

    imuChartView_ = new QChartView(imuChart_);
    imuChartView_->setRenderHint(QPainter::Antialiasing, true);
}

void MainWindow::appendOutputPlot_(const float u0, const float u1, const float u2) {
    u0Series_->append(outputX_, u0);
    u1Series_->append(outputX_, u1);
    u2Series_->append(outputX_, u2);

    if (u0Series_->count() > windowN_) {
        u0Series_->remove(0);
        u1Series_->remove(0);
        u2Series_->remove(0);
    }

    const int left = std::max(0, outputX_ - windowN_);
    outputAxX_->setRange(left, outputX_);
    ++outputX_;
}

void MainWindow::appendManualVoltagePlot_(const float tilt_v, const float pan_v) {
    vTiltSeries_->append(manualX_, tilt_v);
    vPanSeries_->append(manualX_, pan_v);

    if (vTiltSeries_->count() > windowN_) {
        vTiltSeries_->remove(0);
        vPanSeries_->remove(0);
    }

    const int left = std::max(0, manualX_ - windowN_);
    manualAxX_->setRange(left, manualX_);
    ++manualX_;
}

void MainWindow::appendImuPlot_(const float ax, const float ay, const float az, const float tilt_deg) {
    imuAxSeries_->append(imuX_, ax);
    imuAySeries_->append(imuX_, ay);
    imuAzSeries_->append(imuX_, az);
    imuTiltSeries_->append(imuX_, tilt_deg);

    if (imuAxSeries_->count() > windowN_) {
        imuAxSeries_->remove(0);
        imuAySeries_->remove(0);
        imuAzSeries_->remove(0);
        imuTiltSeries_->remove(0);
    }

    const int left = std::max(0, imuX_ - windowN_);
    imuAxX_->setRange(left, imuX_);
    ++imuX_;
}

void MainWindow::onPlotTick_() {
    const std::uint64_t outSeq = outputSequence_.load(std::memory_order_relaxed);
    if (outSeq != lastPlottedOutputSequence_) {
        appendOutputPlot_(
            u0_.load(std::memory_order_relaxed),
            u1_.load(std::memory_order_relaxed),
            u2_.load(std::memory_order_relaxed));
        lastPlottedOutputSequence_ = outSeq;
    }

    const std::uint64_t manSeq = manualSequence_.load(std::memory_order_relaxed);
    if (manSeq != lastPlottedManualSequence_) {
        appendManualVoltagePlot_(
            manualTiltVoltageV_.load(std::memory_order_relaxed),
            manualPanVoltageV_.load(std::memory_order_relaxed));
        lastPlottedManualSequence_ = manSeq;
    }

    const std::uint64_t imuSeq = imuSequence_.load(std::memory_order_relaxed);
    if (imuSeq != lastPlottedImuSequence_) {
        appendImuPlot_(
            imuAx_.load(std::memory_order_relaxed),
            imuAy_.load(std::memory_order_relaxed),
            imuAz_.load(std::memory_order_relaxed),
            imuTiltDeg_.load(std::memory_order_relaxed));
        lastPlottedImuSequence_ = imuSeq;
    }

    refreshStatusText_();
}

QPointF MainWindow::clampToImage_(QPointF p, const int w, const int h) {
    p.setX(std::max(0.0, std::min(p.x(), double(w - 1))));
    p.setY(std::max(0.0, std::min(p.y(), double(h - 1))));
    return p;
}

void MainWindow::drawOverlay_(QImage& img) {
    SunEstimate est{};
    PlatformSetpoint sp{};
    bool haveEst = false;
    bool haveSp = false;

    {
        std::lock_guard<std::mutex> lk(ovMtx_);
        if (haveEst_) {
            est = lastEst_;
            haveEst = true;
        }
        if (haveSp_) {
            sp = lastSp_;
            haveSp = true;
        }
    }

    const float manualTiltV = manualTiltVoltageV_.load(std::memory_order_relaxed);
    const float manualPanV = manualPanVoltageV_.load(std::memory_order_relaxed);
    const std::uint64_t manualSeq = manualSequence_.load(std::memory_order_relaxed);

    const float imuAx = imuAx_.load(std::memory_order_relaxed);
    const float imuAy = imuAy_.load(std::memory_order_relaxed);
    const float imuAz = imuAz_.load(std::memory_order_relaxed);
    const float imuTiltDeg = imuTiltDeg_.load(std::memory_order_relaxed);
    const bool imuValid = imuValid_.load(std::memory_order_relaxed);

    // Draw the lightweight overlay on the already converted preview image.
    QPainter p(&img);
    p.setRenderHint(QPainter::Antialiasing, true);

    const int w = img.width();
    const int h = img.height();
    const double cx0 = w * 0.5;
    const double cy0 = h * 0.5;

    double ex = 0.0;
    double ey = 0.0;
    double conf = 0.0;
    double estx = cx0;
    double esty = cy0;

    if (haveEst) {
        estx = est.cx;
        esty = est.cy;
        ex = estx - cx0;
        ey = esty - cy0;
        conf = est.confidence;
    }

    const double tiltDeg = haveSp ? (sp.tilt_rad * 180.0 / 3.14159265358979323846) : 0.0;
    const double panDeg = haveSp ? (sp.pan_rad * 180.0 / 3.14159265358979323846) : 0.0;

    p.setPen(Qt::NoPen);
    p.setBrush(QColor(0, 0, 0, 145));
    p.drawRoundedRect(QRect(10, 10, 900, 170), 8, 8);

    QFont f = p.font();
    f.setPointSize(11);
    f.setBold(true);
    p.setFont(f);

    p.setPen(Qt::white);
    p.drawText(20, 32, QString("mode=%1   thr=%2   conf=%3")
        .arg(manualMode_ ? "MANUAL" : "AUTO")
        .arg(int(thr_.load()))
        .arg(conf, 0, 'f', 3));

    p.setPen(QColor(255, 200, 80));
    p.drawText(20, 58, QString("manual-source=%1")
        .arg(guiManualOwner_ ? "GUI" : "POT"));

    p.setPen(Qt::yellow);
    p.drawText(20, 84, QString("err-x=%1 px   err-y=%2 px")
        .arg(ex, 0, 'f', 1)
        .arg(ey, 0, 'f', 1));

    p.setPen(Qt::green);
    p.drawText(20, 110, QString("tilt=%1 deg   pan=%2 deg")
        .arg(tiltDeg, 0, 'f', 2)
        .arg(panDeg, 0, 'f', 2));

    p.setPen(QColor(255, 170, 0));
    p.drawText(20, 136, QString("ADS1115: tilt_v=%1   pan_v=%2   seq=%3")
        .arg(manualTiltV, 0, 'f', 3)
        .arg(manualPanV, 0, 'f', 3)
        .arg(manualSeq));

    p.setPen(Qt::cyan);
    p.drawText(20, 162, QString("MPU: valid=%1   ax=%2   ay=%3   az=%4   tilt=%5 deg")
        .arg(yesNo(imuValid))
        .arg(imuAx, 0, 'f', 2)
        .arg(imuAy, 0, 'f', 2)
        .arg(imuAz, 0, 'f', 2)
        .arg(imuTiltDeg, 0, 'f', 2));

    QPointF spPt(cx0, cy0);
    QPointF errPt(estx, esty);
    spPt = clampToImage_(spPt, w, h);
    errPt = clampToImage_(errPt, w, h);

    p.setPen(QPen(QColor(0, 255, 0, 255), 3));
    p.drawLine(spPt, errPt);

    p.setBrush(QColor(0, 255, 0, 220));
    p.setPen(Qt::NoPen);
    QRectF spBox(spPt.x() - 18, spPt.y() - 28, 36, 22);
    p.drawRoundedRect(spBox, 5, 5);
    p.setPen(Qt::black);
    p.drawText(spBox, Qt::AlignCenter, "SP");

    if (haveEst) {
        p.setBrush(QColor(255, 255, 0, 220));
        p.setPen(Qt::NoPen);
        QRectF errBox(errPt.x() - 22, errPt.y() - 28, 44, 22);
        p.drawRoundedRect(errBox, 5, 5);
        p.setPen(Qt::black);
        p.drawText(errBox, Qt::AlignCenter, "ERR");

        p.setPen(Qt::NoPen);
        p.setBrush(QColor(255, 255, 0, 255));
        p.drawEllipse(errPt, 4, 4);
    }
}

void MainWindow::setStatus_(const QString& s) {
    statusLabel_->setText(s);
}

void MainWindow::refreshModeUi_() {
    modeLabel_->setText(QString("Mode: %1").arg(manualMode_ ? "MANUAL" : "AUTO"));

    btnAuto_->setEnabled(!closing_ && manualMode_);
    btnManual_->setEnabled(!closing_ && !manualMode_);
}

void MainWindow::refreshManualSourceUi_() {
    manualSourceLabel_->setText(
        QString("Manual source: %1").arg(guiManualOwner_ ? "GUI" : "Pot"));

    const bool enabled = manualMode_ && !closing_;
    btnUsePot_->setEnabled(enabled && guiManualOwner_);
    btnUseGui_->setEnabled(enabled && !guiManualOwner_);
}

void MainWindow::refreshStatusText_() {
    const float manualTiltV = manualTiltVoltageV_.load(std::memory_order_relaxed);
    const float manualPanV = manualPanVoltageV_.load(std::memory_order_relaxed);
    const std::uint64_t manualSeq = manualSequence_.load(std::memory_order_relaxed);

    const float ax = imuAx_.load(std::memory_order_relaxed);
    const float ay = imuAy_.load(std::memory_order_relaxed);
    const float az = imuAz_.load(std::memory_order_relaxed);
    const float tiltDeg = imuTiltDeg_.load(std::memory_order_relaxed);
    const bool valid = imuValid_.load(std::memory_order_relaxed);

    manualLabel_->setText(
        QString("Manual input: tilt_v=%1  pan_v=%2  seq=%3")
            .arg(manualTiltV, 0, 'f', 3)
            .arg(manualPanV, 0, 'f', 3)
            .arg(manualSeq));

    imuLabel_->setText(
        QString("IMU: valid=%1  ax=%2  ay=%3  az=%4  tilt=%5 deg")
            .arg(yesNo(valid))
            .arg(ax, 0, 'f', 2)
            .arg(ay, 0, 'f', 2)
            .arg(az, 0, 'f', 2)
            .arg(tiltDeg, 0, 'f', 2));

    SunEstimate est{};
    PlatformSetpoint sp{};
    bool haveEst = false;
    bool haveSp = false;

    {
        std::lock_guard<std::mutex> lk(ovMtx_);
        if (haveEst_) {
            est = lastEst_;
            haveEst = true;
        }
        if (haveSp_) {
            sp = lastSp_;
            haveSp = true;
        }
    }

    if (manualMode_) {
        setStatus_(QString("MANUAL mode | source=%1 | tilt=%2 deg | pan=%3 deg | A0=%4 V | A1=%5 V")
            .arg(guiManualOwner_ ? "GUI" : "POT")
            .arg(haveSp ? sp.tilt_rad * 180.0 / 3.14159265358979323846 : 0.0, 0, 'f', 2)
            .arg(haveSp ? sp.pan_rad * 180.0 / 3.14159265358979323846 : 0.0, 0, 'f', 2)
            .arg(manualTiltV, 0, 'f', 3)
            .arg(manualPanV, 0, 'f', 3));
        return;
    }

    if (haveEst) {
        setStatus_(QString("AUTO mode | conf=%1 | cx=%2 | cy=%3")
            .arg(est.confidence, 0, 'f', 3)
            .arg(est.cx, 0, 'f', 1)
            .arg(est.cy, 0, 'f', 1));
    } else {
        setStatus_("AUTO mode | no estimate yet");
    }
}

void MainWindow::setManualEnabled_(const bool enabled) {
    pan_->setEnabled(enabled);
    tilt_->setEnabled(enabled);
    btnSend_->setEnabled(enabled);
    btnZero_->setEnabled(enabled);
    if (btnUsePot_) btnUsePot_->setEnabled(enabled && guiManualOwner_);
    if (btnUseGui_) btnUseGui_->setEnabled(enabled && !guiManualOwner_);
}

void MainWindow::sendManual_() {
    // GUI callbacks only post manual requests; control-side work happens in SystemManager.
    if (!manualMode_) {
        return;
    }

    if (!guiManualOwner_) {
        return;
    }

    sys_.setManualCommandSource(SystemManager::ManualCommandSource::Gui);

    const float panRad = deg2rad_(pan_->value());
    const float tiltRad = deg2rad_(tilt_->value());

    sys_.setManualSetpoint(tiltRad, panRad);
}

} // namespace solar