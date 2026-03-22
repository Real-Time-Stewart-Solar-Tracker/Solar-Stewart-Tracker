#include "MainWindow.hpp"
#include "system/SystemManager.hpp"

#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

#include <QCloseEvent>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QPainter>
#include <QPen>
#include <QPixmap>
#include <QPushButton>
#include <QSlider>
#include <QSplitter>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>

using namespace QtCharts;

namespace solar {
namespace {

bool frameReadableForQt(const std::vector<uint8_t>& data,
                        int width,
                        int height,
                        int strideBytes,
                        PixelFormat format) {
    if (width <= 0 || height <= 0 || strideBytes <= 0) {
        return false;
    }

    const std::size_t bpp = bytesPerPixel(format);
    const std::size_t minStride =
        static_cast<std::size_t>(width) * bpp;

    if (static_cast<std::size_t>(strideBytes) < minStride) {
        return false;
    }

    const std::size_t required =
        static_cast<std::size_t>(strideBytes) *
        static_cast<std::size_t>(height);

    return data.size() >= required;
}

} // namespace

// ------------------------------------------------------------
// Converter worker: converts FrameEvent data into a displayable QImage
// ------------------------------------------------------------
void MainWindow::startConverter_() {
    if (convRunning_.load()) return;

    convRunning_.store(true);
    convThread_ = std::thread(&MainWindow::converterLoop_, this);
}

void MainWindow::stopConverter_() {
    if (!convRunning_.load()) return;

    convRunning_.store(false);
    {
        std::lock_guard<std::mutex> lk(convMutex_);
        convHasFrame_ = true; // wake the worker
    }
    convCv_.notify_all();

    if (convThread_.joinable()) convThread_.join();
}

void MainWindow::converterLoop_() {
    while (convRunning_.load()) {
        std::shared_ptr<std::vector<uint8_t>> buf;
        int w = 0;
        int h = 0;
        int stride = 0;
        PixelFormat format = PixelFormat::Gray8;

        {
            std::unique_lock<std::mutex> lk(convMutex_);
            convCv_.wait(lk, [this] {
                return !convRunning_.load() || convHasFrame_;
            });

            if (!convRunning_.load()) break;

            buf = latestBuf_;
            w = latestW_;
            h = latestH_;
            stride = latestStride_;
            format = latestFormat_;
            convHasFrame_ = false;
        }

        if (!buf || !frameReadableForQt(*buf, w, h, stride, format)) {
            continue;
        }

        QImage img;

        switch (format) {
            case PixelFormat::Gray8: {
                img = QImage(buf->data(),
                             w,
                             h,
                             stride,
                             QImage::Format_Grayscale8).copy();
                break;
            }

            case PixelFormat::RGB888: {
                img = QImage(buf->data(),
                             w,
                             h,
                             stride,
                             QImage::Format_RGB888).copy();
                break;
            }

            case PixelFormat::BGR888: {
                // Qt5 portability: wrap as RGB888 and swap channels into a copied image.
                img = QImage(buf->data(),
                             w,
                             h,
                             stride,
                             QImage::Format_RGB888).rgbSwapped().copy();
                break;
            }
        }

        if (img.isNull()) {
            continue;
        }

        {
            std::lock_guard<std::mutex> lk(imgMutex_);
            latestImg_ = std::move(img);
            imgReady_.store(true);
        }
    }
}

// Qt parent-child ownership: widgets/timers created with a parent are deleted automatically.
// No manual delete required.
MainWindow::MainWindow(solar::SystemManager& sys, QWidget* parent)
    : QMainWindow(parent), sys_(sys)
{
    auto* central = new QWidget(this);
    auto* root = new QVBoxLayout(central);

    // =============================
    // Top row: AUTO / MANUAL / STOP
    // =============================
    auto* row = new QHBoxLayout();
    btnAuto_   = new QPushButton("AUTO", central);
    btnManual_ = new QPushButton("MANUAL", central);
    btnStop_   = new QPushButton("STOP", central);
    btnStop_->setMinimumHeight(48);
    row->addWidget(btnAuto_);
    row->addWidget(btnManual_);
    row->addWidget(btnStop_);
    root->addLayout(row);

    // =============================
    // Status
    // =============================
    status_ = new QLabel("Status: running", central);
    root->addWidget(status_);

    // =============================
    // Threshold controls
    // =============================
    auto* thrRow = new QHBoxLayout();
    thrRow->addWidget(new QLabel("Threshold:", central));

    btnThrMinus_ = new QPushButton("-", central);
    btnThrPlus_  = new QPushButton("+", central);
    thrLabel_    = new QLabel(QString::number(int(thr_.load())), central);
    thrLabel_->setMinimumWidth(50);

    thrRow->addWidget(btnThrMinus_);
    thrRow->addWidget(thrLabel_);
    thrRow->addWidget(btnThrPlus_);
    thrRow->addStretch(1);
    root->addLayout(thrRow);

    // =============================
    // Manual controls
    // =============================
    root->addWidget(new QLabel("Manual setpoint (degrees):", central));

    // Pan
    auto* panRow = new QHBoxLayout();
    panRow->addWidget(new QLabel("Pan (deg)", central));
    pan_ = new QSlider(Qt::Horizontal, central);
    pan_->setRange(-20, 20);
    pan_->setValue(0);
    pan_->setSingleStep(1);
    pan_->setPageStep(5);
    panVal_ = new QLabel("0", central);
    panVal_->setMinimumWidth(40);
    panRow->addWidget(pan_);
    panRow->addWidget(panVal_);
    root->addLayout(panRow);

    // Tilt
    auto* tiltRow = new QHBoxLayout();
    tiltRow->addWidget(new QLabel("Tilt (deg)", central));
    tilt_ = new QSlider(Qt::Horizontal, central);
    tilt_->setRange(-20, 20);
    tilt_->setValue(0);
    tilt_->setSingleStep(1);
    tilt_->setPageStep(5);
    tiltVal_ = new QLabel("0", central);
    tiltVal_->setMinimumWidth(40);
    tiltRow->addWidget(tilt_);
    tiltRow->addWidget(tiltVal_);
    root->addLayout(tiltRow);

    // Manual buttons
    auto* manualBtns = new QHBoxLayout();
    btnSend_ = new QPushButton("SEND", central);
    btnZero_ = new QPushButton("ZERO (0,0)", central);
    manualBtns->addWidget(btnSend_);
    manualBtns->addWidget(btnZero_);
    root->addLayout(manualBtns);

    // ============================================================
    // Splitter area (camera + plot)
    // ============================================================
    auto* splitter = new QSplitter(Qt::Vertical, central);

    // --- Camera container ---
    auto* camContainer = new QWidget(splitter);
    auto* camLayout = new QVBoxLayout(camContainer);
    camLayout->setContentsMargins(0, 0, 0, 0);
    camLayout->addWidget(new QLabel("Camera:", camContainer));

    camView_ = new QLabel(camContainer);
    camView_->setScaledContents(true);
    camView_->setMinimumHeight(320);
    camView_->setStyleSheet("background-color: black;");
    camLayout->addWidget(camView_);

    // --- Plot container ---
    auto* plotContainer = new QWidget(splitter);
    auto* plotLayout = new QVBoxLayout(plotContainer);
    plotLayout->setContentsMargins(0, 0, 0, 0);
    plotLayout->addWidget(new QLabel("Actuator Targets (u0, u1, u2) in degrees:", plotContainer));

    setupPlot_();
    chartView_->setMinimumHeight(360);
    plotLayout->addWidget(chartView_);

    splitter->addWidget(camContainer);
    splitter->addWidget(plotContainer);

    splitter->setStretchFactor(0, 3);
    splitter->setStretchFactor(1, 2);

    root->addWidget(splitter);

    // Window
    setCentralWidget(central);
    resize(1400, 980);
    setWindowTitle("Solar Stewart Tracker");
    setManualEnabled_(false);

    // =============================
    // Subscriptions (store newest, overwrite policy)
    // =============================
    sys_.registerFrameObserver([this](const solar::FrameEvent& fe) {
        if (!frameReadableForQt(fe.data, fe.width, fe.height, fe.stride_bytes, fe.format)) {
            return;
        }

        auto sp = std::make_shared<std::vector<uint8_t>>(fe.data);

        {
            std::lock_guard<std::mutex> lk(convMutex_);
            latestBuf_ = std::move(sp);
            latestW_ = fe.width;
            latestH_ = fe.height;
            latestStride_ = fe.stride_bytes;
            latestFormat_ = fe.format;
            convHasFrame_ = true;
        }
        convCv_.notify_one();
    });

    sys_.registerEstimateObserver([this](const solar::SunEstimate& est) {
        std::lock_guard<std::mutex> lk(ovMtx_);
        lastEst_ = est;
        haveEst_ = true;
    });

    sys_.registerSetpointObserver([this](const solar::PlatformSetpoint& sp) {
        std::lock_guard<std::mutex> lk(ovMtx_);
        lastSp_ = sp;
        haveSp_ = true;
    });

    sys_.registerCommandObserver([this](const solar::ActuatorCommand& cmd) {
        u0_.store(cmd.actuator_targets[0], std::memory_order_relaxed);
        u1_.store(cmd.actuator_targets[1], std::memory_order_relaxed);
        u2_.store(cmd.actuator_targets[2], std::memory_order_relaxed);

        std::lock_guard<std::mutex> lk(ovMtx_);
        lastCmd_ = cmd;
        haveCmd_ = true;
    });

    // Start converter after callbacks are wired
    startConverter_();

    // =============================
    // Timers
    // =============================
    camTimer_ = new QTimer(this);
    connect(camTimer_, &QTimer::timeout, this, [this]() { updateCamera_(); });
    camTimer_->start(33);

    // =============================
    // Buttons
    // =============================
    connect(btnAuto_, &QPushButton::clicked, this, [this]() {
        sys_.exitManual();
        manualMode_ = false;
        setManualEnabled_(false);
        setStatus_("Status: AUTO");
    });

    connect(btnManual_, &QPushButton::clicked, this, [this]() {
        sys_.enterManual();
        manualMode_ = true;
        setManualEnabled_(true);
        setStatus_("Status: MANUAL");
        sendManual_();
    });

    connect(btnStop_, &QPushButton::clicked, this, [this]() {
        if (camTimer_) camTimer_->stop();
        if (plotTimer_) plotTimer_->stop();
        stopConverter_();

        sys_.stop();
        setStatus_("Status: STOPPED");
        btnAuto_->setEnabled(false);
        btnManual_->setEnabled(false);
        btnStop_->setEnabled(false);
        setManualEnabled_(false);
    });

    // Threshold +/- (5 steps)
    connect(btnThrMinus_, &QPushButton::clicked, this, [this]() {
        int t = int(thr_.load());
        t = std::max(0, t - 5);
        thr_.store(static_cast<uint8_t>(t));
        thrLabel_->setText(QString::number(t));
        sys_.setTrackerThreshold(static_cast<uint8_t>(t));
    });

    connect(btnThrPlus_, &QPushButton::clicked, this, [this]() {
        int t = int(thr_.load());
        t = std::min(255, t + 5);
        thr_.store(static_cast<uint8_t>(t));
        thrLabel_->setText(QString::number(t));
        sys_.setTrackerThreshold(static_cast<uint8_t>(t));
    });

    // Sliders live
    connect(pan_, &QSlider::valueChanged, this, [this](int v) {
        panVal_->setText(QString::number(v));
        sendManual_();
    });

    connect(tilt_, &QSlider::valueChanged, this, [this](int v) {
        tiltVal_->setText(QString::number(v));
        sendManual_();
    });

    connect(btnSend_, &QPushButton::clicked, this, [this]() { sendManual_(); });

    connect(btnZero_, &QPushButton::clicked, this, [this]() {
        pan_->setValue(0);
        tilt_->setValue(0);
        sendManual_();
    });
}

MainWindow::~MainWindow() {
    stopConverter_();
}

// ============================================================
// Camera update (UI thread only: overlay + paint)
// ============================================================
void MainWindow::updateCamera_() {
    if (!imgReady_.load()) return;

    QImage img;
    {
        std::lock_guard<std::mutex> lk(imgMutex_);
        img = latestImg_;
    }

    if (img.isNull()) return;

    // Draw overlay on UI thread
    QImage draw = img.copy();
    drawOverlay_(draw);

    camView_->setPixmap(QPixmap::fromImage(draw));
}

// ============================================================
// Plot
// ============================================================
void MainWindow::setupPlot_() {
    s0_ = new QLineSeries();
    s1_ = new QLineSeries();
    s2_ = new QLineSeries();
    s0_->setName("u0");
    s1_->setName("u1");
    s2_->setName("u2");

    chart_ = new QChart();
    chart_->addSeries(s0_);
    chart_->addSeries(s1_);
    chart_->addSeries(s2_);
    chart_->legend()->setVisible(true);
    chart_->setTitle("Servo Commands (degrees)");

    axX_ = new QValueAxis();
    axY_ = new QValueAxis();
    axX_->setTitleText("samples");
    axY_->setTitleText("degrees");
    axY_->setRange(0, 90);

    chart_->addAxis(axX_, Qt::AlignBottom);
    chart_->addAxis(axY_, Qt::AlignLeft);

    s0_->attachAxis(axX_);
    s0_->attachAxis(axY_);
    s1_->attachAxis(axX_);
    s1_->attachAxis(axY_);
    s2_->attachAxis(axX_);
    s2_->attachAxis(axY_);

    chartView_ = new QChartView(chart_);
    chartView_->setMinimumHeight(360);

    plotTimer_ = new QTimer(this);
    connect(plotTimer_, &QTimer::timeout, this, [this]() { onPlotTick_(); });
    plotTimer_->start(33);
}

void MainWindow::onPlotTick_() {
    const float u0 = u0_.load(std::memory_order_relaxed);
    const float u1 = u1_.load(std::memory_order_relaxed);
    const float u2 = u2_.load(std::memory_order_relaxed);

    s0_->append(x_, u0);
    s1_->append(x_, u1);
    s2_->append(x_, u2);

    if (s0_->count() > windowN_) {
        s0_->remove(0);
        s1_->remove(0);
        s2_->remove(0);
    }

    const int left = std::max(0, x_ - windowN_);
    axX_->setRange(left, x_);
    ++x_;
}

// ============================================================
// Overlay
// ============================================================
QPointF MainWindow::clampToImage_(QPointF p, int w, int h) {
    p.setX(std::max(0.0, std::min(p.x(), double(w - 1))));
    p.setY(std::max(0.0, std::min(p.y(), double(h - 1))));
    return p;
}

void MainWindow::drawOverlay_(QImage& img) {
    solar::SunEstimate est{};
    solar::PlatformSetpoint sp{};
    bool haveEst = false;
    bool haveSp = false;

    {
        std::lock_guard<std::mutex> lk(ovMtx_);
        if (haveEst_) { est = lastEst_; haveEst = true; }
        if (haveSp_)  { sp = lastSp_;   haveSp = true; }
    }

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
    const double panDeg  = haveSp ? (sp.pan_rad  * 180.0 / 3.14159265358979323846) : 0.0;

    // info box
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(0, 0, 0, 140));
    p.drawRoundedRect(QRect(10, 10, 560, 95), 8, 8);

    QFont f = p.font();
    f.setPointSize(12);
    f.setBold(true);
    p.setFont(f);

    p.setPen(Qt::white);
    p.drawText(20, 35, QString("thr=%1    conf=%2").arg(int(thr_.load())).arg(conf, 0, 'f', 3));

    p.setPen(Qt::yellow);
    p.drawText(20, 65, QString("er-x=%1 px    er-y=%2 px").arg(ex, 0, 'f', 1).arg(ey, 0, 'f', 1));

    p.setPen(Qt::green);
    p.drawText(20, 95, QString("tilt=%1 deg    pan=%2 deg").arg(tiltDeg, 0, 'f', 2).arg(panDeg, 0, 'f', 2));

    QPointF spPt(cx0, cy0);
    QPointF errPt(estx, esty);
    spPt  = clampToImage_(spPt, w, h);
    errPt = clampToImage_(errPt, w, h);

    // SP box
    p.setBrush(QColor(0, 255, 0, 220));
    p.setPen(Qt::NoPen);
    QRectF spBox(spPt.x() - 18, spPt.y() - 30, 36, 22);
    p.drawRoundedRect(spBox, 5, 5);
    p.setPen(Qt::black);
    p.drawText(spBox, Qt::AlignCenter, "SP");

    // ERR box
    p.setBrush(QColor(255, 255, 0, 220));
    p.setPen(Qt::NoPen);
    QRectF errBox(errPt.x() - 22, errPt.y() - 30, 44, 22);
    p.drawRoundedRect(errBox, 5, 5);
    p.setPen(Qt::black);
    p.drawText(errBox, Qt::AlignCenter, "ERR");

    // Arrow
    p.setPen(QPen(QColor(0, 255, 0, 255), 3));
    p.drawLine(spPt, errPt);

    // Arrow head
    const QPointF v = errPt - spPt;
    const double len = std::hypot(v.x(), v.y());
    if (len > 1.0) {
        const double ux = v.x() / len;
        const double uy = v.y() / len;

        QPointF tip = errPt;
        QPointF left (tip.x() - 14 * ux + 7 * uy, tip.y() - 14 * uy - 7 * ux);
        QPointF right(tip.x() - 14 * ux - 7 * uy, tip.y() - 14 * uy + 7 * ux);

        QPolygonF head;
        head << tip << left << right;
        p.setBrush(QColor(0, 255, 0, 255));
        p.drawPolygon(head);
    }

    if (haveEst) {
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(255, 255, 0, 255));
        p.drawEllipse(errPt, 4, 4);
    }
}

// ============================================================
// Helpers
// ============================================================
void MainWindow::closeEvent(QCloseEvent* e) {
    if (camTimer_) camTimer_->stop();
    if (plotTimer_) plotTimer_->stop();
    stopConverter_();
    sys_.stop();
    e->accept();
}

void MainWindow::setStatus_(const QString& s) {
    status_->setText(s);
}

void MainWindow::setManualEnabled_(bool en) {
    pan_->setEnabled(en);
    tilt_->setEnabled(en);
    btnSend_->setEnabled(en);
    btnZero_->setEnabled(en);
}

void MainWindow::sendManual_() {
    if (!manualMode_) return;

    const int panDeg  = pan_->value();
    const int tiltDeg = tilt_->value();

    sys_.setManualSetpoint(deg2rad_(tiltDeg), deg2rad_(panDeg));

    setStatus_(QString("Status: MANUAL (pan=%1°, tilt=%2°)")
                   .arg(panDeg)
                   .arg(tiltDeg));
}

} // namespace solar