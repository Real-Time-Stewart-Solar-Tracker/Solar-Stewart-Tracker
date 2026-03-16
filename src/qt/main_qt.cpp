#include "app/AppConfig.hpp"
#include "common/Logger.hpp"
#include "MainWindow.hpp"

#include <QApplication>

int main(int argc, char** argv) {
    QApplication app(argc, argv);

    solar::Logger log;
    solar::app::AppConfig cfg = solar::app::defaultConfig();
    cfg.sensor_mode = solar::app::SensorMode::CAMERA; // Qt GUI defaults to camera mode

    solar::MainWindow w(log, cfg);
    w.show();

    return app.exec();
}
