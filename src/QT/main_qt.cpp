#include "app/AppConfig.hpp"
#include "app/SystemFactory.hpp"
#include "common/Logger.hpp"
#include "MainWindow.hpp"

#include <QApplication>

int main(int argc, char** argv) {
    QApplication app(argc, argv);

    solar::Logger log;
    const solar::app::AppConfig cfg = solar::app::defaultConfig();

    auto sys = solar::app::SystemFactory::makeSystem(log, cfg);
    if (!sys || !sys->start()) {
        log.error("Failed to start SystemManager");
        return 1;
    }

    solar::MainWindow w(*sys);
    w.show();

    const int rc = app.exec();

    sys->stop();
    return rc;
}
