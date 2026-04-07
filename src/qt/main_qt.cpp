#include "app/AppConfig.hpp"
#include "app/SystemFactory.hpp"
#include "common/Logger.hpp"
#include "qt/MainWindow.hpp"
#include "system/SystemManager.hpp"

#include <QApplication>

// Qt entry point kept intentionally small.
// Runtime composition belongs in typed configuration helpers and factories.
int main(int argc, char* argv[]) {
    QApplication app(argc, argv);

    solar::Logger log;
    const solar::app::AppConfig cfg = solar::app::defaultQtConfig();

    auto system = solar::app::SystemFactory::makeSystem(log, cfg);
    if (!system) {
        return 1;
    }

    if (!system->start()) {
        return 1;
    }

    solar::MainWindow window(*system);
    window.show();

    const int rc = app.exec();

    system->stop();
    return rc;
}