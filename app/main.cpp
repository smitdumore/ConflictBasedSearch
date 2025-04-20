#include <QApplication>
#include "cbs/simulator.hpp"
#include "cbs/controller.hpp"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    QString mapPath = "../maps/corridor_test.yaml";

    Simulator* sim = new Simulator();
    Controller* controller = new Controller();

    controller->loadMapFromYAML(mapPath);
    controller->connectSimulator(sim);
    controller->runLoop();
 
    sim->show(); // Simulator is a QWidget  // QWidget::show()
    return app.exec();
}
