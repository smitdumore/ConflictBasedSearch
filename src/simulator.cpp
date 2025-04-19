#include <QPainter>

#include "cbs/simulator.hpp"

Simulator::Simulator(QWidget* parent)
    : QWidget(parent), dimX_(0), dimY_(0) {
    setWindowTitle("CBS Simulator");
}

void Simulator::setMap(int dimX, int dimY, const std::unordered_set<Location>& obstacles) {
    if (dimX <= 0 || dimY <= 0 || dimX > 1000 || dimY > 1000) {
        qWarning("Simulator::setMap received invalid grid size: (%d, %d)", dimX, dimY);
        return;
    }

    dimX_ = dimX;
    dimY_ = dimY;
    obstacles_ = obstacles;

    setFixedSize(dimX_ * cellSize_, dimY_ * cellSize_);
    update();
}


void Simulator::paintEvent(QPaintEvent*) {
    QPainter painter(this);

    for (int y = 0; y < dimY_; ++y) {
        for (int x = 0; x < dimX_; ++x) {
            QRect cell(x * cellSize_, y * cellSize_, cellSize_, cellSize_);
            painter.setPen(Qt::gray);

            if (obstacles_.count(Location(x, y))) {
                painter.setBrush(Qt::black);
            } else {
                painter.setBrush(Qt::white);
            }

            painter.drawRect(cell);
        }
    }
}

