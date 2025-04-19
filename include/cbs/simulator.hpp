#pragma once

#include <QWidget>
#include <unordered_set>
#include <vector>
#include "common.hpp"

class Simulator : public QWidget {
    Q_OBJECT

public:
    Simulator(QWidget* parent = nullptr);

    void setMap(int dimX, int dimY, const std::unordered_set<Location>& obstacles);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    int dimX_, dimY_;
    std::unordered_set<Location> obstacles_;
    const int cellSize_ = 50;
};
