#pragma once

#include <QWidget>
#include <QColor>
#include <unordered_set>
#include <vector>
#include "common.hpp"

class Simulator : public QWidget {
    Q_OBJECT

public:
    Simulator(QWidget* parent = nullptr);

    void setMap(int dimX, int dimY, const std::unordered_set<Location>& obstacles);
    void setAgents(const std::vector<State>& starts, const std::vector<Location>& goals);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QColor generateRandomColor() const;
    void drawFlag(QPainter& painter, const QRect& cell, const QColor& color) const;

    int dimX_, dimY_;
    std::unordered_set<Location> obstacles_;
    std::vector<State> agentStarts_;
    std::vector<Location> agentGoals_;
    std::vector<QColor> agentColors_;
    const int cellSize_ = 50;
};
