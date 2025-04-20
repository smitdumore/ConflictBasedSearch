#pragma once

#include <QWidget>
#include <QColor>
#include <QTimer>
#include <QPointF>  // <-- for smooth interpolation drawing
#include <unordered_set>
#include <vector>
#include "common.hpp"
#include "planresult.hpp"

using namespace MultiRobotPlanning;

class Simulator : public QWidget {
    Q_OBJECT

public:
    Simulator(QWidget* parent = nullptr);

    void setMap(const std::vector<std::vector<bool>>& map);
    void setAgents(const std::vector<State>& starts, const std::vector<Location>& goals);
    void visualizeSolution(const std::vector<PlanResult<State, Action, int>>& solution);
    void visualizeTimeStep(const std::vector<PlanResult<State, Action, int>>& solution, int timestep);
    State findStateAtTime(const std::vector<PlanResult<State, Action, int>>& solution, int agentId, int timestep) const;

public slots:
    void startAnimation();
    void stopAnimation();
    void updateTimeStep();

signals:
    void timeStepChanged(int timestep);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    // Drawing utilities
    QColor generateRandomColor() const;
    void drawFlag(QPainter& painter, const QRect& cell, const QColor& color) const;

    // Agent rendering helpers
    State getAgentStateAtTime(size_t agentIdx, int timestep) const;
    QPointF interpolatePosition(const State& start, const State& end, double alpha) const;

    // Map state
    int dimX_, dimY_;
    std::unordered_set<Location> obstacles_;
    std::vector<State> agentStarts_;
    std::vector<Location> agentGoals_;
    std::vector<QColor> agentColors_;

    // Solution data
    std::vector<PlanResult<State, Action, int>> solution_;
    bool hasValidSolution_ = false;
    int currentTimestep_ = 0;
    int maxTimestep_ = 0;

    // Animation control
    QTimer* animationTimer_;
    double interpolationAlpha_ = 0.0;
    const int stepsPerTimestep_ = 5;
    const int animationInterval_ = 100;

    const int cellSize_ = 50;
};
