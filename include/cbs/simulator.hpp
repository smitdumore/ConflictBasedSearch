#pragma once

#include <QWidget>
#include <QColor>
#include <QTimer>
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

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QColor generateRandomColor() const;
    void drawFlag(QPainter& painter, const QRect& cell, const QColor& color) const;
    State getAgentStateAtTime(size_t agentIdx, int timestep) const;

    int dimX_, dimY_;
    std::unordered_set<Location> obstacles_;
    std::vector<State> agentStarts_;
    std::vector<Location> agentGoals_;
    std::vector<QColor> agentColors_;
    
    // Solution visualization
    std::vector<PlanResult<State, Action, int>> solution_;
    bool hasValidSolution_;
    int currentTimestep_;
    int maxTimestep_;
    
    // Animation control
    QTimer* animationTimer_;
    const int animationInterval_ = 500; // milliseconds between steps
    
    const int cellSize_ = 50;
};
