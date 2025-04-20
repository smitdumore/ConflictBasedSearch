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
    
    // Methods for drag and drop
    bool isDraggingAgent() const { return draggedAgentIdx_ >= 0; }
    int getDraggedAgentIdx() const { return draggedAgentIdx_; }
    State getDraggedAgentState() const;
    
    // Method to show a warning message when replanning fails
    void showReplanningWarning(const QString& message);

public slots:
    void startAnimation();
    void stopAnimation();
    void updateTimeStep();

signals:
    void timeStepChanged(int timestep);
    void agentDragged(int agentIdx, State newState);  // Signal for agent dragging

protected:
    void paintEvent(QPaintEvent* event) override;
    
    // Mouse event handlers for dragging
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    // Drawing utilities
    QColor generateRandomColor() const;
    void drawFlag(QPainter& painter, const QRect& cell, const QColor& color) const;

    // Agent rendering helpers
    State getAgentStateAtTime(size_t agentIdx, int timestep) const;
    QPointF interpolatePosition(const State& start, const State& end, double alpha) const;
    
    // Helper methods for agent dragging
    int findAgentAtPosition(const QPoint& pos) const;
    State gridPositionToState(const QPointF& pos) const;
    bool isValidPosition(int x, int y) const;

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
    const int stepsPerTimestep_ = 20;
    const int animationInterval_ = 150;

    // Drag and drop state
    int draggedAgentIdx_ = -1;   // -1 means no agent is being dragged
    QPointF dragOffset_;         // Offset from agent center to drag point
    State draggedAgentOrigState_; // Original state before dragging
    
    // Warning message state
    QString warningMessage_;
    QTimer* warningTimer_;

    const int cellSize_ = 50;
};
