#pragma once

#include <QObject>
#include <QString>
#include <vector>
#include <memory>
#include <unordered_set>
#include "cbs/cbs.hpp"

using namespace MultiRobotPlanning;

class Simulator;  // Forward declaration

class Controller : public QObject {
    Q_OBJECT

public:
    Controller(QObject* parent = nullptr);
    
    bool loadMapFromYAML(const QString& filename);
    void connectSimulator(Simulator* sim);
    
    void triggerPlanner();
    void notifyMapChanged();
    void notifyUIFailure(const QString& reason);
    void visualizeTimeStep(const std::vector<PlanResult<State, Action, int>>& solution, int timestep);
    State getCurrentState(int agentId) const;

    // Accessors for planner
    const std::vector<State>& getAgentStarts() const;
    const std::vector<Location>& getGoals() const;
    const std::unordered_set<Location>& getObstacles() const;
    int getDimX() const;
    int getDimY() const;

private:
    // Map + agent data
    Simulator* simulator_;
    std::vector<State> starts_;
    std::vector<Location> goals_;
    std::unordered_set<Location> obstacles_;
    int dimX_;
    int dimY_;
    bool mapChanged_;
    QString lastUIError_;
    std::unique_ptr<Environment> environment_;
    std::unique_ptr<CBS<State, Action, int, Conflict, Constraints, Environment>> planner_;
    std::vector<PlanResult<State, Action, int>> solution_;
    int currentTimestep_;
};
