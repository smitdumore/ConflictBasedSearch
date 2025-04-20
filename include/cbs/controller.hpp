#pragma once

#include <QObject>
#include <QString>
#include <vector>
#include <unordered_set>
#include "cbs/cbs.hpp"

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

    // Accessors for planner
    const std::vector<State>& getAgentStarts() const;
    const std::vector<Location>& getGoals() const;
    const std::unordered_set<Location>& getObstacles() const;
    int getDimX() const;
    int getDimY() const;

private:
    // Map + agent data
    int dimX_, dimY_;
    std::unordered_set<Location> obstacles_;
    std::vector<Location> goals_;
    std::vector<State> starts_;

    // CBS Planner (with default parameters)
    std::unique_ptr<Environment> environment_;
    std::unique_ptr<CBS<State, Action, int, Conflict, Constraints, Environment>> planner_;
    
    Simulator* simulator_;

    // Flags and errors
    bool mapChanged_;
    QString lastUIError_;
};
