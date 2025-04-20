#include "cbs/controller.hpp"
#include "cbs/simulator.hpp"
#include <yaml-cpp/yaml.h>
#include <QDebug>

Controller::Controller(QObject* parent)
    : QObject(parent), simulator_(nullptr), mapChanged_(false) {}

bool Controller::loadMapFromYAML(const QString& filename) {
    try {
        YAML::Node config = YAML::LoadFile(filename.toStdString());

        if (!config["map"] || !config["map"]["dimensions"]) {
            qDebug() << "Missing map dimensions in YAML";
            return false;
        }

        const auto& dim = config["map"]["dimensions"];
        dimX_ = dim[0].as<int>();
        dimY_ = dim[1].as<int>();

        if (dimX_ <= 0 || dimY_ <= 0 || dimX_ > 1000 || dimY_ > 1000) {
            qDebug() << "Invalid map dimensions:" << dimX_ << "x" << dimY_;
            return false;
        }

        obstacles_.clear();
        for (const auto& node : config["map"]["obstacles"]) {
            if (node.size() != 2) continue;
            int x = node[0].as<int>();
            int y = node[1].as<int>();
            if (x >= 0 && x < dimX_ && y >= 0 && y < dimY_)
                obstacles_.insert(Location(x, y));
        }

        starts_.clear();
        goals_.clear();
        for (const auto& agent : config["agents"]) {
            const auto& start = agent["start"];
            const auto& goal = agent["goal"];
            if (start.size() != 2 || goal.size() != 2) continue;

            int sx = start[0].as<int>(), sy = start[1].as<int>();
            int gx = goal[0].as<int>(), gy = goal[1].as<int>();

            if (sx < 0 || sx >= dimX_ || sy < 0 || sy >= dimY_ ||
                gx < 0 || gx >= dimX_ || gy < 0 || gy >= dimY_) {
                qDebug() << "Agent has invalid position. Skipping.";
                continue;
            }

            starts_.emplace_back(State(0, sx, sy));
            goals_.emplace_back(Location(gx, gy));
        }

        // Initialize the CBS planner
        bool disappearAtGoal = false;  // You can make this configurable if needed
        int spaceSlack = 2;  // You can make this configurable if needed
        int maxNodes = 10000;  // You can make this configurable if needed

        environment_ = std::make_unique<Environment>(dimX_, dimY_, obstacles_, goals_, disappearAtGoal, spaceSlack);
        planner_ = std::make_unique<CBS<State, Action, int, Conflict, Constraints, Environment>>(*environment_, maxNodes);

        qDebug() << "Map loaded successfully from:" << filename;
        return true;

    } catch (const YAML::Exception& e) {
        qDebug() << "Failed to load YAML:" << e.what();
        return false;
    }
}

void Controller::notifyMapChanged() {
    mapChanged_ = true;
}

void Controller::notifyUIFailure(const QString& reason) {
    lastUIError_ = reason;
    qDebug() << "UI failure reported:" << reason;
}

void Controller::triggerPlanner() {
    if (!planner_ || !simulator_) return;

    std::vector<PlanResult<State, Action, int>> solution;
    bool success = planner_->search(starts_, solution);

    if (!success) {
        qDebug() << "Planner failed to generate a plan.";
        return;
    }

    // Calculate statistics
    int cost = 0;
    int makespan = 0;
    for (const auto& s : solution) {
        cost += s.cost;
        makespan = std::max<int>(makespan, s.cost);
    }
    qDebug() << "Planning successful! Cost:" << cost << "Makespan:" << makespan;

    // TODO: Add a method in Simulator to visualize the solution
    // simulator_->visualizeSolution(solution);
    mapChanged_ = false;
}

void Controller::connectSimulator(Simulator* sim) {
    simulator_ = sim;

    if (dimX_ <= 0 || dimY_ <= 0 || dimX_ > 1000 || dimY_ > 1000) {
        qWarning("Refusing to set simulator map: invalid dimensions (%d, %d)", dimX_, dimY_);
        return;
    }

    simulator_->setMap(dimX_, dimY_, obstacles_);
    simulator_->setAgents(starts_, goals_);
}

// Accessors
const std::vector<State>& Controller::getAgentStarts() const { return starts_; }
const std::vector<Location>& Controller::getGoals() const { return goals_; }
const std::unordered_set<Location>& Controller::getObstacles() const { return obstacles_; }
int Controller::getDimX() const { return dimX_; }
int Controller::getDimY() const { return dimY_; }
