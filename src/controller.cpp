#include "cbs/controller.hpp"
#include "cbs/simulator.hpp"
#include <yaml-cpp/yaml.h>
#include <QDebug>
#include <QThread>
#include <QTimer>
#include <QApplication>

Controller::Controller(QObject* parent)
    : QObject(parent), simulator_(nullptr), mapChanged_(false), currentTimestep_(0) {}

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

void Controller::runLoop() {
    if (!planner_ || !simulator_) return;

    // Initial planning
    solution_.clear();
    bool success = planner_->search(starts_, solution_);

    if (!success) {
        qDebug() << "Initial planning failed";
        return;
    }

    // Find makespan (max time)
    int makespan = 0;
    for (const auto& s : solution_) {
        makespan = std::max<int>(makespan, s.cost);
    }

    // Send solution to simulator and start animation
    simulator_->visualizeSolution(solution_);
    simulator_->startAnimation();

    // Connect to simulator's animation timer for replanning
    //connect(simulator_, &Simulator::timeStepChanged, this, [this](int timestep) {
        //currentTimestep_ = timestep;
        
        // Check if replanning needed
        // if (mapChanged_) {
        //     // Replan from current states
        //     std::vector<State> currentStates;
        //     for (size_t i = 0; i < solution_.size(); i++) {
        //         currentStates.push_back(getCurrentState(i));
        //     }

        //     // Try replanning
        //     std::vector<PlanResult<State, Action, int>> newSolution;
        //     if (planner_->search(currentStates, newSolution)) {
        //         solution_ = newSolution;
        //         simulator_->visualizeSolution(solution_);
        //         simulator_->startAnimation();
        //     }
        //     mapChanged_ = false;
        // }
    //});
}

State Controller::getCurrentState(int agentId) const {
    if (!simulator_) return State(-1, -1, -1);
    return simulator_->findStateAtTime(solution_, agentId, currentTimestep_);
}

void Controller::connectSimulator(Simulator* sim) {
    simulator_ = sim;

    if (dimX_ <= 0 || dimY_ <= 0 || dimX_ > 1000 || dimY_ > 1000) {
        qWarning("Refusing to set simulator map: invalid dimensions (%d, %d)", dimX_, dimY_);
        return;
    }

    // Convert obstacles to 2D boolean grid
    std::vector<std::vector<bool>> map(dimY_, std::vector<bool>(dimX_, false));
    for (const auto& obs : obstacles_) {
        if (obs.x >= 0 && obs.x < dimX_ && obs.y >= 0 && obs.y < dimY_) {
            map[obs.y][obs.x] = true;
        }
    }

    simulator_->setMap(map);
    simulator_->setAgents(starts_, goals_);
    
    // Connect the agentDragged signal to our handler
    connect(simulator_, &Simulator::agentDragged, this, &Controller::onAgentDragged);
    connect(simulator_, &Simulator::timeStepChanged, this, [this](int timestep) {
        currentTimestep_ = timestep;
    });
}

// Slot for handling agent dragging
void Controller::onAgentDragged(int agentIdx, State newState) {
    qDebug() << "Agent" << agentIdx << "dragged to position (" 
             << newState.x << "," << newState.y << ") at time" << currentTimestep_;
    
    // Store the current timestep for reference
    int dragTimestep = currentTimestep_;
    
    // Stop the animation while we replan
    simulator_->stopAnimation();
    
    // Replan with the new agent position
    if (replanFromCurrentStates(agentIdx, newState)) {
        qDebug() << "Successfully replanned after agent drag";
        
        // Update the simulator with the new solution - this will adjust the timestep
        simulator_->visualizeSolution(solution_);
        
        // Start animation from the current timestep
        simulator_->startAnimation();
    } else {
        qDebug() << "Failed to replan after agent drag";
        
        // Show warning message to the user
        simulator_->showReplanningWarning("Failed to find a new path. Agent returned to original position.");
        
        // Resume animation from the original position
        simulator_->startAnimation();
    }
}

bool Controller::replanFromCurrentStates(int draggedAgentIdx, State newDraggedState) {
    if (!planner_ || !simulator_) {
        qDebug() << "Can't replan: planner or simulator not initialized";
        return false;
    }
    
    qDebug() << "===== Starting Replanning =====";
    qDebug() << "Current timestep:" << currentTimestep_;
    qDebug() << "Dragged agent:" << draggedAgentIdx << "to position (" 
             << newDraggedState.x << "," << newDraggedState.y << ")";
    
    // The key issue: When we replan, we need to:
    // 1. Use time=0 for all new start states (the planner expects this)
    // 2. Ensure we're using the exact current positions

    // Create a new vector for start states
    std::vector<State> newStartStates;
    
    // Get current positions of all agents
    for (size_t i = 0; i < solution_.size(); i++) {
        if (static_cast<int>(i) == draggedAgentIdx) {
            // Use the new position for the dragged agent
            int x = std::max(0, std::min(newDraggedState.x, dimX_ - 1));
            int y = std::max(0, std::min(newDraggedState.y, dimY_ - 1));
            newStartStates.push_back(State(0, x, y));  // Always use time=0 for planning
            qDebug() << "Agent" << i << "(dragged) replanning from (" << x << "," << y << ")";
        } else {
            // For other agents, get current position directly from the solution at current timestep
            State current(-1, -1, -1);
            
            // Find the state nearest to the current timestep
            for (const auto& [state, time] : solution_[i].states) {
                if (time <= currentTimestep_) {
                    current = state;
                } else {
                    break;  // We've passed the current timestep
                }
            }
            
            // Sanity check - if we couldn't find a state, use the first one
            if (current.x == -1 && current.y == -1) {
                if (!solution_[i].states.empty()) {
                    current = solution_[i].states.front().first;
                } else {
                    current = starts_[i];  // Fallback to original start
                }
            }
            
            // Create a new state with time=0 for planning
            newStartStates.push_back(State(0, current.x, current.y));
            qDebug() << "Agent" << i << "replanning from (" << current.x << "," << current.y << ")";
        }
    }
    
    // Verify no agents are at the same position
    for (size_t i = 0; i < newStartStates.size(); i++) {
        for (size_t j = i + 1; j < newStartStates.size(); j++) {
            if (newStartStates[i].x == newStartStates[j].x && newStartStates[i].y == newStartStates[j].y) {
                qDebug() << "Replanning failed: Agents" << i << "and" << j 
                         << "are at the same position (" 
                         << newStartStates[i].x << "," << newStartStates[i].y << ")";
                return false;
            }
        }
    }
    
    // Ensure no agents are on obstacles
    for (size_t i = 0; i < newStartStates.size(); i++) {
        if (obstacles_.find(Location(newStartStates[i].x, newStartStates[i].y)) != obstacles_.end()) {
            qDebug() << "Replanning failed: Agent" << i << "is on an obstacle at (" 
                     << newStartStates[i].x << "," << newStartStates[i].y << ")";
            return false;
        }
    }
    
    // Create a fresh environment and planner with the same goals
    bool disappearAtGoal = false;
    int spaceSlack = 2;
    int maxNodes = 10000;
    
    // Re-initialize the environment with the same settings but current positions
    environment_ = std::make_unique<Environment>(dimX_, dimY_, obstacles_, goals_, disappearAtGoal, spaceSlack);
    planner_ = std::make_unique<CBS<State, Action, int, Conflict, Constraints, Environment>>(*environment_, maxNodes);
    
    // Try to plan with the new start positions
    std::vector<PlanResult<State, Action, int>> newSolution;
    qDebug() << "Replanning paths from current positions...";
    bool success = planner_->search(newStartStates, newSolution);
    
    if (success) {
        qDebug() << "Successfully replanned!";
        solution_ = newSolution;
        return true;
    }
    
    qDebug() << "Failed to replan!";
    return false;
}

// Accessors
const std::vector<State>& Controller::getAgentStarts() const { return starts_; }
const std::vector<Location>& Controller::getGoals() const { return goals_; }
const std::unordered_set<Location>& Controller::getObstacles() const { return obstacles_; }
int Controller::getDimX() const { return dimX_; }
int Controller::getDimY() const { return dimY_; }
