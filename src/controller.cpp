#include "cbs/controller.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <yaml-cpp/yaml.h>

Controller::Controller()
    : running_(false), paused_(false), initialized_(false), timeMultiplier_(1.0f), secondsPerTimestep_(0.5f),
      timeAccumulator_(0.0f), currentTimestep_(0), interpolationAlpha_(0.0f), maxTimestep_(0),
      dimX_(0), dimY_(0), draggedAgentIdx_(-1), isDragging_(false)
{
    // Create simulator
    simulator_ = std::make_unique<Simulator>();
    
    // Initialize last mouse position
    lastMousePosition_ = sf::Vector2i(0, 0);
}

Controller::~Controller() {
    stop();
}

bool Controller::loadMapFromYAML(const std::string& filename) {
    try {
        YAML::Node config = YAML::LoadFile(filename);

        if (!config["map"] || !config["map"]["dimensions"]) {
            std::cerr << "Missing map dimensions in YAML" << std::endl;
            return false;
        }

        const auto& dim = config["map"]["dimensions"];
        dimX_ = dim[0].as<int>();
        dimY_ = dim[1].as<int>();

        if (dimX_ <= 0 || dimY_ <= 0 || dimX_ > 1000 || dimY_ > 1000) {
            std::cerr << "Invalid map dimensions: " << dimX_ << "x" << dimY_ << std::endl;
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
                std::cerr << "Agent has invalid position. Skipping." << std::endl;
                continue;
            }

            // State constructor takes (time, x, y)
            starts_.emplace_back(State(0, sx, sy));
            goals_.emplace_back(Location(gx, gy));
        }

        // Initialize simulator and planner
        if (!setupSimulator() || !initializeVizWindow() || !initializePlanner() || !computeInitialPlan()) {
            return false;
        }
        
        // Mark as fully initialized
        initialized_ = true;
        std::cout << "Map loaded successfully from: " << filename << std::endl;
        return true;

    } catch (const YAML::Exception& e) {
        std::cerr << "Failed to load YAML: " << e.what() << std::endl;
        return false;
    }
}

bool Controller::setupSimulator() {
    if (simulator_) {
        simulator_->setMap(std::vector<std::vector<bool>>(dimY_, std::vector<bool>(dimX_, false)));
        
        for (const auto& obstacle : obstacles_) {
            std::vector<std::vector<bool>> tempMap(dimY_, std::vector<bool>(dimX_, false));
            tempMap[obstacle.y][obstacle.x] = true;
            simulator_->setMap(tempMap);
        }
        
        simulator_->setAgents(starts_, goals_);
        
        return true;
    }
    return false;
}

bool Controller::initializeVizWindow() {
    if (simulator_) {
        int width = std::max(500, dimX_ * 50);
        int height = std::max(300, dimY_ * 50);
        bool success = simulator_->createWindow(width, height, "CBS Simulator");
        initialized_ = success;
        return success;
    }
    return false;
}

bool Controller::initializePlanner() {
    // Initialize the environment and planner
    bool disappearAtGoal = false;
    int spaceSlack = 2;
    int maxNodes = 10000;

    environment_ = std::make_unique<Environment>(dimX_, dimY_, obstacles_, goals_, disappearAtGoal, spaceSlack);
    planner_ = std::make_unique<CBS<State, Action, int, Conflict, Constraints, Environment>>(*environment_, maxNodes);

    return true;
}

void Controller::run() {
    if (!initialized_) {
        std::cerr << "Controller not initialized, cannot run" << std::endl;
        return;
    }

    running_ = true;
    sf::Clock clock;
    
    while (running_ && simulator_ && simulator_->isWindowOpen()) {
        // Process events (includes drag handling)
        processEvents();
        
        // Get elapsed time
        float deltaTime = clock.restart().asSeconds();
        
        // Update simulation logic (taking into account pausing)
        update(deltaTime);
        
        // Render the current state
        if (simulator_) {
            simulator_->render(solution_, currentTimestep_, interpolationAlpha_);
            simulator_->display();
        }
        
        // Short sleep to prevent using 100% CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

bool Controller::update(float deltaTime) {
    static int lastTimestep = -1;
    static int updateCounter = 0;
    
    // Print simulation state on first call
    if (updateCounter == 0) {
        std::cout << "Update starting: maxTimestep=" << maxTimestep_ << std::endl;
        std::cout << "Solution has " << solution_.size() << " agents" << std::endl;
    }
    
    // If we're dragging, don't update the timestep
    if (isDragging_) {
        return true;
    }
    
    // For the stub simulator, manually advance time
    updateCounter++;
    if (!paused_ && updateCounter % 50 == 0) {  // Every ~0.5 seconds (assuming 10ms sleep)
        // Don't advance timestep if maxTimestep is 0 (invalid plan)
        if (maxTimestep_ <= 0) {
            std::cout << "Warning: No valid plan to simulate (maxTimestep=" << maxTimestep_ << ")" << std::endl;
            return true;
        }
        
        // Force timestep advancement
        currentTimestep_++;
        
        // Print timestep information when it changes
        if (currentTimestep_ != lastTimestep) {
            std::cout << "Current timestep: " << currentTimestep_ 
                      << " / " << maxTimestep_ << std::endl;
            lastTimestep = currentTimestep_;
        }
        
        // Check if we've reached the end of the plan
        if (currentTimestep_ >= maxTimestep_) {
            currentTimestep_ = maxTimestep_;
            paused_ = true;  // Auto-pause at the end
            std::cout << "Simulation complete - reached maximum timestep" << std::endl;
            return false;
        }
    }
    
    return true;
}

void Controller::render() {
    // Render the current state of the simulation
    simulator_->render(solution_, currentTimestep_, interpolationAlpha_);
    simulator_->display();
}

void Controller::pause() {
    paused_ = true;
}

void Controller::resume() {
    paused_ = false;
}

void Controller::reset() {
    currentTimestep_ = 0;
    timeAccumulator_ = 0.0f;
    interpolationAlpha_ = 0.0f;
    paused_ = false;
}

void Controller::stop() {
    running_ = false;
}

bool Controller::dragAgent(int agentIdx, int x, int y) {
    // Validate agent index
    if (agentIdx < 0 || agentIdx >= static_cast<int>(starts_.size())) {
        return false;
    }
    
    // Convert to valid grid position
    x = std::max(0, std::min(x, dimX_ - 1));
    y = std::max(0, std::min(y, dimY_ - 1));
    
    // Check if position is valid (not an obstacle)
    if (!simulator_->isValidPosition(x, y)) {
        std::cout << "Cannot move agent to invalid position: (" << x << "," << y << ")" << std::endl;
        simulator_->showMessage("Invalid position (" + std::to_string(x) + "," + std::to_string(y) + ")", 2.0f);
        return false;
    }
    
    // Create updated state - CORRECT ORDER IS (time, x, y)
    State updatedState(0, x, y);
    
    // Update the start position
    starts_[agentIdx] = updatedState;
    
    // Update the solution
    if (agentIdx < static_cast<int>(solution_.size()) && !solution_[agentIdx].states.empty()) {
        // Create a new state list with the updated position
        std::vector<std::pair<State, int>> newStates;
        newStates.emplace_back(updatedState, currentTimestep_);
        solution_[agentIdx].states = newStates;
    }
    
    // Print information about the drag
    std::cout << "Agent " << agentIdx << " moved to position (" << x << "," << y << ")" << std::endl;
    
    // Show a message in the simulator
    if (simulator_) {
        simulator_->showMessage("Agent " + std::to_string(agentIdx) + 
                               " moved to (" + std::to_string(x) + "," + std::to_string(y) + ")", 3.0f);
    }
    
    // Force a render update
    if (simulator_) {
        simulator_->render(solution_, currentTimestep_, interpolationAlpha_);
        simulator_->display();
    }
    
    return true;
}

// For backward compatibility
bool Controller::handleAgentDrag(int agentIdx, int x, int y) {
    return dragAgent(agentIdx, x, y);
}

bool Controller::computeInitialPlan() {
    // Check prerequisites
    if (!environment_ || !planner_) {
        std::cerr << "Environment or planner not initialized" << std::endl;
        return false;
    }
    
    std::cout << "Starting initial planning for " << starts_.size() << " agents" << std::endl;
    
    // Clear any existing solution
    solution_.clear();
    
    // Run the CBS planner
    bool success = planner_->search(starts_, solution_);
    
    if (!success) {
        std::cerr << "Initial planning failed" << std::endl;
        return false;
    }
    
    std::cout << "Initial planning succeeded" << std::endl;
    std::cout << "Solution size: " << solution_.size() << std::endl;
    
    // Find maximum timestep (makespan)
    maxTimestep_ = 0;
    for (const auto& s : solution_) {
        std::cout << "Agent solution has " << s.states.size() << " states" << std::endl;
        if (!s.states.empty()) {
            int agentMaxTime = s.states.back().second;
            std::cout << "Agent max time: " << agentMaxTime << std::endl;
            maxTimestep_ = std::max(maxTimestep_, agentMaxTime);
        }
    }
    
    std::cout << "Makespan (max timesteps): " << maxTimestep_ << std::endl;
    
    // Reset simulation to start state
    // Force our assumption
    currentTimestep_ = 0;
    interpolationAlpha_ = 0.0f;
    timeAccumulator_ = 0.0f;
    
    return true;
}

bool Controller::replanFromCurrentStates(int draggedAgentIdx, State newDraggedState) {
    // This method has been simplified to just display a message
    std::cout << "Replanning is disabled in this version." << std::endl;
    
    if (simulator_) {
        simulator_->showMessage("Replanning is disabled in this version.", 3.0f);
    }
    
    return false;
}

State Controller::getCurrentAgentPosition(size_t agentIdx) const {
    if (agentIdx >= solution_.size()) {
        return State(-1, -1, -1);
    }
    
    State currentPosition(-1, -1, -1);
            
            // Find the state nearest to the current timestep
    for (const auto& [state, time] : solution_[agentIdx].states) {
                if (time <= currentTimestep_) {
            currentPosition = state;
                } else {
                    break;  // We've passed the current timestep
                }
            }
            
    // If no state found, use the start position
    if (currentPosition.x == -1 && currentPosition.y == -1) {
        return agentIdx < starts_.size() ? starts_[agentIdx] : State(-1, -1, -1);
    }
    
    return currentPosition;
}

bool Controller::validateAgentPositions(const std::vector<State>& positions) const {
    // Check for overlapping agents
    for (size_t i = 0; i < positions.size(); i++) {
        for (size_t j = i + 1; j < positions.size(); j++) {
            if (positions[i].x == positions[j].x && positions[i].y == positions[j].y) {
                std::cerr << "Agents " << i << " and " << j << " overlap at ("
                          << positions[i].x << "," << positions[i].y << ")" << std::endl;
                return false;
            }
        }
    }
    
    // Check if any agent is on an obstacle
    for (size_t i = 0; i < positions.size(); i++) {
        if (obstacles_.find(Location(positions[i].x, positions[i].y)) != obstacles_.end()) {
            std::cerr << "Agent " << i << " is on an obstacle at ("
                      << positions[i].x << "," << positions[i].y << ")" << std::endl;
            return false;
        }
    }
    
    return true;
}

void Controller::resetSimulation() {
    currentTimestep_ = 0;
    interpolationAlpha_ = 0.0f;
    timeAccumulator_ = 0.0f;
    
    // Reset to initial state
    std::cout << "Simulation reset" << std::endl;
}

void Controller::processEvents() {
    if (simulator_) {
        simulator_->processEvents();
        
        // Get current mouse position for drag handling
        sf::Vector2i mousePos = simulator_->getMousePosition();
        
        // Check if mouse position changed
        bool mouseMoved = (mousePos.x != lastMousePosition_.x || mousePos.y != lastMousePosition_.y);
        lastMousePosition_ = mousePos;
        
        // Mouse pressed - check for agent selection
        if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
            if (draggedAgentIdx_ < 0) {
                // Not dragging yet - check for agent click
                int agentIdx = -1;
                if (simulator_->checkAgentClick(mousePos.x, mousePos.y, agentIdx, solution_, currentTimestep_)) {
                    // Start dragging the agent
                    draggedAgentIdx_ = agentIdx;
                    isDragging_ = true;
                    // Pause the simulation during drag
                    if (!paused_) {
                        paused_ = true;
                    }
                    std::cout << "Started dragging agent " << agentIdx << std::endl;
                }
            }
            else if (isDragging_ && mouseMoved) {
                // Already dragging and mouse moved - visualize agent at cursor position
                simulator_->drawDraggedAgent(draggedAgentIdx_, mousePos.x, mousePos.y);
            }
        } 
        else if (draggedAgentIdx_ >= 0) {
            // Mouse released after dragging - place agent at the new position
            sf::Vector2i worldPos = simulator_->screenToWorld(mousePos.x, mousePos.y);
            
            // Check if position is valid
            if (simulator_->isValidPosition(worldPos.x, worldPos.y)) {
                // Valid position - finalize the drag by placing the agent
                // CORRECT ORDER IS (time, x, y)
                State updatedState(0, worldPos.x, worldPos.y);
                
                // Update the start position for the agent
                if (draggedAgentIdx_ < static_cast<int>(starts_.size())) {
                    starts_[draggedAgentIdx_] = updatedState;
                    
                    // Also update the solution to reflect the new position
                    if (draggedAgentIdx_ < static_cast<int>(solution_.size()) && !solution_[draggedAgentIdx_].states.empty()) {
                        // Create a new state list with the updated position at the current time
                        std::vector<std::pair<State, int>> newStates;
                        
                        // Add the current position at the current time
                        newStates.emplace_back(updatedState, currentTimestep_);
                        
                        // Replace the solution for this agent
                        solution_[draggedAgentIdx_].states = newStates;
                    }
                    
                    std::cout << "Agent " << draggedAgentIdx_ << " moved to (" 
                              << worldPos.x << "," << worldPos.y << ")" << std::endl;
                              
                    // Show a message in the simulator
                    simulator_->showMessage("Agent " + std::to_string(draggedAgentIdx_) + 
                                          " moved to (" + std::to_string(worldPos.x) + 
                                          "," + std::to_string(worldPos.y) + ")", 3.0f);
                    
                    // Force a re-render to show the updated position
                    simulator_->render(solution_, currentTimestep_, interpolationAlpha_);
                    simulator_->display();
                    
                    // Exit the application after moving an agent
                    std::cout << "Agent moved. Exiting application." << std::endl;
                    stop();
                }
            } else {
                // Invalid position - show error message
                simulator_->showMessage("Invalid position for agent " + 
                                       std::to_string(draggedAgentIdx_), 3.0f);
            }
            
            // Reset drag state
            draggedAgentIdx_ = -1;
            isDragging_ = false;
        }
    }
} 