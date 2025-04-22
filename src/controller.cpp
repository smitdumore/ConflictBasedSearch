#include "cbs/controller.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>

Controller::Controller()
    : running_(false), paused_(false), timeMultiplier_(1.0f), secondsPerTimestep_(0.5f),
      timeAccumulator_(0.0f), currentTimestep_(0), interpolationAlpha_(0.0f), maxTimestep_(0),
      dimX_(0), dimY_(0), draggedAgentIdx_(-1)
{
    // Create simulator
    simulator_ = std::make_unique<Simulator>();
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

            starts_.emplace_back(State(0, sx, sy));
            goals_.emplace_back(Location(gx, gy));
        }

        std::cout << "Map loaded successfully from: " << filename << std::endl;
        return true;

    } catch (const YAML::Exception& e) {
        std::cerr << "Failed to load YAML: " << e.what() << std::endl;
        return false;
    }
}

bool Controller::setupSimulator() {
    // Create 2D map for simulator
    std::vector<std::vector<bool>> mapGrid(dimY_, std::vector<bool>(dimX_, false));
    for (const auto& obs : obstacles_) {
        if (obs.x >= 0 && obs.x < dimX_ && obs.y >= 0 && obs.y < dimY_) {
            mapGrid[obs.y][obs.x] = true;
        }
    }

    // Set map and agents in simulator
    simulator_->setMap(mapGrid);
    simulator_->setAgents(starts_, goals_);
}

bool Controller::initializeVizWindow() {
    // Create a window for visualization
    int windowWidth = dimX_ * 50; // 50 pixels per cell
    int windowHeight = dimY_ * 50;
    bool windowCreated = simulator_->createWindow(windowWidth, windowHeight, "CBS Simulator");
    
    if (!windowCreated) {
        return false;
    }
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
    if (!running_) {
        running_ = true;
        paused_ = false;
        
        // Main simulation loop
        auto lastTime = std::chrono::high_resolution_clock::now();
        
        while (running_ && simulator_->isWindowOpen()) {
            // Process window events
            processEvents();
            
            // Calculate delta time
            auto currentTime = std::chrono::high_resolution_clock::now();
            float deltaTime = std::chrono::duration<float>(currentTime - lastTime).count();
            lastTime = currentTime;
            
            // Update simulation
            if (!paused_) {
                update(deltaTime); // advances currentTimestep 
            }
            
            // Render current state
            render();
            
            // Keep a consistent frame rate
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
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
    
    // For the stub simulator, manually advance time
    updateCounter++;
    if (updateCounter % 50 == 0) {  // Every ~0.5 seconds (assuming 10ms sleep)
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
    
    // Print information about the drag
    std::cout << "Drag recorded: Agent " << agentIdx << " to position (" << x << "," << y << ")" << std::endl;
    
    // Show a message in the Controller instead of attempting to replan
    if (simulator_) {
        simulator_->showMessage("Drag detected: Agent " + std::to_string(agentIdx) + 
                               " → (" + std::to_string(x) + "," + std::to_string(y) + ")", 3.0f);
    }
    
    // Return success without actually replanning
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
        
        // Check for mouse clicks
        if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
            sf::Vector2i mousePos = simulator_->getMousePosition();
            int agentIdx = -1;
            
            if (simulator_->checkAgentClick(mousePos.x, mousePos.y, agentIdx, solution_, currentTimestep_)) {
                // Start dragging the agent
                draggedAgentIdx_ = agentIdx;
                std::cout << "Started dragging agent " << agentIdx << std::endl;
            }
        } else if (draggedAgentIdx_ >= 0) {
            // If mouse released after dragging, finalize the drag
            sf::Vector2i mousePos = simulator_->getMousePosition();
            sf::Vector2i worldPos = simulator_->screenToWorld(mousePos.x, mousePos.y);
            
            // Attempt to drag the agent to the new position
            dragAgent(draggedAgentIdx_, worldPos.x, worldPos.y);
            
            // Reset dragged agent
            draggedAgentIdx_ = -1;
        }
    }
} 