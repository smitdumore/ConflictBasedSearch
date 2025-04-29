#pragma once

#include <vector>
#include <memory>
#include <string>
#include <chrono>
#include "common.hpp"
#include "planresult.hpp"
#include "environment.hpp"
#include "cbs.hpp"
#include "simulator.hpp"
#include <yaml-cpp/yaml.h>
#include <SFML/Graphics.hpp>

/**
 * The Controller class manages the core simulation loop and state.
 * It coordinates between the planner (CBS) and visualization (Simulator).
 */
class Controller {
public:
    Controller();
    ~Controller();

    // Setup
    bool loadMapFromYAML(const std::string& filename);
    bool initializePlanner();
    bool initializeVizWindow();
    bool computeInitialPlan();
    bool setupSimulator();
    
    // Main loop
    void run();
    bool update(float deltaTime);
    void render();
    
    // Control
    void pause();
    void resume();
    void reset();
    void stop();
    bool isRunning() const { return running_; }
    bool isPaused() const { return paused_; }
    
    // Time control
    void setTimeMultiplier(float multiplier) { timeMultiplier_ = multiplier; }
    float getTimeMultiplier() const { return timeMultiplier_; }
    
    // Add these methods for seconds per timestep
    void setSecondsPerTimestep(float seconds) { secondsPerTimestep_ = seconds; }
    float getSecondsPerTimestep() const { return secondsPerTimestep_; }
    
    // Timestep accessors
    int getCurrentTimestep() const { return currentTimestep_; }
    int getMaxTimestep() const { return maxTimestep_; }
    
    // Agent interaction
    bool dragAgent(int agentIdx, int x, int y);
    
private:
    // Simulation state
    bool running_;
    bool paused_;
    bool initialized_;
    float timeMultiplier_;
    float secondsPerTimestep_;
    float timeAccumulator_;
    
    // Timestep tracking
    int currentTimestep_;
    float interpolationAlpha_;
    int maxTimestep_;
    
    // Map data
    int dimX_;
    int dimY_;
    std::unordered_set<Location> obstacles_;
    std::vector<State> starts_;
    std::vector<Location> goals_;
    
    // Slack control
    int defaultSpaceSlack_;
    int AllowableSlackTolerance; 
    
    // Planning components
    std::unique_ptr<Environment> environment_;
    std::unique_ptr<CBS<State, Action, int, Conflict, Constraints, Environment>> planner_;
    std::vector<PlanResult<State, Action, int>> solution_;
    
    // Visualization
    std::unique_ptr<Simulator> simulator_;
    
    // Agent handling
    int draggedAgentIdx_;
    
    // Agent slack tracking
    std::vector<int> agentSlack_;  // Tracks remaining slack for each agent
    
    // Current drag state
    sf::Vector2i lastMousePosition_;
    bool isDragging_;
    
    // Methods
    bool replanFromCurrentStates(int draggedAgentIdx = -1, State newDraggedState = State(-1, -1, -1));
    State getCurrentAgentPosition(size_t agentIdx) const;
    bool validateAgentPositions(const std::vector<State>& positions) const;
    void resetSimulation();
    void processEvents();
    bool handleAgentDrag(int agentIdx, int x, int y);
    bool checkCollisionAtPosition(int agentIdx, int x, int y, int timestep, int& collidingAgent) const;
}; 