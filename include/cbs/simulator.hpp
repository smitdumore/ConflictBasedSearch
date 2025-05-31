#pragma once

#include <SFML/Graphics.hpp>
#include <unordered_set>
#include <vector>
#include <string>
#include "common.hpp"
#include "planresult.hpp"

using namespace MultiRobotPlanning;

/**
 * The Simulator class handles visualization of the map, agents, and paths.
 * It uses SFML for rendering and provides a simple interface for the main simulation loop.
 */
class Simulator {
public:
    Simulator();
    ~Simulator();

    // Window management
    bool createWindow(int width, int height, const std::string& title);
    void closeWindow();
    bool isWindowOpen() const;
    void processEvents();
    sf::RenderWindow& getWindow() { return window_; }
    
    // Map and agent setup
    void setMap(const std::vector<std::vector<bool>>& map);
    void setAgents(const std::vector<State>& starts, const std::vector<Location>& goals);
    
    // Rendering methods
    void clear();
    void render(const std::vector<PlanResult<State, Action, int>>& solution, 
                int currentTimestep, double interpolationAlpha, float opacity = 1.0f);
    void display();
    
    // Mouse interaction
    bool checkAgentClick(int mouseX, int mouseY, int& outAgentIdx, 
                        const std::vector<PlanResult<State, Action, int>>& solution,
                        int currentTimestep);
    sf::Vector2i getMousePosition() const;
    
    // UI display
    void showMessage(const std::string& message, float duration = 3.0f);
    
    // Utility methods
    State findStateAtTime(const std::vector<PlanResult<State, Action, int>>& solution, 
                         int agentId, int timestep) const;
    bool isValidPosition(int x, int y) const;
    sf::Vector2f worldToScreen(float x, float y) const;
    sf::Vector2i screenToWorld(int x, int y) const;
    
    // Add new method for drawing a dragged agent
    void drawDraggedAgent(int agentIdx, int x, int y);

private:
    // SFML window and rendering
    sf::RenderWindow window_;
    sf::Font font_;
    sf::Clock messageClock_;
    
    // Drawing helpers
    void drawGrid();
    void drawObstacles();
    void drawGoals();
    void drawAgents(const std::vector<PlanResult<State, Action, int>>& solution, 
                   int currentTimestep, double interpolationAlpha, float opacity = 1.0f);
    void drawPaths(const std::vector<PlanResult<State, Action, int>>& solution, float opacity = 1.0f);
    void drawUI(int currentTimestep);
    void drawMessage();
    sf::Color generateRandomColor() const;
    
    // Utility methods
    sf::Vector2f interpolatePosition(const State& start, const State& end, double alpha) const;
    
    // Map data
    int dimX_;
    int dimY_;
    std::unordered_set<Location> obstacles_;
    std::vector<State> agentStarts_;
    std::vector<Location> agentGoals_;
    std::vector<sf::Color> agentColors_;
    
    // UI state
    std::string message_;
    float messageTimeRemaining_;
    
    // Constants
    const int cellSize_ = 50;
};
