#include "cbs/simulator.hpp"
#include <iostream>
#include <random>

Simulator::Simulator() 
    : dimX_(0), dimY_(0), messageTimeRemaining_(0.0f) {
    std::cout << "Simulator initialized" << std::endl;
    
    // Load font for text rendering
    if (!font_.loadFromFile("/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf")) {
        std::cerr << "Warning: Failed to load font. Text rendering will be disabled." << std::endl;
    }
}

Simulator::~Simulator() {
    if (window_.isOpen()) {
        window_.close();
    }
    std::cout << "Simulator destroyed" << std::endl;
}

void Simulator::setMap(const std::vector<std::vector<bool>>& map) {
    if (map.empty() || map[0].empty()) {
        std::cerr << "Error: Empty map provided to simulator" << std::endl;
        return;
    }

    dimY_ = map.size();
    dimX_ = map[0].size();
    obstacles_.clear();
    
    // Convert 2D grid to obstacle set
    for (int y = 0; y < dimY_; ++y) {
        for (int x = 0; x < dimX_; ++x) {
            if (map[y][x]) {
                obstacles_.insert(Location(x, y));
            }
        }
    }
    
    std::cout << "Map set with dimensions " << dimX_ << "x" << dimY_ 
              << " and " << obstacles_.size() << " obstacles" << std::endl;
}

void Simulator::setAgents(const std::vector<State>& starts, const std::vector<Location>& goals) {
    // Only set if we haven't set agents before, otherwise keep the existing positions
    // for any agents that might have been moved
    if (agentStarts_.empty()) {
    agentStarts_ = starts;
    }
    agentGoals_ = goals;
    
    // Generate random colors for agents if we don't have them already
    if (agentColors_.empty()) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> distrib(50, 200);
        
        for (size_t i = 0; i < starts.size(); ++i) {
            sf::Color color(distrib(gen), distrib(gen), distrib(gen));
            agentColors_.push_back(color);
        }
    }
    
    std::cout << "Set " << starts.size() << " agents with goals" << std::endl;
}

bool Simulator::createWindow(int width, int height, const std::string& title) {
    try {
        // Create SFML window
        window_.create(sf::VideoMode(width, height), title, sf::Style::Titlebar | sf::Style::Close);
        window_.setFramerateLimit(60);
        
        std::cout << "Created window: " << width << "x" << height << " - " << title << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error creating window: " << e.what() << std::endl;
        return false;
    }
}

bool Simulator::isWindowOpen() const {
    return window_.isOpen();
}

void Simulator::render(const std::vector<PlanResult<State, Action, int>>& solution, int timestep, double alpha) {
    if (!window_.isOpen()) return;
    
    window_.clear(sf::Color(240, 240, 240)); // Light gray background
    
    // Draw grid
    drawGrid();
    
    // Draw obstacles
    drawObstacles();
    
    // Draw goals
    drawGoals();
    
    // Draw paths first (so they appear underneath the agents)
    drawPaths(solution);
    
    // Draw agents
    drawAgents(solution, timestep, alpha);
    
    // Draw UI overlay
    drawUI(timestep);
    
    // Draw any message
    drawMessage();
}

void Simulator::display() {
    if (window_.isOpen()) {
        window_.display();
    }
}

void Simulator::processEvents() {
    if (!window_.isOpen()) return;
    
    sf::Event event;
    while (window_.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
            window_.close();
        }
    }
    
    // Update message timer
    if (messageTimeRemaining_ > 0) {
        messageTimeRemaining_ -= messageClock_.restart().asSeconds();
    }
}

sf::Vector2i Simulator::getMousePosition() const {
    return sf::Mouse::getPosition(window_);
}

sf::Vector2i Simulator::screenToWorld(int x, int y) const {
    return sf::Vector2i(x / cellSize_, y / cellSize_);
}

bool Simulator::checkAgentClick(int x, int y, int& agentIdx, const std::vector<PlanResult<State, Action, int>>& solution, int timestep) {
    for (size_t i = 0; i < solution.size(); ++i) {
        State agentState = findStateAtTime(solution, i, timestep);
        
        // Convert agent position to screen coordinates
        sf::Vector2f screenPos = worldToScreen(agentState.x, agentState.y);
        
        // Check if click is within agent circle (radius is cellSize_/3)
        float radius = cellSize_ / 3.0f;
        float dx = screenPos.x + radius - x;
        float dy = screenPos.y + radius - y;
        if (dx*dx + dy*dy <= radius*radius) {
            agentIdx = i;
            return true;
        }
    }
    
    return false;
}

void Simulator::showMessage(const std::string& message, float duration) {
    message_ = message;
    messageTimeRemaining_ = duration;
    messageClock_.restart();
    std::cout << "Simulator message: " << message << std::endl;
}

bool Simulator::isValidPosition(int x, int y) const {
    // Check if position is within bounds and not an obstacle
    return x >= 0 && x < dimX_ && y >= 0 && y < dimY_ && 
           obstacles_.find(Location(x, y)) == obstacles_.end();
}

State Simulator::findStateAtTime(const std::vector<PlanResult<State, Action, int>>& solution, int agentId, int timestep) const {
    if (agentId < 0 || agentId >= static_cast<int>(solution.size())) {
        return State(-1, -1, -1);
    }
    
    const auto& states = solution[agentId].states;
    if (states.empty()) {
        return agentStarts_[agentId];
    }
    
    // Find the state at or before the given timestep
    for (size_t i = 0; i < states.size(); ++i) {
        if (states[i].second > timestep) {
            // Return previous state if this one is past the timestep
            return i > 0 ? states[i-1].first : states[0].first;
        }
    }
    
    // If we get here, return the last state
    return states.back().first;
}

sf::Vector2f Simulator::worldToScreen(float x, float y) const {
    return sf::Vector2f(x * cellSize_, y * cellSize_);
}

// Private drawing methods
void Simulator::drawGrid() {
    for (int x = 0; x <= dimX_; ++x) {
        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f(x * cellSize_, 0), sf::Color(200, 200, 200)),
            sf::Vertex(sf::Vector2f(x * cellSize_, dimY_ * cellSize_), sf::Color(200, 200, 200))
        };
        window_.draw(line, 2, sf::Lines);
    }
    
    for (int y = 0; y <= dimY_; ++y) {
        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f(0, y * cellSize_), sf::Color(200, 200, 200)),
            sf::Vertex(sf::Vector2f(dimX_ * cellSize_, y * cellSize_), sf::Color(200, 200, 200))
        };
        window_.draw(line, 2, sf::Lines);
    }
}

void Simulator::drawObstacles() {
    sf::RectangleShape rect;
    rect.setSize(sf::Vector2f(cellSize_, cellSize_));
    rect.setFillColor(sf::Color(100, 100, 100)); // Dark gray
    
    for (const auto& obstacle : obstacles_) {
        rect.setPosition(obstacle.x * cellSize_, obstacle.y * cellSize_);
        window_.draw(rect);
    }
}

void Simulator::drawGoals() {
    sf::CircleShape goalMarker;
    goalMarker.setRadius(cellSize_ / 3.0f);
    goalMarker.setOutlineThickness(2);
    goalMarker.setOutlineColor(sf::Color::Black);
    goalMarker.setPointCount(4); // Diamond shape
    
    for (size_t i = 0; i < agentGoals_.size(); ++i) {
        const auto& goal = agentGoals_[i];
        
        // Use agent's color for the goal
        sf::Color goalColor = i < agentColors_.size() ? agentColors_[i] : sf::Color::Yellow;
        goalColor.a = 128; // Semi-transparent
        
        goalMarker.setFillColor(goalColor);
        goalMarker.setPosition(
            goal.x * cellSize_ + cellSize_/2.0f - goalMarker.getRadius(),
            goal.y * cellSize_ + cellSize_/2.0f - goalMarker.getRadius()
        );
        
        window_.draw(goalMarker);
    }
}

void Simulator::drawPaths(const std::vector<PlanResult<State, Action, int>>& solution) {
    for (size_t i = 0; i < solution.size(); ++i) {
        const auto& states = solution[i].states;
        if (states.empty()) continue; // Skip if no states
        
        // Get agent color but make it slightly transparent
        sf::Color pathColor = i < agentColors_.size() ? agentColors_[i] : sf::Color::Red;
        pathColor.a = 180; // Semi-transparent
        
        if (states.size() >= 2) {
            // Create a vertex array for the path line with thicker line
            sf::VertexArray pathLines(sf::Triangles);
            
            // Create thicker lines using triangles
            float lineThickness = 3.0f;
            
            for (size_t j = 0; j < states.size() - 1; ++j) {
                const auto& state1 = states[j].first;
                const auto& state2 = states[j + 1].first;
                
                sf::Vector2f pos1 = worldToScreen(state1.x, state1.y);
                sf::Vector2f pos2 = worldToScreen(state2.x, state2.y);
                
                // Center in cell
                pos1.x += cellSize_ / 2.0f;
                pos1.y += cellSize_ / 2.0f;
                pos2.x += cellSize_ / 2.0f;
                pos2.y += cellSize_ / 2.0f;
                
                // Calculate the direction vector and its perpendicular
                sf::Vector2f direction = pos2 - pos1;
                float length = std::sqrt(direction.x * direction.x + direction.y * direction.y);
                
                if (length > 0) {
                    direction /= length;
                    sf::Vector2f perpendicular(-direction.y, direction.x);
                    
                    // Calculate the four corners of the line segment
                    sf::Vector2f offset = perpendicular * lineThickness / 2.0f;
                    sf::Vector2f v1 = pos1 + offset;
                    sf::Vector2f v2 = pos1 - offset;
                    sf::Vector2f v3 = pos2 + offset;
                    sf::Vector2f v4 = pos2 - offset;
                    
                    // Add two triangles to form a rectangle
                    pathLines.append(sf::Vertex(v1, pathColor));
                    pathLines.append(sf::Vertex(v2, pathColor));
                    pathLines.append(sf::Vertex(v3, pathColor));
                    
                    pathLines.append(sf::Vertex(v2, pathColor));
                    pathLines.append(sf::Vertex(v4, pathColor));
                    pathLines.append(sf::Vertex(v3, pathColor));
                }
            }
            
            // Draw the path
            window_.draw(pathLines);
        }
        
        // Draw small circles at each path point
        sf::CircleShape pointMarker;
        pointMarker.setRadius(4.0f); // Slightly larger circle
        
        for (size_t j = 0; j < states.size(); ++j) {
            const auto& state = states[j].first;
            sf::Vector2f position = worldToScreen(state.x, state.y);
            
            // Center in cell
            position.x += cellSize_ / 2.0f - pointMarker.getRadius();
            position.y += cellSize_ / 2.0f - pointMarker.getRadius();
            
            pointMarker.setFillColor(pathColor);
            pointMarker.setOutlineThickness(1.0f);
            pointMarker.setOutlineColor(sf::Color::White);
            pointMarker.setPosition(position);
            window_.draw(pointMarker);
        }
        
        // Add timestep labels for key points
        if (font_.getInfo().family != "") {
            // Only show labels for start, end, and a few points in between if it's a long path
            std::vector<size_t> indicesToShow;
            
            if (states.size() <= 3) {
                // For short paths, show all timesteps
                for (size_t j = 0; j < states.size(); ++j) {
                    indicesToShow.push_back(j);
                }
            } else {
                // For longer paths, show start, end, and some points in between
                indicesToShow.push_back(0); // Start point
                
                // Add some intermediate points
                size_t step = states.size() / 3;
                if (step > 0) {
                    for (size_t j = step; j < states.size() - 1; j += step) {
                        indicesToShow.push_back(j);
                    }
                }
                
                indicesToShow.push_back(states.size() - 1); // End point
            }
            
            // Draw the selected timestep labels
            for (size_t idx : indicesToShow) {
                const auto& state = states[idx].first;
                int time = states[idx].second;
                
                sf::Vector2f position = worldToScreen(state.x, state.y);
                
                sf::Text timeText;
                timeText.setFont(font_);
                timeText.setString(std::to_string(time));
                timeText.setCharacterSize(12);
                
                // Create a background for the text
                sf::RectangleShape textBg;
                sf::FloatRect textBounds = timeText.getLocalBounds();
                textBg.setSize(sf::Vector2f(textBounds.width + 6, textBounds.height + 6));
                textBg.setFillColor(sf::Color(255, 255, 255, 200)); // Semi-transparent white
                textBg.setOutlineColor(pathColor);
                textBg.setOutlineThickness(1.0f);
                
                // Position text and background
                textBg.setPosition(
                    position.x + cellSize_ / 2.0f - textBg.getSize().x / 2.0f,
                    position.y + cellSize_ / 2.0f - textBg.getSize().y - 12.0f
                );
                
                timeText.setFillColor(sf::Color::Black);
                timeText.setPosition(
                    position.x + cellSize_ / 2.0f - textBounds.width / 2.0f - 1.0f,
                    position.y + cellSize_ / 2.0f - textBounds.height - 15.0f
                );
                
                window_.draw(textBg);
                window_.draw(timeText);
            }
        }
    }
}

void Simulator::drawAgents(const std::vector<PlanResult<State, Action, int>>& solution, int timestep, double alpha) {
    sf::CircleShape agentShape;
    agentShape.setRadius(cellSize_ / 3.0f);
    
    for (size_t i = 0; i < solution.size(); ++i) {
        // Check if this is timestep 0 and we should use the agent's start position
        // instead of the solution position
        sf::Vector2f position;
        
        if (timestep == 0 && i < agentStarts_.size()) {
            // At timestep 0, use the possibly updated start position
            const auto& start = agentStarts_[i];
            position = worldToScreen(start.x, start.y);
        } else {
            // For other timesteps, use the original solution (interpolated)
            State currentState = findStateAtTime(solution, i, timestep);
            State nextState = findStateAtTime(solution, i, timestep + 1);
            
            // Interpolate between current and next position
            if (currentState.x == nextState.x && currentState.y == nextState.y) {
                position = worldToScreen(currentState.x, currentState.y);
            } else {
                position = sf::Vector2f(
                    (currentState.x * (1.0f - alpha) + nextState.x * alpha) * cellSize_,
                    (currentState.y * (1.0f - alpha) + nextState.y * alpha) * cellSize_
                );
            }
        }
        
        // Set color and position
        sf::Color agentColor = i < agentColors_.size() ? agentColors_[i] : sf::Color::Red;
        agentShape.setFillColor(agentColor);
        agentShape.setPosition(
            position.x + cellSize_/2.0f - agentShape.getRadius(),
            position.y + cellSize_/2.0f - agentShape.getRadius()
        );
        
        // Draw agent number
        window_.draw(agentShape);
        
        if (font_.getInfo().family != "") {
            sf::Text text;
            text.setFont(font_);
            text.setString(std::to_string(i));
            text.setCharacterSize(cellSize_ / 3);
            text.setFillColor(sf::Color::White);
            
            // Center text on agent
            sf::FloatRect textBounds = text.getLocalBounds();
            text.setPosition(
                position.x + cellSize_/2.0f - textBounds.width/2.0f,
                position.y + cellSize_/2.0f - textBounds.height
            );
            
            window_.draw(text);
        }
    }
}

void Simulator::drawUI(int timestep) {
    if (font_.getInfo().family == "") return;
    
    sf::Text text;
    text.setFont(font_);
    text.setCharacterSize(20);
    text.setFillColor(sf::Color::Black);
    text.setPosition(10, 10);
    text.setString("Timestep: " + std::to_string(timestep));
    
    window_.draw(text);
}

void Simulator::drawMessage() {
    if (messageTimeRemaining_ <= 0 || font_.getInfo().family == "") return;
    
    sf::Text text;
    text.setFont(font_);
    text.setString(message_);
    text.setCharacterSize(24);
    text.setFillColor(sf::Color::Red);
    
    // Center text at bottom of screen
    sf::FloatRect textBounds = text.getLocalBounds();
    text.setPosition(
        window_.getSize().x / 2.0f - textBounds.width / 2.0f,
        window_.getSize().y - 50
    );
    
    window_.draw(text);
}

void Simulator::drawDraggedAgent(int agentIdx, int x, int y) {
    if (!window_.isOpen() || agentIdx < 0 || agentIdx >= static_cast<int>(agentColors_.size())) {
        return;
    }
    
    // Convert mouse position to grid coordinates
    sf::Vector2i gridPos = screenToWorld(x, y);
    
    // Create ghost agent shape
    sf::CircleShape ghostAgent;
    ghostAgent.setRadius(cellSize_ / 3.0f);
    
    // Use agent's color but semi-transparent
    sf::Color agentColor = agentColors_[agentIdx];
    agentColor.a = 128; // Semi-transparent
    ghostAgent.setFillColor(agentColor);
    
    // Draw border
    ghostAgent.setOutlineThickness(2.0f);
    ghostAgent.setOutlineColor(sf::Color(255, 255, 255, 200)); // White outline
    
    // Position at mouse cursor
    sf::Vector2f worldPos = worldToScreen(gridPos.x, gridPos.y);
    ghostAgent.setPosition(
        worldPos.x + cellSize_/2.0f - ghostAgent.getRadius(),
        worldPos.y + cellSize_/2.0f - ghostAgent.getRadius()
    );
    
    // Draw the ghost agent
    window_.draw(ghostAgent);
    
    // Display agent number
    if (font_.getInfo().family != "") {
        sf::Text text;
        text.setFont(font_);
        text.setString(std::to_string(agentIdx));
        text.setCharacterSize(cellSize_ / 3);
        text.setFillColor(sf::Color::White);
        
        // Center text on agent
        sf::FloatRect textBounds = text.getLocalBounds();
        text.setPosition(
            worldPos.x + cellSize_/2.0f - textBounds.width/2.0f,
            worldPos.y + cellSize_/2.0f - textBounds.height
        );
        
        window_.draw(text);
    }
    
    // Display grid coordinates
    if (font_.getInfo().family != "") {
        sf::Text posText;
        posText.setFont(font_);
        posText.setString("(" + std::to_string(gridPos.x) + "," + std::to_string(gridPos.y) + ")");
        posText.setCharacterSize(16);
        posText.setFillColor(sf::Color::Black);
        
        // Position text above agent
        sf::FloatRect textBounds = posText.getLocalBounds();
        posText.setPosition(
            worldPos.x + cellSize_/2.0f - textBounds.width/2.0f,
            worldPos.y - textBounds.height - 5
        );
        
        window_.draw(posText);
    }
    
    // Display feedback if position is invalid
    if (!isValidPosition(gridPos.x, gridPos.y)) {
        sf::CircleShape invalidMarker;
        invalidMarker.setRadius(cellSize_ / 2.0f);
        invalidMarker.setFillColor(sf::Color(255, 0, 0, 64)); // Red with low opacity
        invalidMarker.setPosition(
            worldPos.x + cellSize_/2.0f - invalidMarker.getRadius(),
            worldPos.y + cellSize_/2.0f - invalidMarker.getRadius()
        );
        
        window_.draw(invalidMarker);
    }
    
    // We need to display immediately to avoid flicker
    display();
}