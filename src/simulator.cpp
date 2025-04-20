#include <QPainter>
#include <QRandomGenerator>
#include <QTimer>

#include "cbs/simulator.hpp"

Simulator::Simulator(QWidget* parent)
    : QWidget(parent), dimX_(0), dimY_(0), 
      hasValidSolution_(false), currentTimestep_(0), maxTimestep_(0) {
    setWindowTitle("CBS Simulator");
    
    // Setup animation timer
    animationTimer_ = new QTimer(this);
    connect(animationTimer_, &QTimer::timeout, this, &Simulator::updateTimeStep);
}

QColor Simulator::generateRandomColor() const {
    auto* rng = QRandomGenerator::global();
    // Generate vibrant colors by ensuring at least one component is high
    int r = rng->bounded(256);
    int g = rng->bounded(256);
    int b = rng->bounded(256);
    
    // Ensure color is vibrant enough
    const int minBrightness = 150;
    const int maxBrightness = 230;  // Not too bright to ensure text is visible
    
    int max_component = qMax(qMax(r, g), b);
    if (max_component < minBrightness) {
        double scale = double(minBrightness) / max_component;
        r = qMin(255, int(r * scale));
        g = qMin(255, int(g * scale));
        b = qMin(255, int(b * scale));
    }
    
    // Scale down if too bright
    max_component = qMax(qMax(r, g), b);
    if (max_component > maxBrightness) {
        double scale = double(maxBrightness) / max_component;
        r = int(r * scale);
        g = int(g * scale);
        b = int(b * scale);
    }
    
    return QColor(r, g, b);
}

void Simulator::setMap(const std::vector<std::vector<bool>>& map) {
    if (map.empty() || map[0].empty()) {
        qWarning("Simulator::setMap received empty map");
        return;
    }
    
    dimX_ = map[0].size();
    dimY_ = map.size();
    
    if (dimX_ > 1000 || dimY_ > 1000) {
        qWarning("Simulator::setMap received invalid grid size: (%d, %d)", dimX_, dimY_);
        return;
    }
    
    // Convert map to obstacles set
    obstacles_.clear();
    for (int y = 0; y < dimY_; ++y) {
        for (int x = 0; x < dimX_; ++x) {
            if (map[y][x]) {
                obstacles_.insert(Location(x, y));
            }
        }
    }
    
    update();
}

void Simulator::setAgents(const std::vector<State>& starts, const std::vector<Location>& goals) {
    if (starts.size() != goals.size()) {
        qWarning("Simulator::setAgents received mismatched starts and goals sizes");
        return;
    }
    
    agentStarts_ = starts;
    agentGoals_ = goals;
    agentColors_.clear();
    
    // Generate colors for agents
    for (size_t i = 0; i < starts.size(); ++i) {
        agentColors_.push_back(generateRandomColor());
    }
    
    update();
}

void Simulator::visualizeSolution(const std::vector<PlanResult<State, Action, int>>& solution) {
    solution_ = solution;
    hasValidSolution_ = true;
    
    // Find maximum timestep (makespan)
    maxTimestep_ = 0;
    for (const auto& agentPlan : solution_) {
        if (!agentPlan.states.empty()) {
            maxTimestep_ = std::max(maxTimestep_, 
                                  static_cast<int>(agentPlan.states.back().second));
        }
    }
    
    // Reset visualization
    currentTimestep_ = 0;
    update();
}

void Simulator::startAnimation() {
    if (!hasValidSolution_) return;
    currentTimestep_ = 0;
    animationTimer_->start(animationInterval_);
}

void Simulator::stopAnimation() {
    animationTimer_->stop();
}

void Simulator::updateTimeStep() {
    if (!hasValidSolution_) return;
    
    currentTimestep_++;
    if (currentTimestep_ > maxTimestep_) {
        stopAnimation();
        currentTimestep_ = maxTimestep_;
    }
    update();
}

State Simulator::getAgentStateAtTime(size_t agentIdx, int timestep) const {
    if (!hasValidSolution_ || agentIdx >= solution_.size()) {
        return agentStarts_[agentIdx]; // Return start position if no solution
    }

    const auto& agentPlan = solution_[agentIdx];
    
    // Find the state at or before the current timestep
    for (const auto& state : agentPlan.states) {
        if (state.second == timestep) {
            return state.first;
        }
        if (state.second > timestep) {
            break;
        }
    }
    
    // If we're past the last state, return the final position
    if (!agentPlan.states.empty()) {
        return agentPlan.states.back().first;
    }
    
    return agentStarts_[agentIdx];
}

void Simulator::drawFlag(QPainter& painter, const QRect& cell, const QColor& color) const {
    // Calculate flag dimensions relative to cell size
    int flagpoleWidth = cellSize_ / 10;
    int flagHeight = cellSize_ / 2;
    int flagWidth = cellSize_ / 2;
    
    // Calculate positions
    int centerX = cell.center().x();
    int bottomY = cell.bottom() - cellSize_ / 4;  // Raise from bottom a bit
    
    // Draw flagpole
    painter.setPen(Qt::NoPen);
    painter.setBrush(color);
    painter.drawRect(centerX - flagpoleWidth/2, 
                    bottomY - flagHeight,
                    flagpoleWidth, 
                    flagHeight);
    
    // Draw flag
    QPolygon flag;
    flag << QPoint(centerX + flagpoleWidth/2, bottomY - flagHeight)
         << QPoint(centerX + flagpoleWidth/2 + flagWidth, bottomY - flagHeight + flagHeight/3)
         << QPoint(centerX + flagpoleWidth/2, bottomY - flagHeight + flagHeight/2);
    
    painter.setBrush(color);
    painter.drawPolygon(flag);
}

void Simulator::visualizeTimeStep(const std::vector<PlanResult<State, Action, int>>& solution, int timestep) {
    solution_ = solution;
    currentTimestep_ = timestep;
    
    // Calculate max timestep (makespan)
    maxTimestep_ = 0;
    for (const auto& plan : solution_) {
        if (!plan.states.empty()) {
            maxTimestep_ = std::max(maxTimestep_, static_cast<int>(plan.states.back().second));
        }
    }

    hasValidSolution_ = true;
    update(); // Trigger paintEvent() 
}

State Simulator::findStateAtTime(const std::vector<PlanResult<State, Action, int>>& solution, int agentId, int timestep) const {
    if (agentId < 0 || agentId >= static_cast<int>(solution.size())) {
        return State(-1, -1, -1);  // Invalid state
    }
    
    const auto& plan = solution[agentId];
    
    // Find the state at the given timestep
    for (const auto& [state, t] : plan.states) {
        if (t == timestep) {
            return state;
        }
        if (t > timestep) {
            break;
        }
    }
    
    // If we didn't find the exact timestep, return the last state
    if (!plan.states.empty()) {
        return plan.states.back().first;
    }
    
    return State(-1, -1, -1);  // Invalid state
}

void Simulator::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Draw grid and obstacles
    for (int y = 0; y < dimY_; ++y) {
        for (int x = 0; x < dimX_; ++x) {
            QRect cell(x * cellSize_, y * cellSize_, cellSize_, cellSize_);
            painter.setPen(Qt::gray);

            if (obstacles_.count(Location(x, y))) {
                painter.setBrush(Qt::black);
            } else {
                painter.setBrush(Qt::white);
            }

            painter.drawRect(cell);
        }
    }

    // Draw agents at their current positions
    for (size_t i = 0; i < agentStarts_.size(); ++i) {
        const QColor& agentColor = agentColors_[i];
        
        // Get current state from solution
        State currentState = agentStarts_[i];  // Initialize with start state
        if (i < solution_.size()) {
            currentState = findStateAtTime(solution_, i, currentTimestep_);
        }

        QRect currentCell(currentState.x * cellSize_, 
                         currentState.y * cellSize_, 
                         cellSize_, cellSize_);
        
        // Draw filled circle with agent color
        painter.setPen(Qt::NoPen);
        painter.setBrush(agentColor);
        painter.drawEllipse(currentCell.center(), cellSize_/3, cellSize_/3);
        
        // Draw agent number in black
        painter.setPen(Qt::black);
        painter.setFont(QFont("Arial", cellSize_/3));
        painter.drawText(currentCell, Qt::AlignCenter, QString::number(i));

        // Draw goal position
        if (i < agentGoals_.size()) {
            const Location& goal = agentGoals_[i];
            QRect goalCell(goal.x * cellSize_, goal.y * cellSize_, 
                         cellSize_, cellSize_);
            drawFlag(painter, goalCell, agentColor);
        }
    }

    // Draw timestep counter
    painter.setPen(Qt::black);
    painter.setFont(QFont("Arial", 12));
    painter.drawText(10, 20, QString("Time: %1").arg(currentTimestep_));
}

