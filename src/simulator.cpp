#include <QPainter>
#include <QRandomGenerator>
#include <QTimer>
#include <QMouseEvent>
#include <algorithm> 
#include <QDebug>

#include "cbs/simulator.hpp"

Simulator::Simulator(QWidget* parent)
    : QWidget(parent), dimX_(0), dimY_(0),
      hasValidSolution_(false), currentTimestep_(0), maxTimestep_(0),
      interpolationAlpha_(0.0), stepsPerTimestep_(20), animationInterval_(150),
      draggedAgentIdx_(-1), draggedAgentOrigState_(-1, -1, -1) {

    setWindowTitle("CBS Simulator");
    setMouseTracking(true);  // Enable mouse tracking for smooth dragging

    animationTimer_ = new QTimer(this);
    connect(animationTimer_, &QTimer::timeout, this, &Simulator::updateTimeStep);
    
    // Initialize warning timer
    warningTimer_ = new QTimer(this);
    warningTimer_->setSingleShot(true);
    connect(warningTimer_, &QTimer::timeout, this, [this]() {
        warningMessage_.clear();
        update();
    });
}

QColor Simulator::generateRandomColor() const {
    auto* rng = QRandomGenerator::global();
    int r = rng->bounded(256), g = rng->bounded(256), b = rng->bounded(256);
    const int minBrightness = 150, maxBrightness = 230;

    int max_component = std::max({r, g, b});  
    if (max_component < minBrightness) {
        double scale = double(minBrightness) / max_component;
        r = qMin(255, int(r * scale));
        g = qMin(255, int(g * scale));
        b = qMin(255, int(b * scale));
    }
    max_component = std::max({r, g, b});  
    if (max_component > maxBrightness) {
        double scale = double(maxBrightness) / max_component;
        r = int(r * scale);
        g = int(g * scale);
        b = int(b * scale);
    }

    return QColor(r, g, b);
}

void Simulator::setMap(const std::vector<std::vector<bool>>& map) {
    if (map.empty() || map[0].empty()) return;

    dimX_ = map[0].size();
    dimY_ = map.size();
    if (dimX_ > 1000 || dimY_ > 1000) return;

    obstacles_.clear();
    for (int y = 0; y < dimY_; ++y)
        for (int x = 0; x < dimX_; ++x)
            if (map[y][x]) obstacles_.insert(Location(x, y));

    setFixedSize(dimX_ * cellSize_, dimY_ * cellSize_);
    setMinimumSize(size());
    setMaximumSize(size());
    update();
}

void Simulator::setAgents(const std::vector<State>& starts, const std::vector<Location>& goals) {
    if (starts.size() != goals.size()) return;

    agentStarts_ = starts;
    agentGoals_ = goals;
    agentColors_.clear();

    for (size_t i = 0; i < starts.size(); ++i)
        agentColors_.push_back(generateRandomColor());

    update();
}

void Simulator::visualizeSolution(const std::vector<PlanResult<State, Action, int>>& solution) {
    solution_ = solution;
    hasValidSolution_ = true;
    
    // After replanning, the new solution starts at time 0, but we need to adjust it to 
    // appear to continue from the current time step
    if (currentTimestep_ > 0) {
        qDebug() << "Adjusting solution time from current timestep:" << currentTimestep_;
        
        // Create time-adjusted copies of all agent solutions
        for (auto& agentSolution : solution_) {
            // Shift all timestamps forward by currentTimestep_
            for (auto& [state, time] : agentSolution.states) {
                time += currentTimestep_;
            }
        }
    }
    
    // Find the maximum timestep across all agent paths
    maxTimestep_ = 0;
    for (const auto& plan : solution_) {
        if (!plan.states.empty()) {
            maxTimestep_ = std::max(maxTimestep_, static_cast<int>(plan.states.back().second));
        }
    }
    
    // Don't reset the currentTimestep_ - continue from where we were before
    interpolationAlpha_ = 0.0;
    update();
}

void Simulator::startAnimation() {
    if (!hasValidSolution_) return;
    currentTimestep_ = 0;
    interpolationAlpha_ = 0.0;
    animationTimer_->start(animationInterval_);
}

void Simulator::stopAnimation() {
    animationTimer_->stop();
}

void Simulator::updateTimeStep() {
    if (!hasValidSolution_) return;

    interpolationAlpha_ += 1.0 / stepsPerTimestep_;

    if (interpolationAlpha_ >= 1.0) {
        interpolationAlpha_ = 0.0;
        currentTimestep_++;
        if (currentTimestep_ > maxTimestep_) {
            stopAnimation();
            currentTimestep_ = maxTimestep_;
        }
    }

    update();
}

State Simulator::findStateAtTime(const std::vector<PlanResult<State, Action, int>>& solution, int agentId, int timestep) const {
    if (agentId < 0 || agentId >= static_cast<int>(solution.size())) {
        return State(-1, -1, -1);
    }
    
    const auto& plan = solution[agentId];
    
    // If the plan is empty, return an invalid state
    if (plan.states.empty()) {
        return State(-1, -1, -1);
    }
    
    // If timestep is before the first state, return the first state
    if (timestep <= plan.states.front().second) {
        return plan.states.front().first;
    }
    
    // If timestep is after the last state, return the last state
    if (timestep >= plan.states.back().second) {
        return plan.states.back().first;
    }
    
    // Search for the exact state at the timestep
    for (size_t i = 0; i < plan.states.size() - 1; ++i) {
        const auto& [state1, t1] = plan.states[i];
        const auto& [state2, t2] = plan.states[i + 1];
        
        if (t1 == timestep) {
            // Exact match for timestep
            return state1;
        }
        
        if (t1 < timestep && t2 > timestep) {
            // We're between two states, interpolate based on time
            double ratio = static_cast<double>(timestep - t1) / (t2 - t1);
            int x = state1.x + static_cast<int>(ratio * (state2.x - state1.x));
            int y = state1.y + static_cast<int>(ratio * (state2.y - state1.y));
            
            return State(timestep, x, y);
        }
    }
    
    // If we get here, check if the last state matches
    if (plan.states.back().second == timestep) {
        return plan.states.back().first;
    }
    
    // Fallback to the last state (this should not happen with proper data)
    return plan.states.back().first;
}

QPointF Simulator::interpolatePosition(const State& start, const State& end, double alpha) const {
    double x = start.x + alpha * (end.x - start.x);
    double y = start.y + alpha * (end.y - start.y);
    return QPointF(x * cellSize_ + cellSize_ / 2, y * cellSize_ + cellSize_ / 2);
}

void Simulator::drawFlag(QPainter& painter, const QRect& cell, const QColor& color) const {
    int flagpoleWidth = cellSize_ / 10;
    int flagHeight = cellSize_ / 2;
    int flagWidth = cellSize_ / 2;
    int centerX = cell.center().x();
    int bottomY = cell.bottom() - cellSize_ / 4;

    painter.setPen(Qt::NoPen);
    painter.setBrush(color);
    painter.drawRect(centerX - flagpoleWidth / 2, bottomY - flagHeight, flagpoleWidth, flagHeight);

    QPolygon flag;
    flag << QPoint(centerX + flagpoleWidth / 2, bottomY - flagHeight)
         << QPoint(centerX + flagpoleWidth / 2 + flagWidth, bottomY - flagHeight + flagHeight / 3)
         << QPoint(centerX + flagpoleWidth / 2, bottomY - flagHeight + flagHeight / 2);

    painter.setBrush(color);
    painter.drawPolygon(flag);
}

// New helper methods for agent dragging
int Simulator::findAgentAtPosition(const QPoint& pos) const {
    if (!hasValidSolution_) return -1;
    
    for (size_t i = 0; i < solution_.size(); ++i) {
        State state = findStateAtTime(solution_, i, currentTimestep_);
        QPointF agentPos = interpolatePosition(state, 
                                            findStateAtTime(solution_, i, currentTimestep_ + 1), 
                                            interpolationAlpha_);
        
        QPointF agentCenter(agentPos.x(), agentPos.y());
        double distance = QLineF(agentCenter, pos).length();
        
        // Check if click is within agent radius
        if (distance < cellSize_ * 0.3) {
            return static_cast<int>(i);
        }
    }
    
    return -1;  // No agent found at click position
}

State Simulator::gridPositionToState(const QPointF& pos) const {
    int gridX = static_cast<int>(pos.x() / cellSize_);
    int gridY = static_cast<int>(pos.y() / cellSize_);
    
    // Clamp to grid boundaries
    gridX = std::max(0, std::min(gridX, dimX_ - 1));
    gridY = std::max(0, std::min(gridY, dimY_ - 1));
    
    return State(currentTimestep_, gridX, gridY);
}

bool Simulator::isValidPosition(int x, int y) const {
    // Check if position is within grid boundaries
    if (x < 0 || x >= dimX_ || y < 0 || y >= dimY_) {
        return false;
    }
    
    // Check if position is an obstacle
    if (obstacles_.count(Location(x, y)) > 0) {
        return false;
    }
    
    return true;
}

State Simulator::getDraggedAgentState() const {
    if (draggedAgentIdx_ < 0) {
        return State(-1, -1, -1);  // Invalid state
    }
    return draggedAgentOrigState_;
}

// Mouse event handlers
void Simulator::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton && hasValidSolution_) {
        draggedAgentIdx_ = findAgentAtPosition(event->pos());
        
        if (draggedAgentIdx_ >= 0) {
            // Store the original state before dragging
            draggedAgentOrigState_ = findStateAtTime(solution_, draggedAgentIdx_, currentTimestep_);
            
            // Calculate offset from agent center to click point for smooth dragging
            State agentState = draggedAgentOrigState_;
            QPointF agentCenter = QPointF(agentState.x * cellSize_ + cellSize_ / 2, 
                                        agentState.y * cellSize_ + cellSize_ / 2);
            dragOffset_ = agentCenter - event->pos();
            
            // Pause animation while dragging
            if (animationTimer_->isActive()) {
                animationTimer_->stop();
            }
        }
    }
    
    QWidget::mousePressEvent(event);
}

void Simulator::mouseMoveEvent(QMouseEvent* event) {
    if (draggedAgentIdx_ >= 0) {
        // Just update the UI when dragging, we'll calculate positions in paintEvent
        update();
    }
    
    QWidget::mouseMoveEvent(event);
}

void Simulator::mouseReleaseEvent(QMouseEvent* event) {
    if (draggedAgentIdx_ >= 0) {
        // Calculate final position
        QPointF adjustedPos = event->pos() + dragOffset_;
        
        // Convert to grid coordinates and snap to the closest cell
        State newState = gridPositionToState(adjustedPos);
        
        // Check if position is valid (not an obstacle and within grid)
        if (isValidPosition(newState.x, newState.y)) {
            // Emit signal with new agent position for controller to handle replanning
            emit agentDragged(draggedAgentIdx_, newState);
        } else {
            // If invalid position, show a warning and resume animation
            showReplanningWarning("Invalid position! Agent returned to original position.");
            startAnimation();
        }
        
        // Reset drag state
        draggedAgentIdx_ = -1;
    }
    
    QWidget::mouseReleaseEvent(event);
}

void Simulator::showReplanningWarning(const QString& message) {
    warningMessage_ = message;
    update();
    
    // Clear the warning after 3 seconds
    warningTimer_->start(3000);
}

void Simulator::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    for (int y = 0; y < dimY_; ++y) {
        for (int x = 0; x < dimX_; ++x) {
            QRect cell(x * cellSize_, y * cellSize_, cellSize_, cellSize_);
            painter.setPen(Qt::gray);
            painter.setBrush(obstacles_.count(Location(x, y)) ? Qt::black : Qt::white);
            painter.drawRect(cell);
        }
    }

    // Draw goals
    for (size_t i = 0; i < agentGoals_.size(); ++i) {
        QRect cell(agentGoals_[i].x * cellSize_, agentGoals_[i].y * cellSize_, cellSize_, cellSize_);
        drawFlag(painter, cell, agentColors_[i]);
    }

    if (hasValidSolution_) {
        for (size_t i = 0; i < solution_.size(); ++i) {
            // If this agent is being dragged, use the dragged position
            if (draggedAgentIdx_ == static_cast<int>(i)) {
                QPointF dragPos = QCursor::pos() - this->mapToGlobal(QPoint(0, 0)) + dragOffset_;
                
                // Convert to grid coordinates for snapping
                State dragState = gridPositionToState(dragPos);
                
                // Only draw if valid position
                if (isValidPosition(dragState.x, dragState.y)) {
                    // Draw at the snapped grid position
                    QPointF center(dragState.x * cellSize_ + cellSize_ / 2, 
                                  dragState.y * cellSize_ + cellSize_ / 2);
                    
                    // Draw with slight transparency while dragging
                    QColor dragColor = agentColors_[i];
                    dragColor.setAlpha(180);
                    
                    painter.setBrush(dragColor);
                    painter.setPen(QPen(Qt::black, 2, Qt::DashLine));
                    painter.drawEllipse(center, cellSize_ * 0.3, cellSize_ * 0.3);
                    
                    painter.setPen(Qt::black);
                    painter.setFont(QFont("Arial", cellSize_ / 3));
                    painter.drawText(QRectF(center.x() - 10, center.y() - 10, 20, 20), 
                                    Qt::AlignCenter, QString::number(i));
                } else {
                    // Draw with red outline at cursor position if invalid
                    QPointF cursorPos = dragPos;
                    
                    QColor invalidColor = QColor(255, 80, 80, 180); // Semi-transparent red
                    
                    painter.setBrush(invalidColor);
                    painter.setPen(QPen(Qt::red, 2, Qt::DashLine));
                    painter.drawEllipse(cursorPos, cellSize_ * 0.3, cellSize_ * 0.3);
                    
                    painter.setPen(Qt::black);
                    painter.setFont(QFont("Arial", cellSize_ / 3));
                    painter.drawText(QRectF(cursorPos.x() - 10, cursorPos.y() - 10, 20, 20), 
                                    Qt::AlignCenter, QString::number(i));
                }
            } else {
                // Draw normally for non-dragged agents
                State currentState = findStateAtTime(solution_, i, currentTimestep_);
                State nextState = findStateAtTime(solution_, i, currentTimestep_ + 1);
                QPointF pos = interpolatePosition(currentState, nextState, interpolationAlpha_);

                painter.setBrush(agentColors_[i]);
                painter.setPen(QPen(Qt::black, 2));
                painter.drawEllipse(pos, cellSize_ * 0.3, cellSize_ * 0.3);

                painter.setPen(Qt::black);
                painter.setFont(QFont("Arial", cellSize_ / 3));
                painter.drawText(QRectF(pos.x() - 10, pos.y() - 10, 20, 20), Qt::AlignCenter, QString::number(i));
            }
        }
    }

    painter.setPen(Qt::black);
    painter.setFont(QFont("Arial", 12));
    painter.drawText(10, 20, QString("Time: %1").arg(currentTimestep_));
    
    // If an agent is being dragged, show informative text
    if (draggedAgentIdx_ >= 0) {
        painter.setPen(Qt::red);
        painter.setFont(QFont("Arial", 12, QFont::Bold));
        painter.drawText(10, 40, QString("Dragging Agent %1 - Will Replan").arg(draggedAgentIdx_));
    }
    
    // Draw warning message if it exists
    if (!warningMessage_.isEmpty()) {
        QRectF messageRect = QRectF(10, height() - 40, width() - 20, 30);
        
        // Draw a semi-transparent background for the message
        painter.setBrush(QColor(255, 200, 200, 220));
        painter.setPen(Qt::NoPen);
        painter.drawRoundedRect(messageRect, 5, 5);
        
        painter.setPen(Qt::red);
        painter.setFont(QFont("Arial", 12, QFont::Bold));
        painter.drawText(messageRect, Qt::AlignCenter, warningMessage_);
    }
}
