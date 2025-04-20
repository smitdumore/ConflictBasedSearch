#include <QPainter>
#include <QRandomGenerator>
#include <QTimer>
#include <algorithm> 

#include "cbs/simulator.hpp"

Simulator::Simulator(QWidget* parent)
    : QWidget(parent), dimX_(0), dimY_(0),
      hasValidSolution_(false), currentTimestep_(0), maxTimestep_(0),
      interpolationAlpha_(0.0), stepsPerTimestep_(10), animationInterval_(30) {

    setWindowTitle("CBS Simulator");

    animationTimer_ = new QTimer(this);
    connect(animationTimer_, &QTimer::timeout, this, &Simulator::updateTimeStep);
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
    maxTimestep_ = 0;

    for (const auto& plan : solution_)
        if (!plan.states.empty())
            maxTimestep_ = std::max(maxTimestep_, static_cast<int>(plan.states.back().second));

    currentTimestep_ = 0;
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
    if (agentId < 0 || agentId >= static_cast<int>(solution.size())) return State(-1, -1, -1);
    const auto& plan = solution[agentId];

    for (const auto& [state, t] : plan.states) {
        if (t == timestep) return state;
        if (t > timestep) break;
    }

    if (!plan.states.empty()) return plan.states.back().first;
    return State(-1, -1, -1);
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

    painter.setPen(Qt::black);
    painter.setFont(QFont("Arial", 12));
    painter.drawText(10, 20, QString("Time: %1").arg(currentTimestep_));
}
