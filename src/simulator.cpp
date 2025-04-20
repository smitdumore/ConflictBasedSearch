#include <QPainter>
#include <QRandomGenerator>

#include "cbs/simulator.hpp"

Simulator::Simulator(QWidget* parent)
    : QWidget(parent), dimX_(0), dimY_(0) {
    setWindowTitle("CBS Simulator");
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

void Simulator::setMap(int dimX, int dimY, const std::unordered_set<Location>& obstacles) {
    if (dimX <= 0 || dimY <= 0 || dimX > 1000 || dimY > 1000) {
        qWarning("Simulator::setMap received invalid grid size: (%d, %d)", dimX, dimY);
        return;
    }

    dimX_ = dimX;
    dimY_ = dimY;
    obstacles_ = obstacles;

    setFixedSize(dimX_ * cellSize_, dimY_ * cellSize_);
    update();
}

void Simulator::setAgents(const std::vector<State>& starts, const std::vector<Location>& goals) {
    agentStarts_ = starts;
    agentGoals_ = goals;
    
    // Generate random colors for agents
    agentColors_.clear();
    for (size_t i = 0; i < starts.size(); ++i) {
        agentColors_.push_back(generateRandomColor());
    }
    
    update();
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

    // Draw agents and their goals
    for (size_t i = 0; i < agentStarts_.size(); ++i) {
        const QColor& agentColor = agentColors_[i];
        
        // Draw start position (dotted circle)
        const State& start = agentStarts_[i];
        QRect startCell(start.x * cellSize_, start.y * cellSize_, cellSize_, cellSize_);
        
        // Draw filled circle with agent color
        painter.setPen(Qt::NoPen);
        painter.setBrush(agentColor);
        painter.drawEllipse(startCell.center(), cellSize_/3, cellSize_/3);
        
        // Draw agent number in black
        painter.setPen(Qt::black);
        painter.setFont(QFont("Arial", cellSize_/3));
        painter.drawText(startCell, Qt::AlignCenter, QString::number(i));

        // Draw goal position if available
        if (i < agentGoals_.size()) {
            const Location& goal = agentGoals_[i];
            QRect goalCell(goal.x * cellSize_, goal.y * cellSize_, cellSize_, cellSize_);
            
            // Draw flag at goal position
            drawFlag(painter, goalCell, agentColor);
        }
    }
}

