#include "gridmap.h"
#include <QDebug>

GridMap::GridMap(int width, int height, QWidget* parent)
    : QWidget(parent)
    , width_(width)
    , height_(height)
    , cellSize_(30)
    , start_({0, 0})
    , goal_({width-1, height-1})
    , isDrawingWall_(false)
    , isErasingWall_(false)
    , isDraggingStart_(false)
    , isDraggingGoal_(false)
{
    setFixedSize(width_ * cellSize_, height_ * cellSize_);
    map_.resize(width_, std::vector<bool>(height_, true));
    setMouseTracking(true);
}

void GridMap::setStart(const Point& start) {
    if (start.x >= 0 && start.x < width_ && start.y >= 0 && start.y < height_) {
        start_ = start;
        clearPath();
        update();
    }
}

void GridMap::setGoal(const Point& goal) {
    if (goal.x >= 0 && goal.x < width_ && goal.y >= 0 && goal.y < height_) {
        goal_ = goal;
        clearPath();
        update();
    }
}

void GridMap::clearPath() {
    path_.clear();
    update();
}

void GridMap::resetMap() {
    for (auto& row : map_) {
        std::fill(row.begin(), row.end(), true);
    }
    clearPath();
    update();
}

void GridMap::setPath(const std::vector<Point>& path) {
    path_ = path;
    update();
}

Point GridMap::screenToGrid(const QPoint& pos) const {
    return Point{pos.x() / cellSize_, pos.y() / cellSize_};
}

QPoint GridMap::gridToScreen(const Point& pos) const {
    return QPoint(pos.x * cellSize_, pos.y * cellSize_);
}

void GridMap::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    drawGrid(painter);
    drawWalls(painter);
    drawPath(painter);
    drawStartGoal(painter);
}

void GridMap::drawGrid(QPainter& painter) const {
    painter.setPen(QPen(Qt::lightGray, 1));
    
    for (int x = 0; x <= width_; ++x) {
        painter.drawLine(x * cellSize_, 0, x * cellSize_, height_ * cellSize_);
    }
    for (int y = 0; y <= height_; ++y) {
        painter.drawLine(0, y * cellSize_, width_ * cellSize_, y * cellSize_);
    }
}

void GridMap::drawWalls(QPainter& painter) const {
    painter.setPen(Qt::NoPen);
    painter.setBrush(Qt::black);
    
    for (int x = 0; x < width_; ++x) {
        for (int y = 0; y < height_; ++y) {
            if (!map_[x][y]) {
                painter.fillRect(x * cellSize_, y * cellSize_,
                               cellSize_, cellSize_, Qt::black);
            }
        }
    }
}

void GridMap::drawPath(QPainter& painter) const {
    if (path_.empty()) return;

    painter.setPen(QPen(Qt::blue, 2));
    for (size_t i = 1; i < path_.size(); ++i) {
        QPoint p1 = gridToScreen(path_[i-1]);
        QPoint p2 = gridToScreen(path_[i]);
        painter.drawLine(p1.x() + cellSize_/2, p1.y() + cellSize_/2,
                        p2.x() + cellSize_/2, p2.y() + cellSize_/2);
    }
}

void GridMap::drawStartGoal(QPainter& painter) const {
    painter.setPen(Qt::NoPen);
    painter.setBrush(Qt::green);
    QPoint startPos = gridToScreen(start_);
    painter.drawEllipse(startPos.x() + cellSize_/4, startPos.y() + cellSize_/4,
                       cellSize_/2, cellSize_/2);

    painter.setBrush(Qt::red);
    QPoint goalPos = gridToScreen(goal_);
    painter.drawEllipse(goalPos.x() + cellSize_/4, goalPos.y() + cellSize_/4,
                       cellSize_/2, cellSize_/2);
}

void GridMap::mousePressEvent(QMouseEvent* event) {
    Point gridPos = screenToGrid(event->pos());
    if (gridPos.x < 0 || gridPos.x >= width_ || gridPos.y < 0 || gridPos.y >= height_)
        return;

    if (event->button() == Qt::LeftButton) {
        if (gridPos == start_) {
            isDraggingStart_ = true;
        }
        else if (gridPos == goal_) {
            isDraggingGoal_ = true;
        }
        else {
            isDrawingWall_ = true;
            if (map_[gridPos.x][gridPos.y]) {
                map_[gridPos.x][gridPos.y] = false;
                clearPath();
                update();
            }
        }
    }
    else if (event->button() == Qt::RightButton) {
        isErasingWall_ = true;
        if (!map_[gridPos.x][gridPos.y]) {
            map_[gridPos.x][gridPos.y] = true;
            clearPath();
            update();
        }
    }
}

void GridMap::mouseMoveEvent(QMouseEvent* event) {
    Point gridPos = screenToGrid(event->pos());
    if (gridPos.x < 0 || gridPos.x >= width_ || gridPos.y < 0 || gridPos.y >= height_)
        return;

    if (isDraggingStart_) {
        if (gridPos != goal_ && map_[gridPos.x][gridPos.y]) {
            setStart(gridPos);
        }
    }
    else if (isDraggingGoal_) {
        if (gridPos != start_ && map_[gridPos.x][gridPos.y]) {
            setGoal(gridPos);
        }
    }
    else if (isDrawingWall_ && map_[gridPos.x][gridPos.y]) {
        if (gridPos != start_ && gridPos != goal_) {
            map_[gridPos.x][gridPos.y] = false;
            clearPath();
            update();
        }
    }
    else if (isErasingWall_ && !map_[gridPos.x][gridPos.y]) {
        map_[gridPos.x][gridPos.y] = true;
        clearPath();
        update();
    }
}

void GridMap::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        isDrawingWall_ = false;
        isDraggingStart_ = false;
        isDraggingGoal_ = false;
    }
    else if (event->button() == Qt::RightButton) {
        isErasingWall_ = false;
    }
} 
