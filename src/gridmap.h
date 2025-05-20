#ifndef GRIDMAP_H
#define GRIDMAP_H

#include <vector>
#include <QWidget>
#include <QPainter>
#include <QMouseEvent>
#include "astar.h"

class GridMap : public QWidget {
    Q_OBJECT

public:
    explicit GridMap(int width, int height, QWidget* parent = nullptr);

    void setStart(const Point& start);
    void setGoal(const Point& goal);
    void clearPath();
    void resetMap();
    
    const std::vector<std::vector<bool>>& getMap() const { return map_; }
    Point getStart() const { return start_; }
    Point getGoal() const { return goal_; }
    void setPath(const std::vector<Point>& path);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    int width_;
    int height_;
    int cellSize_;
    std::vector<std::vector<bool>> map_;  
    Point start_;
    Point goal_;
    std::vector<Point> path_;
    bool isDrawingWall_;
    bool isErasingWall_;
    bool isDraggingStart_;
    bool isDraggingGoal_;

    Point screenToGrid(const QPoint& pos) const;
    QPoint gridToScreen(const Point& pos) const;
    void drawGrid(QPainter& painter) const;
    void drawWalls(QPainter& painter) const;
    void drawPath(QPainter& painter) const;
    void drawStartGoal(QPainter& painter) const;
};

#endif // GRIDMAP_H 
