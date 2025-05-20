#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <queue>
#include <unordered_map>
#include <functional>
#include <cmath>
#include <algorithm>
#include <chrono>

struct Point {
    int x, y;
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
    bool operator!=(const Point& other) const {
        return !(*this == other);
    }
};

struct PathInfo {
    std::vector<Point> path; 
    double pathLength;        
    int expandedNodes;      
    int visitedNodes;     
    double executionTime;  
    bool pathFound;  
};

struct Node {
    Point pos;
    double g; 
    double h;
    double f; 
    Node* parent;
    int direction; 

    Node(Point p, double g_cost = 0, double h_cost = 0, Node* par = nullptr, int dir = -1)
        : pos(p), g(g_cost), h(h_cost), f(g_cost + h_cost), parent(par), direction(dir) {}
};

struct CompareNode {
    bool operator()(const Node* a, const Node* b) const {
        return a->f > b->f;
    }
};

class PathPlanner {
public:
    enum class HeuristicType {
        MANHATTAN,
        EUCLIDEAN,
        CHEBYSHEV
    };

    PathPlanner(int width, int height) : width_(width), height_(height) {}
    virtual ~PathPlanner() {}

    virtual PathInfo findPath(Point start, Point goal, 
                            const std::vector<std::vector<bool>>& map,
                            HeuristicType heuristic = HeuristicType::MANHATTAN) = 0;

protected:
    int width_;
    int height_;

    static double manhattanDistance(const Point& p1, const Point& p2);
    static double euclideanDistance(const Point& p1, const Point& p2);
    static double chebyshevDistance(const Point& p1, const Point& p2);
    
    bool isValid(int x, int y) const;
    double calculatePathLength(const std::vector<Point>& path) const;
};

class AStar : public PathPlanner {
public:
    AStar(int width, int height);
    virtual ~AStar();

    virtual PathInfo findPath(Point start, Point goal,
                            const std::vector<std::vector<bool>>& map,
                            HeuristicType heuristic = HeuristicType::MANHATTAN) override;

protected:
    const std::vector<std::pair<int, int>> fourDirections_ = {
        {-1, 0}, {1, 0}, {0, -1}, {0, 1}
    };
    
    const std::vector<std::pair<int, int>> eightDirections_ = {
        {-1, -1}, {-1, 0}, {-1, 1},
        {0, -1},           {0, 1},
        {1, -1},  {1, 0},  {1, 1}
    };

    std::vector<Point> getNeighbors(const Point& p, HeuristicType heuristic) const;
    std::vector<Point> reconstructPath(Node* endNode) const;
    void clearNodes(std::unordered_map<int, Node*>& nodes);
    const std::vector<std::pair<int, int>>& getDirections(HeuristicType heuristic) const;
};

class DirectionalAStar : public AStar {
public:
    DirectionalAStar(int width, int height);
    virtual PathInfo findPath(Point start, Point goal,
                            const std::vector<std::vector<bool>>& map,
                            HeuristicType heuristic = HeuristicType::EUCLIDEAN) override;

protected:
    int getDirectionIndex(const Point& from, const Point& to) const;
    double getDirectionChangeCost(int currentDir, int newDir) const;
    std::vector<std::pair<Point, double>> getDirectionalNeighbors(
        const Point& p, int currentDir, HeuristicType heuristic) const;
};

class ThetaStar : public AStar {
public:
    ThetaStar(int width, int height);
    virtual PathInfo findPath(Point start, Point goal,
                            const std::vector<std::vector<bool>>& map,
                            HeuristicType heuristic = HeuristicType::EUCLIDEAN) override;

private:
    bool lineOfSight(const Point& start, const Point& end,
                    const std::vector<std::vector<bool>>& map) const;
    void updateVertex(Node* s, Node* s2, Node* parent,
                     const std::vector<std::vector<bool>>& map);
};

class BiDirectionalAStar : public AStar {
public:
    BiDirectionalAStar(int width, int height);
    virtual PathInfo findPath(Point start, Point goal,
                            const std::vector<std::vector<bool>>& map,
                            HeuristicType heuristic = HeuristicType::MANHATTAN) override;

private:
    std::vector<Point> mergePaths(Node* nodeF, Node* nodeB) const;
};

#endif // ASTAR_H 
