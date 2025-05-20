#include "astar.h"
#include <QDebug>

double PathPlanner::manhattanDistance(const Point& p1, const Point& p2) {
    return std::abs(p1.x - p2.x) + std::abs(p1.y - p2.y);
}

double PathPlanner::euclideanDistance(const Point& p1, const Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

double PathPlanner::chebyshevDistance(const Point& p1, const Point& p2) {
    return std::max(std::abs(p1.x - p2.x), std::abs(p1.y - p2.y));
}

bool PathPlanner::isValid(int x, int y) const {
    return x >= 0 && x < width_ && y >= 0 && y < height_;
}

double PathPlanner::calculatePathLength(const std::vector<Point>& path) const {
    double length = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        length += euclideanDistance(path[i-1], path[i]);
    }
    return length;
}

AStar::AStar(int width, int height) : PathPlanner(width, height) {}

AStar::~AStar() {}

const std::vector<std::pair<int, int>>& AStar::getDirections(HeuristicType heuristic) const {
    return (heuristic == HeuristicType::MANHATTAN) ? fourDirections_ : eightDirections_;
}

std::vector<Point> AStar::getNeighbors(const Point& p, HeuristicType heuristic) const {
    std::vector<Point> neighbors;
    const auto& directions = getDirections(heuristic);
    for (const auto& dir : directions) {
        int newX = p.x + dir.first;
        int newY = p.y + dir.second;
        if (isValid(newX, newY)) {
            neighbors.push_back({newX, newY});
        }
    }
    return neighbors;
}

std::vector<Point> AStar::reconstructPath(Node* endNode) const {
    std::vector<Point> path;
    Node* current = endNode;
    while (current != nullptr) {
        path.push_back(current->pos);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void AStar::clearNodes(std::unordered_map<int, Node*>& nodes) {
    for (auto& pair : nodes) {
        delete pair.second;
    }
    nodes.clear();
}

PathInfo AStar::findPath(Point start, Point goal,
                        const std::vector<std::vector<bool>>& map,
                        HeuristicType heuristic) {
    auto startTime = std::chrono::high_resolution_clock::now();
    PathInfo result;
    result.pathFound = false;
    result.expandedNodes = 0;
    result.visitedNodes = 0;

    std::unordered_map<int, Node*> nodes;
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> openSet;
    std::unordered_map<int, bool> closedSet;

    Node* startNode = new Node(start);
    nodes[start.x * height_ + start.y] = startNode;
    openSet.push(startNode);

    while (!openSet.empty()) {
        Node* current = openSet.top();
        openSet.pop();

        if (current->pos == goal) {
            result.path = reconstructPath(current);
            result.pathFound = true;
            result.pathLength = calculatePathLength(result.path);
            clearNodes(nodes);
            
            auto endTime = std::chrono::high_resolution_clock::now();
            result.executionTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
            result.expandedNodes = closedSet.size();
            return result;
        }

        int currentKey = current->pos.x * height_ + current->pos.y;
        if (closedSet[currentKey]) {
            continue;
        }
        closedSet[currentKey] = true;
        result.expandedNodes++;

        for (const Point& neighbor : getNeighbors(current->pos, heuristic)) {
            result.visitedNodes++;
            
            if (!map[neighbor.x][neighbor.y]) {
                continue;
            }

            int neighborKey = neighbor.x * height_ + neighbor.y;
            if (closedSet[neighborKey]) {
                continue;
            }

            double moveCost;
            if (heuristic == HeuristicType::MANHATTAN) {
                moveCost = 1.0;
            } else {
                moveCost = (neighbor.x - current->pos.x != 0 && 
                           neighbor.y - current->pos.y != 0) ? 1.414 : 1.0;
            }
            double newG = current->g + moveCost;

            Node* neighborNode = nodes[neighborKey];
            if (neighborNode == nullptr) {
                double h;
                switch (heuristic) {
                    case HeuristicType::MANHATTAN:
                        h = manhattanDistance(neighbor, goal);
                        break;
                    case HeuristicType::EUCLIDEAN:
                        h = euclideanDistance(neighbor, goal);
                        break;
                    case HeuristicType::CHEBYSHEV:
                        h = chebyshevDistance(neighbor, goal);
                        break;
                    default:
                        h = manhattanDistance(neighbor, goal);
                }
                neighborNode = new Node(neighbor, newG, h, current);
                nodes[neighborKey] = neighborNode;
                openSet.push(neighborNode);
            }
            else if (newG < neighborNode->g) {
                neighborNode->g = newG;
                neighborNode->f = newG + neighborNode->h;
                neighborNode->parent = current;
            }
        }
    }

    clearNodes(nodes);
    auto endTime = std::chrono::high_resolution_clock::now();
    result.executionTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    result.expandedNodes = closedSet.size();
    return result;
}

// DirectionalAStar implementation
DirectionalAStar::DirectionalAStar(int width, int height) : AStar(width, height) {}

int DirectionalAStar::getDirectionIndex(const Point& from, const Point& to) const {
    int dx = to.x - from.x;
    int dy = to.y - from.y;
    dx = dx ? dx / std::abs(dx) : 0;
    dy = dy ? dy / std::abs(dy) : 0;

    static const std::vector<std::pair<int, int>> directions = {
        {-1, -1}, {-1, 0}, {-1, 1},
        {0, -1},  {0, 0},  {0, 1},
        {1, -1},  {1, 0},  {1, 1}
    };

    for (int i = 0; i < directions.size(); ++i) {
        if (directions[i].first == dx && directions[i].second == dy) {
            return i;
        }
    }
    return -1;
}

double DirectionalAStar::getDirectionChangeCost(int currentDir, int newDir) const {
    if (currentDir == -1 || newDir == -1) return 0.0;
    
    int diff = std::abs(currentDir - newDir);
    if (diff > 4) diff = 8 - diff;
    
    const double turnCost = 0.1;
    return diff * turnCost;
}

std::vector<std::pair<Point, double>> DirectionalAStar::getDirectionalNeighbors(
    const Point& p, int currentDir, HeuristicType heuristic) const {
    
    std::vector<std::pair<Point, double>> neighbors;
    const auto& directions = getDirections(heuristic);

    for (size_t i = 0; i < directions.size(); ++i) {
        int newX = p.x + directions[i].first;
        int newY = p.y + directions[i].second;
        
        if (!isValid(newX, newY)) continue;

        Point newPoint{newX, newY};
        int newDir = getDirectionIndex(p, newPoint);
        
        double moveCost = (directions[i].first != 0 && directions[i].second != 0) ? 1.414 : 1.0;
        moveCost += getDirectionChangeCost(currentDir, newDir);

        neighbors.push_back({newPoint, moveCost});
    }

    return neighbors;
}

PathInfo DirectionalAStar::findPath(Point start, Point goal,
                                  const std::vector<std::vector<bool>>& map,
                                  HeuristicType heuristic) {
    auto startTime = std::chrono::high_resolution_clock::now();
    PathInfo result;
    result.pathFound = false;
    result.expandedNodes = 0;
    result.visitedNodes = 0;

    qDebug() << "DirectionalAStar开始搜索路径: 起点(" << start.x << "," << start.y 
             << ") 终点(" << goal.x << "," << goal.y << ")";

    if (!map[start.x][start.y] || !map[goal.x][goal.y]) {
        qDebug() << "起点或终点在障碍物上，搜索终止";
        auto endTime = std::chrono::high_resolution_clock::now();
        result.executionTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
        return result;
    }

    std::unordered_map<int, Node*> nodes;
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> openSet;
    std::unordered_map<int, bool> closedSet;

    double h_start;
    switch (heuristic) {
        case HeuristicType::MANHATTAN:
            h_start = manhattanDistance(start, goal);
            break;
        case HeuristicType::EUCLIDEAN:
            h_start = euclideanDistance(start, goal);
            break;
        case HeuristicType::CHEBYSHEV:
            h_start = chebyshevDistance(start, goal);
            break;
        default:
            h_start = euclideanDistance(start, goal);
    }

    Node* startNode = new Node(start, 0, h_start);
    nodes[start.x * height_ + start.y] = startNode;
    openSet.push(startNode);

    qDebug() << "初始化完成，开始主循环";

    while (!openSet.empty()) {
        Node* current = openSet.top();
        openSet.pop();

        qDebug() << "当前节点:(" << current->pos.x << "," << current->pos.y 
                 << ") f=" << current->f << " g=" << current->g << " h=" << current->h;

        if (current->pos == goal) {
            qDebug() << "找到目标点！";
            result.path = reconstructPath(current);
            result.pathFound = true;
            result.pathLength = calculatePathLength(result.path);
            
            qDebug() << "路径长度:" << result.pathLength;
            
            for (auto& pair : nodes) {
                delete pair.second;
            }
            
            auto endTime = std::chrono::high_resolution_clock::now();
            result.executionTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
            result.expandedNodes = closedSet.size();
            return result;
        }

        int currentKey = current->pos.x * height_ + current->pos.y;
        if (closedSet[currentKey]) {
            continue;
        }
        closedSet[currentKey] = true;
        result.expandedNodes++;

        auto neighbors = getDirectionalNeighbors(current->pos, current->direction, heuristic);
        for (const auto& [neighbor, moveCost] : neighbors) {
            result.visitedNodes++;
            
            if (!map[neighbor.x][neighbor.y]) {
                continue;
            }

            int neighborKey = neighbor.x * height_ + neighbor.y;
            if (closedSet[neighborKey]) {
                continue;
            }

            double newG = current->g + moveCost;
            Node* neighborNode = nodes[neighborKey];

            if (!neighborNode) {
                double h;
                switch (heuristic) {
                    case HeuristicType::MANHATTAN:
                        h = manhattanDistance(neighbor, goal);
                        break;
                    case HeuristicType::EUCLIDEAN:
                        h = euclideanDistance(neighbor, goal);
                        break;
                    case HeuristicType::CHEBYSHEV:
                        h = chebyshevDistance(neighbor, goal);
                        break;
                    default:
                        h = euclideanDistance(neighbor, goal);
                }
                int newDir = getDirectionIndex(current->pos, neighbor);
                neighborNode = new Node(neighbor, newG, h, current, newDir);
                nodes[neighborKey] = neighborNode;
                openSet.push(neighborNode);
                qDebug() << "创建新节点 f=" << neighborNode->f;
            }
            else if (newG < neighborNode->g) {
                qDebug() << "更新节点 old_g=" << neighborNode->g << " new_g=" << newG;
                neighborNode->g = newG;
                neighborNode->f = newG + neighborNode->h;
                neighborNode->parent = current;
                neighborNode->direction = getDirectionIndex(current->pos, neighbor);
            }
        }
    }

    qDebug() << "未找到路径";
    for (auto& pair : nodes) {
        delete pair.second;
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    result.executionTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    result.expandedNodes = closedSet.size();
    return result;
}

// ThetaStar implementation
ThetaStar::ThetaStar(int width, int height) : AStar(width, height) {}

bool ThetaStar::lineOfSight(const Point& start, const Point& end,
                           const std::vector<std::vector<bool>>& map) const {
    int x0 = start.x, y0 = start.y;
    int x1 = end.x, y1 = end.y;
    
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    
    int x = x0;
    int y = y0;
    
    int n = 1 + dx + dy;
    int x_inc = (x1 > x0) ? 1 : -1;
    int y_inc = (y1 > y0) ? 1 : -1;
    
    int error = dx - dy;
    dx *= 2;
    dy *= 2;

    for (; n > 0; --n) {
        if (!isValid(x, y) || !map[x][y]) {
            return false;
        }

        if (error > 0) {
            x += x_inc;
            error -= dy;
        } else {
            y += y_inc;
            error += dx;
        }
    }
    
    return true;
}

void ThetaStar::updateVertex(Node* s, Node* s2, Node* parent,
                           const std::vector<std::vector<bool>>& map) {
    if (parent && lineOfSight(parent->pos, s2->pos, map)) {
        double newG = parent->g + euclideanDistance(parent->pos, s2->pos);
        if (newG < s2->g) {
            s2->g = newG;
            s2->f = newG + s2->h;
            s2->parent = parent;
        }
    } else {
        double newG = s->g + euclideanDistance(s->pos, s2->pos);
        if (newG < s2->g) {
            s2->g = newG;
            s2->f = newG + s2->h;
            s2->parent = s;
        }
    }
}

PathInfo ThetaStar::findPath(Point start, Point goal,
                           const std::vector<std::vector<bool>>& map,
                           HeuristicType heuristic) {
    auto startTime = std::chrono::high_resolution_clock::now();
    PathInfo result;
    result.pathFound = false;
    result.expandedNodes = 0;
    result.visitedNodes = 0;

    qDebug() << "Theta*开始搜索路径: 起点(" << start.x << "," << start.y 
             << ") 终点(" << goal.x << "," << goal.y << ")";

    if (!map[start.x][start.y] || !map[goal.x][goal.y]) {
        qDebug() << "起点或终点在障碍物上，搜索终止";
        auto endTime = std::chrono::high_resolution_clock::now();
        result.executionTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
        return result;
    }

    std::unordered_map<int, Node*> nodes;
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> openSet;
    std::unordered_map<int, bool> closedSet;

    Node* startNode = new Node(start);
    nodes[start.x * height_ + start.y] = startNode;
    openSet.push(startNode);

    while (!openSet.empty()) {
        Node* current = openSet.top();
        openSet.pop();

        if (current->pos == goal) {
            std::vector<Point> path;
            Node* node = current;
            while (node) {
                path.push_back(node->pos);
                node = node->parent;
            }
            std::reverse(path.begin(), path.end());
            
            result.path = path;
            result.pathFound = true;
            result.pathLength = calculatePathLength(path);
            
            qDebug() << "路径长度:" << result.pathLength;
            
            for (auto& pair : nodes) {
                delete pair.second;
            }
            
            auto endTime = std::chrono::high_resolution_clock::now();
            result.executionTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
            result.expandedNodes = closedSet.size();
            return result;
        }

        int currentKey = current->pos.x * height_ + current->pos.y;
        if (closedSet[currentKey]) {
            continue;
        }
        closedSet[currentKey] = true;
        result.expandedNodes++;

        for (const Point& neighbor : getNeighbors(current->pos, heuristic)) {
            result.visitedNodes++;
            
            if (!map[neighbor.x][neighbor.y]) {
                continue;
            }

            int neighborKey = neighbor.x * height_ + neighbor.y;
            if (closedSet[neighborKey]) {
                continue;
            }

            Node* neighborNode = nodes[neighborKey];
            if (!neighborNode) {
                double h = euclideanDistance(neighbor, goal);
                neighborNode = new Node(neighbor, std::numeric_limits<double>::infinity(), h);
                nodes[neighborKey] = neighborNode;
                openSet.push(neighborNode);
                qDebug() << "创建新节点 f=" << neighborNode->f;
            }

            updateVertex(current, neighborNode, current->parent, map);
        }
    }

    qDebug() << "未找到路径";
    for (auto& pair : nodes) {
        delete pair.second;
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    result.executionTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    result.expandedNodes = closedSet.size();
    return result;
}

// BiDirectionalAStar implementation
BiDirectionalAStar::BiDirectionalAStar(int width, int height) : AStar(width, height) {}

std::vector<Point> BiDirectionalAStar::mergePaths(Node* nodeF, Node* nodeB) const {
    std::vector<Point> path;
    
    Node* current = nodeF;
    while (current != nullptr) {
        path.push_back(current->pos);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
    
    current = nodeB->parent;  //跳过重复的相遇点
    while (current != nullptr) {
        path.push_back(current->pos);
        current = current->parent;
    }
    
    return path;
}

PathInfo BiDirectionalAStar::findPath(Point start, Point goal,
                                    const std::vector<std::vector<bool>>& map,
                                    HeuristicType heuristic) {
    auto startTime = std::chrono::high_resolution_clock::now();
    PathInfo result;
    result.pathFound = false;
    result.expandedNodes = 0;
    result.visitedNodes = 0;

    qDebug() << "BiDirectionalAStar开始搜索路径: 起点(" << start.x << "," << start.y 
             << ") 终点(" << goal.x << "," << goal.y << ")";

    if (!map[start.x][start.y] || !map[goal.x][goal.y]) {
        qDebug() << "起点或终点在障碍物上，搜索终止";
        auto endTime = std::chrono::high_resolution_clock::now();
        result.executionTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
        return result;
    }

    std::unordered_map<int, Node*> nodesF;
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> openSetF;
    std::unordered_map<int, bool> closedSetF;

    std::unordered_map<int, Node*> nodesB;
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> openSetB;
    std::unordered_map<int, bool> closedSetB;

    Node* startNode = new Node(start);
    nodesF[start.x * height_ + start.y] = startNode;
    openSetF.push(startNode);

    Node* goalNode = new Node(goal);
    nodesB[goal.x * height_ + goal.y] = goalNode;
    openSetB.push(goalNode);

    double bestCost = std::numeric_limits<double>::infinity();
    Node* bestNodeF = nullptr;
    Node* bestNodeB = nullptr;

    while (!openSetF.empty() && !openSetB.empty()) {
        // 前向搜索一步
        if (!openSetF.empty()) {
            Node* currentF = openSetF.top();
            openSetF.pop();

            qDebug() << "前向搜索当前节点:(" << currentF->pos.x << "," << currentF->pos.y 
                     << ") f=" << currentF->f << " g=" << currentF->g << " h=" << currentF->h;

            int currentKeyF = currentF->pos.x * height_ + currentF->pos.y;
            if (closedSetF[currentKeyF]) {
                continue;
            }
            closedSetF[currentKeyF] = true;
            result.expandedNodes++;

            if (nodesB[currentKeyF] != nullptr) {
                double totalCost = currentF->g + nodesB[currentKeyF]->g;
                if (totalCost < bestCost) {
                    bestCost = totalCost;
                    bestNodeF = currentF;
                    bestNodeB = nodesB[currentKeyF];
                    qDebug() << "找到更好的相遇点:(" << currentF->pos.x << "," << currentF->pos.y 
                             << ") 总代价=" << totalCost;
                }
            }

            for (const Point& neighbor : getNeighbors(currentF->pos, heuristic)) {
                result.visitedNodes++;
                
                if (!map[neighbor.x][neighbor.y]) {
                    continue;
                }

                int neighborKey = neighbor.x * height_ + neighbor.y;
                if (closedSetF[neighborKey]) {
                    continue;
                }

                double moveCost = (heuristic == HeuristicType::MANHATTAN) ? 1.0 :
                    ((neighbor.x - currentF->pos.x != 0 && 
                      neighbor.y - currentF->pos.y != 0) ? 1.414 : 1.0);
                double newG = currentF->g + moveCost;

                Node* neighborNode = nodesF[neighborKey];
                if (!neighborNode) {
                    double h;
                    switch (heuristic) {
                        case HeuristicType::MANHATTAN:
                            h = manhattanDistance(neighbor, goal);
                            break;
                        case HeuristicType::EUCLIDEAN:
                            h = euclideanDistance(neighbor, goal);
                            break;
                        case HeuristicType::CHEBYSHEV:
                            h = chebyshevDistance(neighbor, goal);
                            break;
                        default:
                            h = manhattanDistance(neighbor, goal);
                    }
                    neighborNode = new Node(neighbor, newG, h, currentF);
                    nodesF[neighborKey] = neighborNode;
                    openSetF.push(neighborNode);
                    qDebug() << "前向搜索创建新节点:(" << neighbor.x << "," << neighbor.y 
                             << ") f=" << neighborNode->f;
                }
                else if (newG < neighborNode->g) {
                    qDebug() << "前向搜索更新节点:(" << neighbor.x << "," << neighbor.y 
                             << ") old_g=" << neighborNode->g << " new_g=" << newG;
                    neighborNode->g = newG;
                    neighborNode->f = newG + neighborNode->h;
                    neighborNode->parent = currentF;
                }
            }
        }

        if (!openSetB.empty()) {
            Node* currentB = openSetB.top();
            openSetB.pop();

            qDebug() << "后向搜索当前节点:(" << currentB->pos.x << "," << currentB->pos.y 
                     << ") f=" << currentB->f << " g=" << currentB->g << " h=" << currentB->h;

            int currentKeyB = currentB->pos.x * height_ + currentB->pos.y;
            if (closedSetB[currentKeyB]) {
                continue;
            }
            closedSetB[currentKeyB] = true;
            result.expandedNodes++;

            if (nodesF[currentKeyB] != nullptr) {
                double totalCost = nodesF[currentKeyB]->g + currentB->g;
                if (totalCost < bestCost) {
                    bestCost = totalCost;
                    bestNodeF = nodesF[currentKeyB];
                    bestNodeB = currentB;
                    qDebug() << "找到更好的相遇点:(" << currentB->pos.x << "," << currentB->pos.y 
                             << ") 总代价=" << totalCost;
                }
            }

            for (const Point& neighbor : getNeighbors(currentB->pos, heuristic)) {
                result.visitedNodes++;
                
                if (!map[neighbor.x][neighbor.y]) {
                    continue;
                }

                int neighborKey = neighbor.x * height_ + neighbor.y;
                if (closedSetB[neighborKey]) {
                    continue;
                }

                double moveCost = (heuristic == HeuristicType::MANHATTAN) ? 1.0 :
                    ((neighbor.x - currentB->pos.x != 0 && 
                      neighbor.y - currentB->pos.y != 0) ? 1.414 : 1.0);
                double newG = currentB->g + moveCost;

                Node* neighborNode = nodesB[neighborKey];
                if (!neighborNode) {
                    double h;
                    switch (heuristic) {
                        case HeuristicType::MANHATTAN:
                            h = manhattanDistance(neighbor, start);
                            break;
                        case HeuristicType::EUCLIDEAN:
                            h = euclideanDistance(neighbor, start);
                            break;
                        case HeuristicType::CHEBYSHEV:
                            h = chebyshevDistance(neighbor, start);
                            break;
                        default:
                            h = manhattanDistance(neighbor, start);
                    }
                    neighborNode = new Node(neighbor, newG, h, currentB);
                    nodesB[neighborKey] = neighborNode;
                    openSetB.push(neighborNode);
                    qDebug() << "后向搜索创建新节点:(" << neighbor.x << "," << neighbor.y 
                             << ") f=" << neighborNode->f;
                }
                else if (newG < neighborNode->g) {
                    qDebug() << "后向搜索更新节点:(" << neighbor.x << "," << neighbor.y 
                             << ") old_g=" << neighborNode->g << " new_g=" << newG;
                    neighborNode->g = newG;
                    neighborNode->f = newG + neighborNode->h;
                    neighborNode->parent = currentB;
                }
            }
        }
    }

    if (bestNodeF != nullptr && bestNodeB != nullptr) {
        qDebug() << "找到最优路径！";
        result.path = mergePaths(bestNodeF, bestNodeB);
        result.pathFound = true;
        result.pathLength = calculatePathLength(result.path);
        qDebug() << "路径长度:" << result.pathLength;
    } else {
        qDebug() << "未找到路径";
    }

    for (auto& pair : nodesF) {
        delete pair.second;
    }
    for (auto& pair : nodesB) {
        delete pair.second;
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    result.executionTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    result.expandedNodes = closedSetF.size() + closedSetB.size();
    return result;
} 
