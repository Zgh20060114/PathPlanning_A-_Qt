#include "mainwindow.h"
#include <QMessageBox>
#include "astar.h"

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setupUI();
}

void MainWindow::setupUI() {
    QWidget* centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);
    QVBoxLayout* mainLayout = new QVBoxLayout(centralWidget);
    QHBoxLayout* controlLayout = new QHBoxLayout();

    gridMap_ = new GridMap(40, 40, this);

    QLabel* algorithmLabel = new QLabel("算法:", this);
    algorithmComboBox_ = new QComboBox(this);
    algorithmComboBox_->addItem("A*");
    algorithmComboBox_->addItem("方向引导A*");
    algorithmComboBox_->addItem("Theta*");
    algorithmComboBox_->addItem("双向A*");

    QLabel* heuristicLabel = new QLabel("启发函数:", this);
    heuristicComboBox_ = new QComboBox(this);
    heuristicComboBox_->addItem("曼哈顿距离", static_cast<int>(PathPlanner::HeuristicType::MANHATTAN));
    heuristicComboBox_->addItem("欧几里得距离", static_cast<int>(PathPlanner::HeuristicType::EUCLIDEAN));
    heuristicComboBox_->addItem("切比雪夫距离", static_cast<int>(PathPlanner::HeuristicType::CHEBYSHEV));

    findPathButton_ = new QPushButton("查找路径", this);
    resetMapButton_ = new QPushButton("重置地图", this);
    clearPathButton_ = new QPushButton("清除路径", this);

    statsTextEdit_ = new QTextEdit(this);
    statsTextEdit_->setReadOnly(true);
    statsTextEdit_->setMaximumHeight(100);
    statsTextEdit_->setPlaceholderText("性能统计信息将在这里显示");

    controlLayout->addWidget(algorithmLabel);
    controlLayout->addWidget(algorithmComboBox_);
    controlLayout->addWidget(heuristicLabel);
    controlLayout->addWidget(heuristicComboBox_);
    controlLayout->addWidget(findPathButton_);
    controlLayout->addWidget(resetMapButton_);
    controlLayout->addWidget(clearPathButton_);
    controlLayout->addStretch();

    mainLayout->addWidget(gridMap_);
    mainLayout->addLayout(controlLayout);
    mainLayout->addWidget(statsTextEdit_);

    connect(findPathButton_, &QPushButton::clicked, this, &MainWindow::onFindPathClicked);
    connect(resetMapButton_, &QPushButton::clicked, this, &MainWindow::onResetMapClicked);
    connect(clearPathButton_, &QPushButton::clicked, this, &MainWindow::onClearPathClicked);
    connect(algorithmComboBox_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onAlgorithmChanged);

    setWindowTitle("路径规划");
}

PathPlanner* MainWindow::createPathPlanner(int index) {
    int width = 40, height = 40;
    switch (index) {
        case 0: return new AStar(width, height);
        case 1: return new DirectionalAStar(width, height);
        case 2: return new ThetaStar(width, height);
        case 3: return new BiDirectionalAStar(width, height);
        default: return new AStar(width, height);
    }
}

void MainWindow::updateStats(const PathInfo& pathInfo) {
    QString stats;
    if (pathInfo.pathFound) {
        stats = QString("路径长度: %1\n执行时间: %2 ms\n扩展节点数: %3\n访问节点数: %4")
                    .arg(pathInfo.pathLength, 0, 'f', 2)
                    .arg(pathInfo.executionTime, 0, 'f', 2)
                    .arg(pathInfo.expandedNodes)
                    .arg(pathInfo.visitedNodes);
    } else {
        stats = "未找到路径\n";
        stats += QString("执行时间: %1 ms\n扩展节点数: %2\n访问节点数: %3")
                    .arg(pathInfo.executionTime, 0, 'f', 2)
                    .arg(pathInfo.expandedNodes)
                    .arg(pathInfo.visitedNodes);
    }
    statsTextEdit_->setText(stats);
}

void MainWindow::onFindPathClicked() {
    PathPlanner* planner = createPathPlanner(algorithmComboBox_->currentIndex());
    PathPlanner::HeuristicType heuristic = static_cast<PathPlanner::HeuristicType>(
        heuristicComboBox_->currentData().toInt());
    
    Point start = gridMap_->getStart();
    Point goal = gridMap_->getGoal();
    const std::vector<std::vector<bool>>& map = gridMap_->getMap();
    
    PathInfo result = planner->findPath(start, goal, map, heuristic);
    
    updateStats(result);
    if (result.pathFound) {
        gridMap_->setPath(result.path);
    }
    
    delete planner;
}

void MainWindow::onResetMapClicked() {
    gridMap_->resetMap();
    statsTextEdit_->clear();
}

void MainWindow::onClearPathClicked() {
    gridMap_->clearPath();
    statsTextEdit_->clear();
}

void MainWindow::onAlgorithmChanged(int index) {
    if (index == 1 || index == 2) {
        heuristicComboBox_->setCurrentIndex(1); 
    }
} 
