#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QTextEdit>
#include "gridmap.h"
#include "astar.h"

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);

private slots:
    void onFindPathClicked();
    void onResetMapClicked();
    void onClearPathClicked();
    void onAlgorithmChanged(int index);

private:
    GridMap* gridMap_;
    QComboBox* algorithmComboBox_;
    QComboBox* heuristicComboBox_;
    QPushButton* findPathButton_;
    QPushButton* resetMapButton_;
    QPushButton* clearPathButton_;
    QTextEdit* statsTextEdit_;

    void setupUI();
    void updateStats(const PathInfo& pathInfo);
    PathPlanner* createPathPlanner(int index);
};

#endif // MAINWINDOW_H 
