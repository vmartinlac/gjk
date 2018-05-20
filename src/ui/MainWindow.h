#pragma once

#include <memory>
#include <QMainWindow>

class World;
class QTimer;
class ViewerWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    MainWindow(
        std::shared_ptr<World> world,
        QWidget* parent=nullptr);

protected slots:

    void initSimulation();
    void runSimulation(bool);
    void simulationStep();
    void about();

protected:

    ViewerWidget* _viewer;
    std::shared_ptr<World> _world;
    QTimer* _timer_simulation;
};

