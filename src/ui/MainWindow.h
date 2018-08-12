#pragma once

#include <memory>
#include <QMainWindow>

class Engine;
class World;
class QAction;
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
    void startSimulation();
    void stopSimulation();

    void simulationStarted();
    void simulationStopped();

    void about();

protected:

    QAction* _start_action;
    QAction* _stop_action;
    ViewerWidget* _viewer;
    std::shared_ptr<World> _world;
    Engine* _engine;
};

