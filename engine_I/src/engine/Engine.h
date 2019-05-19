
#pragma once

#include <QThread>
#include "Solver.h"

class Engine : public QThread
{
public:

    Engine(QObject* parent);

    void setWorld( std::shared_ptr<World> world );

protected:

    void run() override;

protected:

    Solver m_solver;
};

