#pragma once

#include <QTimer>
#include <vector>
#include "BodyInstance.h"
#include "Spring.h"

class Solver : public QObject
{
    Q_OBJECT

public:

    Solver();
    ~Solver();
    
    static Solver* instance()
    {
        return _instance;
    }

public slots:

    void home();
    void startSimulation();
    void stopSimulation();
    void step();

protected:

    void retrieveCurrentState(Eigen::VectorXd& X);
    void computeStateDerivative(const Eigen::VectorXd& X, Eigen::VectorXd& f);
    void normalizeState(Eigen::VectorXd& X);
    void applyState(const Eigen::VectorXd& X);
    BodyState extractIndividualState(const Eigen::VectorXd& X, int id);
    void computeTimestep(const Eigen::VectorXd& X, double maxdt, double& dt, bool& completed);

protected:

    void explicitEulerMethod(double maxdt, double& dt, bool& completed);
    void semiimplicitEulerMethod(double maxdt, double& dt, bool& completed);
    void RK4Method(double maxdt, double& dt, bool& completed);

protected:

    QTimer* _timer;
    int _timestep;

    static Solver* _instance;
};

