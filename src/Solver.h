#pragma once

#include <QTimer>
//#include <QThread>
#include <vector>
#include "Body.h"
#include "Spring.h"

class Solver : public QObject
{
    Q_OBJECT
public:

    Solver();
    ~Solver();
    
    void addBody( BodyPtr body );

    void addSpring( SpringPtr spring );

    static Solver* instance()
    {
        return _instance;
    }

    osg::Node* node() { return _node; }

public slots:

    void home();
    void startSimulation();
    void stopSimulation();
    void clear();

protected slots:

    void step();

protected:

    void syncRepresentation();
    void retrieveCurrentState(Eigen::VectorXd& X);
    void computeStateDerivative(const Eigen::VectorXd& X, Eigen::VectorXd& f);
    void normalizeState(Eigen::VectorXd& X);
    void applyState(const Eigen::VectorXd& X);
    Body::State extractIndividualState(const Eigen::VectorXd& X, int id);
    void computeTimestep(const Eigen::VectorXd& X, double maxdt, double& dt, bool& completed);

protected:

    class CrankNicholsonMethod;
    class ExplicitEulerMethod;
    class RK4Method;

protected:

    int _numBodies;
    int _dim;
    osg::ref_ptr< osg::Group > _node;
    std::vector< BodyPtr > _bodies;
    std::vector< SpringPtr > _springs;
    QTimer* _timer;
    int _timestep;
    double _time;
    static Solver* _instance;
    Eigen::Vector3d _gravity;
    double _linearViscosity;
    double _angularViscosity;
};

