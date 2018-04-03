#include <QTime>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <iostream>
#include <memory>
#include "Solver.h"
#include "Collision.h"

// declarations of classes in charge of solving ODE.

class Solver::CrankNicholsonMethod
{
public:
    CrankNicholsonMethod(Solver* solver, double theta=0.5);
    void run(double maxdt, double& dt, bool& completed);
protected:
    /*
    At each timestep, we solve :
    $$ F( X_{n+1} ) = G( X_n ) $$
    */
    void functionF(const Eigen::VectorXd& X, double delta_t, Eigen::VectorXd& Y);
    void functionG(const Eigen::VectorXd& X, double delta_t, Eigen::VectorXd& Y);
    void inverseF(const Eigen::VectorXd& Y, const Eigen::VectorXd& X0, double delta_t, Eigen::VectorXd& X);
    void constructJacobianOfF(const Eigen::VectorXd& X, double delta_t, Eigen::SparseMatrix<double>& jacobian);
protected:
    double _theta;
    Solver* _solver;
};

class Solver::RK4Method
{
public:
    RK4Method(Solver* solver);
    void run(double maxdt, double& dt, bool& completed);
protected:
    Solver* _solver;
};

class Solver::ExplicitEulerMethod
{
public:
    ExplicitEulerMethod(Solver* solver);
    void run(double maxdt, double& dt, bool& completed);
protected:
    Solver* _solver;
};

class Solver::SemiimplicitEulerMethod
{
public:
    SemiimplicitEulerMethod(Solver* solver);
    void run(double maxdt, double& dt, bool& completed);
protected:
    Solver* _solver;
};

// solver class.

Solver* Solver::_instance = nullptr;

Solver::Solver()
{
    if(_instance != nullptr)
    {
        std::abort();
    }

    _instance = this;

    _node = new osg::Group;

    _numBodies = 0;
    _dim = 0;

    _timestep = 1000 / 60;

    _timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(step()));

    _gravity = Eigen::Vector3d{0.0, 0.0, -9.81};
    _linearViscosity = 5.0e3;
    _angularViscosity = 1.0e5;

    //////
    _linearViscosity = 0.0;
    _angularViscosity = 0.0;
    //////

    // create axes node.

    {
        const double r = 0.1;
        const double l = 5.0;

        osg::ref_ptr<osg::Sphere> sphere = new osg::Sphere( osg::Vec3(0.0, 0.0, 0.0), 2.0*r );
        osg::ref_ptr<osg::Cylinder> cylx = new osg::Cylinder( osg::Vec3(0.5*l, 0.0, 0.0), r, l);
        osg::ref_ptr<osg::Cylinder> cyly = new osg::Cylinder( osg::Vec3(0.0, 0.5*l, 0.0), r, l);
        osg::ref_ptr<osg::Cylinder> cylz = new osg::Cylinder( osg::Vec3(0.0, 0.0, 0.5*l), r, l);
        cylx->setRotation( osg::Quat(0.0, -M_SQRT2*0.5, 0.0, M_SQRT2*0.5) );
        cyly->setRotation( osg::Quat(-M_SQRT2*0.5, 0.0, 0.0, M_SQRT2*0.5) );

        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->addDrawable( new osg::ShapeDrawable( sphere )); 
        geode->addDrawable( new osg::ShapeDrawable( cylx )); 
        geode->addDrawable( new osg::ShapeDrawable( cyly )); 
        geode->addDrawable( new osg::ShapeDrawable( cylz )); 

        _node->addChild(geode);
    }
}

Solver::~Solver()
{
    if(_instance == nullptr)
    {
        std::abort();
    }

    _instance = nullptr;
}

void Solver::addBody(BodyPtr body)
{
    _bodies.push_back( body );
    _node->addChild( body->getRepresentation() );

    body->setId(_numBodies);
    _numBodies++;
    _dim = _numBodies*13;
}

void Solver::addSpring( SpringPtr spring )
{
    _springs.push_back( spring );
    _node->addChild( spring->getRepresentation() );
}

void Solver::home()
{
    for(BodyPtr& b : _bodies)
    {
        b->currentState() = b->initialState();
    }
    syncRepresentation();
}

void Solver::startSimulation()
{
    _timer->start(_timestep);
}

void Solver::stopSimulation()
{
    _timer->stop();
}

void Solver::clear()
{
    _bodies.clear();
    _springs.clear();
    _numBodies = 0;
    _dim = 0;
}

void Solver::syncRepresentation()
{
   for(BodyPtr& b : _bodies)
   {
      b->syncRepresentation();
   }

   for(SpringPtr& s : _springs)
   {
      s->syncRepresentation();
   }
}

void Solver::step()
{
    bool go_on = true;
    double time_left = double(_timestep) * 1.0e-3;
    int num_iterations = 0;

    while(go_on)
    {
        bool completed;
        double dt;

        CrankNicholsonMethod method(this);
        //RK4Method method(this);
        //ExplicitEulerMethod method(this);

        method.run(time_left, dt, completed);

        go_on = !completed;
        time_left -= dt;

        // detect collisions.

        /*
        for(int i=0; i<_bodies.size(); i++)
        {
            BodyPtr b1 = _bodies[i];
            b1->representationState() = b1->collisionDetectionState();
            if( b1->isMoving() )
            {
                bool collision = false;

                for(int j=0; collision == false && j<_bodies.size(); j++)
                {
                    if(j != i)
                    {
                        BodyPtr b2 = _bodies[j];

                        Eigen::Vector3d collision_point;

                        collision = Collision::detect(
                            b1.get(),
                            b2.get(),
                            collision_point);
                    }
                }
            }
        }
        */

        num_iterations++;
    }

    std::cout << "num_iterations = " << num_iterations << std::endl;

    syncRepresentation();
}

void Solver::retrieveCurrentState(Eigen::VectorXd& X)
{
    X.resize( _dim );

    for(int i=0; i<_numBodies; i++)
    {
        Body::State& s = _bodies[i]->currentState();

        X.segment<3>(13*i+0) = s.position;
        X.segment<4>(13*i+3) = s.attitude.coeffs();
        X.segment<3>(13*i+7) = s.linear_momentum;
        X.segment<3>(13*i+10) = s.angular_momentum;
    }
}

void Solver::normalizeState(Eigen::VectorXd& X)
{
    assert( X.size() == _dim );

    for(int i=0; i<_numBodies; i++)
    {
        // we could just do X.segment<4>(13*i+3).normalize().
        Eigen::Quaterniond u;
        u.coeffs() = X.segment<4>(13*i+3);
        u.normalize();
        X.segment<4>(13*i+3) = u.coeffs();
    }
}

void Solver::applyState(const Eigen::VectorXd& X)
{
    assert( X.size() == _dim );

    for(int i=0; i<_numBodies; i++)
    {
        BodyPtr body = _bodies[i];
        if(body->isMoving())
        {
            body->currentState() = extractIndividualState(X, i);
        }
    }
}

void Solver::computeStateDerivative(const Eigen::VectorXd& X, Eigen::VectorXd& f)
{
    assert( X.size() == _dim );

    f.resize(_dim);

    for(int i=0; i<_numBodies; i++)
    {
        BodyPtr body = _bodies[i];

        if(body->isFixed())
        {
            f.segment<13>(13*i).setZero();
        }
        else
        {
            Body::State state = extractIndividualState(X, i);

            const Eigen::Vector3d linear_velocity = state.linear_momentum / body->getMass();
            const Eigen::Vector3d angular_velocity = body->getInertiaTensorSolver().solve( state.angular_momentum );

            Eigen::Vector3d resultant_force = Eigen::Vector3d::Zero();
            Eigen::Vector3d resultant_torque = Eigen::Vector3d::Zero();

            resultant_force += body->getMass() * _gravity;
            resultant_force -= _linearViscosity * linear_velocity;
            resultant_torque -= _angularViscosity * angular_velocity;

            const int id = body->getId();

            f.segment<3>(13*id+0) = linear_velocity;
            f.segment<4>(13*id+3) = 0.5 * ( state.attitude * Eigen::Quaterniond(0.0, angular_velocity.x(), angular_velocity.y(), angular_velocity.z()) ).coeffs();
            f.segment<3>(13*id+7) = resultant_force;
            f.segment<3>(13*id+10) = resultant_torque;
        }
    }

    for(SpringPtr spring : _springs)
    {
        Body* B1 = spring->getBody1();
        Body* B2 = spring->getBody2();

        const int id1 = B1->getId();
        const int id2 = B2->getId();

        Body::State S1 = extractIndividualState(X, id1);
        Body::State S2 = extractIndividualState(X, id2);

        const Eigen::Vector3d P1 = S1.position + S1.attitude * spring->getAnchor1();
        const Eigen::Vector3d P2 = S2.position + S2.attitude * spring->getAnchor2();

        const double L = (P2 - P1).norm();
        const double cte = spring->getElasticityCoefficient() * (L - spring->getFreeLength()) / spring->getFreeLength();
        const Eigen::Vector3d F = (P2-P1).normalized() * cte;

        if(B1->isMoving())
        {
            f.segment<3>(13*id1+7) += F;
            f.segment<3>(13*id1+10) += spring->getAnchor1().cross( B1->currentState().attitude.inverse() * F );
        }

        if(B2->isMoving())
        {
            f.segment<3>(13*id2+7) -= F;
            f.segment<3>(13*id2+10) -= spring->getAnchor2().cross( B2->currentState().attitude.inverse() * F );
        }
    }
}

Body::State Solver::extractIndividualState(const Eigen::VectorXd& X, int id)
{
    Body::State s;

    s.position = X.segment<3>(13*id+0);
    s.attitude.coeffs() = X.segment<4>(13*id+3);
    s.attitude.normalize();
    s.linear_momentum = X.segment<3>(13*id+7);
    s.angular_momentum = X.segment<3>(13*id+10);

    return s;
}

void Solver::computeTimestep(const Eigen::VectorXd& X, double maxdt, double& dt, bool& completed)
{
    completed = true;
    dt = maxdt;

    for(int i=0; i<_numBodies; i++)
    {
        BodyPtr body = _bodies[i];
        Body::State state = extractIndividualState(X, i);

        const Eigen::Vector3d linear_velocity = state.linear_momentum / body->getMass();
        const Eigen::Vector3d angular_velocity = body->getInertiaTensorSolver().solve( state.angular_momentum );

        const double bs_radius = body->getBoundingSphere().radius;
        const double max_dist = bs_radius * 0.01;
        const double dt1 = max_dist / linear_velocity.norm();
        const double dt2 = max_dist / std::max(1.0e-5, angular_velocity.norm()*bs_radius);
        const double dt3 = std::min(dt1, dt2);

        if(dt3 < dt)
        {
            completed = false;
            dt = dt3;
        }
    }
}

// Crank-Nicholson method for solving the ODE.

Solver::CrankNicholsonMethod::CrankNicholsonMethod(Solver* solver, double theta)
{
    _theta = theta;
    _solver = solver;
}

void Solver::CrankNicholsonMethod::run(double maxdt, double& dt, bool& completed)
{
    Eigen::VectorXd Xpre;
    _solver->retrieveCurrentState(Xpre);

    _solver->computeTimestep(Xpre, maxdt, dt, completed);

    Eigen::VectorXd G;
    functionG(Xpre, dt, G);

    Eigen::VectorXd Xpost;
    inverseF(G, Xpre, dt, Xpost);

    _solver->applyState(Xpost);
}

void Solver::CrankNicholsonMethod::functionF(const Eigen::VectorXd& X, double delta_t, Eigen::VectorXd& Y)
{
    Eigen::VectorXd f;
    _solver->computeStateDerivative(X, f);

    Y = (1.0/delta_t)*X - (1.0-_theta)*f;
}

void Solver::CrankNicholsonMethod::functionG(const Eigen::VectorXd& X, double delta_t, Eigen::VectorXd& Y)
{
    Eigen::VectorXd f;
    _solver->computeStateDerivative(X, f);

    Y = (1.0/delta_t)*X + _theta*f;
}

void Solver::CrankNicholsonMethod::inverseF(const Eigen::VectorXd& Y, const Eigen::VectorXd& X0, double delta_t, Eigen::VectorXd& X)
{
    X = X0;

    int max_iterations = 300;
    bool go_on = true;
    while(go_on && max_iterations > 0)
    {
        Eigen::VectorXd F;
        functionF(X, delta_t, F);

        Eigen::SparseMatrix<double> jacobian;
        constructJacobianOfF(X, delta_t, jacobian);

        Eigen::ConjugateGradient< Eigen::SparseMatrix<double> > solver;
        solver.compute(jacobian);

        Eigen::VectorXd DX = solver.solve(Y-F);
        X += DX;
        _solver->normalizeState(X);

        go_on = (DX.lpNorm<1>() > 1.0e-2);
        max_iterations--;
    }
}

void Solver::CrankNicholsonMethod::constructJacobianOfF(const Eigen::VectorXd& X, double delta_t, Eigen::SparseMatrix<double>& jacobian)
{
    // TODO !
    throw;

    const int d = _solver->_dim;

    Eigen::VectorXi reservation(d);

    for(int i=0; i<_solver->_numBodies; i++)
    {
        reservation.segment<3>(13*i+0) = Eigen::VectorXi::Constant(3, 2);
        reservation.segment<4>(13*i+3) = Eigen::VectorXi::Constant(4, 3);
        reservation.segment<6>(13*i+7) = Eigen::VectorXi::Constant(6, 2);
    }

    for(SpringPtr spring : _solver->_springs)
    {
        const int id1 = spring->getBody1()->getId();
        const int id2 = spring->getBody2()->getId();

        reservation.segment<6>(13*id1+7) += Eigen::VectorXi::Constant(6, 1);
        reservation.segment<6>(13*id2+7) += Eigen::VectorXi::Constant(6, 1);
    }

    jacobian.resize(d, d);
    jacobian.setZero();
    jacobian.reserve(reservation);

    for(int i=0; i<d; i++)
    {
        jacobian.coeffRef(i, i) += 1.0/delta_t;
    }

    const double cte = -(1.0-_theta);

    for(int i=0; i<_solver->_numBodies; i++)
    {
        BodyPtr body = _solver->_bodies[i];
        Body::State state = _solver->extractIndividualState(X, i);

        jacobian.coeffRef(13*i+0, 13*i+7) = 1.0 / body->getMass();
        jacobian.coeffRef(13*i+1, 13*i+8) = 1.0 / body->getMass();
        jacobian.coeffRef(13*i+2, 13*i+9) = 1.0 / body->getMass();

        jacobian.coeffRef(13*i+7, 13*i+7) = -_solver->_linearViscosity / body->getMass();
        jacobian.coeffRef(13*i+8, 13*i+8) = -_solver->_linearViscosity / body->getMass();
        jacobian.coeffRef(13*i+9, 13*i+9) = -_solver->_linearViscosity / body->getMass();
    }

    for(SpringPtr spring : _solver->_springs)
    {
        ;
    }
}

// Explicit Euler method.

Solver::ExplicitEulerMethod::ExplicitEulerMethod(Solver* solver)
{
    _solver = solver;
}

void Solver::ExplicitEulerMethod::run(double maxdt, double& dt, bool& completed)
{
    completed = true;
    dt = maxdt;

    Eigen::VectorXd X;
    _solver->retrieveCurrentState(X);

    _solver->computeTimestep(X, maxdt, dt, completed);

    Eigen::VectorXd f;
    _solver->computeStateDerivative(X, f);

    X += dt*f;

    _solver->normalizeState(X);
    _solver->applyState(X);
}

// Runge Kutta 4 method.

Solver::RK4Method::RK4Method(Solver* solver)
{
    _solver = solver;
}

void Solver::RK4Method::run(double maxdt, double& dt, bool& completed)
{
    completed = true;
    dt = maxdt;

    Eigen::VectorXd X;
    _solver->retrieveCurrentState(X);

    _solver->computeTimestep(X, maxdt, dt, completed);

    Eigen::VectorXd f1;
    _solver->computeStateDerivative(X, f1);

    Eigen::VectorXd f2;
    Eigen::VectorXd X2 = X + 0.5*dt*f1;
    _solver->normalizeState(X2);
    _solver->computeStateDerivative(X2, f2);

    Eigen::VectorXd f3;
    Eigen::VectorXd X3 = X + 0.5*dt*f2;
    _solver->normalizeState(X3);
    _solver->computeStateDerivative(X3, f3);

    Eigen::VectorXd f4;
    Eigen::VectorXd X4 = X + dt*f3;
    _solver->normalizeState(X4);
    _solver->computeStateDerivative(X4, f4);

    Eigen::VectorXd f = (1.0/6.0) * ( f1 + 2.0*f2 + 2.0*f3 + f4 );
    X += dt*f;
    _solver->normalizeState(X);
    _solver->applyState(X);
}

// Semiimplicit Euler method.

Solver::SemiimplicitEulerMethod::SemiimplicitEulerMethod(Solver* solver)
{
    _solver = solver;
}

void Solver::SemiimplicitEulerMethod::run(double maxdt, double& dt, bool& completed)
{
    // TODO !
    throw;
    /*
    completed = true;
    dt = maxdt;

    Eigen::VectorXd X;
    _solver->retrieveCurrentState(X);

    _solver->computeTimestep(X, maxdt, dt, completed);

    Eigen::VectorXd f;
    _solver->computeStateDerivative(X, f);

    X += dt*f;
    _solver->normalizeState(X);

    Eigen::VectorXd f;
    _solver->computeStateDerivative(X, f);

    _solver->applyState(X);
    */
}
