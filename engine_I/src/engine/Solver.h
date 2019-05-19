
#pragma once

#include <memory>
#include "World.h"

class Solver
{
public:

    Solver();

    void setWorld( std::shared_ptr<World> world );

    void step(double dt);

protected:

    std::shared_ptr<World> m_world;
    int m_num_bodies;
    int m_num_links;

protected:

    void retrieveCurrentState(Eigen::VectorXd& X);
    void computeStateDerivative(const Eigen::VectorXd& X, Eigen::VectorXd& f);
    void normalizeState(Eigen::VectorXd& X);
    void applyState(const Eigen::VectorXd& X);
    BodyState extractIndividualState(const Eigen::VectorXd& X, int id);
    void computeTimestep(const Eigen::VectorXd& X, double mindt, double maxdt, double& dt, bool& completed);

    void integrate_explicit_euler(
        double mindt, // input : minimal dt
        double maxdt, // input : maximal dt
        double& dt, // output : effective dt
        bool& completed); // output : (completed == true) <=> (dt == maxdt)

    void integrate_semiimplicit_euler(
        double mindt, // input : minimal dt
        double maxdt, // input : maximal dt
        double& dt, // output : effective dt
        bool& completed); // output : (completed == true) <=> (dt == maxdt)
};
