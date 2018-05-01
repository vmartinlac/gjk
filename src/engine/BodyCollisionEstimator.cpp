#include <iostream>
#include "BodyCollisionEstimator.h"

BodyCollisionEstimatorSA::BodyCollisionEstimatorSA()
{
    _epsilon = 1.0e-4;
}

void BodyCollisionEstimatorSA::run(
    std::shared_ptr<BodyInstance> b1,
    std::shared_ptr<BodyInstance> b2,
    BodyInstance::KindOfState k )
{
    Eigen::Vector3d X = b1->project(b1->state(k).position, k);

    int max_iter = 2000;
    bool go_on = true;

    _converged = true;

    //std::cout << "DÉBUT CALCUL POINTS PROXIMAUX" << std::endl;
    while(go_on)
    {
        if(max_iter <= 0)
        {
            std::cout << "Computation of proximal points between convexes failed !" << std::endl;
            _converged = false;
            go_on = false;
        }
        else
        {
            const double theta = 0.5;
            Eigen::Vector3d Y = b2->project( X, k );
            Eigen::Vector3d X_post = (1.0-theta)*X + theta*b1->project( Y, k );

            go_on = (X_post - X).norm() > _epsilon;

            X = X_post;
        }

        max_iter--;
    }

    _closest1 = X;
    _closest2 = b2->project(X, k);

    /*
    std::cout << "distance = " << distance() << std::endl;
    std::cout << "pt1 = " << pts[0].transpose() << std::endl;
    std::cout << "pt2 = " << pts[1].transpose() << std::endl;
    std::cout << "FIN CALCUL POINTS PROXIMAUX" << std::endl;
    */
}
