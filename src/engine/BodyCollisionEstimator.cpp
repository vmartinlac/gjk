#include <iostream>
#include "BodyCollisionEstimator.h"

BodyCollisionEstimator::BodyCollisionEstimator()
{
    _epsilon = 1.0e-3;
}

void BodyCollisionEstimator::run(
    std::shared_ptr<BodyInstance> b1,
    std::shared_ptr<BodyInstance> b2,
    BodyInstance::KindOfState k )
{
    std::shared_ptr<BodyInstance> bodies[2];
    bodies[0] = b1;
    bodies[1] = b2;

    const Eigen::Vector3d direction{1.0, 0.0, 0.0};

    Eigen::Vector3d pts[2];
    pts[0] = bodies[0]->support(direction, k);
    pts[1] = bodies[1]->support(direction, k);

    int max_iter = 100;
    bool go_on = true;

    _converged = true;

    std::cout << "DÉBUT CALCUL POINTS PROXIMAUX" << std::endl;
    while(go_on)
    {
        if(max_iter <= 0)
        {
            std::cout << "Number of iterations exceeded !" << std::endl;
            _converged = false;
            go_on = false;
        }
        else
        {
            go_on = false;

            for(int i=0; i<2; i++)
            {
                const int j = (i+1)%2;
                Eigen::Vector3d pt_post = bodies[j]->project( pts[i], k );
                if( (pts[j] - pt_post).norm() > _epsilon )
                {
                    go_on = true;
                }
                pts[j] = pt_post;
            }
        }

        max_iter--;
    }

    _overlap = bodies[0]->indicator(pts[1], k) && bodies[1]->indicator(pts[0], k);
    _closest1 = pts[0];
    _closest2 = pts[1];

    std::cout << "overlap = " << overlap() << std::endl;
    std::cout << "distance = " << distance() << std::endl;
    std::cout << "pt1 = " << pts[0].transpose() << std::endl;
    std::cout << "pt2 = " << pts[1].transpose() << std::endl;
    std::cout << "FIN CALCUL POINTS PROXIMAUX" << std::endl;
}
