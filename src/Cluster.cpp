#include <iostream>
#include <map>
#include "Cluster.h"

void Cluster::process()
{
    if( collisions.empty() )
    {
        if( bodies.size() != 1) throw std::logic_error("logic error");

        bodies.front()->currentState() = bodies.front()->collisionState();
    }
    else
    {
        if( bodies.size() <= 1) throw std::logic_error("logic error");

        /*
        for( std::shared_ptr<BodyInstance>& b : bodies)
        {
            b->setFixed();
        }
        */

        std::map<int, int> gid_to_lid;

        {
            int counter = 0;
            for( std::shared_ptr<BodyInstance>& b : bodies)
            {
                gid_to_lid[b->getId()] = counter;
                counter++;
            }
            if( counter != bodies.size() ) throw std::logic_error("logic error");
        }

        const int dim = 6*bodies.size() + 3*collisions.size();

        Eigen::SparseMatrix<double> A(dim, dim);
        Eigen::VectorXd B(dim);

        for( std::map<int,int>::iterator it=gid_to_lid.begin(); it!=gid_to_lid.end(); it++)
        {
            const int gid = it->first;
            const int lid = it->second;

            ;
        }
    }
}

