#include <iostream>
#include <map>
#include "Cluster.h"
#include "World.h"
#include "BodyModel.h"
#include "Utils.h"

void Cluster::process()
{
    World* w = World::instance();

    if( collisions.empty() )
    {
        if( bodies.size() != 1) throw std::logic_error("logic error");

        bodies.front()->currentState() = bodies.front()->collisionState();
    }
    else
    {
        if( bodies.size() <= 1) throw std::logic_error("logic error");

        for( Collision& c : collisions)
        {
            if( c.getBody1()->isFixed() && c.getBody2()->isFixed() )
            {
                throw std::runtime_error("Collision between fixed bodies is not supported.");
            }
        }

        std::map<int, int> body_gid_to_lid;

        int num_bodies = 0;
        for( std::shared_ptr<BodyInstance>& b : bodies)
        {
            body_gid_to_lid[b->getId()] = num_bodies;
            num_bodies++;
        }
        if( num_bodies != bodies.size() ) throw std::logic_error("logic error");

        const int dim = 6*bodies.size() + 3*collisions.size();

        //Eigen::SparseMatrix<double> A(dim, dim);
        Eigen::MatrixXd A(dim, dim);
        Eigen::VectorXd B(dim);

        A.setZero();
        B.setZero();

        // A*X = B
        // X = [V1, W1, V2, W2, ..., P1, P2, ...]

        for( std::map<int,int>::iterator it=body_gid_to_lid.begin(); it!=body_gid_to_lid.end(); it++)
        {
            const int gid = it->first;
            const int lid = it->second;

            std::shared_ptr<BodyInstance> body = w->getBodies()[gid];

            Eigen::Matrix3d R = body->collisionState().attitude.toRotationMatrix();

            A.block<3,3>(lid*6+0, lid*6+0) = body->getModel()->getMass() * Eigen::Matrix3d::Identity();
            A.block<3,3>(lid*6+3, lid*6+3) = R * body->getModel()->getInertiaTensor() * R.transpose();

            if(body->isMoving())
            {
                B.segment<3>(lid*6+0) = body->collisionState().linear_momentum;
                B.segment<3>(lid*6+3) = body->collisionState().angular_momentum;
            }
            else
            {
                B.segment<3>(lid*6+0).setZero();
                B.segment<3>(lid*6+3).setZero();
            }
        }

        for( int i=0; i<collisions.size(); i++ )
        {
            Collision& c = collisions[i];

            std::shared_ptr<BodyInstance> body1 = c.getBody1();
            std::shared_ptr<BodyInstance> body2 = c.getBody2();

            const int lid1 = body_gid_to_lid[body1->getId()];
            const int lid2 = body_gid_to_lid[body2->getId()];

            const Eigen::Vector3d collision_point = c.getCollisionPoint();
            const Eigen::Matrix3d collision_frame = c.getCollisionFrame();

            Eigen::Matrix3d R1 = Utils::crossProductMatrix(collision_point - body1->collisionState().position);
            Eigen::Matrix3d R2 = Utils::crossProductMatrix(collision_point - body2->collisionState().position);

            Eigen::Vector3d V1 = body1->getLinearVelocityWF( body1->collisionState() );
            Eigen::Vector3d V2 = body2->getLinearVelocityWF( body2->collisionState() );
            Eigen::Vector3d omega1 = body1->getAngularVelocityWF( body1->collisionState() );
            Eigen::Vector3d omega2 = body2->getAngularVelocityWF( body2->collisionState() );

            const int base = 6*num_bodies + 3*i;

            A.block<3,3>(base, 6*lid1+0) = - collision_frame.transpose();
            A.block<3,3>(base, 6*lid1+3) = collision_frame.transpose()*R1;

            A.block<3,3>(base, 6*lid2+0) = collision_frame.transpose();
            A.block<3,3>(base, 6*lid2+3) = - collision_frame.transpose()*R2;

            B.segment<2>(base).setZero();
            const double restitution = 0.3;
            B(base+2) = - restitution * collision_frame.col(2).dot( V2 - R2*omega2 - V1 + R1*omega1 );

            if(body1->isMoving())
            {
                A.block<3,3>(6*lid1+0, base) = Eigen::Matrix3d::Identity();
                A.block<3,3>(6*lid1+3, base) = R1;
            }
            if(body2->isMoving())
            {
                A.block<3,3>(6*lid2+0, base) = -Eigen::Matrix3d::Identity();
                A.block<3,3>(6*lid2+3, base) = -R2;
            }
        }

        Eigen::FullPivLU< Eigen::MatrixXd > solver;

        solver.compute(A);

        if(solver.isInvertible())
        {
            const Eigen::VectorXd X = solver.solve(B);

            for( std::map<int,int>::iterator it=body_gid_to_lid.begin(); it!=body_gid_to_lid.end(); it++)
            {
                const int gid = it->first;
                const int lid = it->second;

                std::shared_ptr<BodyInstance> body = w->getBodies()[gid];

                if( body->isMoving() )
                {
                    body->currentState().linear_momentum = body->getModel()->getMass() * X.segment<3>(6*lid);
                    body->currentState().angular_momentum = body->getModel()->getInertiaTensor() * X.segment<3>(6*lid+3);
                }
                else
                {
                    body->currentState().linear_momentum.setZero();
                    body->currentState().angular_momentum.setZero();
                }
            }
        }
        else
        {
            std::cout << "Warning : non invertible matrix encountered in collision response module." << std::endl;
        }
    }
}

