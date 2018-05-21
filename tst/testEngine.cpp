#include <QtTest>
#include <iostream>
#include "PhysicalConstants.h"
#include "testEngine.h"
#include "BodyModel.h"
#include "BodyInstance.h"

void testEngine::initTestCase()
{
    _model_ball = std::shared_ptr<BodyModel>(new SphereBody(1.0, CTE_WOOD_DENSITY));
    _model_box = std::shared_ptr<BodyModel>(new BoxBody(Eigen::Vector3d{2.0, 4.0, 6.0}, CTE_WOOD_DENSITY));
    _linear_velocity << 1.0, 0.1, 2.3;
    _angular_velocity << 3.0, 4.0, 1.0;

    _ball = std::make_shared<BodyInstance>(_model_ball);
    _ball->initialState().position << 10.0, 0.0, 0.0;
    _ball->initialState().attitude.setFromTwoVectors(
        Eigen::Vector3d{1.0, 0.0, 0.0},
        Eigen::Vector3d{1.0, 1.0, 1.0});
    _ball->initialState().linear_momentum = _model_ball->getMass() * _linear_velocity;
    _ball->initialState().angular_momentum =
        _ball->initialState().attitude *
        _model_ball->getInertiaTensor() *
        _ball->initialState().attitude.inverse()  *
        _angular_velocity;
    _ball->currentState() = _ball->initialState();

    _box = std::make_shared<BodyInstance>(_model_box);
    _box->initialState().position << 10.0, 0.0, 0.0;
    _box->initialState().attitude.setFromTwoVectors(
        Eigen::Vector3d{1.0, 0.0, 0.0},
        Eigen::Vector3d{1.0, 2.0, 3.0});
    _box->initialState().linear_momentum = _model_box->getMass() * _linear_velocity;
    _box->initialState().angular_momentum =
        _box->initialState().attitude *
        _model_box->getInertiaTensor() *
        _box->initialState().attitude.inverse()  *
        _angular_velocity;
    _box->currentState() = _box->initialState();
}

void testEngine::cleanupTestCase()
{
    _model_box.reset();
    _model_ball.reset();
    _box.reset();
    _ball.reset();
}

void testEngine::testBodyComputations()
{
    // inertia tensor of the ball is the same in all frames.
    QVERIFY( ( _ball->getModel()->getInertiaTensor() - _ball->getInertiaTensorWF(_ball->currentState()) ).array().cwiseAbs().maxCoeff() < 1.0e-9 );

    // check that the inertia tensor in world frame is computed correctly.
    {
        const Eigen::Matrix3d R = _box->currentState().attitude.toRotationMatrix();
        QVERIFY( ( _box->getInertiaTensorWF(_box->currentState()) - R*_box->getModel()->getInertiaTensor()*R.transpose() ).array().cwiseAbs().maxCoeff() < 1.0e-9 );
    }

    QVERIFY( _angular_velocity.isApprox(_box->getAngularVelocityWF(_box->currentState()) ) );
    QVERIFY( _angular_velocity.isApprox(_ball->getAngularVelocityWF(_ball->currentState()) ) );
    QVERIFY( _linear_velocity.isApprox(_box->getLinearVelocityWF(_box->currentState()) ) );
    QVERIFY( _linear_velocity.isApprox(_ball->getLinearVelocityWF(_ball->currentState()) ) );

    QVERIFY( Eigen::Matrix3d::Identity().isApprox( _model_ball->getInverseOfInertiaTensor() * _model_ball->getInertiaTensor() ) );
    QVERIFY( Eigen::Matrix3d::Identity().isApprox( _model_box->getInverseOfInertiaTensor() * _model_box->getInertiaTensor() ) );

    QVERIFY( Eigen::Matrix3d::Identity().isApprox( _ball->getInertiaTensorWF(_ball->currentState()) * _ball->getInverseOfInertiaTensorWF(_ball->currentState()) ) );
    QVERIFY( Eigen::Matrix3d::Identity().isApprox( _box->getInertiaTensorWF(_box->currentState()) * _box->getInverseOfInertiaTensorWF(_box->currentState()) ) );
}

QTEST_MAIN(testEngine)

