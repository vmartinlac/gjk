#pragma once

#include <QObject>
#include <memory>
#include <Eigen/Eigen>

class BodyModel;
class BodyInstance;

class testEngine : public QObject
{
    Q_OBJECT
private slots:
    void initTestCase();
    void cleanupTestCase();
    void testBodyComputations();
protected:
    std::shared_ptr<BodyModel> _model_ball;
    std::shared_ptr<BodyInstance> _ball;
    std::shared_ptr<BodyModel> _model_box;
    std::shared_ptr<BodyInstance> _box;
    Eigen::Vector3d _angular_velocity;
    Eigen::Vector3d _linear_velocity;
};
