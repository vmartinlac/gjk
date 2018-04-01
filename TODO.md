* calculer point de contact lors de collision.
* améliorer implantation GJK.
* comment recenser les forces qui s'exercent sur un objet ?
* comment traiter les collisions.
* tester la résolution des équations d'Euler.


struct BodyState
{
    Eigen::Vector3d position;
    Eigen::Quaterniond attitude;
    Eigen::Vector3d linear_momentum;
    Eigen::Vector3d angular_momentum;
};

class BodyModel
{
public:

    virtual BodyInstance* createInstance() = 0;

    virtual isBox();
    virtual isSphere();

    virtual getBoundingSphere() = 0;

private:

    double _mass;
};

class BodyInstance
{
public:

    BodyInstance(
        osg::PositionAttitudeTransform* representation,
        BodyModel* model);

    osg::PositionAttitudeTransform* getRepresentation();

    void setState(const BodyState& new_state);
    BodyState getState();

    bool isMoving() { return _moving; }
    void setMoving(bool val) { _moving = val; }

    int getId();
    void setId(int id);

    BodyModel* getModel();

protected:

    int _id;
    osg::ref_ptr<osg::PositionAttitudeTransform> _node;
    BodyModel* _model;
    BodyState _state;
    bool _moving;
}

class Solver
{
}
