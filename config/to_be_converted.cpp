class Preset3 : public Preset
{
public:
    const char* name() override
    {
        return "Collar";
    }

    std::shared_ptr<World> apply() override
    {
        World::Builder b;

        std::shared_ptr<BodyModel> model = std::shared_ptr<BodyModel>(new SphereBody(1.0, CTE_WOOD_DENSITY));

        const double L = 4.0; // distance between consecutive balls / free length of springs.
        Eigen::Vector3d dir{1.0, 0.0, 0.0}; // direction along which balls are aligned.
        const int N = 4; // number of balls.

        std::shared_ptr<BodyInstance> sphere = std::make_shared<BodyInstance>(model);
        sphere->initialState().position << 2.0, 0.0, 2.0;
        sphere->setFixed();
        b.addBody(sphere);

        const double R = model->asSphere()->getRadius();

        for(int i=1; i<N; i++)
        {
            std::shared_ptr<BodyInstance> new_sphere = std::make_shared<BodyInstance>(model);
            new_sphere->setMoving();
            new_sphere->initialState().position = sphere->initialState().position + (L + 2*R) * dir;
            b.addBody( new_sphere );

            Spring* spring = new Spring();
            spring->setFreeLength(L);
            spring->setDampingCoefficient(9.81*model->getMass()*1.0);
            spring->setElasticityCoefficient( model->getMass()*9.81/0.3 );
            spring->setBody1(sphere);
            spring->setBody2(new_sphere);
            spring->setAnchor1(R*dir);
            spring->setAnchor2(-R*dir);
            b.addLink( std::shared_ptr<Link>(spring) );

            sphere = new_sphere;
        }

        //sphere->setFixed();

        std::shared_ptr<BodyModel> model_ground(new BoxBody(Eigen::Vector3d{50.0, 50.0, 0.5}, CTE_WOOD_DENSITY)); 
        std::shared_ptr<BodyInstance> body_ground = std::make_shared<BodyInstance>(model_ground);
        body_ground->initialState().position << 0.0, 0.0, -40.0;

        b.addBody(body_ground);

        b.setAngularViscosity(1000.0);

        return b.build();
    }
};

class Preset5 : public Preset
{
public:
    const char* name() override
    {
        return "Cubes falling and bouncing.";
    }

    std::shared_ptr<World> apply() override
    {
        std::shared_ptr<BodyModel> model_cube = std::shared_ptr<BodyModel>(new BoxBody(
            Eigen::Vector3d{2.0, 2.0, 2.0}, CTE_WOOD_DENSITY));

        std::shared_ptr<BodyModel> model_plate = std::shared_ptr<BodyModel>(new BoxBody(
            Eigen::Vector3d{25.0, 15.0, 1.0}, CTE_WOOD_DENSITY));

        std::shared_ptr<BodyInstance> cube1 = std::make_shared<BodyInstance>(model_cube);
        cube1->setMoving();
        cube1->initialState().position << 0.0, 0.0, 0.0;

        std::shared_ptr<BodyInstance> cube2 = std::make_shared<BodyInstance>(model_cube);
        cube2->setMoving();
        cube2->initialState().position << 0.0, 0.0, 15.0;

        std::shared_ptr<BodyInstance> plate1 = std::make_shared<BodyInstance>(model_plate);
        plate1->initialState().position << 0.0, 0.0, -20.0;
        plate1->initialState().attitude = Eigen::AngleAxisd( 0.2*M_PI, Eigen::Vector3d::UnitX());

        std::shared_ptr<BodyInstance> plate2 = std::make_shared<BodyInstance>(model_plate);
        plate2->initialState().position << 0.0, -15.0, -30.0;
        plate2->initialState().attitude = Eigen::AngleAxisd( -0.2*M_PI, Eigen::Vector3d::UnitX());

        std::shared_ptr<BodyInstance> plate3 = std::make_shared<BodyInstance>(model_plate);
        plate3->initialState().position << 0.0, 0.0, -40.0;
        plate3->initialState().attitude = Eigen::AngleAxisd( 0.3*M_PI, Eigen::Vector3d::UnitX());

        World::Builder b;

        b.addBody(cube1);
        b.addBody(cube2);
        b.addBody(plate1);
        b.addBody(plate2);
        b.addBody(plate3);

        return b.build();
    }
};

class Preset6 : public Preset
{
public:
    const char* name() override
    {
        return "Two spheres fall, bounce and collide.";
    }

    std::shared_ptr<World> apply() override
    {
        std::shared_ptr<BodyModel> model_ball = std::shared_ptr<BodyModel>(new SphereBody(
            1.0, CTE_WOOD_DENSITY));

        std::shared_ptr<BodyModel> model_plate = std::shared_ptr<BodyModel>(new BoxBody(
            Eigen::Vector3d{25.0, 15.0, 1.0}, CTE_WOOD_DENSITY));

        std::shared_ptr<BodyInstance> ball1 = std::make_shared<BodyInstance>(model_ball);
        ball1->setMoving();
        ball1->initialState().position << 0.0, 0.0, 20.0;

        std::shared_ptr<BodyInstance> ball2 = std::make_shared<BodyInstance>(model_ball);
        ball2->setMoving();
        ball2->initialState().position << 0.0, -20.0, 20.0;

        std::shared_ptr<BodyInstance> plate1 = std::make_shared<BodyInstance>(model_plate);
        plate1->initialState().position << 0.0, 0.0, -20.0;
        plate1->initialState().attitude = Eigen::AngleAxisd( 0.1*M_PI, Eigen::Vector3d::UnitX());

        std::shared_ptr<BodyInstance> plate2 = std::make_shared<BodyInstance>(model_plate);
        plate2->initialState().position << 0.0, -20.0, -20.0;
        plate2->initialState().attitude = Eigen::AngleAxisd( -0.1*M_PI, Eigen::Vector3d::UnitX());

        World::Builder b;

        b.addBody(ball1);
        b.addBody(ball2);
        b.addBody(plate1);
        b.addBody(plate2);
        b.setRestitution(0.55);

        return b.build();
    }
};

class Preset7 : public Preset
{
public:
    const char* name() override
    {
        return "Joints";
    }

    std::shared_ptr<World> apply() override
    {
        std::shared_ptr<BodyModel> model_ball = std::shared_ptr<BodyModel>(new SphereBody(1.0, CTE_WOOD_DENSITY));

        std::shared_ptr<BodyInstance> ball0 = std::make_shared<BodyInstance>(model_ball);
        ball0->initialState().position << -4.0, 0.0, 0.0;

        std::shared_ptr<BodyInstance> ball1 = std::make_shared<BodyInstance>(model_ball);
        ball1->setMoving();
        ball1->initialState().position << 0.0, 0.0, 0.0;

        std::shared_ptr<BodyInstance> ball2 = std::make_shared<BodyInstance>(model_ball);
        ball2->setMoving();
        ball2->initialState().position << 2.0, 0.0, 0.0;

        std::shared_ptr<Link> joint = std::shared_ptr<Link>(new Joint);
        joint->setBody1(ball1);
        joint->setBody2(ball2);
        joint->setAnchor1(Eigen::Vector3d{1.0, 0.0, 0.0});
        joint->setAnchor2(Eigen::Vector3d{-1.0, 0.0, 0.0});

        std::shared_ptr<Link> spring = std::shared_ptr<Link>(new Spring);
        spring->setBody1(ball0);
        spring->setBody2(ball1);
        spring->setAnchor1(Eigen::Vector3d{1.0, 0.0, 0.0});
        spring->setAnchor2(Eigen::Vector3d{-1.0, 0.0, 0.0});
        spring->asSpring()->setFreeLength(2.0);
        spring->asSpring()->setDampingCoefficient(9.81*model_ball->getMass()*1.0);
        spring->asSpring()->setElasticityCoefficient( model_ball->getMass()*9.81/1.0 );

        World::Builder b;

        b.addBody(ball0);
        b.addBody(ball1);
        b.addBody(ball2);
        b.addLink(joint);
        b.addLink(spring);

        return b.build();
    }
};
