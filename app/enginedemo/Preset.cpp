#include <QDialog>
#include <QListWidget>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <vector>
#include <memory>
#include "PhysicalConstants.h"
#include "BodyInstance.h"
#include "BodyModel.h"
#include "Preset.h"
#include "Spring.h"
#include "World.h"

class Preset
{
public:
    virtual const char* name() = 0;
    virtual void apply() = 0;
};

class Preset1 : public Preset
{
public:
    const char* name() override
    {
        return "Three balls colliding";
    }

    void apply() override
    {
        std::shared_ptr<BodyModel> m(new SphereBody(1.0, CTE_WOOD_DENSITY));
        m->asSphere()->setColor(0.5, 0.8, 0.5);

        std::shared_ptr<BodyInstance> b1(new BodyInstance(m));
        b1->initialState().position << -10.0, 0.0, 1.6;
        b1->initialState().linear_momentum = Eigen::Vector3d{5.0, 0.0, 0.0} * m->getMass();
        b1->setMoving();

        std::shared_ptr<BodyInstance> b2(new BodyInstance(m));
        b2->initialState().position << -10.0, 0.0, -1.6;
        b2->initialState().linear_momentum = Eigen::Vector3d{5.0, 0.0, 0.0} * m->getMass();
        b2->setMoving();

        std::shared_ptr<BodyInstance> b3(new BodyInstance(m));
        b3->initialState().position << 10.0, 0.0, 0.0;
        b3->initialState().linear_momentum = Eigen::Vector3d{-5.0, 0.0, 0.0} * m->getMass();
        b3->setMoving();

        World* w = World::instance();
        w->setRestitution(1.0);
        w->setMargin(0.05);
        w->setGravity(Eigen::Vector3d::Zero());
        w->addBody(b1);
        w->addBody(b2);
        w->addBody(b3);
        w->build();
    }
};

class Preset2 : public Preset
{
public:
    const char* name() override
    {
        return "Collisions + pendulum";
    }

    void apply() override
    {
        std::shared_ptr<BodyModel> m1(new BoxBody(Eigen::Vector3d{60.0, 40.0, 0.5}, CTE_WOOD_DENSITY)); 

        std::shared_ptr<BodyModel> m2(new SphereBody(1.0, CTE_WOOD_DENSITY));
        m2->asSphere()->setColor(0.7, 0.1, 0.1);

        std::shared_ptr<BodyModel> m3(new BoxBody(Eigen::Vector3d{10.0, 2.0, 4.0}, CTE_WOOD_DENSITY)); 

        std::shared_ptr<BodyInstance> b1(new BodyInstance(m1));

        std::shared_ptr<BodyInstance> b2(new BodyInstance(m2));
        b2->initialState().position << -2.0, 0.0, 10.0;
        b2->setMoving();

        std::shared_ptr<BodyInstance> b3(new BodyInstance(m2));
        b3->initialState().position << 2.0, 0.0, 10.0;

        std::shared_ptr<BodyInstance> b4(new BodyInstance(m2));
        b4->initialState().position << -10.0, 0.0, 10.0;
        //b4->setMoving();

        std::shared_ptr<BodyInstance> b5(new BodyInstance(m3));
        b5->initialState().position << -12.0, 0.0, 20.0;
        b5->setMoving();

        std::shared_ptr<Spring> spring(new Spring);
        spring->setBody1(b2);
        spring->setBody2(b3);
        spring->setAnchor1(Eigen::Vector3d{1.0, 0.0, 0.0});
        spring->setAnchor2(Eigen::Vector3d{-1.0, 0.0, 0.0});
        spring->setFreeLength(2.0);
        spring->setElasticityCoefficient(9.81*m2->getMass()/0.3);
        spring->setDampingCoefficient(9.81*m2->getMass()/0.3);

        World* w = World::instance();
        w->addBody(b1);
        w->addBody(b2);
        w->addBody(b3);
        w->addBody(b4);
        w->addBody(b5);
        w->addSpring(spring);
        w->build();
    }
};

class Preset3 : public Preset
{
public:
    const char* name() override
    {
        return "Collar";
    }

    void apply() override
    {
        World* world = World::instance();

        std::shared_ptr<BodyModel> model = std::shared_ptr<BodyModel>(new SphereBody(1.0, CTE_WOOD_DENSITY));

        const double L = 4.0; // distance between consecutive balls / free length of springs.
        Eigen::Vector3d dir{1.0, 0.0, 0.0}; // direction along which balls are aligned.
        const int N = 4; // number of balls.

        std::shared_ptr<BodyInstance> sphere = std::make_shared<BodyInstance>(model);
        sphere->initialState().position << 2.0, 0.0, 2.0;
        sphere->setFixed();
        world->addBody(sphere);

        const double R = model->asSphere()->getRadius();

        for(int i=1; i<N; i++)
        {
            std::shared_ptr<BodyInstance> new_sphere = std::make_shared<BodyInstance>(model);
            new_sphere->setMoving();
            new_sphere->initialState().position = sphere->initialState().position + (L + 2*R) * dir;
            world->addBody( new_sphere );

            std::shared_ptr<Spring> spring = std::make_shared<Spring>();
            spring->setFreeLength(L);
            spring->setDampingCoefficient(9.81*model->getMass()*1.0);
            spring->setElasticityCoefficient( model->getMass()*9.81/0.3 );
            spring->setBody1(sphere);
            spring->setBody2(new_sphere);
            spring->setAnchor1(R*dir);
            spring->setAnchor2(-R*dir);
            world->addSpring( spring );

            sphere = new_sphere;
        }

        //sphere->setFixed();

        std::shared_ptr<BodyModel> model_ground(new BoxBody(Eigen::Vector3d{50.0, 50.0, 0.5}, CTE_WOOD_DENSITY)); 
        std::shared_ptr<BodyInstance> body_ground = std::make_shared<BodyInstance>(model_ground);
        body_ground->initialState().position << 0.0, 0.0, -40.0;

        world->addBody(body_ground);

        world->setAngularViscosity(1000.0);

        world->build();
    }
};

/*
class Preset4 : public Preset
{
public:
    const char* name() override
    {
        return "Perpetual movement";
    }

    void apply() override
    {
        World* world = World::instance();

        std::shared_ptr<BodyModel> model_sphere = std::shared_ptr<BodyModel>(new SphereBody(0.5, CTE_WOOD_DENSITY));

        std::shared_ptr<BodyInstance> P1 = std::make_shared<BodyInstance>(model_sphere);
        std::shared_ptr<BodyInstance> P2 = std::make_shared<BodyInstance>(model_sphere);
        std::shared_ptr<BodyInstance> P3 = std::make_shared<BodyInstance>(model_sphere);
        P1->initialState().position << -3.0, -5.0, 0.0;
        P2->initialState().position << 3.0, -5.0, 0.0;
        P3->initialState().position << 0.0, 5.0, 0.0;

        world->addBody(P1);
        world->addBody(P2);
        world->addBody(P3);

        std::shared_ptr<BodyModel> model_box = std::shared_ptr<BoxBody>(new BoxBody(Eigen::Vector3d{3.0, 0.5, 9.0}, CTE_WOOD_DENSITY));

        std::shared_ptr<BodyInstance> D1 = std::make_shared<BodyInstance>(model_box);
        D1->initialState().position << 0.0, -5.0, 6.5;
        D1->setMoving();

        world->addBody(D1);

        std::shared_ptr<Spring> S1 = std::make_shared<Spring>();
        S1->setFreeLength(0);
        //S1->setDampingCoefficient(9.81*model_box->getMass()*1.0);
        S1->setElasticityCoefficient( 0.0*model_box->getMass()*9.81/0.3 );
        S1->setBody1(P1);
        S1->setBody2(D1);
        S1->setAnchor1(Eigen::Vector3d::Zero());
        S1->setAnchor2(Eigen::Vector3d{-1.5, 0.0, -4.5});
        world->addSpring( S1 );

        std::shared_ptr<Spring> S2 = std::make_shared<Spring>();
        S2->setFreeLength(0);
        //S2->setDampingCoefficient(9.81*model_box->getMass()*1.0);
        S2->setElasticityCoefficient( 0.0*model_box->getMass()*9.81/0.3 );
        S2->setBody1(P2);
        S2->setBody2(D1);
        S2->setAnchor1(Eigen::Vector3d::Zero());
        S2->setAnchor2(Eigen::Vector3d{1.5, 0.0, -4.5});
        world->addSpring( S2 );

        world->build();
    }
};
*/

class Preset5 : public Preset
{
public:
    const char* name() override
    {
        return "Cubes falling and bouncing.";
    }

    void apply() override
    {
        World* world = World::instance();

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

        world->addBody(cube1);
        world->addBody(cube2);
        world->addBody(plate1);
        world->addBody(plate2);
        world->addBody(plate3);

        world->build();
    }
};

class Preset6 : public Preset
{
public:
    const char* name() override
    {
        return "Two spheres fall, bounce and collide.";
    }

    void apply() override
    {
        World* world = World::instance();

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

        world->addBody(ball1);
        world->addBody(ball2);
        world->addBody(plate1);
        world->addBody(plate2);
        world->setRestitution(0.55);

        world->build();
    }
};

class Preset7 : public Preset
{
public:
    const char* name() override
    {
        return "Some balls";
    }

    void apply() override
    {
        World* world = World::instance();

        std::shared_ptr<BodyModel> model_ball = std::shared_ptr<BodyModel>(new SphereBody(
            1.0, CTE_WOOD_DENSITY));

        std::shared_ptr<BodyInstance> ball1 = std::make_shared<BodyInstance>(model_ball);
        ball1->setMoving();
        ball1->initialState().position << 0.0, 0.0, 0.0;
        ball1->initialState().linear_momentum = Eigen::Vector3d{9.0, 0.0, 0.0} * model_ball->getMass();

        std::shared_ptr<BodyInstance> ball2 = std::make_shared<BodyInstance>(model_ball);
        ball2->setMoving();
        ball2->initialState().position << 20.0, 0.0, 0.0;

        std::shared_ptr<BodyInstance> ball3 = std::make_shared<BodyInstance>(model_ball);
        ball3->setMoving();
        ball3->initialState().position << 0.0, 10.0, 0.0;
        ball3->initialState().linear_momentum = Eigen::Vector3d{9.0, 0.0, 0.0} * model_ball->getMass();

        std::shared_ptr<BodyInstance> ball4 = std::make_shared<BodyInstance>(model_ball);
        ball4->initialState().position << 20.0, 10.0, 0.0;

        world->addBody(ball1);
        world->addBody(ball2);
        world->addBody(ball3);
        world->addBody(ball4);
        world->setRestitution(1.0);
        world->setGravity(Eigen::Vector3d::Zero());

        world->build();
    }
};

bool choose_and_build_world()
{
    std::vector< std::shared_ptr<Preset> > presets;
    presets.emplace_back(new Preset1());
    presets.emplace_back(new Preset2());
    presets.emplace_back(new Preset3());
    //presets.emplace_back(new Preset4());
    presets.emplace_back(new Preset5());
    presets.emplace_back(new Preset6());
    presets.emplace_back(new Preset7());

    QLabel* lbl = new QLabel("Which world do you want to build ?");

    QListWidget* list = new QListWidget;
    for(int i=0; i<presets.size(); i++)
    {
        std::shared_ptr<Preset>& preset = presets[i];

        QListWidgetItem* item = new QListWidgetItem( preset->name() );
        item->setData(Qt::UserRole, i);
        list->addItem(item);
    }

    QPushButton* btn = new QPushButton("OK");

    QVBoxLayout* lay = new QVBoxLayout;
    lay->addWidget(lbl);
    lay->addWidget(list);
    lay->addWidget(btn);

    QDialog* dlg = new QDialog;
    dlg->setLayout(lay);
    dlg->setWindowTitle("Physics Engine Demo");

    QObject::connect( btn, &QPushButton::clicked, dlg, &QDialog::accept);
    QObject::connect( list, &QListWidget::itemDoubleClicked, dlg, &QDialog::accept);

    bool ret = false;

    if( dlg->exec() == QDialog::Accepted )
    {
        const int i = list->currentItem()->data(Qt::UserRole).toInt();
        if( 0 <= i && i < presets.size() )
        {
            ret = true;
            presets[i]->apply();
        }
    }
    
    delete dlg;

    return ret;
}

