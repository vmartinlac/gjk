
#pragma once

#include <QPicture>
#include <QPainter>
#include <QImage>
#include <iostream>
#include <algorithm>
#include <eigen3/Eigen/Eigen>

namespace gjk
{
    template<int Dim>
    using Vector = Eigen::Matrix<double, Dim, 1>;

    template<int Dim>
    using SimplexPoints = Eigen::Matrix<double, Dim, Dim+1, Eigen::ColMajor>;

    template<int Dim>
    void distanceSubalgorithm(
        const Vector<Dim>& target,
        SimplexPoints<Dim>& points,
        int& num_points,
        Vector<Dim>& proximal );

    template<int Dim>
    Eigen::VectorXd projectInAffineSpan(
        const Vector<Dim>& target,
        SimplexPoints<Dim>& points,
        int& num_points);

    template<int Dim, typename SupportFunction>
    bool findClosestPoint(
        const SupportFunction& support,
        const Vector<Dim>& target,
        bool useguess,
        Vector<Dim>& closest,
        bool& targetisinside );

    template<int Dim, typename SupportFunction1, typename SupportFunction2>
    bool findClosestPoints(
        const SupportFunction1& support1,
        const SupportFunction2& support2,
        bool useguess,
        Vector<Dim>& pt1,
        Vector<Dim>& pt2,
        bool& collide);

    template< int Dim, typename SupportFunction1, typename SupportFunction2 >
    bool testCollision(
        const SupportFunction1& support1,
        const SupportFunction2& support2,
        bool& collide );
}

template<int Dim, typename SupportFunction1, typename SupportFunction2>
bool gjk::findClosestPoints(
    const SupportFunction1& support1,
    const SupportFunction2& support2,
    bool useguess,
    Vector<Dim>& pt1,
    Vector<Dim>& pt2,
    bool& collide)
{
    if(useguess == false)
    {
        Vector<Dim> direction;
        direction.setZero();
        direction(0) = 1.0;
        pt1 = support1(direction);
        pt2 = support2(direction);
    }

    // TODO.
    throw std::runtime_error("not implemented");
}

template< int Dim, typename SupportFunction1, typename SupportFunction2 >
bool gjk::testCollision( const SupportFunction1& support1, const SupportFunction2& support2, bool& collide )
{
    auto support = [&support1, &support2] (const Vector<Dim>& dir) -> gjk::Vector<Dim>
    {
        return support2(dir) - support1(-dir);
    };

    Vector<Dim> closest;
    Vector<Dim> target;
    target.setZero();
    return findClosestPoint(support, target, false, closest, collide);
}

template<int Dim, typename SupportFunction>
bool gjk::findClosestPoint(const SupportFunction& support, const Vector<Dim>& target, bool useguess, Vector<Dim>& closest, bool& targetisinside)
{
    QPicture picture;
    QTransform transform;
    QSize size;

    {

    QVector<QPointF> pt_lst;
    double range[4] = {0.0, 0.0, 0.0, 0.0};
    for(double theta=0.0; theta<2.0*M_PI; theta+=1.0*M_PI/180.0)
    {
        gjk::Vector<2> pt = support( gjk::Vector<2>{ cos(theta), sin(theta) } );
        QPointF pt2(pt(0), pt(1));
        pt_lst.append(pt2);
        range[0] = std::min(range[0], pt(0));
        range[1] = std::max(range[1], pt(0));
        range[2] = std::min(range[2], pt(1));
        range[3] = std::max(range[3], pt(1));
    }

    range[0] = std::min(range[0], target(0));
    range[1] = std::max(range[1], target(0));
    range[2] = std::min(range[2], target(1));
    range[3] = std::max(range[3], target(1));

    {
        const double width = (range[1] - range[0])*1.2;
        const double height = (range[3] - range[2])*1.2;
        range[0] -= 0.2*width;
        range[1] += 0.2*width;
        range[2] -= 0.1*height;
        range[3] += 0.1*height;
    }

    size = QSize(480, 480*(range[3]-range[2])/(range[1]-range[0]));

    transform =
        QTransform()
        .translate( size.width()/2, size.height()/2)
        .scale(size.width()/(range[1]-range[0]), size.height()/(range[3]-range[2]))
        .translate( -0.5*(range[0]+range[1]), -0.5*(range[2]+range[3]));

    std::transform(pt_lst.begin(), pt_lst.end(), pt_lst.begin(), [&transform] (const QPointF& A) -> QPointF { return transform.map(A); } );

    QPainter painter;

    painter.begin(&picture);
    painter.setBrush(Qt::white);
    painter.drawPolygon(&pt_lst.front(), pt_lst.size());
    painter.setBrush(Qt::red);
    painter.drawEllipse( transform.map(QPointF(target(0), target(1))), 5.0, 5.0 );
    painter.end();

    }

    Vector<Dim> p;
    Vector<Dim> q;

    if(useguess)
    {
        p = closest;
    }
    else
    {
        Vector<Dim> direction;
        direction.setZero();
        direction(0) = 1.0;
        p = support( direction );
    }

    int num_points = 0;
    SimplexPoints<Dim> points;

    bool ret = true;
    int max_iter = 50;

    bool go_on = true;

    while( go_on )
    {
        const Vector<Dim> direction = target - p;
        const double distance = direction.norm();

        if( max_iter <= 0 )
        {
            go_on = false;
            ret = false;
            targetisinside = false;

            std::cout << "GJK : max number of iterations reached !" << std::endl;
        }
        else if( distance < 1.0e-5 || num_points == Dim+1 )
        {
            go_on = false;
            ret = true;
            targetisinside = true;
        }
        else
        {
            q = support( direction );

            points.col(num_points) = q;
            num_points++;

            Vector<Dim> p_next;
            distanceSubalgorithm(target, points, num_points, p_next);

            if( (p_next - p).norm() < 1.0e-5 )
            {
                go_on = false;
                ret = true;
                targetisinside = false;
            }

            p = p_next;
        }

        //
        //if(max_iter < 30)
        {
            QImage img(size, QImage::Format_RGB888);
            img.fill(Qt::white);
            QPainter painter;
            painter.begin(&img);
            painter.drawPicture(0, 0, picture);

            painter.setBrush(Qt::NoBrush);
            painter.drawEllipse( transform.map(QPointF(p(0), p(1))), 8.0, 8.0);

            painter.setBrush(Qt::black);
            for(int i=0; i<num_points; i++)
            {
                Vector<2> pt = points.col(i);
                painter.drawEllipse( transform.map(QPointF(pt(0), pt(1))), 5.0, 5.0);
            }
            painter.end();
            img.save("gjk_" + QString::number(max_iter) + ".png");
        }
        //

        max_iter--;
    }

    closest = p;

    return ret;
}

template<int Dim>
Eigen::VectorXd gjk::projectInAffineSpan(
    const Vector<Dim>& target,
    SimplexPoints<Dim>& points,
    int& num_points)
{
    Eigen::MatrixXd A(num_points, num_points);
    Eigen::VectorXd Y(num_points);

    A.row(0).fill(1.0);
    Y(0) = 1.0;

    for(int i=1; i<num_points; i++)
    {
       for(int j=0; j<num_points; j++)
       {
          A(i, j) = ( points.col(i) - points.col(0) ).dot( points.col(j) );
       }
       Y(i) = ( points.col(i) - points.col(0) ).dot( target );
    }

    Eigen::FullPivHouseholderQR< Eigen::MatrixXd > solver;
    solver.compute(A);

    if( solver.isInvertible() == false )
    {
        std::cout << "Non invertible matrix in GJK" << std::endl;
    }

    return solver.solve(Y);
}

template<int Dim>
void gjk::distanceSubalgorithm(
    const Vector<Dim>& target,
    SimplexPoints<Dim>& points,
    int& num_points,
    Vector<Dim>& proximal )
{
    if(num_points == 0)
    {
        throw std::runtime_error("Internal error");
    }
    else if(num_points == 1)
    {
        proximal = points.col(0);
    }
    else
    {
        Eigen::VectorXd X = projectInAffineSpan(
            target,
            points,
            num_points);

        int new_num_points = 0;
        SimplexPoints<Dim> new_points;

        for(int i=0; i<num_points; i++)
        {
            if( X(i) > 0.0 )
            {
                new_points.col(new_num_points) = points.col(i);
                new_num_points++;
            }
        }

        if( new_num_points == 0 )
        {
            throw std::runtime_error("GJK failed.");
        }
        else if(new_num_points == 1)
        {
            proximal = new_points.col(0);
        }
        else
        {
            Eigen::VectorXd Y = projectInAffineSpan(
                target,
                new_points,
                new_num_points);
            proximal = new_points.leftCols(new_num_points) * Y;
        }

        num_points = new_num_points;
        points.swap(new_points);
    }
}

/*
template<>
void gjk::distanceSubalgorithm(
    const Vector<2>& target,
    SimplexPoints<2>& points,
    int& num_points,
    Vector<2>& proximal )
{
    if(num_points == 0)
    {
        throw std::runtime_error("bad arguments to GJK");
    }
    else if(num_points == 1)
    {
        proximal = points.col(0);
    }
    else if(num_points == 2)
    {
        const Vector<2> dir = points.col(1) - points.col(0);
        const double dist = dir.norm();
        if(dist < 1.0e-8)
        {
            proximal = points.col(0);
            num_points = 1;
        }
        else
        {
            const double u = dir.dot(target - points.col(0)) / dist;
            if(u <= 0)
            {
                num_points = 1;
                proximal = points.col(0);
            }
            else if(u >= dist)
            {
                num_points = 1;
                proximal = points.col(1);
            }
            else
            {
                proximal = points.col(0) + u*dir/dist;
            }
        }
    }
    else if(num_points == 3)
    {
        std::cout << "not implemented" << std::endl;
    }
    else
    {
        throw std::runtime_error("bad number of points");
    }
}

*/
