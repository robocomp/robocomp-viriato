//
// Created by robolab on 17/01/20.
//

#ifndef PROJECT_NAVIGATION_H
#define PROJECT_NAVIGATION_H



#include <innermodel/innermodel.h>
#include <math.h>

#include <genericworker.h>
#include <Laser.h>
#include "collisions.h"
#include <QPolygonF>
#include <QPointF>
#include "innerviewer.h"
#include <cppitertools/chain.hpp>
#include <cppitertools/range.hpp>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/zip.hpp>
#include <cppitertools/filter.hpp>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/slice.hpp>
#include <algorithm>
#include <localPerson.h>
#include <typeinfo>
#include <Eigen/Dense>

// Map
struct TMapDefault
{
};

struct TContDefault
{
    TContDefault(){};
};

template<typename TMap = TMapDefault, typename TController = TContDefault>
class Navigation
{
    public:

        // Target
        struct Target
        {
            QPointF pos;
            float ang;
        };
        Target current_target;

        float KE;
        float KI;

        TMap grid;
        TController controller;

        void initialize(const std::shared_ptr<InnerModel> &innerModel_,
                        std::shared_ptr< RoboCompCommonBehavior::ParameterList > configparams_,
                        RoboCompOmniRobot::OmniRobotPrxPtr omnirobot_proxy_,
                        QGraphicsScene *scene_)
        {
            qDebug()<<"Navigation - "<< __FUNCTION__;

            innerModel = innerModel_;
            configparams = configparams_;
            scene = scene_;
            //viewer = viewer_;

            omnirobot_proxy = omnirobot_proxy_;
            stopRobot();
             //grid can't be initialized if the robot is moving

            collisions =  std::make_shared<Collisions>();

            collisions->initialize(innerModel, configparams);
            grid.initialize(collisions, scene);

            controller.initialize(innerModel,configparams);

            robotXWidth = std::stof(configparams->at("RobotXWidth").value);
            robotZLong = std::stof(configparams->at("RobotZLong").value);
            robotBottomLeft     = QVec::vec3( - robotXWidth / 2 - 100, 0, - robotZLong / 2 - 100);
            robotBottomRight    = QVec::vec3( + robotXWidth / 2 + 100, 0, - robotZLong / 2 - 100);
            robotTopRight       = QVec::vec3( + robotXWidth / 2 + 100, 0, + robotZLong / 2 + 100);
            robotTopLeft        = QVec::vec3( - robotXWidth / 2 - 100, 0, + robotZLong / 2 + 100);

        //    reloj.restart();
        };

        void updateInnerModel(const std::shared_ptr<InnerModel> &innerModel_)
        {
            qDebug()<<"Navigation - "<< __FUNCTION__;
            innerModel = innerModel_;
            controller.updateInnerModel(innerModel);
        };

        void update(const RoboCompLaser::TLaserData &laserData_, Target &target, bool needsReplaning)
        {
            currentRobotPose = innerModel->transformS6D("world","robot");
            updateLaserPolygon(laserData_);
            currentRobotPolygon = getRobotPolygon();
            currentRobotNose = getRobotNose();

            if ( needsReplaning )
            {
                std::list<QPointF> path = grid.computePath(currentRobotNose, target.pos);
                pathPoints.clear(); pathPoints.reserve(path.size());
                std::copy(std::begin(path), std::end(path), std::back_inserter(pathPoints));

                if (path.empty())
                {
                    qDebug() << __FUNCTION__ << "Path not found. Returning";
                    stopRobot();
                    return;
                }
            }
            computeForces(pathPoints, laserData_);
            cleanPoints();
            addPoints();

            auto [blocked, active, turning, xVel,zVel,rotVel] = controller.update(pathPoints, laserData_, current_target.pos, currentRobotPose);
            omnirobot_proxy->setSpeedBase(xVel,zVel,rotVel);
            draw_path(pathPoints);
        };

        void stopRobot()
        {
            qDebug()<<"Navigation - "<< __FUNCTION__;
            omnirobot_proxy->setSpeedBase(0,0,0);
        }

    private:
        std::shared_ptr<Collisions> collisions;
        std::shared_ptr<InnerModel> innerModel;
        std::shared_ptr<InnerViewer> viewer;
        std::shared_ptr<RoboCompCommonBehavior::ParameterList> configparams;

        RoboCompOmniRobot::OmniRobotPrxPtr omnirobot_proxy;
        typedef struct { float dist; float angle;} LocalPointPol;

        // ElasticBand
        std::vector<QPointF> pathPoints;

        //Draw
        QGraphicsScene *scene;

        const float ROBOT_LENGTH = 500;
        const float ROAD_STEP_SEPARATION = ROBOT_LENGTH * 0.9;

        bool targetBehindRobot = false;

         //Integrating time
        QTime reloj = QTime::currentTime();

        QPointF lastPointInPath, currentRobotNose;

        QPolygonF currentRobotPolygon, laser_poly;
        std::vector<QPointF> laser_cart;
        QVec currentRobotPose;

        float robotXWidth, robotZLong; //robot dimensions read from config
        QVec robotBottomLeft, robotBottomRight, robotTopRight, robotTopLeft;

        bool gridChanged = false;

        vector<QPolygonF> intimateSpaces,personalSpaces,socialSpaces, totalAffordances;
        vector<QPolygonF> affordancesBlocked;

        std::map<float, vector<QPolygonF>> mapCostObjects;

    ////////// GRID RELATED METHODS //////////
    void updateFreeSpaceMap(bool drawGrid = true)
    {
        qDebug()<<"Navigation - "<< __FUNCTION__;

        grid.resetGrid();

        //To set occupied
        for (auto &&poly_intimate : iter::chain(intimateSpaces, affordancesBlocked))
            grid.markAreaInGridAs(poly_intimate, false);

        for(auto [cost,polygonVec] : mapCostObjects)
        {
            for (auto polygon : polygonVec)
                grid.modifyCostInGrid(polygon, cost);
        }

        for (auto &&poly_soc : socialSpaces)
            grid.modifyCostInGrid(poly_soc, 8.0);

        for (auto &&poly_per : personalSpaces)
            grid.modifyCostInGrid(poly_per, 10.0);


        //if(drawGrid) grid.draw(viewer);

    }
    void draw_path(std::vector<QPointF> &path)
    {
        static std::vector<QGraphicsLineItem *> scene_road_points;
        ///////////////////////
        // Preconditions
        ///////////////////////
        if (path.size() == 0)
            return;

        //clear previous points
        for (QGraphicsLineItem* item : scene_road_points)
            scene->removeItem((QGraphicsItem*)item);
        scene_road_points.clear();

        /// Draw all points
        QGraphicsLineItem *line1, *line2;
        std::string color;
        for (unsigned int i = 1; i < path.size(); i++)
            for(auto &&p_pair : iter::sliding_window(path, 2))
            {
                if(p_pair.size() < 2)
                    continue;
                Eigen::Vector2d a_point(p_pair[0].x(), p_pair[0].y());
                Eigen::Vector2d b_point(p_pair[1].x(), p_pair[1].y());
                Eigen::Vector2d dir = a_point - b_point;
                Eigen::Vector2d dir_perp = dir.unitOrthogonal();
                Eigen::ParametrizedLine segment = Eigen::ParametrizedLine<double, 2>::Through(a_point, b_point);
                Eigen::ParametrizedLine<double, 2> segment_perp((a_point+b_point)/2, dir_perp);
                auto left = segment_perp.pointAt(50);
                auto right = segment_perp.pointAt(-50);
                QLineF qsegment(QPointF(a_point.x(), a_point.y()), QPointF(b_point.x(), b_point.y()));
                QLineF qsegment_perp(QPointF(left.x(), left.y()), QPointF(right.x(), right.y()));

                if(i == 1 or i == path.size()-1)
                    color = "#00FF00"; //Green

                line1 = scene->addLine(qsegment, QPen(QBrush(QColor(QString::fromStdString(color))), 20));
                line2 = scene->addLine(qsegment_perp, QPen(QBrush(QColor(QString::fromStdString("#F0FF00"))), 20));

                line1->setZValue(2000);
                line2->setZValue(2000);
                scene_road_points.push_back(line1);
                scene_road_points.push_back(line2);
            }
    }

    ////////// CONTROLLER RELATED METHODS //////////
    RoboCompLaser::TLaserData computeLaser(RoboCompLaser::TLaserData laserData)
    {
    //        qDebug()<<"Navigation - "<< __FUNCTION__;

        auto lasernode = innerModel->getNode<InnerModelLaser>(QString("laser"));

        RoboCompLaser::TLaserData laserCombined;
        laserCombined = laserData;


        for (const auto &polyline : iter::chain(intimateSpaces,affordancesBlocked))
        {
            float min = std::numeric_limits<float>::max();
            float max = std::numeric_limits<float>::min();

            for (const auto &polylinePoint: polyline)
            {
                LocalPointPol lPol;
                QVec pInLaser = innerModel->transform("laser", QVec::vec3(polylinePoint.x(), 0, polylinePoint.y()), "world");
                lPol.angle = atan2(pInLaser.x(), pInLaser.z());
                lPol.dist = sqrt(pow(pInLaser.x(),2) + pow(pInLaser.z(),2));

                if( lPol.angle < min ) min = lPol.angle;
                if( lPol.angle > max ) max = lPol.angle;
            }

            //Recorremos todas las muestras del laser
            for (auto &laserSample: laserCombined)
            {
                //Compruebo que la muestra del laser corta a la polilinea. Es decir si esta comprendida entre el maximo y el minimo de antes
                if (laserSample.angle >= min and laserSample.angle <= max and fabs(max-min) < 3.14 )
                {
                    QVec lasercart = lasernode->laserTo(QString("laser"),laserSample.dist, laserSample.angle);

                    //recta que une el 0,0 con el punto del laser
                    QVec robotL = innerModel->transform("robot", "laser");
                    QLineF laserline(QPointF(robotL.x(), robotL.z()), QPointF(lasercart.x(), lasercart.z()));

                    auto previousPoint = polyline[polyline.size()-1];
                    QVec previousPointInLaser = innerModel->transform("laser", (QVec::vec3(previousPoint.x(), 0, previousPoint.y())), "world");

                    for (const auto &polylinePoint: polyline)
                    {
                        QVec currentPointInLaser = innerModel->transform("laser", (QVec::vec3(polylinePoint.x(), 0, polylinePoint.y())), "world");
                        QLineF polygonLine(QPointF(previousPointInLaser.x(), previousPointInLaser.z()), QPointF(currentPointInLaser.x(), currentPointInLaser.z()));

                        QPointF intersection;
                        auto intersectionType = laserline.intersect(polygonLine, &intersection);
                        float dist = QVector2D(intersection.x()-robotL.x(),intersection.y()-robotL.z()).length();

                        if ((intersectionType == QLineF::BoundedIntersection) and (dist<laserSample.dist))
                            laserSample.dist =  dist;

                        previousPointInLaser = currentPointInLaser;
                    }
                }
            }
        }
        return laserCombined;
    }

    bool isVisible(QPointF p)
    {
        QVec pointInLaser = innerModel->transform("laser", QVec::vec3(p.x(),0,p.y()),"world");
        return laser_poly.containsPoint(QPointF(pointInLaser.x(),pointInLaser.z()), Qt::OddEvenFill);
    }

    void computeForces(const std::vector<QPointF> &path, const RoboCompLaser::TLaserData &lData)
    {
        if (path.size() < 3)
        {
            return;
        }

        int pointIndex = 0;
        int nonVisiblePointsComputed = 0;

        // Go through points using a sliding windows of 3
        for (auto &group : iter::sliding_window(path, 3))
        {
            if (group.size() < 3)
                break; // break if too short

    //        if (group[0] == pathPoints[0])
    //            continue;

            auto p1 = QVector2D(group[0]);
            auto p2 = QVector2D(group[1]);
            auto p3 = QVector2D(group[2]);
            auto p = group[1];

            float min_dist;
            QVector2D force;

            if ((isVisible(p) == false))// if not visible (computed before) continue
            {

                auto [obstacleFound, vectorForce] = grid.vectorToClosestObstacle(p);

                if ((!obstacleFound) or (nonVisiblePointsComputed > 10))
                {
    //                qDebug () << "No obstacles found ";
                    nonVisiblePointsComputed++;

                    continue;
                }
                else
                {
    //                qDebug()<< "--- Obstacle found in grid ---";
                    min_dist = vectorForce.length() - (ROBOT_LENGTH / 2);
                    if (min_dist <= 0)
                        min_dist = 0.01;
                    force = vectorForce;

                }
                nonVisiblePointsComputed++;
            }
            else
            {
                std::vector<std::tuple<float, QVector2D, QPointF>> distances;
                // Apply to all laser points a functor to compute the distances to point p2
                std::transform(std::begin(laser_cart), std::end(laser_cart), std::back_inserter(distances), [p, this](QPointF &t) { //lasercart is updated in UpdateLaserPolygon
                    // compute distante from laser tip to point minus RLENGTH/2 or 0 and keep it positive
                    float dist = (QVector2D(p) - QVector2D(t)).length() - (ROBOT_LENGTH / 2);
                    if (dist <= 0)
                        dist = 0.01;
                    return std::make_tuple(dist,  QVector2D(p)-QVector2D(t), t);
                });

                // compute min distance
                auto min = std::min_element(std::begin(distances), std::end(distances), [](auto &a, auto &b) {
                    return std::get<float>(a) < std::get<float>(b);
                });
                min_dist = std::get<float>(*min);
                //QPointF min_angle = std::get<QPointF>(*min);
    //            qDebug()<< "Point "<< p << " --  min dist " << min_dist << "--- min angle "<< min_angle;
                force = std::get<QVector2D>(*min);
            }

            // INTERNAL curvature forces on p2
            QVector2D iforce = ((p1 - p2) / (p1 - p2).length() + (p3 - p2) / (p3 - p2).length());

            // EXTERNAL forces. We need the minimun distance from each point to the obstacle(s). we compute the shortest laser ray to each point in the path
            // compute minimun distances to each point within the laser field

            // rescale min_dist so 1 is ROBOT_LENGTH
            float magnitude = (1.f / ROBOT_LENGTH) * min_dist;
            // compute inverse square law
            magnitude = 10.f / (magnitude * magnitude);
            //if(magnitude > 25) magnitude = 25.;
            QVector2D f_force = magnitude * force.normalized();
            //qDebug() << magnitude << f_force;

            // Remove tangential component of repulsion force by projecting on line tangent to path (base_line)
            QVector2D base_line = (p1 - p3).normalized();
            const QVector2D itangential = QVector2D::dotProduct(f_force, base_line) * base_line;
            f_force = f_force - itangential;

    //        qDebug()<< "[NAVIGATION]"<< __FUNCTION__<< " --- i force " << iforce << "f force "<< f_force;
            // update node pos
            auto total = (KI * iforce) + (KE * f_force);

    //
            // limiters CHECK!!!!!!!!!!!!!!!!!!!!!!
            if (total.length() > 30)
                total = 8 * total.normalized();
            if (total.length() < -30)
                total = -8 * total.normalized();
    //        qDebug()<< "[NAVIGATION]"<< __FUNCTION__<< "---total forces = " << total;
            // move node only if they do not exit the laser polygon and do not get inside objects or underneath the robot.
            QPointF temp_p = p + total.toPointF();

    //        qDebug() << "Total force "<< total.toPointF()<< " New Point "<< temp_p;

            if (isVisible(temp_p)
//            if (isPointVisitable(temp_p)
                    and (!currentRobotPolygon.containsPoint(temp_p, Qt::OddEvenFill))
                    and (std::none_of(std::begin(intimateSpaces), std::end(intimateSpaces),[temp_p](const auto &poly) { return poly.containsPoint(temp_p, Qt::OddEvenFill);}))
    //                and (std::none_of(std::begin(personalSpaces), std::end(personalSpaces),[temp_p](const auto &poly) { return poly.containsPoint(temp_p, Qt::OddEvenFill);}))
                )
            {

                auto it = find_if(pathPoints.begin(), pathPoints.end(), [p] (auto & s) {
                    return (s.x() == p.x() and s.y() == p.y() );
                } );

                if (it != pathPoints.end())
                {
                   int index = std::distance(pathPoints.begin(), it);
                   pathPoints[index] = temp_p;
                }
            }
            pointIndex++;
        }

//        if(isVisible(currentRobotNose))
//        {
//            pathPoints[0] = currentRobotNose;
//            //drawRoad();
//        }
//        else
//        {
//            this->current_target.lock();
//                current_target.blocked.store(true);
//            this->current_target.unlock();
//            qDebug()<< "Robot Nose not visible -- NEEDS REPLANNING ";
//        }
        return;
    }

    void addPoints()
    {
    //        qDebug()<<"Navigation - "<< __FUNCTION__;

        std::vector<std::tuple<int, QPointF>> points_to_insert;
        for (auto &&[k, group] : iter::enumerate(iter::sliding_window(pathPoints, 2)))
        {
            auto &p1 = group[0];
            auto &p2 = group[1];

            if (isVisible(p1) == false or isVisible(p2) == false) //not visible
                continue;

            float dist = QVector2D(p1 - p2).length();

            if (dist > ROAD_STEP_SEPARATION)
            {
                float l = 0.9 * ROAD_STEP_SEPARATION / dist; //Crucial que el punto se ponga mas cerca que la condición de entrada
                QLineF line(p1, p2);
                points_to_insert.push_back(std::make_tuple(k + 1, QPointF{line.pointAt(l)}));
            }
            //qDebug() << __FUNCTION__ << k;
        }
        for (const auto &[l, p] : iter::enumerate(points_to_insert))
        {
            if(!currentRobotPolygon.containsPoint(std::get<QPointF>(p), Qt::OddEvenFill))
            {
    //                qDebug()<< "Add points  " << std::get<QPointF>(p);
                pathPoints.insert(pathPoints.begin() + std::get<int>(p) + l, std::get<QPointF>(p));
            }
        }
    //        qDebug() << __FUNCTION__ << "points inserted " << points_to_insert.size();
    }

    void cleanPoints()
    {
    //        qDebug()<<"Navigation - "<< __FUNCTION__;

        std::vector<QPointF> points_to_remove;
        for (const auto &group : iter::sliding_window(pathPoints, 2))
        {
            const auto &p1 = group[0];
            const auto &p2 = group[1];

            if ((!isVisible(p1)) or (!isVisible(p2))) //not visible
                continue;

            if (p2 == lastPointInPath)
                break;
            // check if p1 was marked to erase in the previous iteration
            if (std::find(std::begin(points_to_remove), std::end(points_to_remove), p1) != std::end(points_to_remove))
                continue;

            float dist = QVector2D(p1 - p2).length();
            if (dist < 0.5 * ROAD_STEP_SEPARATION)
                points_to_remove.push_back(p2);

            else if(currentRobotPolygon.containsPoint(p2, Qt::OddEvenFill))
            {
    //            qDebug()<<"-------------" << __FUNCTION__ << "------------- Removing point inside robot ";
                points_to_remove.push_back(p2);
            }

        }


        for (auto &&p : points_to_remove)
        {
            pathPoints.erase(std::remove_if(pathPoints.begin(), pathPoints.end(), [p](auto &r) { return p == r; }), pathPoints.end());

        }
    }

    ///////////////////////////////////////
    QPolygonF getRobotPolygon()
    {
    //        qDebug()<<"Navigation - "<< __FUNCTION__;

        QPolygonF robotP;

        auto bLWorld = innerModel->transform ("world", robotBottomLeft ,"base_mesh");
        auto bRWorld = innerModel->transform ("world", robotBottomRight ,"base_mesh");
        auto tRWorld = innerModel->transform ("world", robotTopRight ,"base_mesh");
        auto tLWorld = innerModel->transform ("world", robotTopLeft ,"base_mesh");


        robotP << QPointF(bLWorld.x(),bLWorld.z());
        robotP << QPointF(bRWorld.x(),bRWorld.z());
        robotP << QPointF(tRWorld.x(),tRWorld.z());
        robotP << QPointF(tLWorld.x(),tLWorld.z());

        FILE *fd = fopen("robot.txt", "w");
        for (const auto &r: robotP)
        {
            fprintf(fd, "%d %d\n", (int)r.x(), (int)r.y());
        }

        fprintf(fd, "%d %d\n", (int)robotP[0].x(), (int)robotP[0].y());

        fclose(fd);


        return robotP;
    }

    void updateLaserPolygon(const RoboCompLaser::TLaserData &lData)
    {
    //        qDebug()<<"Navigation - "<< __FUNCTION__;

        laser_poly.clear(); //stores the points of the laser in lasers refrence system
        laser_cart.clear();
        auto lasernode = innerModel->getNode<InnerModelLaser>(QString("laser"));

        for (const auto &l : lData)
        {
            //convert laser polar coordinates to cartesian
            QVec laserc = lasernode->laserTo(QString("laser"),l.dist, l.angle);
            QVec laserWord = lasernode->laserTo(QString("world"),l.dist, l.angle);
    //        QVec laserWorld = innerModel->transform("world",QVec::vec3(laserc.x(),0,laserc.y()),"laser");

            laser_poly << QPointF(laserc.x(),laserc.z());
            laser_cart.push_back(QPointF(laserWord.x(),laserWord.z()));
        }
    }

    QPointF getRobotNose()
    {
    //        qDebug()<<"Navigation - "<< __FUNCTION__;
        auto robot = QPointF(currentRobotPose.x(),currentRobotPose.z());

    //    return (robot + QPointF( (robotZLong/2 + 200) * sin(currentRobotPose.ry()), (robotZLong/2 + 200) * cos(currentRobotPose.ry())));
        return (robot + QPointF(300*sin(currentRobotPose.ry()),300*cos(currentRobotPose.ry())));

    }

    void drawRoad()
    {
//    //        qDebug()<<"Navigation - "<< __FUNCTION__;
//
//        ///////////////////////
//        // Preconditions
//        ///////////////////////
//        if (pathPoints.size() == 0)
//            return;
//
//
//        try	{ viewer->removeNode("points");} catch(const QString &s){	qDebug() <<"drawRoad" <<s; };
//        try	{ viewer->addTransform_ignoreExisting("points","world");} catch(const QString &s){qDebug()<<"drawRoad" << s; };
//
//        try
//        {
//            ///////////////////
//            //Draw all points
//            //////////////////
//            for (int i = 1; i < pathPoints.size(); i++)
//            {
//                QPointF &w = pathPoints[i];
//                QPointF &wAnt = pathPoints[i - 1];
//                if (w == wAnt) //avoid calculations on equal points
//                    continue;
//
//                QLine2D l(QVec::vec2(wAnt.x(),wAnt.y()), QVec::vec2(w.x(),w.y()));
//                QLine2D lp = l.getPerpendicularLineThroughPoint(QVec::vec2(w.x(), w.y()));
//                QVec normal = lp.getNormalForOSGLineDraw();  //3D vector
//                QString item = "p_" + QString::number(i);
//                viewer->addTransform_ignoreExisting(item, "points", QVec::vec6(w.x(), 10, w.y(), 0, 0, 0));
//
//
//                if(i == 1)
//                {
//                    viewer->drawLine(item + "_point", item, QVec::zeros(3), normal, 500, 40, "#FF0000");  //Rojo
//                }
//                else if (i == pathPoints.size()-1)
//                    viewer->drawLine(item + "_point", item, QVec::zeros(3), normal, 500, 40, "#FF0000");  //Rojo
//
//                else if (isVisible(w))
//                    viewer->drawLine(item + "_point", item, QVec::zeros(3), normal, 500, 40, "#00FFF0");
//                else
//                    viewer->drawLine(item + "_point", item, QVec::zeros(3), normal, 500, 40, "#A200FF");  //Morado
//
//            }
//        }
//        catch(const QString &s){qDebug()<<"drawRoad" << s;}
//    //        qDebug()<<"END "<<__FUNCTION__;
    }
};


#endif //PROJECT_NAVIGATION_H
