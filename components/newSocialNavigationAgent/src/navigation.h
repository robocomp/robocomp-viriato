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
        struct Target : public std::mutex
        {
            QPointF p;
            std::atomic_bool active = false;
            std::atomic_bool blocked = true;
        };

        Target current_target;
        bool robotAutoMov = false;

        void initialize(const std::shared_ptr<InnerModel> &innerModel_, const std::shared_ptr<InnerViewer> &viewer_,
                std::shared_ptr< RoboCompCommonBehavior::ParameterList > configparams_, OmniRobotPrx omnirobot_proxy_)
        {
            qDebug()<<"NAVIGATION -- " <<__FUNCTION__;

            innerModel = innerModel_;

            configparams = configparams_;
            viewer = viewer_;

            omnirobot_proxy = omnirobot_proxy_;
            omnirobot_proxy->setSpeedBase(0,0,0); //grid can't be initialized if the robot is moving

            collisions =  std::make_shared<Collisions>();

            collisions->initialize(innerModel, configparams);
            grid.initialize(collisions);
            grid.draw(viewer);
            controller.initialize(innerModel,configparams);


            robotXWidth = std::stof(configparams->at("RobotXWidth").value);
            robotZLong = std::stof(configparams->at("RobotZLong").value);

            reloj.restart();

        };

        void updateInnerModel(const std::shared_ptr<InnerModel> &innerModel_)
        {
            qDebug()<<"NAVIGATION ->" <<__FUNCTION__;
            innerModel = innerModel_;
            controller.updateInnerModel(innerModel);

        };

        void update(const RoboCompLaser::TLaserData &laserData_, bool personMoved)
        {

            RoboCompLaser::TLaserData laserData;
            laserData = computeLaser(laserData_);
            currentRobotPose = innerModel->transformS6D("world","robot");
            currentRobotPolygon = getRobotPolygon();

            if(personMoved)
            {
                this->current_target.lock();
                    current_target.blocked.store(true);
                this->current_target.unlock();
            }

            if (checkPathState() == false)
                return;


            updateLaserPolygon(laserData);
            computeForces(points, laserData);
            cleanPoints();
            addPoints();

            auto [blocked, active, xVel,zVel,rotVel] = controller.update(points, laserData, current_target.p, currentRobotPose);

            if (blocked)
            {
                this->current_target.lock();
                    current_target.blocked.store(true);
                this->current_target.unlock();

                omnirobot_proxy->setSpeedBase(0,0,0);


            }
            if (!active)
            {
                this->current_target.lock();
                    current_target.active.store(false);
                this->current_target.unlock();

                points.clear();

                omnirobot_proxy->setSpeedBase(0,0,0);

                if(robotAutoMov) newRandomTarget();
            }

            if (!blocked and active ) omnirobot_proxy->setSpeedBase(xVel,zVel,rotVel);


            drawRoad();

        };

        bool checkPathState()
        {
            qDebug() << __FUNCTION__;


            if (current_target.active.load() == true)
            {

                if (current_target.blocked.load()==true) {

                    if (findNewPath()==false) {

                        this->current_target.lock();
                            current_target.active.store(false);
                        this->current_target.unlock();

                        points.clear();

                        if(robotAutoMov) newRandomTarget();

                        return false;
                    }

                    else{

                        this->current_target.lock();
                            this->current_target.blocked.store(false);
                        this->current_target.unlock();

                        reloj.restart();
                    }
                }

                return true;
            }

            else
                return false;

        }


        void newRandomTarget()
        {

            auto hmin = std::min(collisions->outerRegion.left(), collisions->outerRegion.right());
            auto width = std::max(collisions->outerRegion.left(), collisions->outerRegion.right()) - hmin;
            auto vmin = std::min(collisions->outerRegion.top(), collisions->outerRegion.bottom());
            auto height = std::max(collisions->outerRegion.top(), collisions->outerRegion.bottom()) - vmin;

            auto x = hmin + (double)rand() * width / (double)RAND_MAX;
            auto z = vmin + (double)rand() * height/ (double)RAND_MAX;

            this->current_target.lock();
            current_target.active.store(true);
            current_target.blocked.store(true);
            current_target.p = QPointF(x,z);

            this->current_target.unlock();

            qDebug()<<"NAVIGATION -- " <<__FUNCTION__ <<"New Random Target" << current_target.p;

        }

        void newTarget(QPointF newT)
        {
            qDebug()<<"NAVIGATION -- " <<__FUNCTION__ <<"New Target arrived "<< newT;

            this->current_target.lock();
                current_target.active.store(true);
                current_target.blocked.store(true);
                current_target.p = newT;

            this->current_target.unlock();
        }

        const TMap& getMap() const { return grid; };


        void updatePolylines(SNGPersonSeq persons_, SNGPolylineSeq intimate_seq,SNGPolylineSeq personal_seq,SNGPolylineSeq social_seq,
                SNGPolylineSeq object_seq, SNGPolylineSeq object_lowProbVisited ,SNGPolylineSeq object_mediumProbVisited ,SNGPolylineSeq object_highProbVisited, SNGPolylineSeq objectsblocking_seq)
        {
            qDebug()<<"NAVIGATION -- " <<__FUNCTION__;

            polylines_intimate.clear();
            polylines_personal.clear();
            polylines_social.clear();
            polylines_objects_total.clear();
            polylines_lowVisited.clear();
            polylines_mediumVisited.clear();
            polylines_highVisited.clear();
            polylines_objects_blocked.clear();

            for (auto intimate: intimate_seq)
            {
                QPolygonF polygon;
                for (auto i : intimate)
                    polygon << QPointF(i.x, i.z);
                polylines_intimate.push_back(polygon);
            }

            for (auto personal : personal_seq)
            {
                QPolygonF polygon;
                for (auto p : personal)
                    polygon << QPointF(p.x, p.z);
                polylines_personal.push_back(polygon);
            }

            for (auto social : social_seq)
            {
                QPolygonF polygon;
                for (auto s : social)
                    polygon << QPointF(s.x, s.z);
                polylines_social.push_back(polygon);
            }

            for (auto object : object_seq)
            {
                QPolygonF polygon;
                for (auto o : object)
                    polygon << QPointF(o.x, o.z);
                polylines_objects_total.push_back(polygon);
            }

            for (auto object : object_lowProbVisited)
            {
                QPolygonF polygon;
                for (auto o : object)
                    polygon << QPointF(o.x, o.z);
                polylines_lowVisited.push_back(polygon);
            }

            for (auto object : object_mediumProbVisited)
            {
                QPolygonF polygon;
                for (auto o : object)
                    polygon << QPointF(o.x, o.z);
                polylines_mediumVisited.push_back(polygon);
            }

            for (auto object : object_highProbVisited)
            {
                QPolygonF polygon;
                for (auto o : object)
                    polygon << QPointF(o.x, o.z);
                polylines_highVisited.push_back(polygon);
            }

            for (auto object_block : objectsblocking_seq)
            {
                QPolygonF polygon;
                for (auto ob : object_block)
                    polygon << QPointF(ob.x, ob.z);
                polylines_objects_blocked.push_back(polygon);
            }

            updateFreeSpaceMap();

        };




    private:
        std::shared_ptr<Collisions> collisions;
        std::shared_ptr<InnerModel> innerModel;
        std::shared_ptr<InnerViewer> viewer;
        std::shared_ptr<RoboCompCommonBehavior::ParameterList> configparams;

        OmniRobotPrx omnirobot_proxy;

        typedef struct { float dist; float angle;} LocalPointPol;

        TMap grid;
        TController controller;

        std::vector<QPolygonF> polylines_intimate,polylines_personal,polylines_social,polylines_objects_total,polylines_objects_blocked, polylines_lowVisited ,polylines_mediumVisited ,polylines_highVisited;

        // ElasticBand
        std::vector<QPointF> points;

        const float ROBOT_LENGTH = 400;

//        float KE = 3.0;
//        float KI = 120;
        float KE = 6.0;
        float KI = 200;

        const float ROAD_STEP_SEPARATION = ROBOT_LENGTH * 0.9;


    //Integrating time
        QTime reloj = QTime::currentTime();

        QPointF lastPointInPath;

        QPolygonF currentRobotPolygon, laser_poly;
        std::vector<QPointF> laser_cart;
        QVec currentRobotPose;
        float robotXWidth, robotZLong;


    ////////// GRID RELATED METHODS //////////
    void updateFreeSpaceMap()
    {
        qDebug()<<"NAVIGATION -- " <<__FUNCTION__;

        // First remove polygons from last iteration respecting cell occupied by furniture
        grid.resetGrid();

        //To set occupied
        for (auto &&poly_intimate : iter::chain(polylines_intimate, polylines_objects_blocked))
            grid.markAreaInGridAs(poly_intimate, false);

        //To modify cost
        for (auto &&poly_object : polylines_objects_total)
            grid.modifyCostInGrid(poly_object, 1.5);

        for (auto &&poly_l : polylines_lowVisited)
            grid.modifyCostInGrid(poly_l, 2);

        for (auto &&poly_m : polylines_mediumVisited)
            grid.modifyCostInGrid(poly_m, 2.5);

        for (auto &&poly_h : polylines_highVisited)
            grid.modifyCostInGrid(poly_h, 3);

        for (auto &&poly_soc : polylines_social)
            grid.modifyCostInGrid(poly_soc, 4.0);

        for (auto &&poly_per : polylines_personal)
            grid.modifyCostInGrid(poly_per, 6.0);

        qDebug()<<"drawing grid";

        grid.draw(viewer);

    }

    ////////// CONTROLLER RELATED METHODS //////////
    RoboCompLaser::TLaserData computeLaser(RoboCompLaser::TLaserData laserData)
    {

        auto lasernode = innerModel->getNode<InnerModelLaser>(QString("laser"));

        RoboCompLaser::TLaserData laserCombined;
        laserCombined = laserData;


        for (const auto &polyline : iter::chain(polylines_intimate,polylines_objects_blocked))
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

    //CONTROLLER METHODS

    bool findNewPath()
    {
        qDebug()<<"NAVIGATION -- " <<__FUNCTION__;

        points.clear();

        // extract target from current_path
        this->current_target.lock();
            auto target = this->current_target.p;
        this->current_target.unlock();

        QPointF robotNose = getRobotNose();

        //mark spece under the robot as occupied
        grid.markAreaInGridAs(currentRobotPolygon, false);

        std::list<QPointF> path = grid.computePath(QPointF(robotNose.x() ,robotNose.y()), target);


        if (path.size() > 0)
        {
            points.push_back(robotNose);

            for (const QPointF &p : path)
                points.push_back(p);

            lastPointInPath = points[points.size()-1];
            grid.markAreaInGridAs(currentRobotPolygon, true);

            return true;
        }
        else
        {
            qDebug() << __FUNCTION__ << "Path not found";
            grid.markAreaInGridAs(currentRobotPolygon, true);


            return false;
        }

    }


    bool isVisible(QPointF p)
    {
        QVec pointInLaser = innerModel->transform("laser", QVec::vec3(p.x(),0,p.y()),"world");
        return laser_poly.containsPoint(QPointF(pointInLaser.x(),pointInLaser.z()), Qt::OddEvenFill);
    }


    void computeForces(const std::vector<QPointF> &path, const RoboCompLaser::TLaserData &lData)
    {
        qDebug() << __FUNCTION__;

        if (path.size() < 3) {
            return;
        }

        // Go through points using a sliding windows of 3
        for (auto group : iter::sliding_window(path, 3))
        {
            if (group.size() < 3)
                break; // break if too short

            auto p1 = QVector2D(group[0]);
            auto p2 = QVector2D(group[1]);
            auto p3 = QVector2D(group[2]);
            auto p = group[1];

            if (isVisible(p) == false) // if not visible (computed before) continue
                continue;

            // INTERNAL curvature forces on p2
            QVector2D iforce = ((p1 - p2) / (p1 - p2).length() + (p3 - p2) / (p3 - p2).length());

            // EXTERNAL forces. We need the minimun distance from each point to the obstacle(s). we compute the shortest laser ray to each point in the path
            // compute minimun distances to each point within the laser field

            std::vector<std::tuple<float, QVector2D>> distances;
            // Apply to all laser points a functor to compute the distances to point p2
            std::transform(std::begin(laser_cart), std::end(laser_cart), std::back_inserter(distances), [p, this](QPointF &t) { //lasercart is updated in UpdateLaserPolygon
                // compute distante from laser tip to point minus RLENGTH/2 or 0 and keep it positive
                float dist = (QVector2D(p) - QVector2D(t)).length() - (ROBOT_LENGTH / 2);
                if (dist <= 0)
                    dist = 0.01;
                return std::make_tuple(dist, QVector2D(p) - QVector2D(t));
            });

            // compute min distance
            auto min = std::min_element(std::begin(distances), std::end(distances), [](auto &a, auto &b) { return std::get<float>(a) < std::get<float>(b); });
            float min_dist = std::get<float>(*min);

            QVector2D force = std::get<QVector2D>(*min);
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

            // update node pos
            auto total = (KI * iforce) + (KE * f_force);

            // limiters CHECK!!!!!!!!!!!!!!!!!!!!!!
            if (total.length() > 30)
                total = 8 * total.normalized();
            if (total.length() < -30)
                total = -8 * total.normalized();

            // move node only if they do not exit the laser polygon and do not get inside objects or underneath the robot.
            QPointF temp_p = p + total.toPointF();
            if(isVisible(temp_p)
                    and (currentRobotPolygon.containsPoint(temp_p, Qt::OddEvenFill) == false)
                    and (std::none_of(std::begin(polylines_intimate), std::end(polylines_intimate),[temp_p](const auto &poly) { return poly.containsPoint(temp_p, Qt::OddEvenFill);})))
                p = temp_p;
        }

        QPointF robotNose = getRobotNose();
        points[0] = robotNose;

    }


    void addPoints()
    {
        qDebug() << __FUNCTION__;

        std::vector<std::tuple<int, QPointF>> points_to_insert;
        for (auto &&[k, group] : iter::enumerate(iter::sliding_window(points, 2)))
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
            points.insert(points.begin() + std::get<int>(p) + l, std::get<QPointF>(p));
        }
//        qDebug() << __FUNCTION__ << "points inserted " << points_to_insert.size();
    }

    void cleanPoints()
    {
        qDebug() << __FUNCTION__;

        std::vector<QPointF> points_to_remove;
        for (const auto &group : iter::sliding_window(points, 2))
        {
            const auto &p1 = group[0];
            const auto &p2 = group[1];

            if ((isVisible(p1)== false) or (isVisible(p2) == false)) //not visible
                continue;

            if (p2 == lastPointInPath)
                break;
            // check if p1 was marked to erase in the previous iteration
            if (std::find(std::begin(points_to_remove), std::end(points_to_remove), p1) != std::end(points_to_remove))
                continue;

            float dist = QVector2D(p1 - p2).length();
            if (dist < 0.5 * ROAD_STEP_SEPARATION)
                points_to_remove.push_back(p2);
        }


        for (auto &&p : points_to_remove)
        {
            points.erase(std::remove_if(points.begin(), points.end(), [p](auto &r) { return p == r; }), points.end());

        }
    }

    QPolygonF getRobotPolygon()
    {
        QPolygonF robotP;

        auto bottomLeft     = QVec::vec3(- robotXWidth/2 -100, 0, - robotZLong/2-100);
        auto bottomRight    = QVec::vec3(+ robotXWidth/2 +100, 0, - robotZLong/2-100);
        auto topRight       = QVec::vec3( + robotXWidth/2 +100, 0 , + robotZLong/2 +100);
        auto topLeft        = QVec::vec3(- robotXWidth/2 -100, 0, + robotZLong/2+100);

        auto bLWorld = innerModel->transform ("world", bottomLeft ,"base_mesh");
        auto bRWorld = innerModel->transform ("world", bottomRight ,"base_mesh");
        auto tRWorld = innerModel->transform ("world", topRight ,"base_mesh");
        auto tLWorld = innerModel->transform ("world", topLeft ,"base_mesh");


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
        qDebug() << __FUNCTION__;

        laser_poly.clear(); //stores the points of the laser in lasers refrence system
        laser_cart.clear();
        auto lasernode = innerModel->getNode<InnerModelLaser>(QString("laser"));

        for (const auto &l : lData)
        {
            //convert laser polar coordinates to cartesian
            QVec laserc = lasernode->laserTo(QString("laser"),l.dist, l.angle);
            laser_poly << QPointF(laserc.x(),laserc.z());
            laser_cart.push_back(QPointF(laserc.x(),laserc.z()));
        }

//        FILE *fd = fopen("laserPoly.txt", "w");
//        for (const auto &lp : laser_poly)
//        {
//            QVec p = innerModel->transform("world",QVec::vec3(lp.x(),0,lp.y()),"laser");
//            fprintf(fd, "%d %d\n", (int)p.x(), (int)p.z());
//        }
//        fclose(fd);

    }

    QPointF getRobotNose()
    {
        auto robot = QPointF(currentRobotPose.x(),currentRobotPose.z());
        return (robot + QPointF( ROAD_STEP_SEPARATION *sin(currentRobotPose.ry()), ROAD_STEP_SEPARATION *cos(currentRobotPose.ry())));
    }

    void drawRoad()
    {
        qDebug() << __FUNCTION__;

        ///////////////////////
        // Preconditions
        ///////////////////////
        if (points.size() == 0)
            return;


        try	{ viewer->removeNode("points");} catch(const QString &s){	qDebug() <<"drawRoad" <<s; };
        try	{ viewer->addTransform_ignoreExisting("points","world");} catch(const QString &s){qDebug()<<"drawRoad" << s; };

        try
        {
            ///////////////////
            //Draw all points
            //////////////////
            for (int i = 1; i < points.size(); i++)
            {
                QPointF &w = points[i];
                QPointF &wAnt = points[i - 1];
                if (w == wAnt) //avoid calculations on equal points
                    continue;

                QLine2D l(QVec::vec2(wAnt.x(),wAnt.y()), QVec::vec2(w.x(),w.y()));
                QLine2D lp = l.getPerpendicularLineThroughPoint(QVec::vec2(w.x(), w.y()));
                QVec normal = lp.getNormalForOSGLineDraw();  //3D vector
                QString item = "p_" + QString::number(i);
                viewer->addTransform_ignoreExisting(item, "points", QVec::vec6(w.x(), 10, w.y(), 0, 0, 0));


                if(i == 1)
                {
                    viewer->drawLine(item + "_point", item, QVec::zeros(3), normal, 500, 40, "#007CFF");  //Azul
                }
                else if (i == points.size()-1)
                    viewer->drawLine(item + "_point", item, QVec::zeros(3), normal, 500, 40, "#FF0000");  //Rojo

//                else if (isVisible(w))
//                    viewer->drawLine(item + "_point", item, QVec::zeros(3), normal, 500, 40, "#00ECFF"); //TAKE WIDTH FROM PARAMS!!!
                else
                    viewer->drawLine(item + "_point", item, QVec::zeros(3), normal, 500, 40, "#00ECFF");  //Morado


            }

        }
        catch(const QString &s){qDebug()<<"drawRoad" << s;}
        qDebug()<<"END "<<__FUNCTION__;
    }



};


#endif //PROJECT_NAVIGATION_H
