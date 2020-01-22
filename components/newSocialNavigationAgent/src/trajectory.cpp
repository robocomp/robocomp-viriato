//
// Created by robolab on 9/01/20.
//

#include "trajectory.h"

void Trajectory::initialize(const std::shared_ptr<InnerModel> &innerModel_,
        const std::shared_ptr<InnerViewer> &viewer_,
        const std::shared_ptr< RoboCompCommonBehavior::ParameterList > &configparams_)

{
    qDebug()<<"Initializing Trajectory";

    innerModel = innerModel_;
    viewer = viewer_;
    configparams = configparams_;

    try{ robotname = configparams->at("RobotName").value;} catch(const std::exception &e){ std::cout << e.what() << " Trajectory::initialize - No Robot name defined in config. Using default 'robot' " << std::endl;}

    collisions =  std::make_shared<Collisions>();
    collisions->initialize(innerModel, configparams);
    initGrid();


}

void Trajectory::update(const RoboCompLaser::TLaserData &laserData_)
{
    laserData = computeLaser(laserData_);
    //    computeVisibility(points, laser_polygon);
//    cleanPath(); // might go in a faster timer
//    controller();
//    updateRobot();
//    checkProgress();

}

void Trajectory::updatePolylines(SNGPersonSeq persons_, SNGPolylineSeq intimate_seq,SNGPolylineSeq personal_seq,SNGPolylineSeq social_seq,SNGPolylineSeq object_seq,SNGPolylineSeq objectsblocking_seq)
{

    polylines_intimate.clear();
    polylines_personal.clear();
    polylines_social.clear();
    polylines_objects_total.clear();
    polylines_objects_blocked.clear();

    toAvoidLaser.clear();

    for (auto intimate : intimate_seq)
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

   for (auto object_block : objectsblocking_seq)
    {
        QPolygonF polygon;
        for (auto ob : object_block)
            polygon << QPointF(ob.x, ob.z);
        polylines_objects_blocked.push_back(polygon);
    }

    //iter::chain??
    toAvoidLaser.insert(toAvoidLaser.end(), polylines_objects_blocked.begin(),polylines_objects_blocked.end());
    toAvoidLaser.insert(toAvoidLaser.end(), polylines_intimate.begin(),polylines_intimate.end());

    updateFreeSpaceMap();


}

void Trajectory::initGrid()
{
    try
    {
        dimensions.TILE_SIZE = int(TILE_SIZE_);
        dimensions.HMIN = std::min(collisions->outerRegion.left(), collisions->outerRegion.right());
        dimensions.WIDTH = std::max(collisions->outerRegion.left(), collisions->outerRegion.right()) - dimensions.HMIN;
        dimensions.VMIN = std::min(collisions->outerRegion.top(), collisions->outerRegion.bottom());
        dimensions.HEIGHT = std::max(collisions->outerRegion.top(), collisions->outerRegion.bottom()) - dimensions.VMIN;

    }
    catch(const std::exception &e)
    {
        std::cout << "Exception " << e.what() << " Trajectory::initialize(). OuterRegion parameters not found in config file" << std::endl;
        throw e;
    }

//    grid.initialize(dimensions,  collisions);
    grid.draw(viewer.get());

}

void Trajectory::resetGrid()
{
    toSetFree.clear();
    toResetCost.clear();

    toSetFree.insert(toSetFree.end(), prev_polylines_intimate.begin(),prev_polylines_intimate.end());
    toSetFree.insert(toSetFree.end(), prev_polylines_objects_blocked.begin(),prev_polylines_objects_blocked.end());

    toResetCost.insert(toResetCost.end(), prev_polylines_personal.begin(),prev_polylines_personal.end());
    toResetCost.insert(toResetCost.end(), prev_polylines_social.begin(),prev_polylines_social.end());
    toResetCost.insert(toResetCost.end(), prev_polylines_objects_total.begin(),prev_polylines_objects_total.end());


    for (auto &&prev : toSetFree)
    {
        grid.markAreaInGridAs(prev, true);
    }

    for (auto &&prev_cost : toResetCost)
    {
        grid.modifyCostInGrid(prev_cost, 1.0);
    }
}

void Trajectory::updateFreeSpaceMap()
{
    // First remove polygons from last iteration respecting cell occupied by furniture
    resetGrid();

    //To set occupied
    for (auto &&poly_intimate : polylines_intimate)
        grid.markAreaInGridAs(poly_intimate, false);

    for (auto &&poly_object_block : polylines_objects_blocked)
        grid.markAreaInGridAs(poly_object_block, false);

    //To modify cost
    for (auto &&poly_object : polylines_objects_total)
        grid.modifyCostInGrid(poly_object, 1.5);

    for (auto &&poly_soc : polylines_social)
        grid.modifyCostInGrid(poly_soc, 2.0);

    for (auto &&poly_per : polylines_personal)
        grid.modifyCostInGrid(poly_per, 4.0);



    prev_polylines_intimate = polylines_intimate;
    prev_polylines_personal = polylines_personal;
    prev_polylines_social = polylines_social;
    prev_polylines_objects_total = polylines_objects_total;
    prev_polylines_objects_blocked = polylines_objects_blocked;

    grid.draw(viewer.get());

}

RoboCompLaser::TLaserData Trajectory::computeLaser(RoboCompLaser::TLaserData laserData)
{

    auto lasernode = innerModel->getNode<InnerModelLaser>(QString("laser"));

    RoboCompLaser::TLaserData laserCombined;
    laserCombined = laserData;

    FILE *fd = fopen("entradaL.txt", "w");
    for (const auto &laserSample: laserData)
    {
        QVec vv =  lasernode->laserTo(QString("world"), laserSample.dist, laserSample.angle);
        fprintf(fd, "%d %d\n", (int)vv(0), (int)vv(2));
    }
    fclose(fd);



    FILE *fd3 = fopen("polygonL.txt", "w");

    for (const auto &polyline : toAvoidLaser)
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

            fprintf(fd3, "%d %d\n", (int)pInLaser.x(), (int)pInLaser.z());

        }

        //Recorremos todas las muestras del laser
        for (auto &laserSample: laserCombined)
        {
            //Compruebo que la muestra del laser corta a la polilinea. Es decir si esta comprendida entre el maximo y el minimo de antes
            if (laserSample.angle >= min and laserSample.angle <= max and fabs(max-min) < 3.14 )
            {
                QVec lasercart = lasernode->laserTo(QString("laser"),laserSample.dist, laserSample.angle);
                //recta que une el 0,0 con el punto del laser
                auto laser_in_robot = innerModel->transform("world", "laser");
//                auto laser_in_robot = innerModel->transform("world", "robot"); //robotPose

                QLineF laserline(QPointF(laser_in_robot.x(), laser_in_robot.z()), QPointF(lasercart.x(), lasercart.z()));
//                QLineF laserline(QPointF(0, 0), QPointF(lasercart.x(), lasercart.z()));
//
                auto previousPoint = polyline[polyline.size()-1];
                QVec previousPointInLaser = innerModel->transform("laser", (QVec::vec3(previousPoint.x(), 0, previousPoint.y())), "world");

                for (const auto &polylinePoint: polyline)
                {
                    QVec currentPointInLaser = innerModel->transform("laser", (QVec::vec3(polylinePoint.x(), 0, polylinePoint.y())), "world");
                    QLineF polygonLine(QPointF(previousPointInLaser.x(), previousPointInLaser.z()), QPointF(currentPointInLaser.x(), currentPointInLaser.z()));

                    QPointF intersection;
                    auto intersectionType = laserline.intersect(polygonLine, &intersection);
                    float dist = QVector2D(intersection.x()-laser_in_robot.x(),intersection.y()-laser_in_robot.z()).length();

                    if ((intersectionType == QLineF::BoundedIntersection) and (dist<laserSample.dist))
                        laserSample.dist =  dist;


                    previousPointInLaser = currentPointInLaser;

                }
            }
        }
    }
    fclose(fd3);

    FILE *fd1 = fopen("salidaL.txt", "w");
    for (auto &laserSample: laserCombined)
    {
        QVec vv =  lasernode->laserTo(QString("world"), laserSample.dist, laserSample.angle);
        fprintf(fd1, "%d %d\n", (int)vv(0), (int)vv(2));
    }
    fclose(fd1);

    return laserCombined;
}

