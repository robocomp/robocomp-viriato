//
// Created by robolab on 9/01/20.
//

#include "trajectory.h"

void Trajectory::initialize(const std::shared_ptr<InnerModel> &innerModel_,
        const std::shared_ptr<InnerViewer> &viewer_,
        const std::shared_ptr< RoboCompCommonBehavior::ParameterList > &configparams_,
        LaserPrx laser_prx,
        OmniRobotPrx omnirobot_proxy)

{
    qDebug()<<"Initializing Trajectory";


    innerModel = innerModel_;
    viewer = viewer_;
    configparams = configparams_;
    qDebug()<<"-----1----";
    laser_proxy = laser_prx;

    qDebug()<<"-----2----";
    try{ robotname = configparams->at("RobotName").value;} catch(const std::exception &e){ std::cout << e.what() << " Sampler::initialize - No Robot name defined in config. Using default 'robot' " << std::endl;}
    qDebug()<<"-----3----";

    collisions =  std::make_shared<Collisions>();
    collisions->initialize(innerModel, configparams);
    initGrid();


}

void Trajectory::update(const std::shared_ptr<InnerModel> &innerModel_)
{
    innerModel = innerModel_;
    RoboCompLaser::TLaserData laserData;
    TBaseState baseState;

    try{
//        laserData  = laser_proxy->getLaserData();
        laserData  =laser_proxy->getLaserAndBStateData(baseState);

    }
    catch(const Ice::Exception &e){std::cout <<"SHIT" <<e.what() << std::endl;};
    computeLaser(laserData);
//    modifyLaser(laserData);
    //    computeVisibility(points, laser_polygon);
//    cleanPath(); // might go in a faster timer
//    controller();
//    updateRobot();
//    checkProgress();

}

void Trajectory::updatePolylines(const std::shared_ptr<InnerModel> &innerModel_, SNGPersonSeq persons_, SNGPolylineSeq intimate_seq,SNGPolylineSeq personal_seq,SNGPolylineSeq social_seq,SNGPolylineSeq object_seq,SNGPolylineSeq objectsblocking_seq)
{
    innerModel = innerModel_;

    polylines_intimate.clear();
    polylines_personal.clear();
    polylines_social.clear();
    polylines_objects_total.clear();
    polylines_objects_blocked.clear();

    intimate_spaces.clear();
    intimate_spaces = intimate_seq;

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

    updateFreeSpaceMap();


}


///ADMIN GRID
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
        //robocomp::exception ex("OuterRegion parameters not found in config file");
        throw e;
    }

    grid.initialize(dimensions,  collisions);
//    grid.initialize(dimensions, TCell{0, true, false, 1.f});
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
    {
        grid.markAreaInGridAs(poly_intimate, false);
    }

    for (auto &&poly_object_block : polylines_objects_blocked)
    {
        grid.markAreaInGridAs(poly_object_block, false);
    }

    //To modify cost
    for (auto &&poly_object : polylines_objects_total)
    {
        grid.modifyCostInGrid(poly_object, 1.5);
    }

    for (auto &&poly_soc : polylines_social)
    {
        grid.modifyCostInGrid(poly_soc, 2.0);
    }

    for (auto &&poly_per : polylines_personal)
    {
        grid.modifyCostInGrid(poly_per, 4.0);
    }


    prev_polylines_intimate = polylines_intimate;
    prev_polylines_personal = polylines_personal;
    prev_polylines_social = polylines_social;
    prev_polylines_objects_total = polylines_objects_total;
    prev_polylines_objects_blocked = polylines_objects_blocked;

    grid.draw(viewer.get());

}




void Trajectory::computeLaser(RoboCompLaser::TLaserData laserData)
{
    const float LASER_DIST_STEP = 0.05;

    FILE *fd = fopen("entradaL.txt", "w");
    auto lasernode = innerModel->getNode<InnerModelLaser>(QString("laser"));
    for (const auto &laserSample: laserData)
    {
        QVec vv = lasernode->laserTo(QString("world"),laserSample.dist, laserSample.angle);
//        QVec vv = innerModel->laserTo("world", "laser", laserSample.dist, laserSample.angle);
//        QVec vv = innerModel->transform("world", QVec::vec3(laserSample.dist * sin(laserSample.angle), 0, laserSample.dist * cos(laserSample.angle)), "laser");
        fprintf(fd, "%d %d\n", (int)vv(0), (int)vv(2));
    }
    fclose(fd);

    for (auto &&l : laserData)
    {
//        QLineF line (QPointF(innerModel->transform("laser", QVec::vec3(0, 0, 0), "world")), QPointF(innerModel->transform("laser", QVec::vec3(l.dist * sin(l.angle), 0, l.dist * cos(l.angle)), "world")));
        QLineF line (QPointF(innerModel->transform("world", QVec::vec3(0, 0, 0), "laser")), QPointF(innerModel->laserTo("world", "laser", l.dist, l.angle)));
        float step = 100.f / line.length();
        for (auto t : iter::range(0.f, 1.f, LASER_DIST_STEP))
        {
            auto point = line.pointAt(t);
            auto pointInWorld = (QPointF(innerModel->transform("world", QVec::vec3(point.x(), 0, point.y()), "laser")));
//            auto pointInWorld = (QPointF(innerModel->laserTo("world", "laser"), );


            if (std::any_of(std::begin(polylines_intimate), std::end(polylines_intimate), [pointInWorld](auto &box) { return box.containsPoint(pointInWorld,Qt::OddEvenFill); }))
            {
                l.dist = QVector2D(point - line.pointAt(0)).length() - (step * 2);
                break;
            }
        }
    }

    FILE *fd2 = fopen("salidaL.txt", "w");
    for (auto &laserSample: laserData)
    {
//        QVec vv = innerModel->laserTo("world", "laser", laserSample.dist, laserSample.angle);
        QVec vv = innerModel->transform("world", QVec::vec3(laserSample.dist * sin(laserSample.angle), 0, laserSample.dist * cos(laserSample.angle)), "laser");
        fprintf(fd2, "%d %d\n", (int)vv(0), (int)vv(2));
    }
    fclose(fd2);



}


RoboCompLaser::TLaserData Trajectory::modifyLaser(RoboCompLaser::TLaserData laserData)
{
// 	SNGPolyline acho;
// 	acho.resize(2);
// 	acho[0].x = 0;
// 	acho[0].z = 3000;
// 	acho[1].x = 5000;
// 	acho[1].z = 0;
// 	l.push_back(acho);

    FILE *fd = fopen("entradaL.txt", "w");
    for (auto &laserSample: laserData)
    {
        QVec vv = innerModel->laserTo("world", "laser", laserSample.dist, laserSample.angle);
        fprintf(fd, "%f %f\n", vv(0), vv(2));
    }
    fclose(fd);

    RoboCompLaser::TLaserData laserCombined;
    laserCombined = laserData;

    for (auto polyline : intimate_spaces)
    {
        float min = std::numeric_limits<float>::max();
        float max = std::numeric_limits<float>::min();

        for (auto polylinePoint: polyline)
        {
            LocalPointPol lPol;
            QVec pInLaser = innerModel->transform("laser", QVec::vec3(polylinePoint.x, 0, polylinePoint.z), "world");
            lPol.angle = atan2(pInLaser.x(), pInLaser.z());
            if( lPol.angle < min ) min = lPol.angle;
            if( lPol.angle > max ) max = lPol.angle;
        }
// 		printf("MIN %f MAX %f\n", min, max);
        //Recorremos todas las muestras del laser
// 		auto laserSample = laserCombined[laserCombined.size()/2];
        for (auto &laserSample: laserCombined)
        {
            //Compruebo que la muestra del laser corta a la polilinea. Es decir si esta comprendida entre el maximo y el minimo de antes
// 			printf("LASER %f (%f)\n", laserSample.angle, laserSample.dist);
            if (laserSample.angle >= min and laserSample.angle <= max and fabs(max-min) < 3.14)
            {
                QVec lasercart = innerModel->laserTo("laser", "laser", laserSample.dist, laserSample.angle);
                //recta que une el 0,0 con el punto del laser
                QLine2D laserline(QVec::vec2(0,0), QVec::vec2(lasercart.x(), lasercart.z()));

                auto previousPoint = polyline[polyline.size()-1];
                QVec previousPointInLaser = innerModel->transform("laser", (QVec::vec3(previousPoint.x, 0, previousPoint.z)), "world");
                // For each polyline's point

                for (auto polylinePoint: polyline)
                {
                    QVec currentPointInLaser = innerModel->transform("laser", (QVec::vec3(polylinePoint.x, 0, polylinePoint.z)), "world");

                    QVec intersection = laserline.intersectionPoint(QLine2D(QVec::vec2(previousPointInLaser.x(),previousPointInLaser.z()),QVec::vec2(currentPointInLaser.x(),currentPointInLaser.z())));

                    //Una vez sacada la interseccion se comprueba que esta dentro del segmento. Para ello se calculan los angulos de los puntos actual y previo
                    float pAngle = atan2(previousPointInLaser.x(), previousPointInLaser.z());
                    float cAngle = atan2(currentPointInLaser.x(), currentPointInLaser.z());

                    const float m = std::min<float>(cAngle, pAngle);
                    const float M = std::max<float>(cAngle, pAngle);
// 					printf("angulo medida: %f   p:%f  c:%f\n", laserSample.angle, cAngle, pAngle);

                    if (laserSample.angle >= m and laserSample.angle <= M and fabs(M-m) < 3.14)
                    {
                        float distint = sqrt (pow(intersection.x(),2)+pow(intersection.y(),2));
                        if (distint<laserSample.dist) laserSample.dist= distint;
                    }

                    previousPointInLaser = currentPointInLaser;

                }
            }
        }
    }

    FILE *fd3 = fopen("salidaL.txt", "w");
    for (auto &laserSample: laserCombined)
    {
        QVec vv = innerModel->laserTo("world", "laser", laserSample.dist, laserSample.angle);
        fprintf(fd3, "%f %f\n", vv(0), vv(2));
    }
    fclose(fd3);


    return laserCombined;
}