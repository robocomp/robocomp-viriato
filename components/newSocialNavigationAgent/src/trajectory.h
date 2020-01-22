//
// Created by robolab on 9/01/20.
//

#ifndef TRAJECTORY_H
#define TRAJECTORY_H


#include <innermodel/innermodel.h>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsRectItem>
#include <QGraphicsPolygonItem>
#include <math.h>
#include "grid.h"
#include "CommonBehavior.h"
#include <AGMCommonBehavior.h>
#include <genericworker.h>
#include "innerviewer.h"
#include "collisions.h"
#include <QPolygonF>
#include <QPointF>
#include <cppitertools/sliding_window.hpp>

#define TILE_SIZE_ 250



class Trajectory {

public:

    void initialize(const std::shared_ptr<InnerModel> &innerModel_,
            const std::shared_ptr<InnerViewer> &viewer_,
            const std::shared_ptr< RoboCompCommonBehavior::ParameterList > &configparams_);
    void update(const RoboCompLaser::TLaserData &laserData);
    void updatePolylines(SNGPersonSeq persons_, SNGPolylineSeq intimate,SNGPolylineSeq personal,SNGPolylineSeq social,SNGPolylineSeq object,SNGPolylineSeq objectsblocking);

public slots:
//    void cleanPath();
//    void updateRobot();
//    void controller();


private:
    // Grid
    using TDim = Grid<>::Dimensions;
    TDim dimensions;
    Grid<> grid;

    RoboCompLaser::TLaserData laserData;

    std::shared_ptr<Collisions> collisions;
    std::shared_ptr<InnerViewer> viewer;
    std::shared_ptr<InnerModel> innerModel;
    std::shared_ptr<RoboCompCommonBehavior::ParameterList> configparams;

    std::vector<QPolygonF> polylines_intimate,polylines_personal,polylines_social,polylines_objects_total,polylines_objects_blocked;
    std::vector<QPolygonF> prev_polylines_intimate = {},  prev_polylines_personal = {}, prev_polylines_social = {}, prev_polylines_objects_total = {}, prev_polylines_objects_blocked = {};
    std::vector<QPolygonF> toSetFree, toResetCost, toAvoidLaser;

    std::string robotname = "robot";
    typedef struct { float dist; float angle;} LocalPointPol;


    void initGrid();
    void resetGrid();
    void updateFreeSpaceMap();

    RoboCompLaser::TLaserData computeLaser(RoboCompLaser::TLaserData laserData_);



};

#endif
