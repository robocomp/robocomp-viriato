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

// Map
struct TCell
{
    std::uint32_t id;
    bool free;
    bool visited;
    float cost;

    // method to save the value
    void save(std::ostream &os) const {	os << free << " " << visited; };
    void read(std::istream &is) {	is >> free >> visited ;};
};

class Trajectory {

public:

    void initialize(const std::shared_ptr<InnerModel> &innerModel_,
            const std::shared_ptr<InnerViewer> &viewer_,
            const std::shared_ptr< RoboCompCommonBehavior::ParameterList > &configparams_);
    void update(const RoboCompLaser::TLaserData &laserData);
    void updatePolylines(SNGPersonSeq persons_, SNGPolylineSeq intimate,SNGPolylineSeq personal,SNGPolylineSeq social,SNGPolylineSeq object,SNGPolylineSeq objectsblocking);



private:
    // Grid
    using TDim = Grid<TCell>::Dimensions;
    TDim dimensions;

    Grid<TCell> grid;

    std::shared_ptr<Collisions> collisions;

    std::shared_ptr<InnerModel> innerModel;
    std::shared_ptr<InnerViewer> viewer;
    std::shared_ptr<RoboCompCommonBehavior::ParameterList> configparams;

    std::vector<QPolygonF> polylines_intimate,polylines_personal,polylines_social,polylines_objects_total,polylines_objects_blocked;
    std::vector<QPolygonF> prev_polylines_intimate = {},  prev_polylines_personal = {}, prev_polylines_social = {}, prev_polylines_objects_total = {}, prev_polylines_objects_blocked = {};

    std::vector<QPolygonF> toSetFree, toResetCost, toAvoidLaser;

    std::string robotname = "robot";
    typedef struct { float dist; float angle;} LocalPointPol;

    LaserPrx laser_proxy;

    void initGrid();
    void resetGrid();
    void updateFreeSpaceMap();

    RoboCompLaser::TLaserData  computeLaser(RoboCompLaser::TLaserData laserData);



};

#endif
