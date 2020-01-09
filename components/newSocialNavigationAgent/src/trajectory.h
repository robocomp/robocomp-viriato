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

#define TILE_SIZE_ 250

// Map
struct TCell
{
    std::uint32_t id;
    bool free;
    bool visited;
    QGraphicsEllipseItem* g_item;
    float cost;

    // method to save the value
    void save(std::ostream &os) const {	os << free << " " << visited; };
    void read(std::istream &is) {	is >> free >> visited ;};
};

class Trajectory {

public:
    void initialize(const std::shared_ptr<InnerModel> &innerModel_,
            const std::shared_ptr<InnerViewer> &viewer_,
            const std::shared_ptr< RoboCompCommonBehavior::ParameterList > &configparams_,
            LaserPrx laser_prx,
            OmniRobotPrx omnirobot_proxy);
    void update();
    void updatePolylines();

    bool checkRobotValidStateAtTargetFast(const QVec &targetPos, const QVec &targetRot) const;

private:
    // Grid
    using TDim = Grid<TCell>::Dimensions;
    TDim dimensions;
    Grid<TCell> grid;

    std::shared_ptr<InnerModel> innerModel;
    std::shared_ptr<InnerViewer> viewer;
    std::shared_ptr<RoboCompCommonBehavior::ParameterList> configparams;


    void init_grid();


};

#endif
