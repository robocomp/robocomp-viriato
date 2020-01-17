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
    std::shared_ptr<InnerModel> innerModel;
    std::shared_ptr<InnerViewer> viewer;
    std::shared_ptr<RoboCompCommonBehavior::ParameterList> configparams;

    std::vector<QPolygonF> polylines_intimate,polylines_personal,polylines_social,polylines_objects_total,polylines_objects_blocked;
    std::vector<QPolygonF> prev_polylines_intimate = {},  prev_polylines_personal = {}, prev_polylines_social = {}, prev_polylines_objects_total = {}, prev_polylines_objects_blocked = {};
    std::vector<QPolygonF> toSetFree, toResetCost, toAvoidLaser;

    std::string robotname = "robot";
    typedef struct { float dist; float angle;} LocalPointPol;

    const float ROBOT_LENGTH = 400;
    //const float BALL_MIN = ROBOT_LENGTH/2;
    const float BALL_SIZE = 400;
    const float BALL_MIN = BALL_SIZE / 2;
    float KE = 3.0;
    float KI = 120;
    float KB = 90;
    const float ROAD_STEP_SEPARATION = ROBOT_LENGTH * 0.9;
    const float ROBOT_MAX_ADVANCE_SPEED = 600;
    const float ROBOT_MAX_ROTATION_SPEED = 0.9;
    const float FORCE_DISTANCE_LIMIT = (ROBOT_LENGTH * 1.5); //mm
    const float ROBOT_STEP = (ROBOT_LENGTH * 0.1);
    const float DELTA_H = (ROBOT_LENGTH * 0.1);
    const float FINAL_DISTANCE_TO_TARGET = 100; //mm

    // Target
    struct Target : public std::mutex
    {
        QPointF p;
        std::atomic_bool active = false;
        std::atomic_bool blocked = true;
        std::atomic_bool new_target = false;
        QGraphicsRectItem *item;
    };
    Target current_target;

    // ElasticBand
    std::vector<QGraphicsEllipseItem *> points;
    QGraphicsEllipseItem *first, *last, *robot_nose, *laser_pose;
    QGraphicsPolygonItem *robot;
    QGraphicsRectItem *target;

    // Robot simuation
    timeval lastCommand_timeval;
    float advVelx = 0, advVelz = 0, rotVel = 0;
    QVector2D bumperVel;

    void initGrid();
    void resetGrid();
    void updateFreeSpaceMap();

    RoboCompLaser::TLaserData computeLaser(RoboCompLaser::TLaserData laserData_);
    // This function takes an angle in the range [-3*pi, 3*pi] and wraps it to the range [-pi, pi].
//    float rewrapAngleRestricted(const float angle);
//    float exponentialFunction(float value, float xValue, float yValue, float min);
//    float degreesToRadians(const float angle_);


};

#endif
