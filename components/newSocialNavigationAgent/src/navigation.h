//
// Created by robolab on 17/01/20.
//

#ifndef PROJECT_NAVIGATION_H
#define PROJECT_NAVIGATION_H



#include <innermodel/innermodel.h>
#include <math.h>

#include <Laser.h>
#include "collisions.h"
#include <QPolygonF>
#include <QPointF>
#include <cppitertools/sliding_window.hpp>

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
        void initialize(const std::shared_ptr<InnerModel> &innerModel_, std::shared_ptr< RoboCompCommonBehavior::ParameterList > configparams_){};
        void update(const RoboCompLaser::TLaserData &laserData){
    qDebug()<<__PRETTY_FUNCTION__;
};
        //void updatePolylines(SNGPersonSeq persons_, SNGPolylineSeq intimate,SNGPolylineSeq personal,SNGPolylineSeq social,SNGPolylineSeq object,SNGPolylineSeq objectsblocking){};
};


#endif //PROJECT_NAVIGATION_H
