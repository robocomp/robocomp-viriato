//
// Created by robolab on 24/01/20.
//

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <CommonBehavior.h>
#include "genericworker.h"

class Controller {
public:
    void initialize(const std::shared_ptr<InnerModel> &innerModel_,
            std::shared_ptr<RoboCompCommonBehavior::ParameterList> params);




private:
    std::shared_ptr<InnerModel> innerModel;
    QTime time;
    int delay;
    std::vector<float> baseOffsets;

    // Constants reassigned to the params values
    float MAX_ADV_SPEED;
    float MAX_ROT_SPEED;
    float MAX_SIDE_SPEED;
    float MAX_LAG; //ms
    float ROBOT_RADIUS_MM; //mm
    float ARRIVAL_TOLERANCE = 20.f;  //Default tolerance on arrival
};

#endif //PROJECT_CONTROLLER_H
