//
// Created by robolab on 22/07/20.
//

#ifndef ACTIONEXECUTION_H
#define ACTIONEXECUTION_H
#include <genericworker.h>

class ActionExecution {

public:
    using retActions = std::tuple <bool,QPointF>;

    void initialize(AGMModel::SPtr worldModel_);
    void updateWordModel(AGMModel::SPtr worldModel_);
    void update(std::string action_,  ParameterMap params_);
    ActionExecution::retActions runActions();


private:
    AGMModel::SPtr worldModel;
    std::string action;
    ParameterMap params;


    ActionExecution::retActions action_ChangeRoom();
    QPolygonF getRoomPolyline(AGMModelSymbol::SPtr roomSymbol);


};

#endif //ACTIONEXECUTION_H
