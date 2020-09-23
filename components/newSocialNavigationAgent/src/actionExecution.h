//
// Created by robolab on 22/07/20.
//

#ifndef ACTIONEXECUTION_H
#define ACTIONEXECUTION_H
#include <genericworker.h>
#include <localPerson.h>

class ActionExecution {

public:
    using retActions = std::tuple <bool,QPointF>;

    void initialize(AGMModel::SPtr worldModel_);
    void updateWordModel(AGMModel::SPtr worldModel_);
    void update(std::string action_,  ParameterMap params_);
    ActionExecution::retActions runActions(std::string action_,  ParameterMap params_);


private:
    AGMModel::SPtr worldModel;
    std::string action;
    ParameterMap params;
    bool newActionReceived = false;

    ActionExecution::retActions action_ChangeRoom(ParameterMap params_);
    ActionExecution::retActions action_GoToPerson(ParameterMap params_);
    QPolygonF getRoomPolyline(AGMModelSymbol::SPtr roomSymbol);
    QPointF getPointInSocialSpace(AGMModelSymbol::SPtr personSymbol,AGMModelSymbol::SPtr robotSymbol);
    QPointF getRandomPointInRoom(QPolygonF room);

};

#endif //ACTIONEXECUTION_H
