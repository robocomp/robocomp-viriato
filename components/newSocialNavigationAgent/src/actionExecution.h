//
// Created by robolab on 22/07/20.
//

#ifndef ACTIONEXECUTION_H
#define ACTIONEXECUTION_H
#include <genericworker.h>
#include <localPerson.h>

class ActionExecution {

public:
    string prevRoomTarget;


    using retActions = std::tuple <bool,QPointF>;

    void initialize(AGMModel::SPtr worldModel_);
    void updateWordModel(AGMModel::SPtr worldModel_);
    void update(std::string action_,  RoboCompAGMCommonBehavior::ParameterMap params_);
    ActionExecution::retActions runActions(std::string action_,  RoboCompAGMCommonBehavior::ParameterMap params_, bool testing=false);


private:
    AGMModel::SPtr worldModel;
    std::string action;
    RoboCompAGMCommonBehavior::ParameterMap params;
    bool newActionReceived = false;
    uint32_t previousPerson = -1;
    uint32_t previousPerson_group = -1;

    ActionExecution::retActions action_ChangeRoom(RoboCompAGMCommonBehavior::ParameterMap params_, bool testing=false);
    ActionExecution::retActions action_GoToPerson(RoboCompAGMCommonBehavior::ParameterMap params_);
    ActionExecution::retActions action_GoToGroupOfPeople(RoboCompAGMCommonBehavior::ParameterMap params_);

    QPolygonF getRoomPolyline(AGMModelSymbol::SPtr roomSymbol);
    QPointF getPointInSocialSpace(AGMModelSymbol::SPtr personSymbol,AGMModelSymbol::SPtr robotSymbol);
    QPointF getRandomPointInRoom(QPolygonF room);
    AGMModelSymbol::SPtr getNearestPerson(vector<AGMModelSymbol::SPtr> totalPersons,AGMModelSymbol::SPtr robotSymbol);

};

#endif //ACTIONEXECUTION_H
