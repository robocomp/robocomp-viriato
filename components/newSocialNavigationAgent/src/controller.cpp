//
// Created by robolab on 24/01/20.
//

#include "controller.h"

void Controller::initialize(const std::shared_ptr<InnerModel>& innerModel_,
        std::shared_ptr<RoboCompCommonBehavior::ParameterList> configparams)
{
    qDebug()<< __FUNCTION__;

    innerModel = innerModel_;
    this->time = QTime::currentTime();
    this->delay = delay*1000;	//msecs

    try
    {
        MAX_ADV_SPEED = QString::fromStdString(configparams->at("MaxZSpeed").value).toFloat();
        MAX_ROT_SPEED = QString::fromStdString(configparams->at("MaxRotationSpeed").value).toFloat();
        MAX_SIDE_SPEED = QString::fromStdString(configparams->at("MaxXSpeed").value).toFloat();
        MAX_LAG = std::stof(configparams->at("MinControllerPeriod").value);
        ROBOT_RADIUS_MM =  QString::fromStdString(configparams->at("RobotRadius").value).toFloat();

        qDebug()<< __FUNCTION__ << "CONTROLLER: Params from config:"  << MAX_ADV_SPEED << MAX_ROT_SPEED << MAX_SIDE_SPEED << MAX_LAG << ROBOT_RADIUS_MM;

    }
    catch (const std::out_of_range& oor)
    {   std::cerr << "CONTROLLER. Out of Range error reading parameters: " << oor.what() << '\n'; }


}