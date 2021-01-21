//
// Created by robolab on 24/01/20.
//

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <CommonBehavior.h>
#include "genericworker.h"
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/range.hpp>
#include <tuple>

struct Edge_val{
	bool previous = false;
	bool current = false;
	void set (bool new_state)
	{
		previous = current;
		current = new_state;
	};
	bool rising_edge()
	{
		return (previous == false and current == true);
	};
	bool lowering_edge()
	{
		return (previous == true and current == false);
	};
};


class Controller {
public:
                            //   blocked, active, velx,velz,velrot
    using retUpdate = std::tuple <bool,bool,Edge_val, float,float,float>;

    void initialize(const std::shared_ptr<InnerModel> &innerModel_,
            std::shared_ptr<RoboCompCommonBehavior::ParameterList> params_)
    {
        qDebug()<<"Controller - "<< __FUNCTION__;

        innerModel = innerModel_;
        this->time = QTime::currentTime();
        this->delay = delay*1000;	//msecs

        try
        {
            MAX_ADV_SPEED = QString::fromStdString(params_->at("MaxZSpeed").value).toFloat();
            MAX_ROT_SPEED = QString::fromStdString(params_->at("MaxRotationSpeed").value).toFloat();
            MAX_SIDE_SPEED = QString::fromStdString(params_->at("MaxXSpeed").value).toFloat();
            MAX_LAG = std::stof(params_->at("MinControllerPeriod").value);
            ROBOT_RADIUS_MM =  QString::fromStdString(params_->at("RobotRadius").value).toFloat();

            qDebug()<< __FUNCTION__ << "CONTROLLER: Params from config:"  << MAX_ADV_SPEED << MAX_ROT_SPEED << MAX_SIDE_SPEED << MAX_LAG << ROBOT_RADIUS_MM;

        }
        catch (const std::out_of_range& oor)
        {   std::cerr << "CONTROLLER. Out of Range error reading parameters: " << oor.what() << '\n'; }
    }

    void updateInnerModel(const std::shared_ptr<InnerModel> &innerModel_)
    {
        qDebug()<<"Controller - "<< __FUNCTION__;

        innerModel = innerModel_;
    }

    retUpdate update(std::vector<QPointF> points, RoboCompLaser::TLaserData laserData, QPointF target, QVec robotPose)
    {
//        qDebug()<<"Controller - "<< __FUNCTION__;

        bool active = true;
        bool blocked = false;

        QPointF robot = QPointF(robotPose.x(),robotPose.z());
        QPointF robotNose = robot + QPointF(250*sin(robotPose.ry()),250*cos(robotPose.ry()));

        auto secondPointInPath = points[1];

        // Compute euclidean distance to target
        float euc_dist_to_target = QVector2D(robot - target).length();
//        qDebug()<<"robot pose "<< robot << "target" << target;
//        qDebug()<< "DISTANCE TO TARGET " << euc_dist_to_target << "NUM POINTS "<< points.size();

        if (points.size() < 3 and euc_dist_to_target < FINAL_DISTANCE_TO_TARGET)
        {
            qDebug()<< "·························";
            qDebug()<< "···· TARGET ACHIEVED ····";
            qDebug()<< "·························";
            qDebug()<< " ";

            advVelz = 0;
            rotVel = 0;

            active = false;

            return std::make_tuple(blocked, active, turning, advVelx, advVelz,rotVel);
        }
        QLineF nose(robot, robotNose);
		auto targetInRobot = innerModel->transform("robot", QVec::vec3(target.x(), 0, target.y()), "world");
		targetInRobot.print("targetInRobot");
		

		auto angle = atan2(targetInRobot.x(), targetInRobot.z());
		std::cout<<"target angle "<<angle<<std::endl;
		if(fabs(angle) > 0.7 )
		{
			rotVel = 0.3 * angle;
			rotVel = std::clamp(rotVel, (float)-0.5, (float)0.5);
			advVelz = 0;
			advVelx = 0;
			turning.set(true);
		}
		else
		{
			turning.set(false);
			
			// Proceed through path
			// Compute rotation speed. We use angle between robot's nose and line between first and sucessive points
			// as an estimation of curvature ahead

			std::vector<float> angles;
			auto min_angle = rewrapAngleRestricted(qDegreesToRadians(nose.angleTo(QLineF(robot, secondPointInPath))));
			rotVel = 0.6 * min_angle;
			rotVel = std::clamp(rotVel, (float)-0.5, (float)0.5);

			// Compute advance speed
			std::min(advVelz = MAX_ADV_SPEED * exponentialFunction(rotVel, 0.3, 0.4, 0), euc_dist_to_target);
		}
        // Compute bumper-away speed
        QVector2D total{0, 0};
        for (const auto &l : laserData)
        {
            float limit = (fabs(ROBOT_LENGTH / 2.f * sin(l.angle)) + fabs(ROBOT_LENGTH / 2.f * cos(l.angle))) + 200;
            float diff = limit - l.dist;
            if (diff >= 0)
                total = total + QVector2D(-diff * sin(l.angle), -diff * cos(l.angle));
        }

        bumperVel = total * KB;  // Parameter set in slidebar

        if (abs(bumperVel.x()) < MAX_SIDE_SPEED)
            advVelx = bumperVel.x();


        return std::make_tuple (blocked, active, turning, advVelx, advVelz,rotVel);

    }




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

    const float ROBOT_LENGTH = 500;
    const float FINAL_DISTANCE_TO_TARGET = 250; //mm
    float KB = 2.0;

    float advVelx = 0, advVelz = 0, rotVel = 0;
    QVector2D bumperVel;

	Edge_val turning;
	
    // compute max de gauss(value) where gauss(x)=y  y min
    float exponentialFunction(float value, float xValue, float yValue, float min)
    {
        if (yValue <= 0)
            return 1.f;
        float landa = -fabs(xValue) / log(yValue);
        float res = exp(-fabs(value) / landa);
        return std::max(res, min);
    }

    float rewrapAngleRestricted(const float angle)
    {
        if (angle > M_PI)
            return angle - M_PI * 2;
        else if (angle < -M_PI)
            return angle + M_PI * 2;
        else
            return angle;
    }

};

#endif //PROJECT_CONTROLLER_H
