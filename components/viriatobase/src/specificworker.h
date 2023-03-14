/*
 *    Copyright (C)2018 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
       \brief
       @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

#include "viriato.h"


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
    SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	//Ice Interfaces
    void DifferentialRobot_correctOdometer(int x, int z, float alpha);
    void DifferentialRobot_getBasePose(int &x, int &z, float &alpha);
    void DifferentialRobot_getBaseState(RoboCompGenericBase::TBaseState &state);
    void DifferentialRobot_resetOdometer();
    void DifferentialRobot_setOdometer(RoboCompGenericBase::TBaseState &state);
    void DifferentialRobot_setOdometerPose(int x, int z, float alpha);
    void DifferentialRobot_setSpeedBase(float adv, float rot);
    void DifferentialRobot_stopBase();
    void GenericBase_getBasePose(int &x, int &z, float &alpha);
    void GenericBase_getBaseState(RoboCompGenericBase::TBaseState &state);
    void OmniRobot_correctOdometer(int x, int z, float alpha);
    void OmniRobot_getBasePose(int &x, int &z, float &alpha);
    void OmniRobot_getBaseState(RoboCompGenericBase::TBaseState &state);
    void OmniRobot_resetOdometer();
    void OmniRobot_setOdometer(RoboCompGenericBase::TBaseState &state);
    void OmniRobot_setOdometerPose(int x, int z, float alpha);
    void OmniRobot_setSpeedBase(float advx, float advz, float rot);
    void OmniRobot_stopBase();

	
private:
	//Internal methods
	void correctOdometer(const int x, const int z, const float alpha);
	void getBasePose(int &x, int &z, float &alpha);
	void resetOdometer();
	void setOdometer(const RoboCompGenericBase::TBaseState &state);
	void getBaseState(RoboCompGenericBase::TBaseState &state);
	void setOdometerPose(const int x, const int z, const float alpha);
	void stopBase();
	void setSpeedBase(const float advx, const float advz, const float rot);
	void setSpeedBase(const float adv, const float rot);



	void setWheels();
	void computeOdometry();
	float R, l1, l2;
	QMat M_wheels_2_vels;
	QMat M_vels_2_wheels;
	// Odometry control
	QVec wheelVels, wheelsPos;
	InnerModel *innermodel;
	InnerModelTransform *backPose, *newPose;
	InnerModelTransform *corrBackPose, *corrNewPose;

public slots:
	void compute();
	void initialize(int period);

private:
	Viriato *viriato;
	RoboCompGenericBase::TBaseState *bState_a, *bState_b;    
	QMutex *bstate_swap;
	QMutex *inner_mutex;
	QMutex *wheels_swap;
	QVec *wheelVels_a, *wheelVels_b;
	ofstream myfile;
};

#endif



	
