/*
 *    Copyright (C) 2016 by YOUR NAME HERE
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
#include "specificworker.h"
#include <chrono>
#include <iomanip>
#include <iostream>
/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    bState_a = new RoboCompGenericBase::TBaseState();
    bState_b = new RoboCompGenericBase::TBaseState();
    bstate_swap = new QMutex();
	wheelVels_a = new QVec(4);
	wheelVels_b = new QVec(4);
	inner_mutex = new QMutex();
	wheels_swap = new QMutex();
	
	
	myfile.open ("speed.txt");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	myfile.close();
	delete viriato;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	timer.start(0);

}


bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	QMutexLocker locker(mutex);

	printf("viriato: initializing communication with the robot (stale connection might indicate port misconfiguration)...");
	fflush(stdout);
	viriato = new Viriato(params["ViriatoBase.Port"].value);
	printf(". done!\n");
	
	wheelVels = QVec::vec4(0,0,0,0);

	/// InnerModel
	innermodel = new InnerModel();
	// raw odometry nodes
	backPose = innermodel->newTransform("backPose", "static", innermodel->getRoot(), 0,0,0, 0,0,0, 0);
	innermodel->getRoot()->addChild(backPose);
	newPose = innermodel->newTransform("newPose", "static", backPose, 0,0,0, 0,0,0, 0);
	backPose->addChild(newPose);

	// corrected odometry nodes
	corrBackPose = innermodel->newTransform("corrBackPose", "static", innermodel->getRoot(), 0,0,0, 0,0,0, 0);
	innermodel->getRoot()->addChild(corrBackPose);
	corrNewPose = innermodel->newTransform("corrNewPose", "static", corrBackPose, 0,0,0, 0,0,0, 0);
	corrBackPose->addChild(corrNewPose);
	
	printf("viriato: successfully initialized\n");

	// YEP: OMNI-DIRECTIONAL ROBOTS: https://link.springer.com/chapter/10.1007/978-3-540-70534-5_9
	R  = QString::fromStdString(params["ViriatoBase.WheelRadius"].value).toFloat(); 
	l1 = QString::fromStdString(params["ViriatoBase.DistAxes"].value   ).toFloat();
	l2 = QString::fromStdString(params["ViriatoBase.AxesLength"].value ).toFloat();
	printf("l1: %f\n", l1);
	printf("l2: %f\n", l2);
	printf("r:  %f\n", R);

	// inverse kinematics matrix
	const float ill = 0.5 / (l1 + l2);
	M_wheels_2_vels = QMat(3, 4);
	M_wheels_2_vels(0,0) = +1/4.;
	M_wheels_2_vels(0,1) = +1/4.;
	M_wheels_2_vels(0,2) = +1/4.;
	M_wheels_2_vels(0,3) = +1/4.;
	M_wheels_2_vels(1,0) = +1/4.;
	M_wheels_2_vels(1,1) = -1/4.;
	M_wheels_2_vels(1,2) = -1/4.;
	M_wheels_2_vels(1,3) = +1/4.;
	M_wheels_2_vels(2,0) = +ill;
	M_wheels_2_vels(2,1) = -ill;
	M_wheels_2_vels(2,2) = +ill;
	M_wheels_2_vels(2,3) = -ill;
	M_wheels_2_vels = M_wheels_2_vels.operator*(R); // R instead of 2*pi*R because we use rads/s instead of rev/s
	M_wheels_2_vels.print("M_wheels_2_vels");

	// forward kinematics matrix
	const float ll = 0.5*(l1 + l2);
	M_vels_2_wheels = QMat(4,3);
	M_vels_2_wheels(0,0) = +1.;
	M_vels_2_wheels(1,0) = +1.;
	M_vels_2_wheels(2,0) = +1.;
	M_vels_2_wheels(3,0) = +1.;
	M_vels_2_wheels(0,1) = -1.;
	M_vels_2_wheels(1,1) = +1.;
	M_vels_2_wheels(2,1) = +1.;
	M_vels_2_wheels(3,1) = -1.;
	M_vels_2_wheels(0,2) = +ll; // In contrast with the paper this code is based on, the
	M_vels_2_wheels(1,2) = -ll; // third column of the matrix is inverted because we use
	M_vels_2_wheels(2,2) = +ll; // the left-hand rule for angles.
	M_vels_2_wheels(3,2) = -ll;
	M_vels_2_wheels = M_vels_2_wheels.operator*(1./(R)); // 1/R instead of 1/(2*pi*R) because we use rads/s instead of rev/s
	M_vels_2_wheels.print("M_vels_2_wheels");

	return true;
}

std::string getTimestamp()
{
  // get a precise timestamp as a string
  const auto now = std::chrono::system_clock::now();
  const auto nowAsTimeT = std::chrono::system_clock::to_time_t(now);
  const auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()) % 1000;
  std::stringstream nowSs;
  nowSs
      << std::put_time(std::localtime(&nowAsTimeT), "%a %b %d %Y %T")
      << '.' << std::setfill('0') << std::setw(3) << nowMs.count();
  return nowSs.str();
}

void SpecificWorker::compute()
{
    //	printf("compute\n");
	setWheels();
	computeOdometry();
	const auto now = std::chrono::system_clock::now();
	const auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
	myfile <<getTimestamp()<<" " << bState_a->x<<" "<<bState_a->z<<" "<<bState_a->alpha<<"\n";
	usleep(10000);
}


void SpecificWorker::computeOdometry()
{
	static QVec previousWheelsPos = wheelsPos;
	QVec deltaWheels = (wheelsPos-previousWheelsPos).operator*(0.912*0.912/M_PIl);
	QVec deltaPos = M_wheels_2_vels * deltaWheels;

	deltaPos(1) *= -1;
	previousWheelsPos = wheelsPos;

	QVec newP;
	// Raw odometry
	inner_mutex->lock();
		innermodel->updateTransformValues("newPose",     deltaPos(1), 0, deltaPos(0),       0,       deltaPos(2), 0);
		newP = innermodel->transform("root", "newPose");
		innermodel->updateTransformValues("backPose",        newP(0), 0,     newP(2),       0, bState_b->alpha+deltaPos(2), 0);
		innermodel->updateTransformValues("newPose",               0, 0,           0,       0,                 0, 0);
	
		// Corrected odometry
		QVec coorP;
		innermodel->updateTransformValues("corrNewPose",    deltaPos(1), 0, deltaPos(0),    0,       deltaPos(2), 0);
		coorP = innermodel->transform("root", "corrNewPose");
		innermodel->updateTransformValues("corrBackPose",       newP(0), 0,     newP(2),    0, bState_b->correctedAlpha+deltaPos(2), 0);

		innermodel->updateTransformValues("corrNewPose",              0, 0,           0,    0,                 0, 0);
	inner_mutex->unlock();
	//update bState
	bState_b->x = newP(0);
	bState_b->z = newP(2);
	bState_b->alpha += deltaPos(2);
	bState_b->correctedX = newP(0);
	bState_b->correctedZ = newP(2);
	bState_b->correctedAlpha += deltaPos(2);
	bstate_swap->lock();
        RoboCompGenericBase::TBaseState *temp = bState_a;
        bState_a = bState_b;
        bState_b = temp;
	bstate_swap->unlock();
}

void SpecificWorker::correctOdometer(const int x, const int z, const float alpha)
{
	QMutexLocker locker(inner_mutex);
	innermodel->updateTransformValues("corrBackPose",x, 0,z,0,alpha,0);
}

void SpecificWorker::getBasePose(int &x, int &z, float &alpha)
{
	x     = bState_a->x;
	z     = bState_a->z;
	alpha = bState_a->alpha;
}

void SpecificWorker::resetOdometer()
{
	setOdometerPose(0,0,0);
	correctOdometer(0,0,0);
}

void SpecificWorker::setOdometer(const RoboCompGenericBase::TBaseState &state)
{
	setOdometerPose(state.x,          state.z,          state.alpha);
	correctOdometer(state.correctedX, state.correctedZ, state.correctedAlpha);
}

void SpecificWorker::getBaseState(RoboCompGenericBase::TBaseState &state)
{
//	printf("geRoboCompGenericBase::TBaseState\n");
	state = *bState_a;
}

void SpecificWorker::setOdometerPose(const int x, const int z, const float alpha)
{
	QMutexLocker locker(inner_mutex);
	innermodel->updateTransformValues("backPose",x, 0,z,0,alpha,0);
}

void SpecificWorker::stopBase()
{
	setSpeedBase(0,0,0);
}

void SpecificWorker::setSpeedBase(const float advx, const float advz, const float rotv)
{
	std::cout<<"set speed "<<advx<<" "<<advz<<" "<<rotv<<std::endl;
	const QVec v = QVec::vec3(advz, advx, rotv);
	const QVec wheels = M_vels_2_wheels * v;
//	wheels.print("wheels");
	for (int i=0;i<4;i++)
		wheelVels_a->operator[](i) = wheels[i];
	
	wheels_swap->lock();
		QVec *temp = wheelVels_a;
		wheelVels_a = wheelVels_b;
		wheelVels_b = temp;
	wheels_swap->unlock();

}

void SpecificWorker::setWheels()
{
	double rps2rpm = 60./(2.*M_PI);
	double encoderFactor = 71.0/8.0;
	QVec f = wheelVels_b->operator*(rps2rpm * encoderFactor);
	f = f.operator*(1./0.912);
	
	float r1,r2,r3,r4;
	viriato->setVelocity(    -f(0),   +f(1),   -f(2),   +f(3), r1, r2, r3, r4);
	wheelsPos = QVec::vec4(  -r1,     +r2,     -r3,     +r4).operator/(rps2rpm * encoderFactor);
// 	wheelsPos.print("wheelsPos");
}

void SpecificWorker::setSpeedBase(const float adv, const float rot)
{
	setSpeedBase(0, adv, rot);

}
//*************************
//ICE INTERFACES
//*************************
void SpecificWorker::DifferentialRobot_correctOdometer(const int x, const int z, const float alpha)
{
	correctOdometer(x,z,alpha);
}

void SpecificWorker::DifferentialRobot_getBasePose(int &x, int &z, float &alpha)
{
	getBasePose(x,z,alpha);
}

void SpecificWorker::DifferentialRobot_resetOdometer()
{
	resetOdometer();
}

void SpecificWorker::DifferentialRobot_setOdometer(RoboCompGenericBase::TBaseState &state)
{
	setOdometer(state);
}

void SpecificWorker::DifferentialRobot_getBaseState(RoboCompGenericBase::TBaseState &state)
{
	getBaseState(state);
}

void SpecificWorker::DifferentialRobot_setOdometerPose(const int x, const int z, const float alpha)
{
	setOdometerPose(x,z,alpha);
}

void SpecificWorker::DifferentialRobot_stopBase()
{
	stopBase();
}

void SpecificWorker::DifferentialRobot_setSpeedBase(const float adv, const float rot)
{
	setSpeedBase(adv,rot);
}

void SpecificWorker::GenericBase_getBaseState(RoboCompGenericBase::TBaseState &state)
{
	getBaseState(state);
}

void SpecificWorker::GenericBase_getBasePose(int &x, int &z, float &alpha)
{
	getBasePose(x,z,alpha);
}

void SpecificWorker::OmniRobot_correctOdometer(const int x, const int z, const float alpha)
{
	correctOdometer(x,z,alpha);
}

void SpecificWorker::OmniRobot_getBasePose(int &x, int &z, float &alpha)
{
	getBasePose(x,z,alpha);
}

void SpecificWorker::OmniRobot_resetOdometer()
{
	resetOdometer();
}

void SpecificWorker::OmniRobot_setOdometer(RoboCompGenericBase::TBaseState &state)
{
	setOdometer(state);
}

void SpecificWorker::OmniRobot_getBaseState(RoboCompGenericBase::TBaseState &state)
{
	getBaseState(state);
}

void SpecificWorker::OmniRobot_setOdometerPose(const int x, const int z, const float alpha)
{
	setOdometerPose(x,z,alpha);
}

void SpecificWorker::OmniRobot_stopBase()
{
	stopBase();
}

void SpecificWorker::OmniRobot_setSpeedBase(const float advx, const float advz, const float rot)
{
	setSpeedBase(advx,advz,rot);
}
