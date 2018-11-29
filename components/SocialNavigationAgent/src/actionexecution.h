/*
 * Copyright 2017 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#ifndef ACTIONEXECUTION_H
#define ACTIONEXECUTION_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <boost/format.hpp>
#include <QObject>



#define THRESHOLD 40


class ActionExecution :
public QObject

{

public:
	 ActionExecution();
	~ActionExecution();
	 void Update(std::string a, ParameterMap prs);
	
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	RoboCompCommonBehavior::ParameterList getWorkerParams();	
	
	QMutex *mx;
	
	LoggerPrx logger_proxy;
	TrajectoryRobot2DPrx trajectoryrobot2d_proxy;
	OmniRobotPrx omnirobot_proxy;
	AGMExecutivePrx agmexecutive_proxy;

	void  structuralChange(const RoboCompAGMWorldModel::World & modification);
	
public slots:
 	void compute();
	void readTrajState();

private:
	
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	bool active;
	void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel,std::string m);
	void includeMovementInRobotSymbol(AGMModelSymbol::SPtr robot);


	void go(float x, float z, float alpha=0, bool rot=false, float xRef=0, float zRef=0, float threshold=200);
	void stop();

	void actionExecution();
	int32_t getIdentifierOfRobotsLocation(AGMModel::SPtr &worldModel);
	void setIdentifierOfRobotsLocation(AGMModel::SPtr &worldModel, int32_t identifier);

private:
	std::string action;
	ParameterMap params;
	AGMModel::SPtr worldModel;
    std::shared_ptr<InnerModel> innerModel;
	bool haveTarget;
	QTimer trajReader;
	
	RoboCompCommonBehavior::ParameterList worker_params;
	QMutex *worker_params_mutex;	

	RoboCompTrajectoryRobot2D::NavState planningState;


	// Target info
	RoboCompTrajectoryRobot2D::TargetPose currentTarget;
	
	void manageReachedPose();
	float distanceToNode(std::string reference_name, AGMModel::SPtr model, AGMModelSymbol::SPtr object);
private:
	void action_WaitingToAchieve();
	void action_Stop(bool newAction = true);
	void action_ReachPose(bool newAction = true);
	void action_ChangeRoom(bool newAction = true);
	void action_FindObjectVisuallyInTable(bool newAction = true);
	void action_SetObjectReach(bool newAction = true);
//	void action_GraspObject(bool newAction = true);
	void action_DetectPerson (bool newAction = true);
	void action_HandObject(bool newAction = true);
	void action_NoAction(bool newAction = true);
	void action_HandObject_Offer(bool newAction = true);
	void action_HandObject_leave(bool newAction = true);
	
};


class TimedList
{
	class TimedDatum
	{
	public:
		TimedDatum(float d)
		{
			datum = d;
			datum_time = QTime::currentTime();
		}
		float datum;
		QTime datum_time;
	};

public:
	TimedList(float msecs)
	{
		maxMSec = msecs;
	}
	void add(float datum)
	{
		data.push_back(TimedDatum(datum));
	}
	float getSum()
	{
		while (data.size()>0)
		{
			if (data[0].datum_time.elapsed() > maxMSec)
				data.pop_front();
			else
				break;
		}
		float acc = 0.;
		for (int i=0; i<data.size(); i++)
			acc += data[i].datum;
		return acc;
	}
private:
	float maxMSec;
	QList<TimedDatum> data;
};


#endif  // ACTIONEXECUTION_H
