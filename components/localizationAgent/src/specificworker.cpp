/*
 *    Copyright (C)2019 by YOUR NAME HERE
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



/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	active = false;
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModel = new InnerModel();
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	emit t_pop_data_to_finalize();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		std::string innermodel_path = par.value;
		innerModel = new InnerModel(innermodel_path);
	}
	catch(std::exception e) { qFatal("Error reading config params"); }

	customMachine.start();
	// retrieve model
	try
	{
		RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
		AGMExecutiveTopic_structuralChange(w);
	}
	catch(...)
	{
		printf("The executive is probably not running, waiting for first AGM model publication...");
	}
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
}

void SpecificWorker::compute()
{
}


// compute difference between actual and last value to determine if it should be sent
bool SpecificWorker::enoughDifferenceBState(const RoboCompGenericBase::TBaseState &lastState, const RoboCompGenericBase::TBaseState &newState)
{
	if (fabs(newState.correctedX - lastState.correctedX) > 5 or fabs(newState.correctedZ - lastState.correctedZ) > 5 or fabs(newState.correctedAlpha - lastState.correctedAlpha) > 0.02)
	{
		return true;
	}
	return false;
}
// compute difference between actual and last value to determine if it should be sent
bool SpecificWorker::enoughDifferencePose(const RoboCompFullPoseEstimation::FullPose &lastPose, const RoboCompFullPoseEstimation::FullPose &newPose)
{
	if (fabs(newPose.x - lastPose.x) > 5 or fabs(newPose.z - lastPose.z) > 5 or fabs(newPose.ry - lastPose.ry) > 0.02)
	{
		return true;
	}
	return false;
}


// send corrected position
void SpecificWorker::setCorrectedPosition(const RoboCompGenericBase::TBaseState &bState)
{
	std::cout<<"correct odometer position"<<std::endl;
	try
	{
		omnirobot_proxy->correctOdometer(bState.correctedX, bState.correctedZ, bState.correctedAlpha);
	}
	catch(Ice::Exception &ex)
	{
		std::cout<<ex.what()<<std::endl;
	}
}

//update robot position in model
bool SpecificWorker::publishPoseInModel(const RoboCompFullPoseEstimation::FullPose &pose)
{
	// Get robot's symbol and its identifier
	int32_t robotId=-1;
	robotId = worldModel->getIdentifierByType("robot");
	if (robotId < 0)
	{
		printf("Robot symbol not found, Waiting for the executive...\n");
		usleep(1000000);
		return false;
	}
	AGMModelSymbol::SPtr robot = worldModel->getSymbol(robotId);

	// Get current roomId
	int roomId=-1;
	for (auto edge = robot->edgesBegin(worldModel); edge != robot->edgesEnd(worldModel); edge++)
	{
		const std::pair<int32_t, int32_t> symbolPair = edge->getSymbolPair();

		if (edge->getLabel() == "RT")
		{
			const string secondType = worldModel->getSymbol(symbolPair.first)->symbolType;
			if (symbolPair.second == robotId and secondType == "room")
			{
				roomId = symbolPair.first;
				break;
			}
		}
	}
	if (roomId < 0)
	{
		printf("roomId not found, Waiting for Insert innerModel...\n");
		usleep(1000000);
		return false;
	}

	// Query which should actually be the current room based on the corrected odometry odometry
	int32_t robotIsActuallyInRoom;
	float schmittTriggerLikeThreshold = 80;
	if (pose.z < -schmittTriggerLikeThreshold)
	{
		robotIsActuallyInRoom = 5;
	}
	else if (pose.z > schmittTriggerLikeThreshold)
	{
		robotIsActuallyInRoom = 3;
	}
	else
	{
		robotIsActuallyInRoom = roomId;
	}
	if (roomId != robotIsActuallyInRoom)
	{
		try
		{
			AGMModel::SPtr newModel(new AGMModel(worldModel));

			// Modify IN edge
			newModel->removeEdgeByIdentifiers(robotId, roomId, "in");
			newModel->addEdgeByIdentifiers(robotId, robotIsActuallyInRoom, "in");

			// Move "in" edges for every object IN the robot
			AGMModelSymbol::SPtr newRobot = newModel->getSymbol(robotId);
			for (auto edgeIn = robot->edgesBegin(newModel); edgeIn != robot->edgesEnd(newModel); edgeIn++)
			{
				const std::pair<int32_t, int32_t> symbolPair = edgeIn->getSymbolPair();
				if ( edgeIn->linking=="in" and symbolPair.second==robot->identifier )
				{
					AGMModelSymbol::SPtr objectToMove = newModel->getSymbol(symbolPair.first);
					newModel->removeEdgeByIdentifiers(objectToMove->identifier, roomId, "in");
					newModel->addEdgeByIdentifiers(objectToMove->identifier, robotIsActuallyInRoom, "in");
				}
			}
			// Modify RT edge
			AGMModelEdge edgeRT = newModel->getEdgeByIdentifiers(roomId, robotId, "RT");
			newModel->removeEdgeByIdentifiers(roomId, robotId, "RT");
			try
			{
				edgeRT->setAttribute("tx", float2str(pose.x));
				edgeRT->setAttribute("tz", float2str(pose.z));
				edgeRT->setAttribute("ry", float2str(pose.ry));

				newModel->addEdgeByIdentifiers(robotIsActuallyInRoom, robotId, "RT", edgeRT->attributes);
				AGMMisc::publishModification(newModel, agmexecutive_proxy, "localizationAgent");
				lastPublish = pose;
				rDebug2(("navigationAgent moved robot from room"));
			}
			catch (...)
			{
				printf("Can't update odometry in RT, edge exists but we encountered other problem!!\n");
				return false;
			}
		}
		catch (...)
		{
			printf("Can't update room... do edges exist? !!!\n");
			return false;
		}
	}
	else
	{
		try
		{
			AGMModelEdge edge  = worldModel->getEdgeByIdentifiers(roomId, robotId, "RT");
			try
			{
				//Publish update edge
				printf("\nUpdate model...\n");
				edge->setAttribute("tx", float2str(pose.x));
				edge->setAttribute("tz", float2str(pose.z));
				edge->setAttribute("ry", float2str(pose.ry));
				AGMMisc::publishEdgeUpdate(edge, agmexecutive_proxy);
				lastPublish = pose;
			}
			catch (...)
			{
				printf("Can't update odometry in RT, edge exists but we encountered other problem!!\n");
				return false;
			}
		}
		catch (...)
		{
			printf("Can't update odometry in RT, edge does not exist? !!!\n");
			return false;
		}
	}
	return true;
}

//STATE MACHINE
void SpecificWorker::sm_publish()
{
	std::cout<<"Entered state PUBLISH"<<std::endl;
	publishPoseInModel(poseRead);
	emit t_publish_to_pop_data();
}

void SpecificWorker::sm_pop_data()
{
//	std::cout<<"Entered state pop_data"<<std::endl;
	if(not db.isEmpty())
	{
		poseRead = db.get();
		if(poseRead.source == "realsense" and UWB_DATA >= 10)
		{
			emit t_pop_data_to_read_rs();
		}
		else if(UWB_DATA <= 10)
		{
			emit t_pop_data_to_read_uwb();
		}
	}
	else
	{
		std::cout<<"No data"<<std::endl;
		QTimer::singleShot(200,this, SIGNAL(t_pop_data_to_pop_data()));
	}
}

void SpecificWorker::sm_read_uwb()
{
	std::cout<<"Entered state read_uwb"<<std::endl;
	initial_offset.x += poseRead.x; 
	initial_offset.z += poseRead.z;
	initial_offset.ry += poseRead.ry;
	UWB_DATA ++;
	if (UWB_DATA == 10)
	{
		try
		{
			qDebug()<<"SET INITIAL POSE";
			fullposeestimation_proxy->setInitialPose(initial_offset.x/10.f, 0.f, initial_offset.z/10.f, 0.f, initial_offset.ry/10.f, 0.f);
		}catch(...)
		{
			qDebug()<<"Error setting initialpose to RealSense";
		}
	}
	emit t_read_uwb_to_pop_data();
}

void SpecificWorker::sm_read_rs()
{
	std::cout<<"Entered state read_rs"<<std::endl;
	emit t_read_rs_to_compute_pose();
}

void SpecificWorker::sm_read_april()
{
	std::cout<<"Entered state read_april"<<std::endl;
}

void SpecificWorker::sm_compute_pose()
{
	std::cout<<"Entered state compute_pose"<<std::endl;
	if(enoughDifferencePose(lastPublish, poseRead))
	{
		emit t_compute_pose_to_publish();
	}
 	else
	{
		emit t_compute_pose_to_pop_data();
	}
}

void SpecificWorker::sm_initialize()
{
	std::cout<<"Entered initial state initialize"<<std::endl;
	initial_offset.x = 0.f;
	initial_offset.y = 0.f;
	initial_offset.z = 0.f;
	initial_offset.rx = 0.f;
	initial_offset.ry = 0.f;
	initial_offset.rz = 0.f;
	emit this->t_initialize_to_pop_data();
}

void SpecificWorker::sm_finalize()
{
	std::cout<<"Entered final state finalize"<<std::endl;
}



/// AGM 

bool SpecificWorker::AGMCommonBehavior_reloadConfigAgent()
{
//implementCODE
	return true;
}

bool SpecificWorker::AGMCommonBehavior_activateAgent(const ParameterMap &prs)
{
//implementCODE
	bool activated = false;
	if (setParametersAndPossibleActivation(prs, activated))
	{
		if (not activated)
		{
			return activate(p);
		}
	}
	else
	{
		return false;
	}
	return true;
}

bool SpecificWorker::AGMCommonBehavior_setAgentParameters(const ParameterMap &prs)
{
//implementCODE
	bool activated = false;
	return setParametersAndPossibleActivation(prs, activated);
}

ParameterMap SpecificWorker::AGMCommonBehavior_getAgentParameters()
{
//implementCODE
	return params;
}

void SpecificWorker::AGMCommonBehavior_killAgent()
{
//implementCODE

}

int SpecificWorker::AGMCommonBehavior_uptimeAgent()
{
//implementCODE
	return 0;
}

bool SpecificWorker::AGMCommonBehavior_deactivateAgent()
{
//implementCODE
	return deactivate();
}

StateStruct SpecificWorker::AGMCommonBehavior_getAgentState()
{
//implementCODE
	StateStruct s;
	if (isActive())
	{
		s.state = RoboCompAGMCommonBehavior::StateEnum::Running;
	}
	else
	{
		s.state = RoboCompAGMCommonBehavior::StateEnum::Stopped;
	}
	s.info = p.action.name;
	return s;
}

void SpecificWorker::AGMExecutiveTopic_structuralChange(const RoboCompAGMWorldModel::World &w)
{
//subscribesToCODE
	QMutexLocker lockIM(mutex);
 	AGMModelConverter::fromIceToInternal(w, worldModel);
 
	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
}

void SpecificWorker::AGMExecutiveTopic_edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{
//subscribesToCODE
	QMutexLocker lockIM(mutex);
	for (auto modification : modifications)
	{
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
		AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel);
	}

}

void SpecificWorker::AGMExecutiveTopic_edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
//subscribesToCODE
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel);

}

void SpecificWorker::AGMExecutiveTopic_symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
//subscribesToCODE
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);

}

void SpecificWorker::AGMExecutiveTopic_symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications)
{
//subscribesToCODE
	QMutexLocker l(mutex);
	for (auto modification : modifications)
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);

}

void SpecificWorker::AprilTags_newAprilTagAndPose(const tagsList &tags, const RoboCompGenericBase::TBaseState &bState, const RoboCompJointMotor::MotorStateMap &hState)
{
//subscribesToCODE

}

void SpecificWorker::AprilTags_newAprilTag(const tagsList &tags)
{
	std::cout<<"April received"<<std::endl;
	for(auto tag: tags)
	{
		std::cout << tag.cameraId <<"=>ID(x,y,z,rx,ry,rz): "<<tag.id<<"("<<tag.tx<<","<<tag.ty<<","<<tag.tz<<","<<tag.rx<<","<<tag.ry<<","<<tag.rz<<")"<<std::endl;
	}

}

void SpecificWorker::FullPoseEstimationPub_newFullPose(const RoboCompFullPoseEstimation::FullPose &pose)
{
//	std::cout<<"FullPose received "<<std::endl;
//	std::cout << pose.source <<" (x,y,z,rx,ry,rz): ("<<pose.x<<","<<pose.y<<","<<pose.z<<","<<pose.rx<<","<<pose.ry<<","<<pose.rz<<")"<<std::endl;
	RoboCompFullPoseEstimation::FullPose copy = pose;
	db.put(copy);
}



bool SpecificWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
	printf("<<< setParametersAndPossibleActivation\n");
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
	params.clear();
	for (ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
	{
		params[it->first] = it->second;
	}

	try
	{
		action = params["action"].value;
		std::transform(action.begin(), action.end(), action.begin(), ::tolower);
		//TYPE YOUR ACTION NAME
		if (action == "actionname")
		{
			active = true;
		}
		else
		{
			active = true;
		}
	}
	catch (...)
	{
		printf("exception in setParametersAndPossibleActivation %d\n", __LINE__);
		return false;
	}

	// Check if we should reactivate the component
	if (active)
	{
		active = true;
		reactivated = true;
	}

	printf("setParametersAndPossibleActivation >>>\n");

	return true;
}
void SpecificWorker::sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel)
{
	try
	{
		AGMMisc::publishModification(newModel, agmexecutive_proxy, "localizationAgentAgent");
	}
/*	catch(const RoboCompAGMExecutive::Locked &e)
	{
	}
	catch(const RoboCompAGMExecutive::OldModel &e)
	{
	}
	catch(const RoboCompAGMExecutive::InvalidChange &e)
	{
	}
*/
	catch(const Ice::Exception& e)
	{
		exit(1);
	}
}
