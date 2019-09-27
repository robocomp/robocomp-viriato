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

/**
       \brief
       @author authorname
*/

// THIS IS AN AGENT


#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	bool odometryAndLocationIssues(bool force=false);
	void includeMovementInRobotSymbol(AGMModelSymbol::SPtr robot);
	void setCorrectedPosition(const RoboCompGenericBase::TBaseState &bState);
	bool enoughDifference(const RoboCompGenericBase::TBaseState &lastState, const RoboCompGenericBase::TBaseState &newState);
	void computeNewState();

	bool AGMCommonBehavior_reloadConfigAgent();
	bool AGMCommonBehavior_activateAgent(const ParameterMap &prs);
	bool AGMCommonBehavior_setAgentParameters(const ParameterMap &prs);
	ParameterMap AGMCommonBehavior_getAgentParameters();
	void AGMCommonBehavior_killAgent();
	int AGMCommonBehavior_uptimeAgent();
	bool AGMCommonBehavior_deactivateAgent();
	StateStruct AGMCommonBehavior_getAgentState();
	void AGMExecutiveTopic_structuralChange(const RoboCompAGMWorldModel::World &w);
	void AGMExecutiveTopic_edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications);
	void AGMExecutiveTopic_edgeUpdated(const RoboCompAGMWorldModel::Edge &modification);
	void AGMExecutiveTopic_symbolUpdated(const RoboCompAGMWorldModel::Node &modification);
	void AGMExecutiveTopic_symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications);
	void AprilTags_newAprilTagAndPose(const tagsList &tags, const RoboCompGenericBase::TBaseState &bState, const RoboCompJointMotor::MotorStateMap &hState);
	void AprilTags_newAprilTag(const tagsList &tags);
	void FullPoseEstimationPub_newFullPose(const RoboCompFullPoseEstimation::FullPose &pose);

public slots:
	void compute();
	void initialize(int period);
//Specification slot methods State Machine
	void sm_compute();
	void sm_initialize();
	void sm_finalize();

//--------------------
private:
	RoboCompGenericBase::TBaseState lastState, aprilState, omniState;
	RoboCompCommonBehavior::ParameterList worker_params;
	InnerModel *innerModel;
	std::string action;
	ParameterMap params;
	AGMModel::SPtr worldModel;
	bool active;
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);

};

#endif
