/*
 *    Copyright (C)2020 by YOUR NAME HERE
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

#include <socialrules.h>
#include <grid.h>
#include <controller.h>
#include <navigation.h>
#include <algorithm>

using namespace std;

#define USE_QTGUI
#ifdef USE_QTGUI
	#include "innerviewer.h"
#endif

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	bool AGMCommonBehavior_activateAgent(const ParameterMap &prs);
	bool AGMCommonBehavior_deactivateAgent();
	ParameterMap AGMCommonBehavior_getAgentParameters();
	StateStruct AGMCommonBehavior_getAgentState();
	void AGMCommonBehavior_killAgent();
	bool AGMCommonBehavior_reloadConfigAgent();
	bool AGMCommonBehavior_setAgentParameters(const ParameterMap &prs);
	int AGMCommonBehavior_uptimeAgent();
	void AGMExecutiveTopic_edgeUpdated(const RoboCompAGMWorldModel::Edge &modification);
	void AGMExecutiveTopic_edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications);
	void AGMExecutiveTopic_structuralChange(const RoboCompAGMWorldModel::World &w);
	void AGMExecutiveTopic_symbolUpdated(const RoboCompAGMWorldModel::Node &modification);
	void AGMExecutiveTopic_symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications);
    void AGMExecutiveTopic_selfEdgeAdded(const int nodeid, const string &edgeType, const RoboCompAGMWorldModel::StringDictionary &attributes);
    void AGMExecutiveTopic_selfEdgeDeleted(const int nodeid, const string &edgeType);
	void RCISMousePicker_setPick(const Pick &myPick);

	using InnerPtr = std::shared_ptr<InnerModel>;
	#ifdef USE_QTGUI
		using InnerViewerPtr = std::shared_ptr<InnerViewer>;
	#endif

	SocialRules socialrules;
	bool structuralChange = false;
	bool edgesUpdated = false;

	bool specificWorkerInitialized = false;

public slots:
    void compute();
    void initialize(int period);
    void checkRobotMovState();

    //Specification slot methods State Machine
	void sm_compute();
    void sm_initialize();

	void sm_finalize();

//--------------------
private:
	InnerPtr innerModel;
	std::string action;
	ParameterMap params;

    AGMModel::SPtr worldModel;
	bool active;


    #ifdef USE_QTGUI
        InnerViewerPtr viewer;
    #endif
    std::string robotname = "robot";
    std::shared_ptr<RoboCompCommonBehavior::ParameterList> confParams;


    Navigation<Grid<>,Controller> navigation;

	RoboCompLaser::TLaserData updateLaser();
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);



};

#endif
