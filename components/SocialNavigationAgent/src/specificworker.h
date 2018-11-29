/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
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
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <python2.7/Python.h>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <QFile>
#include "pathfinder.h"
#include <actionexecution.h>
#include <socialrules.h>
#include "safepolylist.h"


//PROBLEMA: con python 3.5 da error al compilar

#include <innermodel/innermodel.h>
#include <boost/format.hpp>

using namespace std;

#define USE_QTGUI
#ifdef USE_QTGUI
	#include "innerviewer.h"
#endif

#define THRESHOLD 40

/**
       \brief
       @author authorname
*/

class SpecificWorker : public GenericWorker
{
Q_OBJECT

public:

	using InnerPtr = std::shared_ptr<InnerModel>;
	#ifdef USE_QTGUI
 		using InnerViewerPtr = std::shared_ptr<InnerViewer>;
 	#endif

	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

//bool para indicar si se ha movido la persona, lo utilizare para imprimir la coordenada de la persona cada vez que se mueva
	bool changepos = false;

	ActionExecution aE; //Class ActionExecution
	SocialRules socialrules; //Class SocialRules

	bool staticperson = false;
	///////////////////////////////////////////////////////////////////////////
	/// SERVANTS
	//////////////////////////////////////////////////////////////////////////
	bool activateAgent(const ParameterMap& prs);
	bool deactivateAgent();
	StateStruct getAgentState();
	ParameterMap getAgentParameters();
	bool setAgentParameters(const ParameterMap& prs);
	void  killAgent();
	Ice::Int uptimeAgent();
	bool reloadConfigAgent();

	void  structuralChange(const RoboCompAGMWorldModel::World & modification);
	void  symbolUpdated(const RoboCompAGMWorldModel::Node& modification);
	void  symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence & modification);
	void  edgeUpdated(const RoboCompAGMWorldModel::Edge& modification);
	void edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications);

	//double agaussian(SNGPerson person, float x, float y);

	NavState getState(){ return pathfinder.getState(); };

	void setHumanSpace(const PolyLineList &polyList){};
	void checkHumanBlock();
	vector <int32_t> previous_blockinglist = {};
	vector <int32_t> previous_softblockinglist = {};
    vector <int32_t> previous_affordanceslist = {};

	//Trajectory
	float goBackwards(const TargetPose &target){return 0.0;};
	void stop(){};
	float goReferenced(const TargetPose &target, const float xRef, const float zRef, const float threshold){return 0;};
	float changeTarget(const TargetPose &target){return 0.0;};
	void mapBasedTarget(const NavigationParameterMap &parameters){};
	float go(const TargetPose &target);
	//float go(const TargetPose &target){pathfinder.go(target.x, -target.z);  return 0.0;};



public slots:
 	void compute();
	//void readTrajState();
// 	SNGPolylineSeq gauss(bool draw=true);
// 	void changevalue(int value);



private:
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	bool active;
	void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel,std::string m);

	std::string action;
	ParameterMap params;
	AGMModel::SPtr worldModel;

	bool haveTarget;
	QTimer trajReader;
	AGMModel::SPtr world;
	RoboCompTrajectoryRobot2D::NavState planningState;
	// Target info
	RoboCompTrajectoryRobot2D::TargetPose currentTarget;

	// New TrajectoryRobot Class
	robocomp::pathfinder::PathFinder pathfinder;
	std::thread thread_pathfinder;


	std::string robotname = "robot";
	RoboCompGenericBase::TBaseState bState;
	InnerPtr innerModel;
	#ifdef USE_QTGUI
		InnerViewerPtr viewer;
	#endif


//CHECK
	//void updateRobotsCognitiveLocation();
//	std::map<int32_t, QPolygonF> roomsPolygons;
//	std::map<int32_t, QPolygonF> extractPolygonsFromModel(AGMModel::SPtr &worldModel);
//	RoboCompOmniRobot::TBaseState bState;

};


#endif


