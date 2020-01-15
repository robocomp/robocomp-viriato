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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include "config.h"
#include <stdint.h>
#include <qlog/qlog.h>

#if Qt5_FOUND
	#include <QtWidgets>
#else
	#include <QtGui>
#endif
#include <ui_mainUI.h>
#include <QStateMachine>
#include <QState>
#include <CommonBehavior.h>

#include <GenericBase.h>
#include <Planning.h>
#include <Laser.h>
#include <OmniRobot.h>
#include <SocialNavigationGaussian.h>
#include <AGMCommonBehavior.h>
#include <AGMExecutive.h>
#include <AGMExecutiveTopic.h>
#include <AGMWorldModel.h>
#include <agm.h>

#include <agm.h>

#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

using namespace std;
using namespace RoboCompGenericBase;
using namespace RoboCompPlanning;
using namespace RoboCompLaser;
using namespace RoboCompOmniRobot;
using namespace RoboCompSocialNavigationGaussian;
using namespace RoboCompAGMCommonBehavior;
using namespace RoboCompAGMExecutive;
using namespace RoboCompAGMExecutiveTopic;
using namespace RoboCompAGMWorldModel;

typedef map <string,::IceProxy::Ice::Object*> MapPrx;


struct BehaviorParameters
{
	RoboCompPlanning::Action action;
	std::vector< std::vector <std::string> > plan;
};

class GenericWorker :
#ifdef USE_QTGUI
	public QWidget, public Ui_guiDlg
#else
	public QObject
 #endif
{
Q_OBJECT
public:
	GenericWorker(MapPrx& mprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);

	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;
	bool activate(const BehaviorParameters& parameters);
	bool deactivate();
	bool isActive() { return active; }


	AGMExecutivePrx agmexecutive_proxy;
	LaserPrx laser_proxy;
	OmniRobotPrx omnirobot_proxy;
	SocialNavigationGaussianPrx socialnavigationgaussian_proxy;

	virtual bool AGMCommonBehavior_activateAgent(const ParameterMap &prs) = 0;
	virtual bool AGMCommonBehavior_deactivateAgent() = 0;
	virtual ParameterMap AGMCommonBehavior_getAgentParameters() = 0;
	virtual StateStruct AGMCommonBehavior_getAgentState() = 0;
	virtual void AGMCommonBehavior_killAgent() = 0;
	virtual bool AGMCommonBehavior_reloadConfigAgent() = 0;
	virtual bool AGMCommonBehavior_setAgentParameters(const ParameterMap &prs) = 0;
	virtual int AGMCommonBehavior_uptimeAgent() = 0;
	virtual void AGMExecutiveTopic_edgeUpdated(const RoboCompAGMWorldModel::Edge &modification) = 0;
	virtual void AGMExecutiveTopic_edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications) = 0;
	virtual void AGMExecutiveTopic_selfEdgeAdded(const int nodeid, const string &edgeType, const RoboCompAGMWorldModel::StringDictionary &attributes) = 0;
	virtual void AGMExecutiveTopic_selfEdgeDeleted(const int nodeid, const string &edgeType) = 0;
	virtual void AGMExecutiveTopic_structuralChange(const RoboCompAGMWorldModel::World &w) = 0;
	virtual void AGMExecutiveTopic_symbolUpdated(const RoboCompAGMWorldModel::Node &modification) = 0;
	virtual void AGMExecutiveTopic_symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications) = 0;

protected:
//State Machine
	QStateMachine defaultMachine;

	QState *computeState;
	QState *initializeState;
	QFinalState *finalizeState;

//-------------------------

	QTimer timer;
	int Period;
	bool active;
	AGMModel::SPtr worldModel;
	BehaviorParameters p;
	ParameterMap params;
	int iter;
	bool setParametersAndPossibleActivation(const RoboCompAGMCommonBehavior::ParameterMap &prs, bool &reactivated);
	RoboCompPlanning::Action createAction(std::string s);

private:


public slots:
//Slots funtion State Machine
	virtual void sm_compute() = 0;
	virtual void sm_initialize() = 0;
	virtual void sm_finalize() = 0;

//-------------------------
	virtual void compute() = 0;
    virtual void initialize(int period) = 0;
	
signals:
	void kill();
//Signals for State Machine
	void t_initialize_to_compute();
	void t_compute_to_compute();
	void t_compute_to_finalize();

//-------------------------
};

#endif
