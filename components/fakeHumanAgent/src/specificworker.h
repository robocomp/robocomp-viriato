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

/**
       \brief
       @author authorname
*/

// THIS IS AN AGENT


#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
//#include <qjoystick/qjoystick.h>
#include <QMessageBox>
#include <QFileDialog>

#define HUMANADVVEL 50
#define HUMANROTVEL 0.1

//#ifdef USE_QTGUI
//    #include <osgviewer/osgview.h>
//  #include <innermodel/innermodelviewer.h>
//#endif



Q_DECLARE_METATYPE(AGMModelEdge)

class SpecificWorker : public GenericWorker
{
Q_OBJECT

	enum TInteraction {
		isBusy, 
		interacting,
		block,
		softBlock,
		unknown
	};

	struct TPerson {
		bool autoMovement;
		int currentPoint;
		int speed;
		std::vector<RoboCompInnerModelManager::Pose3D> points;
		int personSymbolId;
		std::string name;
		RoboCompInnerModelManager::Pose3D pose;
	};
	std::map<int, TPerson> personMap;
	bool moveFlag = false;
	bool setPoseFlag = false;
	RoboCompInnerModelManager::coord3D coordInItem;
	float valorgiro;
	int robotID;

	std::string meshname,scale,rotationz,translationy;

	 QRectF outerRegion;
    int hmin, hmax, vmin, vmax;
	QString prevIdSelected = "";



public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	void initializeUI();
	void readPersonsFromAGM();
	bool includeInRCIS(int id, const RoboCompInnerModelManager::Pose3D &pose, std::string mesh);
	bool removeFromRCIS(int id);
	int includeInAGM(int id,const RoboCompInnerModelManager::Pose3D &pose, std::string mesh);
	bool removeFromAGM(int id);
	void changePersonRoom(int personID, int roomID);
	void cleanListWidget(int personID);

	void move();
	void movePerson(TPerson *person, RoboCompInnerModelManager::coord3D coordInItem, bool global = false);
	RoboCompInnerModelManager::coord3D autoMovePerson(TPerson person);

	TInteraction string2Interaction(std::string interaction);
	void pointsChanged();
	void updatePersonInterfaz(bool enable);


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
public slots:
	void compute();
    void initialize(int period);

	void setPose();
	//void receivedJoyStickEvent(int value, int type, int number);
	
	void autoMovement();
	void addPerson();
	void delPerson();
	void savePoints();
	void loadPoints();
	void personChanged(int index);
	void interactionChanged(int index);
	void addInteraction();
	void removeEdgeAGM();
	void upP ();
	void upR ();
	void downP ();
	void downR ();
	void rightP ();
	void rightR ();
	void leftP ();
	void leftR ();
	void rotar(int valor);
	void cbIndexChanged(int index);
	void giroP();
	void giroR();
	void moverandom();

private:
	std::shared_ptr<InnerModel> innerModel;
//#ifdef USE_QTGUI
//    OsgView *osgView;
//
//#endif

//    InnerModelViewer *innerModelViewer;
	std::string action;
	ParameterMap params;
	AGMModel::SPtr worldModel;
	bool active;
	
//	QTime lastJoystickEvent;
//	QJoyStick *joystick;

//	void regenerateInnerModelViewer();
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	bool sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);
};

#endif

