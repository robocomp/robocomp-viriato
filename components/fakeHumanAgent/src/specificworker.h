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
#include <qjoystick/qjoystick.h>

#define HUMANADVVEL 50
#define HUMANROTVEL 0.1

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
public:
  
	void move();
	void movePerson(TPerson *person, RoboCompInnerModelManager::coord3D coordInItem, bool global = false);
	RoboCompInnerModelManager::coord3D autoMovePerson(TPerson person);
	
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void initializeUI();
	bool includeInRCIS(int id, const RoboCompInnerModelManager::Pose3D &pose, std::string mesh);
	bool removeFromRCIS(int id);
	int includeInAGM(int id,const RoboCompInnerModelManager::Pose3D &pose, std::string mesh);
	bool removeFromAGM(int id);
	void changePersonRoom(int personID, int roomID);
	void cleanListWidget(int personID);
	
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	bool sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);
	bool reloadConfigAgent();
	bool activateAgent(const ParameterMap &prs);
	bool setAgentParameters(const ParameterMap &prs);
	ParameterMap getAgentParameters();
	void killAgent();
	int uptimeAgent();
	bool deactivateAgent();
	StateStruct getAgentState();
	void structuralChange(const RoboCompAGMWorldModel::World &w);
	void edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modification);
	void edgeUpdated(const RoboCompAGMWorldModel::Edge &modification);
	void symbolUpdated(const RoboCompAGMWorldModel::Node &modification);
	void symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modification);
	TInteraction string2Interaction(std::string interaction);
	void pointsChanged();
	void updatePersonInterfaz(bool enable);

public slots:
	void compute();
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
	void giroP();
	void giroR();

private:
	InnerModel *innerModel;
	std::string action;
	ParameterMap params;
	AGMModel::SPtr worldModel;
	bool active;
	
	QTime lastJoystickEvent;
	QJoyStick *joystick;
};

#endif

