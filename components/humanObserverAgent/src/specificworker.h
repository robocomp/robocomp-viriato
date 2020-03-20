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

#include <cmath>

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <QPolygonF>

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


    bool worldModelChanged = false;
    bool ourModelChanged = false;
    std::vector<AGMModelSymbol::SPtr> previousPersonsList;
    struct PersonType
    {
        QString imName; //Nombre del nodo geometrico en AGM
        int id;
        float x;
        float z;
        float rot;

    };

    vector<PersonType> totalPersons;


    struct ObjectType
    {
		QString imName; //Nombre del nodo geometrico en AGM
        int id;
        float x;
        float z;
        float rot;

        QString shape;
        float width;
        float depth;
        float height;
        float inter_space;
        float inter_angle;


    };
    vector<ObjectType> totalObjects;


    float thr_angle_humans = (45*M_PI)/180; //buscar angulo adecuado
    float threshold_dist = 3000; //reducir


    void loadInfoFromAGM();
    bool checkHumanInteraction();
    bool checkObjectInteraction();
    QPolygonF getAffordance(int objectID);



public slots:
    void compute();
    void initialize(int period);
//Specification slot methods State Machine
	void sm_compute();
	void sm_initialize();
	void sm_finalize();


//--------------------
private:
	std::shared_ptr<InnerModel> innerModel;
	std::string action;
	ParameterMap params;
	AGMModel::SPtr worldModel,newModel;
	bool active;
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);

};

#endif
