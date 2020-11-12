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


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:

	SpecificWorker(MapPrx& mprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	bool AGMCommonBehavior_activateAgent(const RoboCompAGMCommonBehavior::ParameterMap &prs);
	bool AGMCommonBehavior_deactivateAgent();
	RoboCompAGMCommonBehavior::ParameterMap AGMCommonBehavior_getAgentParameters();
	RoboCompAGMCommonBehavior::StateStruct AGMCommonBehavior_getAgentState();
	void AGMCommonBehavior_killAgent();
	bool AGMCommonBehavior_reloadConfigAgent();
	bool AGMCommonBehavior_setAgentParameters(const RoboCompAGMCommonBehavior::ParameterMap &prs);
	int AGMCommonBehavior_uptimeAgent();
	void AGMExecutiveTopic_edgeUpdated(const RoboCompAGMWorldModel::Edge &modification);
	void AGMExecutiveTopic_edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications);
	void AGMExecutiveTopic_selfEdgeAdded(const int nodeid, const std::string &edgeType, const RoboCompAGMWorldModel::StringDictionary &attributes);
	void AGMExecutiveTopic_selfEdgeDeleted(const int nodeid, const std::string &edgeType);
	void AGMExecutiveTopic_structuralChange(const RoboCompAGMWorldModel::World &w);
	void AGMExecutiveTopic_symbolUpdated(const RoboCompAGMWorldModel::Node &modification);
	void AGMExecutiveTopic_symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications);

public slots:
	void compute();
    int startup_check();

	void initialize(int period);
//Specification slot methods State Machine
	void sm_compute();
	void sm_initialize();
	void sm_finalize();
//--------------------
    void affordanceSliderChanged(int value);
    void affordanceTimeSliderChanged(int step);
    void affordanceTimeEditChanged(const QTime &time);
    void programTherapy();
    void removeTherapy();
	void drawPersonalSpace();
	void recordData();
    void checkRobotPermission();



private:

    RoboCompSocialNavigationGaussian::SNGPersonSeq sngPersonSeq;
    std::map<int32_t, RoboCompSocialNavigationGaussian::SNGPerson> mapIdPersons;
    vector <RoboCompSocialNavigationGaussian::SNGPersonSeq> interactingPersonsVec; //vector de grupos que interactuan entre ellos

    struct PersonalSpaceType
    {
        RoboCompSocialNavigationGaussian::SNGPolylineSeq intimatePolylines;
        RoboCompSocialNavigationGaussian::SNGPolylineSeq personalPolylines;
        RoboCompSocialNavigationGaussian::SNGPolylineSeq socialPolylines;

        vector<int> spacesSharedWith;
    };

    std::map<int32_t, PersonalSpaceType> mapIdSpaces;

    RoboCompSocialNavigationGaussian::SNGPolylineSeq intimateSpace_seq, personalSpace_seq, socialSpace_seq;


	bool worldModelChanged = false;

	struct Therapy
	{
		int id;
		QTime startT;
		QTime endT;
	};

    struct ObjectType
    {
        int id;
        float x;
        float z;
        float rot;
        string imName;

        float cost = 2.0;
        float prevCost = 2.0;


        RoboCompSocialNavigationGaussian::SNGPolyline affordance;
        bool interacting = false;

        QString shape;
        float width;
        float depth;
        float height;
        float inter_space;
        float inter_angle;


        bool therapiesProgrammed = false;
		vector<Therapy> therapies;

    };

    std::map<QString, ObjectType> mapIdObjects;
    bool costChanged = false;


    //To save data to file
    int32_t robotSymbolId;
    RoboCompSocialNavigationGaussian::SNGPerson robot;

    struct Point {float x;float z;};
    Point point;
    vector <Point> poserobot;
    float totaldist = 0;

    std::map<QString, vector<float>> mapCostsPerHour;

    bool permission_given = false;

    //------------------------------------------//
    std::shared_ptr<InnerModel> innerModel;
	std::string action;
	RoboCompAGMCommonBehavior::ParameterMap params;
	AGMModel::SPtr worldModel, newModel;

	bool active;
	bool setParametersAndPossibleActivation(const RoboCompAGMCommonBehavior::ParameterMap &prs, bool &reactivated);
	void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);
	bool startup_check_flag;

    void updatePeopleInModel();
    void checkInteractions();
    vector <vector<int32_t>> groupInteractingPeople(int32_t id, int32_t pairId,vector<vector<int32_t>> &interactingId);
    void checkObjectAffordance();

    RoboCompSocialNavigationGaussian::SNGPolyline affordanceTrapezoidal(ObjectType obj);
    RoboCompSocialNavigationGaussian::SNGPolyline affordanceRectangular(ObjectType obj);
    RoboCompSocialNavigationGaussian::SNGPolyline affordanceCircular(ObjectType obj);


    std::map<int32_t,bool> peoplePermission;
    int32_t personPermission = -1;
    int indexPerson = 0;
    vector<AGMModelSymbol::SPtr> symbolsToPublish;

    void checkHumanPermissions();

    void calculatePersonalSpaces(RoboCompSocialNavigationGaussian::SNGPersonSeq personGroup);
    void applySocialRules();
	void publishPersonalSpaces();
	void publishAffordances();

	void updatePersonalSpacesInGraph();
	void updateAffordancesInGraph();
	void arrangePersonalSpaces(RoboCompSocialNavigationGaussian::SNGPersonSeq personGroup,RoboCompSocialNavigationGaussian::SNGPolylineSeq intimate,
                               RoboCompSocialNavigationGaussian::SNGPolylineSeq personal, RoboCompSocialNavigationGaussian::SNGPolylineSeq social);

    void checkRobotmov();



};

#endif
