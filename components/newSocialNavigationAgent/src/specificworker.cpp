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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

	active = false;
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
    innerModel = std::make_shared<InnerModel>();

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	emit t_compute_to_finalize();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    try{ robotname = params.at("RobotName").value;}
    catch(const std::exception &e){ std::cout << e.what() << "SpecificWorker::SpecificWorker - Robot name defined in config. Using default 'robot' " << std::endl;}

    confParams  = std::make_shared<RoboCompCommonBehavior::ParameterList>(params);


	defaultMachine.start();

	return true;
}

void SpecificWorker::initialize(int period)
{

	std::cout << "Initialize worker" << std::endl;

    connect(draw_gaussian_button,SIGNAL(clicked()),&socialrules, SLOT(drawGauss()));
    connect(draw_objects_button,SIGNAL(clicked()),&socialrules, SLOT(checkObjectAffordance()));
    connect(save_data_button,SIGNAL(clicked()),&socialrules, SLOT(saveData()));
    connect(gotoperson_button,SIGNAL(clicked()),&socialrules, SLOT(goToPerson()));
    connect(follow_checkbox, SIGNAL (clicked()),&socialrules,SLOT(checkstate()));
    connect(accompany_checkbox, SIGNAL (clicked()),&socialrules,SLOT(checkstate()));
    connect(passonright_checkbox, SIGNAL (clicked()),&socialrules,SLOT(checkstate()));

    socialrules.idselect_combobox = idselect_combobox;
    socialrules.follow_checkbox = follow_checkbox;
    socialrules.accompany_checkbox = accompany_checkbox;
    socialrules.passonright_checkbox = passonright_checkbox;


#ifdef USE_QTGUI
	viewer = std::make_shared<InnerViewer>(innerModel, "Social Navigation");  //InnerViewer copies internally innerModel so it has to be resynchronized
#endif

    try
    {
        RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
        AGMExecutiveTopic_structuralChange(w);
    }
    catch(...)
    {
        printf("The executive is probably not running, waiting for first AGM model publication...");
    }

//    trajectory.initialize(innerModel, viewer, confParams);
    navigation.initialize(innerModel, viewer, confParams);
    socialrules.initialize(worldModel, socialnavigationgaussian_proxy);


	this->Period = period;
	timer.start(Period);
    emit this->t_initialize_to_compute();


}

void SpecificWorker::compute()
{

	updateLaser();

    if (worldModelChanged) {
		auto [changes, totalpersons, intimate_seq, personal_seq, social_seq, object_seq, objectblock_seq] = socialrules.update(worldModel);
		if (changes) //se comprueba si alguna de las personas ha cambiado de posicion
            navigation.updatePolylines(totalpersons, intimate_seq, personal_seq, social_seq, object_seq, objectblock_seq);

        socialrules.checkRobotmov();

        worldModelChanged = false;
    }

    viewer->run();

}
void SpecificWorker::updateLaser()
{
    try{

    laserData  = laser_proxy->getLaserData();
    navigation.update(laserData);

    }

    catch(const Ice::Exception &e){
        std::cout <<"Can't connect to laser --" <<e.what() << std::endl;
    };

}



void SpecificWorker::sm_compute()
{
//	std::cout<<"Entered state compute"<<std::endl;
	compute();
}

void SpecificWorker::sm_initialize()
{
	std::cout<<"Entered initial state initialize"<<std::endl;
}

void SpecificWorker::sm_finalize()
{
	std::cout<<"Entered final state finalize"<<std::endl;
}



///////////////////////////////////////////////////////////////////////////////////////

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

bool SpecificWorker::AGMCommonBehavior_deactivateAgent()
{
//implementCODE
	return deactivate();
}

ParameterMap SpecificWorker::AGMCommonBehavior_getAgentParameters()
{
//implementCODE
	return params;
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

void SpecificWorker::AGMCommonBehavior_killAgent()
{
//implementCODE

}

bool SpecificWorker::AGMCommonBehavior_reloadConfigAgent()
{
//implementCODE
	return true;
}

bool SpecificWorker::AGMCommonBehavior_setAgentParameters(const ParameterMap &prs)
{
//implementCODE
	bool activated = false;
	return setParametersAndPossibleActivation(prs, activated);
}

int SpecificWorker::AGMCommonBehavior_uptimeAgent()
{
//implementCODE
	return 0;
}

void SpecificWorker::AGMExecutiveTopic_selfEdgeAdded(const int nodeid, const string &edgeType, const RoboCompAGMWorldModel::StringDictionary &attributes)
{
//subscribesToCODE
	QMutexLocker lockIM(mutex);
	try { worldModel->addEdgeByIdentifiers(nodeid, nodeid, edgeType, attributes); } catch(...){ printf("Couldn't add an edge. Duplicate?\n"); }

	try { innerModel = std::make_shared<InnerModel>(AGMInner::extractInnerModel(worldModel)); } catch(...) { printf("Can't extract an InnerModel from the current model.\n"); }
}

void SpecificWorker::AGMExecutiveTopic_selfEdgeDeleted(const int nodeid, const string &edgeType)
{
//subscribesToCODE
	QMutexLocker lockIM(mutex);
	try { worldModel->removeEdgeByIdentifiers(nodeid, nodeid, edgeType); } catch(...) { printf("Couldn't remove an edge\n"); }

	try { innerModel = std::make_shared<InnerModel>(AGMInner::extractInnerModel(worldModel)); } catch(...) { printf("Can't extract an InnerModel from the current model.\n"); }
}


void SpecificWorker::AGMExecutiveTopic_edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
//subscribesToCODE

	QMutexLocker locker(mutex);

	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel.get());

    auto symbol1 = worldModel->getSymbolByIdentifier(modification.a);
    auto symbol2 = worldModel->getSymbolByIdentifier(modification.b);

    if(symbol1.get()->symbolType == "person" or symbol2.get()->symbolType == "person")
        worldModelChanged = true;


}

void SpecificWorker::AGMExecutiveTopic_edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{
//subscribesToCODE

	QMutexLocker lockIM(mutex);
	for (auto modification : modifications)
	{
	    auto symbol1 = worldModel->getSymbolByIdentifier(modification.a);
        auto symbol2 = worldModel->getSymbolByIdentifier(modification.b);

        if(symbol1.get()->symbolType == "person" or symbol2.get()->symbolType == "person")
            worldModelChanged = true;

		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
		AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel.get());
	}


}

void SpecificWorker::AGMExecutiveTopic_structuralChange(const RoboCompAGMWorldModel::World &w)
{
	qDebug()<<__PRETTY_FUNCTION__;

	QMutexLocker lockIM(mutex);
	static bool first = true;

	AGMModelConverter::fromIceToInternal(w, worldModel);
	innerModel = std::make_shared<InnerModel>(AGMInner::extractInnerModel(worldModel));

	if (!first)
	{
		innerModel->save("/home/robocomp/robocomp/components/robocomp-viriato/etcSim/innermodel.xml");
	}
	else
		first = false;

	viewer->reloadInnerModel(innerModel);
    worldModelChanged = true;

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
		AGMMisc::publishModification(newModel, agmexecutive_proxy, "socialNavigationAgentAgent");
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
