/*
 *    Copyright (C) 2020 by YOUR NAME HERE
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
#include <QtGui/QPolygonF>
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx,bool startup_check) : GenericWorker(mprx)
{
	active = false;
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
    this->startup_check_flag = startup_check;

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

	innerModel = std::make_shared<InnerModel>(new InnerModel());
	try
	{
		RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
		AGMExecutiveTopic_structuralChange(w);
        robotSymbolId = worldModel->getIdentifierByType("robot");
        qDebug()<< "Robot id is "<<robotSymbolId;
        readRoomPolylines();
	}
	catch(...)
	{
		printf("The executive is probably not running, waiting for first AGM model publication...");
	}


	defaultMachine.start();
	

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
    this->Period = period;
    if(this->startup_check_flag)
    {
        this->startup_check();
    }
    else
    {
        timer.start(Period);
        emit this->t_initialize_to_compute();
    }

}
int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}
void SpecificWorker::compute()
{
    QMutexLocker lockIM(mutex);

	newModel = AGMModel::SPtr(new AGMModel(worldModel));

    if ( updatePeopleRoom() or  updateRobotRoom()) {
		try {
			sendModificationProposal(worldModel, newModel);
		}
		catch (...) {
			std::cout << "No se puede actualizar worldModel" << std::endl;
		}
	}

}


////////////////////////////////////////////////////////////////////

void SpecificWorker::readRoomPolylines()
{
	qDebug() << __FUNCTION__;

	auto rooms = worldModel->getSymbolsByType("room");

	for (auto r : rooms) {

		auto id = r->identifier;
		QPolygonF polygon;

		try {
			auto polyline = QString::fromStdString(worldModel->getSymbolByIdentifier(id)->getAttribute("polyline"));

			for (auto pxz : polyline.split(";"))
			{
				auto p = pxz.split(" ");

				if (p.size() != 2)
					continue;

				auto x = std::stof(p[0].toStdString());
				auto z = std::stof(p[1].toStdString());

				polygon << QPointF(x,z);
			}
			mapRoomPolygon[id] = polygon;
		}

		catch (std::exception& e) {
			std::cout << "Exception reading room "<< id << " polyline: " << e.what() << std::endl;
		}
	}
}

bool SpecificWorker::updatePeopleRoom() {

    bool changesInEdges = false;

	auto vectorPersons = worldModel->getSymbolsByType("person");

	if (vectorPersons.size() == 0) {
		qDebug() << "No persons found";
	}

	for (auto personAGM: vectorPersons) {

		auto id = personAGM->identifier;

		AGMModelSymbol::SPtr personParent = worldModel->getParentByLink(id, "RT");
		AGMModelEdge& edgeRT = worldModel->getEdgeByIdentifiers(personParent->identifier, id, "RT");

		auto x = str2float(edgeRT.attributes["tx"]);
		auto z = str2float(edgeRT.attributes["tz"]);

        int actualRoomID = -1;

        for (auto [roomID,roomPolyline] : mapRoomPolygon)
        {
		    if (roomPolyline.containsPoint(QPointF(x,z), Qt::FillRule::OddEvenFill))
		    {
                actualRoomID = roomID;
                break;
		    }
        }

        if (actualRoomID == -1)
        {
            qDebug()<< "can't locate person "<< id;
            continue;
        }

        int prevRoomID = -1;
        for (auto edge = personAGM->edgesBegin(worldModel); edge != personAGM->edgesEnd(worldModel); edge++)
        {
            const std::pair<int32_t, int32_t> symbolPair = edge->getSymbolPair();
            if (edge->getLabel() == "in")
            {
                const string secondType = worldModel->getSymbol(symbolPair.second)->symbolType;
                if (symbolPair.first == id and secondType == "room")
                {
                    prevRoomID = symbolPair.second;
                    break;
                }
            }
        }


        if (actualRoomID == prevRoomID)
            continue;

        else
        {

            try
            {
                newModel->addEdgeByIdentifiers(id, actualRoomID, "in");

                qDebug ()<<" Se añade el enlace entre " << id << " y "<< actualRoomID;
            }

            catch(...)
            {
                std::cout<<__FUNCTION__<<"Ya existe el enlace"<<std::endl;
            }


			try
			{
				newModel->removeEdgeByIdentifiers(id, prevRoomID, "in");
				qDebug ()<<" Se elimina el enlace entre " << id << " y "<< actualRoomID;
			}

			catch(...)
			{
				std::cout<<__FUNCTION__<<"No existe el enlace"<<std::endl;

			}

            changesInEdges = true;
        }


	}

	return changesInEdges;

}

bool SpecificWorker::updateRobotRoom() {

    bool changesInEdges = false;

    auto currentRobotPose = innerModel->transformS6D("world","robot");
    int actualRoomID = -1;

    for (auto [roomID,roomPolyline] : mapRoomPolygon)
    {
        qDebug()<< roomID<< roomPolyline;
        if (roomPolyline.containsPoint(QPointF(currentRobotPose.x(),currentRobotPose.z()), Qt::FillRule::OddEvenFill))
        {
            qDebug()<< "Contained";
            actualRoomID = roomID;
            break;
        }
    }

    if (actualRoomID == -1)
    {
        qFatal( "can't locate robot");
    }

    int prevRoomID = -1;
    AGMModelSymbol::SPtr robotSymbol = worldModel->getSymbol(robotSymbolId);

    for (auto edge = robotSymbol->edgesBegin(worldModel); edge != robotSymbol->edgesEnd(worldModel); edge++)
    {
        const std::pair<int32_t, int32_t> symbolPair = edge->getSymbolPair();
        if (edge->getLabel() == "in")
        {
            const string secondType = worldModel->getSymbol(symbolPair.second)->symbolType;
            if (symbolPair.first == robotSymbolId and secondType == "room")
            {
                prevRoomID = symbolPair.second;
                break;
            }
        }
    }

    if (actualRoomID == prevRoomID) {
        changesInEdges = false;
    }

    else {

        try
        {
            newModel->addEdgeByIdentifiers(robotSymbolId, actualRoomID, "in");

            qDebug ()<<" Se añade el enlace entre " << robotSymbolId << " y "<< actualRoomID;
        }

        catch(...)
        {
            std::cout<<__FUNCTION__<<"Ya existe el enlace"<<std::endl;
        }


        try
        {
            newModel->removeEdgeByIdentifiers(robotSymbolId, prevRoomID, "in");
            qDebug ()<<" Se elimina el enlace entre " << robotSymbolId << " y "<< actualRoomID;
        }

        catch(...)
        {
            std::cout<<__FUNCTION__<<"No existe el enlace"<<std::endl;

        }

        changesInEdges = true;
    }


    return changesInEdges;


}

////////////////////////////////////////////////////////////////////

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



bool SpecificWorker::AGMCommonBehavior_activateAgent(const RoboCompAGMCommonBehavior::ParameterMap &prs)
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

RoboCompAGMCommonBehavior::ParameterMap SpecificWorker::AGMCommonBehavior_getAgentParameters()
{
//implementCODE
	return params;
}

RoboCompAGMCommonBehavior::StateStruct SpecificWorker::AGMCommonBehavior_getAgentState()
{
//implementCODE
    RoboCompAGMCommonBehavior::StateStruct s;
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

bool SpecificWorker::AGMCommonBehavior_setAgentParameters(const RoboCompAGMCommonBehavior::ParameterMap &prs)
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

//SUBSCRIPTION to edgeUpdated method from AGMExecutiveTopic interface
void SpecificWorker::AGMExecutiveTopic_edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
//subscribesToCODE
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel.get());

}

//SUBSCRIPTION to edgesUpdated method from AGMExecutiveTopic interface
void SpecificWorker::AGMExecutiveTopic_edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{
//subscribesToCODE
	QMutexLocker lockIM(mutex);
	for (auto modification : modifications)
	{
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
		AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel.get());
	}

}

//SUBSCRIPTION to selfEdgeAdded method from AGMExecutiveTopic interface
void SpecificWorker::AGMExecutiveTopic_selfEdgeAdded(const int nodeid, const string &edgeType, const RoboCompAGMWorldModel::StringDictionary &attributes)
{
//subscribesToCODE
	QMutexLocker lockIM(mutex);
 	try { worldModel->addEdgeByIdentifiers(nodeid, nodeid, edgeType, attributes); } catch(...){ printf("Couldn't add an edge. Duplicate?\n"); }
 
	try { innerModel = std::make_shared<InnerModel>(AGMInner::extractInnerModel(worldModel)); } catch(...) { printf("Can't extract an InnerModel from the current model.\n"); }
}

//SUBSCRIPTION to selfEdgeDeleted method from AGMExecutiveTopic interface
void SpecificWorker::AGMExecutiveTopic_selfEdgeDeleted(const int nodeid, const string &edgeType)
{
//subscribesToCODE
	QMutexLocker lockIM(mutex);
 	try { worldModel->removeEdgeByIdentifiers(nodeid, nodeid, edgeType); } catch(...) { printf("Couldn't remove an edge\n"); }
 
	try { innerModel = std::make_shared<InnerModel>(AGMInner::extractInnerModel(worldModel)); } catch(...) { printf("Can't extract an InnerModel from the current model.\n"); }
}

//SUBSCRIPTION to structuralChange method from AGMExecutiveTopic interface
void SpecificWorker::AGMExecutiveTopic_structuralChange(const RoboCompAGMWorldModel::World &w)
{
//subscribesToCODE
	QMutexLocker lockIM(mutex);
 	AGMModelConverter::fromIceToInternal(w, worldModel);
 
	innerModel = std::make_shared<InnerModel>(AGMInner::extractInnerModel(worldModel));
}

//SUBSCRIPTION to symbolUpdated method from AGMExecutiveTopic interface
void SpecificWorker::AGMExecutiveTopic_symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
//subscribesToCODE
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);

}

//SUBSCRIPTION to symbolsUpdated method from AGMExecutiveTopic interface
void SpecificWorker::AGMExecutiveTopic_symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications)
{
//subscribesToCODE
	QMutexLocker l(mutex);
	for (auto modification : modifications)
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);

}



bool SpecificWorker::setParametersAndPossibleActivation(const RoboCompAGMCommonBehavior::ParameterMap &prs, bool &reactivated)
{
	printf("<<< setParametersAndPossibleActivation\n");
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
	params.clear();
	for (RoboCompAGMCommonBehavior::ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
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
		AGMMisc::publishModification(newModel, agmexecutive_proxy, "roomLocalizationAgentAgent");
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
