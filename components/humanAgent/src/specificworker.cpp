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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

	active = false;
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModel = new InnerModel();
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

	try
	{
		RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
		structuralChange(w);
	}
	catch(...)
	{
		printf("The executive is probably not running, waiting for first AGM model publication...");
	}

    Period = 200;
    timer.start(Period);

	return true;
}




void SpecificWorker::compute()
{
    QMutexLocker locker(mutex);
    getHumans();
}


void SpecificWorker::obtainHumanPose(const humansDetected &list_of_humans) {

    reading = true;
    Humans.clear();
    Humans = list_of_humans;
    reading = false;


}

void SpecificWorker::getHumans()
{
    if (reading) return;


    humans_in_world.clear();

    if (previous_humans.size() == 0) //Si no hay personas en el modelo
    {
        qDebug()<<"No hay personas en el mundo";
        for (auto h : Humans)
        {
            if (h.pos.pos_good) //Solo incluyo en AGM si la posición se ha calculado correctamente
            {
                includeInAGM(h.id, h.pos);
                humans_in_world.push_back(h);
            }
        }
    }

    else
    {
        for (auto h : Humans)
        {
            bool found = false;

            for (auto human : previous_humans)
            {
                if (human.id == h.id)
                {
                    found = true;
                    break;
                }
            }

            if (found)
                movePersonInAGM(h.id,h.pos);

            else
            {
                for (int i = 0; i < previous_humans.size(); i++)
                {

                    if (h.IDcamera != previous_humans[i].IDcamera)

                    {
                        auto dist = sqrt(((previous_humans[i].pos.x - h.pos.x)*(previous_humans[i].pos.x - h.pos.x))+(previous_humans[i].pos.z - h.pos.z)*(previous_humans[i].pos.z - h.pos.z));

                        qDebug()<<"DISTANCIA ENTRE " << previous_humans[i].id << " y " << h.id << " = " <<dist;

                        if (dist < 500) //450 es espacio íntimo
                        {
                            movePersonInAGM(previous_humans[i].id,h.pos);
                        }

                        else if (i == (previous_humans.size()-1))//solo insertar si es la última persona en comprobarse
                        {
                            if(h.pos.pos_good)
                            {
                                includeInAGM(h.id, h.pos);
                                humans_in_world.push_back(h);
                            }
                        }
                    }

                    else if (i == (previous_humans.size()-1))
                    {
                        if(h.pos.pos_good)
                        {
                            includeInAGM(h.id, h.pos);
                            humans_in_world.push_back(h);
                        }
                    }
                }
            }
        }
    }

    previous_humans = humans_in_world;
}




void SpecificWorker::includeInAGM(int id,const Pose3D pose)
{
    printf("includeInAGM begins\n");
    std::string meshname;
    std::string scale;
    std::string rotationz;

    if (mesh == 6) mesh = 1;
    switch(mesh)
    {
        case 1:
            meshname = "human01.3ds";
            scale = "12";
            rotationz= "3.1415926535";
            break;
        case 2:
            meshname = "human02.3ds";
            scale = "1.12";
            rotationz= "3.1415926535";
            break;
        case 3:
            meshname = "human03.3ds";
            scale = "8";
            rotationz= "1.57079632679";
            break;
        case 4:
            meshname = "human04.3ds";
            scale = "900";
            rotationz= "0";
            break;
        case 5:meshname = "human05.3ds";
            scale = "800";
            rotationz= "0";
            break;
        case 6:meshname = "human06.3ds";
            scale = "23";
            rotationz= "3.1415926535";
            break;
        default:
            qDebug()<< "Mesh error";
            return;
    }

    mesh++;

    std::string type = "person";
    std::string imName = "person" + std::to_string(id);
    int personSymbolId = -1;
    int idx=0;
    while ((personSymbolId = worldModel->getIdentifierByType(type, idx++)) != -1)
    {
        printf("%d %d\n", idx, personSymbolId);
        if (worldModel->getSymbolByIdentifier(personSymbolId)->getAttribute("imName") == imName)
        {
            printf("found %d!!\n", personSymbolId);
            break;
        }
    }
    if (personSymbolId != -1)
    {
        printf("Person already in the AGM model\n");
        return;
    }

    AGMModel::SPtr newModel(new AGMModel(worldModel));

    // Symbolic part
    AGMModelSymbol::SPtr person = newModel->newSymbol("person");
    personSymbolId = person->identifier;
    printf("Got personSymbolId: %d\n", personSymbolId);
    person->setAttribute("imName", imName);
    person->setAttribute("imType", "transform");
    AGMModelSymbol::SPtr personSt = newModel->newSymbol("personSt" + std::to_string(id));
    printf("person %d status %d\n", person->identifier, personSt->identifier);

    newModel->addEdge(person, personSt, "hasStatus");
    newModel->addEdge(person, personSt, "noReach");
    newModel->addEdge(person, personSt, type);
    newModel->addEdgeByIdentifiers(person->identifier, 3, "in");

    qDebug()<<"Inserting person " <<id << " with pose "<<pose.x <<" "<< pose.z;
//    qFatal("--------------------------------------------------------------------");
    // Geometric part
    std::map<std::string, std::string> edgeRTAtrs;
    edgeRTAtrs["tx"] = std::to_string(pose.x);
    edgeRTAtrs["ty"] = "0";
    edgeRTAtrs["tz"] = std::to_string(pose.z);
    edgeRTAtrs["rx"] = "0";
    edgeRTAtrs["ry"] = std::to_string(pose.ry);
    edgeRTAtrs["rz"] = "0";
    newModel->addEdgeByIdentifiers(100, person->identifier, "RT", edgeRTAtrs);


    AGMModelSymbol::SPtr personMesh = newModel->newSymbol(meshname);
    printf("personMesh %d\n", personMesh->identifier);
    personMesh->setAttribute("collidable", "false");
    personMesh->setAttribute("imName", imName + "_Mesh");
    personMesh->setAttribute("imType", "mesh");
    std::string meshPath = "/home/robocomp/robocomp/components/robocomp-araceli/models/" +meshname;
    personMesh->setAttribute("path", meshPath);
    personMesh->setAttribute("render", "NormalRendering");
    personMesh->setAttribute("scalex", scale);
    personMesh->setAttribute("scaley", scale);
    personMesh->setAttribute("scalez", scale);

    edgeRTAtrs["tx"] = "0";
    edgeRTAtrs["ty"] = "0";
    edgeRTAtrs["tz"] = "0";
    edgeRTAtrs["rx"] = "1.570796326794";
    edgeRTAtrs["ry"] = "0";
    edgeRTAtrs["rz"] = rotationz;
    newModel->addEdge(person, personMesh, "RT", edgeRTAtrs);

    try{ sendModificationProposal(worldModel, newModel); }
    catch(...) {qDebug()<<"Error including the person in the AGM";}


    printf("includeInAGM ends\n");



}

void SpecificWorker::movePersonInAGM(int id,const Pose3D pose)
{
    std::string type = "person";
    std::string imName = "person" + std::to_string(id);
    int personSymbolId = -1;
    int idx=0;
    while ((personSymbolId = worldModel->getIdentifierByType(type, idx++)) != -1)
    {
        if (worldModel->getSymbolByIdentifier(personSymbolId)->getAttribute("imName") == imName)
            break;
    }
    //move in AGM
    AGMModelSymbol::SPtr personParent = worldModel->getParentByLink(personSymbolId, "RT");
    AGMModelEdge &edgeRT  = worldModel->getEdgeByIdentifiers(personParent->identifier, personSymbolId, "RT");

    if (pose.pos_good)
    {
//	    qDebug()<<"Moving person to "<< pose.x << "," <<pose.z;
        edgeRT.attributes["tx"] = float2str(pose.x);
        edgeRT.attributes["ty"] = "0";
        edgeRT.attributes["tz"] = float2str(pose.z);
    }

    if (pose.rot_good)
    {
//        qDebug()<<"Rotating person " <<pose.ry;
        edgeRT.attributes["rx"] = "0";
        edgeRT.attributes["ry"] = float2str(pose.ry);
        edgeRT.attributes["rz"] = "0";
    }

    if(!pose.pos_good and !pose.rot_good)
        return;

    try
    {
        AGMMisc::publishEdgeUpdate(edgeRT, agmexecutive_proxy);
    }
    catch(std::exception& e)
    {
        std::cout<<"Exception moving in AGM: "<<e.what()<<std::endl;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SpecificWorker::reloadConfigAgent()
{
//implementCODE
	return true;
}

bool SpecificWorker::activateAgent(const ParameterMap &prs)
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

bool SpecificWorker::setAgentParameters(const ParameterMap &prs)
{
//implementCODE
	bool activated = false;
	return setParametersAndPossibleActivation(prs, activated);
}

ParameterMap SpecificWorker::getAgentParameters()
{
//implementCODE
	return params;
}

void SpecificWorker::killAgent()
{
//implementCODE

}

int SpecificWorker::uptimeAgent()
{
//implementCODE
	return 0;
}

bool SpecificWorker::deactivateAgent()
{
//implementCODE
	return deactivate();
}

StateStruct SpecificWorker::getAgentState()
{
//implementCODE
	StateStruct s;
	if (isActive())
	{
		s.state = Running;
	}
	else
	{
		s.state = Stopped;
	}
	s.info = p.action.name;
	return s;
}

void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::World &w)
{
    qDebug()<<"structuralChange";
//subscribesToCODE
	QMutexLocker lockIM(mutex);
 	AGMModelConverter::fromIceToInternal(w, worldModel);
 
	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);

}

void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{
//subscribesToCODE
	QMutexLocker lockIM(mutex);
	for (auto modification : modifications)
	{
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
		AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel);
	}

}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
//subscribesToCODE
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel);

}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
//subscribesToCODE
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);

}

void SpecificWorker::symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications)
{
//subscribesToCODE
	QMutexLocker l(mutex);
	for (auto modification : modifications)
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);

}


bool SpecificWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated) {
	printf("<<< setParametersAndPossibleActivation\n");
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
	params.clear();
	for (ParameterMap::const_iterator it = prs.begin(); it != prs.end(); it++) {
		params[it->first] = it->second;
	}

	try {
		action = params["action"].value;
		std::transform(action.begin(), action.end(), action.begin(), ::tolower);
		//TYPE YOUR ACTION NAME
		if (action == "actionname") {
			active = true;
		} else {
			active = true;
		}
	}
	catch (...) {
		printf("exception in setParametersAndPossibleActivation %d\n", __LINE__);
		return false;
	}

	// Check if we should reactivate the component
	if (active) {
		active = true;
		reactivated = true;
	}

	printf("setParametersAndPossibleActivation >>>\n");

	return true;

}

bool SpecificWorker::sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel)
{
	bool result = false;
	try
	{	qDebug()<<"Intentando sendModificationProposal";
		AGMMisc::publishModification(newModel, agmexecutive_proxy, "HumanAgent");
		qDebug()<<"sendModificationProposal";
		result = true;
	}
	catch(const RoboCompAGMExecutive::Locked &e)
	{
		printf("agmexecutive locked...\n");
	}
	catch(const RoboCompAGMExecutive::OldModel &e)
	{
		printf("agmexecutive oldModel...\n");
	}
	catch(const RoboCompAGMExecutive::InvalidChange &e)
	{
		printf("agmexecutive InvalidChange...\n");
	}
	catch(const Ice::Exception& e)
	{
		exit(1);
	}
	return result;
}
