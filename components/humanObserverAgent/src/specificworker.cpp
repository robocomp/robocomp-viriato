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
	try
	{
		RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
		AGMExecutiveTopic_structuralChange(w);
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
	this->Period = 1000;
	timer.start(Period);

    previousPersonsList.clear();

    emit this->t_initialize_to_compute();


}

void SpecificWorker::compute()
{
    QMutexLocker l(mutex);

//    qDebug()<<"worldModelchanged:"<<worldModelChanged;
    //solo queremos mirar las personas cuando otro agente modifica el AGM. Si somos nosotros quien lo modifica no volvemos a observar las personas.
    if (worldModelChanged and !ourModelChanged)
    {
    	loadInfoFromAGM();
    }
    else if (worldModelChanged) {
        ourModelChanged = false;
    }
    worldModelChanged = false;

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


void SpecificWorker::loadInfoFromAGM()
{
	totalPersons.clear();
	totalObjects.clear();

	auto vectorPersons = worldModel->getSymbolsByType("person");

    for (auto p: vectorPersons)
    {
        PersonType person;
//        std::cout<<"Person type " <<p->symbolType <<" Person Identifier: "<< p->identifier<<"\n";

        auto id =  p->identifier;
        AGMModelSymbol::SPtr personParent = worldModel->getParentByLink(id, "RT");
        AGMModelEdge &edgeRT = worldModel->getEdgeByIdentifiers(personParent->identifier, id, "RT");

        person.id = id;
       	person.imName = QString::fromStdString( p->getAttribute("imName"));
       	person.x = str2float(edgeRT.attributes["tx"]);
        person.z = str2float(edgeRT.attributes["tz"]);
		person.rot = str2float(edgeRT.attributes["ry"]);

		cout<< "PERSON " <<person.id<< " x = "<<person.x <<" z = "<<person.z <<" rot "<<person.rot <<endl;
        totalPersons.push_back(person);
    }

	auto vectorObjects = worldModel->getSymbolsByType("object");
    for (auto obj : vectorObjects)
	{
		ObjectType object;
		auto id =  obj->identifier;


		try
		{
			worldModel->getEdge(obj,obj,"interactive");
		}

		catch(...)
		{
			//El objeto no es interactivo
			continue;
		}


		AGMModelSymbol::SPtr objectParent = worldModel->getParentByLink(id, "RT");
		AGMModelEdge &edgeRT  = worldModel->getEdgeByIdentifiers(objectParent->identifier,id, "RT");
		object.id = id;
		object.imName = QString::fromStdString( obj->getAttribute("imName"));
		object.x = str2float(edgeRT.attributes["tx"]);
		object.z = str2float(edgeRT.attributes["tz"]);
		object.rot=str2float(edgeRT.attributes["ry"]);
        object.width=str2float(worldModel->getSymbolByIdentifier(id)->getAttribute("width"));
        object.height=str2float(worldModel->getSymbolByIdentifier(id)->getAttribute("height"));
        object.depth=str2float(worldModel->getSymbolByIdentifier(id)->getAttribute("depth"));		object.inter_space=str2float(worldModel->getSymbolByIdentifier(id)->getAttribute("inter_space"));
		object.inter_angle=str2float(worldModel->getSymbolByIdentifier(id)->getAttribute("inter_angle"));

        object.shape = QString::fromStdString(worldModel->getSymbolByIdentifier(id)->getAttribute("shape"));

        cout<< "OBJECT - interaction space " <<object.inter_space << "mm"<< " x = "<<object.x <<" z = "<<object.z <<" rot "<<object.rot << endl;

		totalObjects.push_back(object);

	}

	qDebug()<<"interaction objects "<< totalObjects.size();


	newModel = AGMModel::SPtr(new AGMModel(worldModel));

	if (checkHumanInteraction() or checkObjectInteraction())
    {
        try
        {
            qDebug()<<"Trying to update worldModel";
            sendModificationProposal(worldModel,newModel);
            ourModelChanged = true;
        }
        catch(...) { std::cout<<"No se puede actualizar worldModel"<<std::endl; }
    }
}


bool SpecificWorker::checkHumanInteraction()
{
    std::cout<<"Entered checkHumanInteraction"<<std::endl;

    bool changeInEdges = false;

    for (int i=0; i<totalPersons.size(); i++)
    {
        for (int j=0; j<totalPersons.size(); j++)
        {
            if (i==j) { continue; } // no calculamos la interacción de una persona consigo misma

            cout<< "Checking interaction between "<< totalPersons[i].id << " and " << totalPersons[j].id<<endl;

            QVec pose1from2 = innerModel->transform(totalPersons[i].imName ,totalPersons[j].imName);
            auto angle1 = atan2 (pose1from2.x(),pose1from2.z());

            QVec pose2from1 = innerModel->transform(totalPersons[j].imName ,totalPersons[i].imName );
            auto angle2 = atan2 (pose2from1.x(),pose2from1.z());

            QVec VI = QVec::vec2((totalPersons[i].x -totalPersons[j].x),(totalPersons[i].z -totalPersons[j].z));
            qDebug()<< "Distancia " << VI.norm2();

            if(abs(angle1) < thr_angle_humans and (abs(angle2) < thr_angle_humans) and (VI.norm2() < threshold_dist))
            {
                qDebug()<<totalPersons[i].id<< "and"<<totalPersons[j].id<< "INTERACTING";

                try
                {
                    newModel->addEdgeByIdentifiers(totalPersons[i].id, totalPersons[j].id, "interacting");
                    newModel->addEdgeByIdentifiers(totalPersons[j].id, totalPersons[i].id, "interacting");
                    changeInEdges = true;
                }

                catch(...) { std::cout<<__FUNCTION__<<" Ya existe el enlace"<<std::endl; }
            }

            else
            {
                try
                {
                    newModel->removeEdgeByIdentifiers(totalPersons[i].id, totalPersons[j].id, "interacting");
                    newModel->removeEdgeByIdentifiers(totalPersons[j].id, totalPersons[i].id, "interacting");
                    changeInEdges = true;
                }

                catch(...) { std::cout<<__FUNCTION__<<" No existe el enlace"<<std::endl; }
            }

        }
    }

    return changeInEdges;
}

bool SpecificWorker::checkObjectInteraction()
{
    cout<<"Entered checkObjectInteraction"<<endl;

    bool changeInEdges = false;

    for (auto person : totalPersons)
	{
		for (auto object : totalObjects)
		{
            QVec VI = QVec::vec2((person.x - object.x),(person.z -object.z));
            QPolygonF affordance;
            if (object.shape == "trapezoid")
                affordance = affordanceTrapezoidal(object);

            if (object.shape == "circle")
                affordance = affordanceCircular(object);

            if (object.shape == "rectangle")
                affordance = affordanceRectangular(object);

//            auto affordance = calculateAffordance(object);

            bool inside_affordance = affordance.containsPoint(QPointF(person.x,person.z),Qt::OddEvenFill);

            if(inside_affordance) qDebug() <<"Person " << person.id << " is inside the affordance of object " << object.imName;

            QVec pose2from1 = innerModel->transform6D(person.imName ,object.imName );
            pose2from1.print("Object from person");
            auto angle = atan2 (pose2from1.x(),pose2from1.z());
            qDebug()<<"Angle of view "<< angle;


            if(inside_affordance and (abs(angle) < thr_angle_humans)) 
			{
            	qDebug()<<"OBJECT INTERACTION";
				try
				{
					newModel->addEdgeByIdentifiers(person.id, object.id, "interacting");
                    changeInEdges = true;
                }

				catch(...) { std::cout<<__FUNCTION__<<" Ya existe el enlace"<<std::endl; }
			}

            else
            {
                try
                {
                    newModel->removeEdgeByIdentifiers(person.id, object.id, "interacting");
                    changeInEdges = true;
                }
                catch(...) { std::cout<<__FUNCTION__<<" No existe el enlace"<<std::endl; }
            }
        }
	}

	return changeInEdges;
}

QPolygonF SpecificWorker::affordanceTrapezoidal(ObjectType obj)
{
    cout << "Entered calculateAffordance"<<endl;
    QPolygonF aff_qp;


    auto left_angle = obj.rot + obj.inter_angle/2;
    auto right_angle = obj.rot - obj.inter_angle/2;

    auto point_x_left = obj.x + obj.inter_space*(cos(M_PI_2 - left_angle));
    auto point_z_left = obj.z + obj.inter_space*(sin( M_PI_2 - left_angle));

    auto point_x_right = obj.x + obj.inter_space*(cos(M_PI_2 - right_angle));
    auto point_z_right = obj.z + obj.inter_space*(sin(M_PI_2 - right_angle));

    aff_qp << QPointF(obj.x - obj.width/2,obj.z);
    aff_qp << QPointF(obj.x + obj.width/2,obj.z);
    aff_qp << QPointF(point_x_left,point_z_left);
    aff_qp << QPointF(point_x_right,point_z_right);

   qDebug()<<aff_qp;

    return aff_qp;

}


QPolygonF SpecificWorker::affordanceRectangular(ObjectType obj)
{
    qDebug()<< __FUNCTION__;

    QPolygonF aff_qp;

    aff_qp << QPointF (obj.x - obj.width/2 - obj.inter_space , obj.z - obj.depth/2 -obj.inter_space) ;

    aff_qp << QPointF ( obj.x + obj.width/2 + obj.inter_space, obj.z - obj.depth/2 -obj.inter_space) ;

    aff_qp << QPointF ( obj.x + obj.width/2 + obj.inter_space, obj.z + obj.depth/2 +obj.inter_space) ;

    aff_qp << QPointF (obj.x - obj.width/2 - obj.inter_space, obj.z + obj.depth/2 + obj.inter_space) ;


    return aff_qp;

}

QPolygonF SpecificWorker::affordanceCircular(ObjectType obj)
{
    qDebug()<< __FUNCTION__;

    QPolygonF aff_qp;

    int points = 50;

    float angle_shift = M_PI*2 / points, phi = 0;

    for (int i = 0; i < points; ++i) {
        phi += angle_shift;
        aff_qp << QPointF ( obj.x + ((obj.width/2 + obj.inter_space)*sin(phi)),obj.z +  ((obj.depth/2 + obj.inter_space)*cos(phi)) );
    }


    return aff_qp;

}




////////////////////////////////////////////////////////////////////////////////////////

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
    worldModelChanged = true;

}

void SpecificWorker::AGMExecutiveTopic_edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{
//subscribesToCODE
	QMutexLocker lockIM(mutex);
	for (auto modification : modifications)
	{
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
		AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel.get());
	}

    worldModelChanged = true;
}

void SpecificWorker::AGMExecutiveTopic_structuralChange(const RoboCompAGMWorldModel::World &w)
{
//subscribesToCODE
	QMutexLocker lockIM(mutex);
 	AGMModelConverter::fromIceToInternal(w, worldModel);
 
	innerModel = std::make_shared<InnerModel>(AGMInner::extractInnerModel(worldModel));

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
		AGMMisc::publishModification(newModel, agmexecutive_proxy, "humanObserverAgentAgent");
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
