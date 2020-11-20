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
SpecificWorker::SpecificWorker(MapPrx& mprx,bool startup_check) : GenericWorker(mprx)
{

	active = false;
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
    innerModel = std::make_shared<InnerModel>();
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
    confParams  = std::make_shared<RoboCompCommonBehavior::ParameterList>(params);

	defaultMachine.start();

	return true;
}

void SpecificWorker::initialize(int period)
{
    QMutexLocker lockIM(mutex);

    std::cout << "Initialize worker" << std::endl;

    connect(autoMov_checkbox, SIGNAL(clicked()),this, SLOT(checkRobotAutoMovState()));
    connect(robotMov_checkbox, SIGNAL(clicked()),this, SLOT(moveRobot()));

    connect(ki_slider, SIGNAL (valueChanged(int)),this,SLOT(forcesSliderChanged(int)));
    connect(ke_slider, SIGNAL (valueChanged(int)),this,SLOT(forcesSliderChanged(int)));

    connect(send_button, SIGNAL(clicked()),this, SLOT(sendRobotTo()));
    connect(stopRobot_button, SIGNAL(clicked()),this, SLOT(stopRobot()));


    forcesSliderChanged();
    moveRobot();
    robotBlocked = false;

#ifdef USE_QTGUI
	viewer = std::make_shared<InnerViewer>(innerModel, "Social Navigation");  //InnerViewer copies internally innerModel so it has to be resynchronized
#endif

    try
    {
        RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
        AGMExecutiveTopic_structuralChange(w);
        robotID = worldModel->getIdentifierByType("robot");


    }
    catch(...)
    {
        printf("The executive is probably not running, waiting for first AGM model publication...");
    }

    navigation.initialize(innerModel, viewer, confParams, omnirobot_proxy);
    actionExecution.initialize(worldModel);



    qDebug()<<"Classes initialized correctly";

    getPeopleBlocking();

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
//    qDebug()<< __FUNCTION__;

//    static QTime reloj = QTime::currentTime();


    QMutexLocker lockIM(mutex);

    bool needsReplaning = false;

    if(personalSpacesChanged)
	{
        qDebug()<<"personalSpacesChanged";
    	getPersonsFromModel();
        auto [intimate,personal,social] = getPolylinesFromModel();
    	navigation.updatePersonalPolylines(intimate,personal,social);
		personalSpacesChanged = false;
		needsReplaning = true;
	}

	if(affordancesChanged)
	{
        qDebug()<<"affordancesChanged";

        auto [mapCostObjects, totalAffordances, blockedAffordances] = getAffordancesFromModel();
        navigation.updateAffordancesPolylines(mapCostObjects,totalAffordances,blockedAffordances);
		affordancesChanged = false;
		needsReplaning = true;
	}

    RoboCompLaser::TLaserData laserData = updateLaser();
	navigation.update(totalPersons, laserData, needsReplaning);
    viewer->run();


    if (active)
    {
        auto list_actions = {"changeroom","gotoperson","gotoaffordance","gotogroupofpeople"};
        if(std::find(std::begin(list_actions), std::end(list_actions), action) == std::end(list_actions))
            stopRobot();

        auto [newTarget, target] = actionExecution.runActions(action,params);
        if (newTarget)
        {
            qDebug()<< " ------- ACTION EXECUTION NEW TARGET------- "<< QString::fromStdString(action) << target;
            bool reachable = navigation.newTarget(target);

//            int searchedPoints = 0;
//
//            if (action == "changeroom")
//            {
//                while(!reachable and searchedPoints<10){
//                    auto [_, target_] = actionExecution.runActions(action,params);
//                    reachable = navigation.newTarget(target_);
//                    searchedPoints++;
//                }
//
//            }

            if (!reachable)
                qDebug()<< "Can't reach new target ";

        }

        if(robotBlockedInAGM)
            checkRobotBlock();
    }

    if (!totalPersons.empty())
        checkHumanBlock();
}

//Check if the robot is still blocked
void SpecificWorker::checkRobotBlock()
{
//    qDebug()<<__FUNCTION__ << QString::fromStdString(actionBlocked) <<  paramsBlocked;
    auto [newTarget, target] = actionExecution.runActions(actionBlocked,paramsBlocked, true);

    if (newTarget)
        navigation.newTarget(target);

    if(navigation.isPointVisitable(target))
    {
        qDebug()<<__FUNCTION__<< "---- ROBOT NOT BLOCKED ----";
        robotBlocked = false;
        actionBlocked = "";
        planBlocked = "";
        paramsBlocked.clear();
    }

}

void SpecificWorker::checkHumanBlock()
{
	QMutexLocker lockIM(mutex);
    static bool first = true;

	newModel = AGMModel::SPtr(new AGMModel(worldModel));

	bool edgesChanged = false;
    string edgeName;
    string currentEdge;

	auto blockingIDs = navigation.blockIDs;
    auto affBlockingIDs = navigation.affBlockIDs;



//    if((prev_blockingIDs == blockingIDs)  and (prev_affBlockingIDs == affBlockingIDs) and robotBlocked)
//        return;

//    qDebug()<< "blocking - prev: " << prev_blockingIDs << " current: " << blockingIDs;
//    qDebug()<< "aff blocking - prev: " << prev_affBlockingIDs << " current: " << affBlockingIDs;
//    qDebug()<< "prev_blockingIDs == blockingIDs " <<(prev_blockingIDs == blockingIDs);
//    qDebug()<< "prev_affBlockingIDs == affBlockingIDs" <<(prev_affBlockingIDs == affBlockingIDs);
//    qDebug()<< "robotBlocked " <<robotBlocked;


    ////////////////////////// human Block ////////////////////////
    edgeName = "is_blocking";

    if((!robotBlocked)
        or ( robotBlocked and (!blockingIDs.empty() or !affBlockingIDs.empty())))
    {

        for (auto id: prev_blockingIDs)
        {
            //no se borra si se va añadir despues
            if(std::find(std::begin(blockingIDs), std::end(blockingIDs), id) != std::end(blockingIDs))
                continue;

            if(removeEdgeModel(id,robotID,edgeName))
                edgesChanged = true;
        }
    }

    for(auto id: blockingIDs)
    {
        if (addEdgeModel(id,robotID,edgeName))
        {
            edgesChanged = true;
        }
    }

    ////////////////////////// affordanceBlock ////////////////////////
    edgeName = "affordance_blocking";

    if((!robotBlocked)
       or (robotBlocked and (!affBlockingIDs.empty() or !blockingIDs.empty())))
    {
        for (auto id: prev_affBlockingIDs)
        {
            if(std::find(std::begin(affBlockingIDs), std::end(affBlockingIDs), id) != std::end(affBlockingIDs))
                continue;

            if(removeEdgeModel(id,robotID,edgeName))
                edgesChanged = true;
        }
    }

    for(auto id: affBlockingIDs)
    {
        if(addEdgeModel(id,robotID,edgeName))
        {
            edgesChanged = true;
        }
    }
     ////////////////////////////////////////////////////////////////////

    blockingEdgesInAGM = !blockingIDs.empty() or !affBlockingIDs.empty();

//    if  (!robotBlockedInAGM and (!blockingIDs.empty() or !affBlockingIDs.empty()))
    if  (!robotBlockedInAGM and blockingEdgesInAGM)
//    if  (blockingEdgesInAGM)
    {
        qDebug()<<" ----- Blocking robot ----- " << currentPlan;
        if(addEdgeModel(robotID,robotID,"blocked"))
        {
            edgesChanged = true;
        }

        robotBlockedInAGM = true;

        robotBlocked = true;

        if (planBlocked == "")
        {
            planBlocked = currentPlan;
            paramsBlocked = params;
            actionBlocked = action;
        }
    }

    else if ((!robotBlocked and robotBlockedInAGM) or first)
    {
        qDebug()<< "ROBOT NOT BLOCKED ---REMOVING BLOCKED EDGE";
        if(removeEdgeModel(robotID,robotID,"blocked"))
        {
            edgesChanged = true;
            robotBlockedInAGM = false;
        }

        auto totalPersonsAGM = worldModel->getSymbolsByType("person");
        for (auto person : totalPersonsAGM)
        {
            for (AGMModelSymbol::iterator edge = person->edgesBegin(worldModel);
                 edge!=person->edgesEnd(worldModel);
                 edge++) {
                auto id = person->identifier;
                auto edgeName = edge->getLabel();
                if ((edgeName=="is_blocking") or (edgeName=="affordance_blocking" )) {
                    if(removeEdgeModel(id,robotID,edgeName))
                        edgesChanged = true;
                }
            }
        }

        actionBlocked = "";
        planBlocked = "";
        paramsBlocked.clear();
        blockingEdgesInAGM = false;
    }


	if(edgesChanged){

		try
		{
		    static QTime reloj = QTime::currentTime();
            reloj.start();
            sendModificationProposal(worldModel, newModel);
            qDebug()<<"sendModificationProposal time "<< reloj.restart()/1000 << " seconds";
		}
		catch(...)
		{
			std::cout<<"No se puede actualizar worldModel"<<std::endl;
		}
	}

    prev_blockingIDs = blockingIDs;
    prev_affBlockingIDs = affBlockingIDs;
    first = false;


}

bool SpecificWorker::removeEdgeModel(int32_t id1, int32_t id2, string edgeName)
{
//    qDebug()<< __FUNCTION__ << id1 << id2 <<  QString::fromStdString(edgeName);

    try
    {
        newModel->removeEdgeByIdentifiers(id1, id2, edgeName);
        qDebug ()<<" Se elimina el enlace " << QString::fromStdString(edgeName) << " de " << id1;
        return true;
    }

    catch(...)
    {
//        std::cout<<__FUNCTION__<<" No existe el enlace " << edgeName  <<std::endl;
        return false;
    }

}

bool SpecificWorker::addEdgeModel(int32_t id1, int32_t id2, string edgeName)
{
    try
    {
        newModel->addEdgeByIdentifiers(id1, id2, edgeName);
        qDebug ()<<" Se añade el enlace " << QString::fromStdString(edgeName) << " de " << id1;
        return true;
    }

    catch(...)
    {
//        std::cout<<__FUNCTION__<<" Ya existe el enlace " << edgeName <<std::endl;
        return false;
    }
}


RoboCompLaser::TLaserData  SpecificWorker::updateLaser()
{
//	qDebug()<<__FUNCTION__;

	RoboCompLaser::TLaserData laserData;

    try
    {
		laserData  = laser_proxy->getLaserData();
    }

    catch(const Ice::Exception &e){ std::cout <<"Can't connect to laser --" <<e.what() << std::endl; };

    return laserData;
}


void SpecificWorker::getPersonsFromModel()
{
	totalPersons.clear();
	auto vectorPersons = worldModel->getSymbolsByType("person");

	for (auto p: vectorPersons) {
		localPerson person;

		auto id = p->identifier;
		AGMModelSymbol::SPtr personParent = worldModel->getParentByLink(id, "RT");
		AGMModelEdge& edgeRT = worldModel->getEdgeByIdentifiers(personParent->identifier, id, "RT");

		person.id = id;
		person.x = str2float(edgeRT.attributes["tx"]);
		person.z = str2float(edgeRT.attributes["tz"]);
		person.angle = str2float(edgeRT.attributes["ry"]);

		totalPersons.push_back(person);
	}
}

void SpecificWorker::getPeopleBlocking()
{
    auto vectorPersons = worldModel->getSymbolsByType("person");
    for (auto person : vectorPersons)
    {

        for (AGMModelSymbol::iterator edge = person->edgesBegin(worldModel);
             edge!=person->edgesEnd(worldModel);
             edge++) {
            if (edge->getLabel()=="is_blocking") {
                const std::pair<int32_t, int32_t> symbolPair = edge->getSymbolPair();
                prev_blockingIDs.push_back(symbolPair.first);
                edgeInModel = "is_blocking";
            }
        }
    }

}


SpecificWorker::retPersonalSpaces SpecificWorker::getPolylinesFromModel()
{
	qDebug()<<__FUNCTION__;

    vector<QPolygonF> intimatePolygon;
    vector<QPolygonF> personalPolygon;
    vector<QPolygonF> socialPolygon;

    vector <vector<QPolygonF>> polylinesSeq {intimatePolygon, personalPolygon, socialPolygon};

    auto peopleAGM = worldModel->getSymbolsByType("person");
    vector<int> IDsAlreadyIncluded;

    for( auto person : peopleAGM)
    {
        int32_t owner = person->identifier;
        QString intimate = QString::fromStdString(person->getAttribute("polyline_intimate"));
        QString personal = QString::fromStdString(person->getAttribute("polyline_personal"));
        QString social = QString::fromStdString(person->getAttribute("polyline_social"));
        QString sharedWith = QString::fromStdString(person->getAttribute("polyline_sharedWith"));

        vector<int> sharedWithIDs;

        for (auto sh : sharedWith.split(" ")) {
            if (sh.toInt() != 0)
                sharedWithIDs.push_back(sh.toInt());
        }

        if(!sharedWithIDs.empty())
        {
//            qDebug()<< "Searching for " << owner;
            if( std::find(std::begin(IDsAlreadyIncluded), std::end(IDsAlreadyIncluded), owner) != std::end(IDsAlreadyIncluded))
                continue;
        }


        vector<QString> polylinesStr = {intimate,personal,social};

        for (auto &&[str, polygonSeq] : iter::zip(polylinesStr, polylinesSeq))
        {
            for(auto pol: str.split(";;"))
            {
                if(pol.size() == 0)
                    continue;

                QPolygonF polygon;

                for (auto pxz : pol.split(";"))
                {
                    auto p = pxz.split(" ");

                    if (p.size() != 2)
                        continue;

                    auto x = std::stof(p[0].toStdString());
                    auto z = std::stof(p[1].toStdString());

                    polygon << QPointF(x,z);
                }

                polygonSeq.push_back(polygon);
            }
		}


		for(auto id : sharedWithIDs) {
            if (std::find(std::begin(IDsAlreadyIncluded), std::end(IDsAlreadyIncluded), id)==std::end(IDsAlreadyIncluded)) {
                IDsAlreadyIncluded.push_back(id);
            }
        }


    }
    return std::make_tuple(polylinesSeq[0],polylinesSeq[1],polylinesSeq[2]);

}

SpecificWorker::retAffordanceSpaces SpecificWorker::getAffordancesFromModel()
{

    qDebug()<<__FUNCTION__;

    vector<QPolygonF> totalAffordances;
    vector<QPolygonF> blockedAffordances;
    std::map<float,vector<QPolygonF>> mapCostObjects;

    auto objectsAGM = worldModel->getSymbolsByType("object");


    for( auto objectAGM : objectsAGM)
    {
        bool interactive = false;
        for (AGMModelSymbol::iterator edge = objectAGM->edgesBegin(worldModel);
             edge!=objectAGM->edgesEnd(worldModel);
             edge++)
        {
            if (edge->getLabel()=="interactive") {
                interactive = true;
            }
        }

        if(!interactive)
            continue;

        QPolygonF object;

        QString polyline = QString::fromStdString(objectAGM->getAttribute("polyline_affordance"));
        float cost = std::stof(objectAGM->getAttribute("cost"));
        bool interacting = (objectAGM->getAttribute("interacting") == "1");

        for(auto pol: polyline.split(";;"))
        {
            if(pol.size() == 0)
                continue;

            for (auto pxz : pol.split(";"))
            {
                auto p = pxz.split(" ");

                if (p.size() != 2)
                    continue;

                auto x = std::stof(p[0].toStdString());
                auto z = std::stof(p[1].toStdString());

                object<< QPointF(x,z);
            }

            mapCostObjects[cost].push_back(object);
            totalAffordances.push_back(object);
            if(interacting) blockedAffordances.push_back(object);

        }
    }
    qDebug()<<"END "<<__FUNCTION__;

    return std::make_tuple(mapCostObjects,totalAffordances,blockedAffordances);
}


void  SpecificWorker::moveRobot()
{
    qDebug()<<__FUNCTION__;

    if(robotMov_checkbox->checkState() == Qt::CheckState(2))
    {
        autoMov_checkbox->setEnabled(true);
        navigation.moveRobot = true;
		navigation.stopMovingRobot = false;
    }

    else
    {
        if(navigation.current_target.active.load())
			navigation.stopMovingRobot = true;

        else
		{
            navigation.moveRobot = false;
			navigation.stopMovingRobot = false;
		}

        autoMov_checkbox->setEnabled(false);

    }

}

void SpecificWorker::stopRobot()
{
    if (navigation.isCurrentTargetActive())
    {
        navigation.deactivateTarget();
        navigation.stopRobot();
    }

}

void  SpecificWorker::checkRobotAutoMovState()
{
//	qDebug()<<__FUNCTION__;

	if(autoMov_checkbox->checkState() == Qt::CheckState(2))
	{
		navigation.robotAutoMov = true;
		navigation.newRandomTarget();
	}

	else
    {
        navigation.robotAutoMov = false;
    }

}


void SpecificWorker::sendRobotTo()
{
    auto x =  x_spinbox->value();
    auto z =  z_spinbox->value();

    navigation.newTarget(QPointF(x,z));

}

void SpecificWorker::
forcesSliderChanged(int value)
{

    navigation.KI = (float) ki_slider -> sliderPosition();
    navigation.KE = (float) ke_slider -> sliderPosition();

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



////////////////////////// SUBSCRIPTIONS /////////////////////////////////////////////

void SpecificWorker::RCISMousePicker_setPick(const RoboCompRCISMousePicker::Pick &myPick)
{
    navigation.newTarget(QPointF(myPick.x,myPick.z));
}

void SpecificWorker::SocialRules_objectsChanged(const RoboCompSocialRules::SRObjectSeq &objectsAffordances)
{
    //subscribesToCODE
    qDebug() << __FUNCTION__ << objectsAffordances.size();

    objects_seq = objectsAffordances;

	affordancesChanged = true;

}

void SpecificWorker::SocialRules_personalSpacesChanged(const RoboCompSocialNavigationGaussian::SNGPolylineSeq &intimateSpaces, const RoboCompSocialNavigationGaussian::SNGPolylineSeq &personalSpaces, const RoboCompSocialNavigationGaussian::SNGPolylineSeq &socialSpaces)
{
    qDebug() << __FUNCTION__;

	intimate_seq = intimateSpaces;
	personal_seq = personalSpaces;
	social_seq = socialSpaces;

	personalSpacesChanged = true;
}

///////////////////////////////////////////////////////////////////////////////////////

bool SpecificWorker::AGMCommonBehavior_activateAgent(const RoboCompAGMCommonBehavior::ParameterMap &prs)
{
    qDebug()<<__FUNCTION__;

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

void SpecificWorker::AGMExecutiveTopic_selfEdgeAdded(const int nodeid, const string &edgeType, const RoboCompAGMWorldModel::StringDictionary &attributes)
{
    qDebug()<<__FUNCTION__;
	QMutexLocker lockIM(mutex);
	try { worldModel->addEdgeByIdentifiers(nodeid, nodeid, edgeType, attributes); } catch(...){ printf("Couldn't add an edge. Duplicate?\n"); }

	try {
		innerModel.reset(AGMInner::extractInnerModel(worldModel));

        viewer->reloadInnerModel(innerModel);
		navigation.updateInnerModel(innerModel);
        personalSpacesChanged = true;
        affordancesChanged = true;

	} catch(...) { printf("Can't extract an InnerModel from the current model.\n"); }



}

void SpecificWorker::AGMExecutiveTopic_selfEdgeDeleted(const int nodeid, const string &edgeType)
{
    qDebug()<<__FUNCTION__;

    QMutexLocker lockIM(mutex);
	try { worldModel->removeEdgeByIdentifiers(nodeid, nodeid, edgeType); } catch(...) { printf("Couldn't remove an edge\n"); }

	try {
        innerModel.reset(AGMInner::extractInnerModel(worldModel));

        viewer->reloadInnerModel(innerModel);
		navigation.updateInnerModel(innerModel);
        personalSpacesChanged = true;
        affordancesChanged = true;

    } catch(...) { printf("Can't extract an InnerModel from the current model.\n"); }
}


void SpecificWorker::AGMExecutiveTopic_edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{

    QMutexLocker lockIM(mutex);


    AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel.get());



}

void SpecificWorker::AGMExecutiveTopic_edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{
//subscribesToCODE

//	qDebug() << __FUNCTION__;

    QMutexLocker lockIM(mutex);

	for (auto modification : modifications)
	{
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
		AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel.get());
	}


}

void SpecificWorker::AGMExecutiveTopic_structuralChange(const RoboCompAGMWorldModel::World &w)
{
    qDebug()<<"-------------------------------"<< __FUNCTION__<< "-------------------------------";

    QMutexLocker lockIM(mutex);

	try {
		AGMModelConverter::fromIceToInternal(w, worldModel);
		innerModel.reset(AGMInner::extractInnerModel(worldModel));

		viewer->reloadInnerModel(innerModel);
		navigation.updateInnerModel(innerModel);
		actionExecution.updateWordModel(worldModel);

	} catch(...) { qDebug()<<__FUNCTION__<<"Can't extract an InnerModel from the current model."; }

	personalSpacesChanged = true;
	affordancesChanged = true;
    worldModelChanged = true;
    qDebug()<<"-------------------------------"<< __FUNCTION__<< "-------------------------------";

}

void SpecificWorker::AGMExecutiveTopic_symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
   // qDebug()<< __FUNCTION__;

	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);

    if (modification.nodeType == "person")
        personalSpacesChanged = true;

    if (modification.nodeType == "object")
        affordancesChanged = true;

    if(affordancesChanged or personalSpacesChanged)
        actionExecution.updateWordModel(worldModel);

}

void SpecificWorker::AGMExecutiveTopic_symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications)
{

   // qDebug()<< __FUNCTION__;

	QMutexLocker l(mutex);

    for (auto modification : modifications)
    {
        AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);

        for(auto node : modifications)
        {
            if (node.nodeType == "person")
                personalSpacesChanged = true;

            if (modification.nodeType == "object")
                affordancesChanged = true;
        }
    }
    if(affordancesChanged or personalSpacesChanged)
    {
        actionExecution.updateWordModel(worldModel);
    }
}

bool SpecificWorker::setParametersAndPossibleActivation(const RoboCompAGMCommonBehavior::ParameterMap &prs, bool &reactivated)
{

	printf("<<< ------------------------- setParametersAndPossibleActivation -------------------------\n");
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
	params.clear();
	for (RoboCompAGMCommonBehavior::ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
	{
		params[it->first] = it->second;
		qDebug()<< QString::fromStdString(it->first) << " " << QString::fromStdString(it->second.value);
	}

	vector<QString> plans;
	bool blockedPlanInPlans = false;

    QString planList = QString::fromStdString(params["plan"].value);

    for (auto plan : planList.split("\n"))
    {
        if(plan == "")
            continue;

        if(planBlocked == plan)
            blockedPlanInPlans = true;

        plans.push_back(plan);

    }
    if (!plans.empty())
        currentPlan = plans[0];

    robotBlocked = blockedPlanInPlans;
    qDebug()<< "robotblocked = "<< robotBlocked;
    qDebug()<< "Blocked plan "<< planBlocked;
    qDebug()<<"Current plan "<< currentPlan;


    try
	{
		action = params["action"].value;
		std::transform(action.begin(), action.end(), action.begin(), ::tolower);
		//TYPE YOUR ACTION NAME
		if (action != "none")
		{
			active = true;
		}
		else
		{
		    qDebug()<< " Action none --- END MISSION ---";
            active = false;

            robotBlocked = false;
            actionBlocked = action;
            currentPlan = "none";
            planBlocked = "";
            paramsBlocked.clear();
            actionExecution.prevRoomTarget ="";
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

        actionExecution.update(action,params);

    }

	printf("<<< ------------------------- setParametersAndPossibleActivation -------------------------\n");

	return true;
}

void SpecificWorker::sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel)
{
    qDebug()<<__FUNCTION__;
    QMutexLocker locker(mutex);

    try
    {
        AGMMisc::publishModification(newModel, agmexecutive_proxy, std::string( "SocialnavigationAgent"));
    }
    catch(const RoboCompAGMExecutive::Locked &e)
    {
        printf("modelo bloqueado\n");

    }
    catch(const RoboCompAGMExecutive::OldModel &e)
    {
        printf("modelo viejo\n");
    }
    catch(const RoboCompAGMExecutive::InvalidChange &e)
    {
        printf("modelo invalido\n");
    }
    catch(const Ice::Exception& e)
    {
        printf("sendModificationProposal --- unknown exception \n");
//        exit(1);
    }
}
