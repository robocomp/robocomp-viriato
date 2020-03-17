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

    forcesSliderChanged();
    moveRobot();


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

    navigation.initialize(innerModel, viewer, confParams, omnirobot_proxy);

    qDebug()<<"Classes initialized correctly";

    this->Period = period;
    timer.start(period);


    emit this->t_initialize_to_compute();

}

void SpecificWorker::compute()
{
//    qDebug()<< __FUNCTION__;

//    static QTime reloj = QTime::currentTime();

    bool needsReplaning = false;

    if(personalSpacesChanged)
	{
        getPolylinesFromModel();
    	navigation.updatePersonalPolylines(intimate_seq, personal_seq, social_seq);
		personalSpacesChanged = false;
		needsReplaning = true;
	}

	if(affordancesChanged)
	{
    	navigation.updateAffordancesPolylines(objects_seq);
		affordancesChanged = false;
		needsReplaning = true;
	}

    QMutexLocker lockIM(mutex);

    RoboCompLaser::TLaserData laserData = updateLaser();
	navigation.update(laserData, needsReplaning);

    viewer->run();

//    qDebug()<< "Compute time " <<reloj.restart();

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

void SpecificWorker::getPolylinesFromModel()
{

	qDebug()<<__FUNCTION__;
    intimate_seq.clear();
    personal_seq.clear();
    social_seq.clear();


    auto personalSpaces = worldModel->getSymbolsByType("personalSpace");


    for( auto space : personalSpaces)
    {

        QString intimate = QString::fromStdString(space->getAttribute("intimate"));
        QString personal = QString::fromStdString(space->getAttribute("personal"));
        QString social = QString::fromStdString(space->getAttribute("social"));
        qDebug()<< intimate;
        qDebug()<< "-------";

        vector<QString> polylinesStr = {intimate,personal,social};
        SNGPolylineSeq intimateSeq, personalSeq, socialSeq;
        vector <SNGPolylineSeq> polylinesSeq {intimateSeq, personalSeq, socialSeq};

        for (auto&&[str, polyline] : iter::zip(polylinesStr, polylinesSeq))
        {
            for(auto pol: str.split(";;"))
            {
                if(pol.size() == 1)
                    continue;

                SNGPolyline intimatePol;

                for (auto pxz : pol.split(";"))
                {

                    SNGPoint2D point;
                    auto p = pxz.split(" ");

                    if (p.size() != 2)
                        continue;

                    point.x = std::stof(p[0].toStdString());
                    point.z = std::stof(p[1].toStdString());


                    qDebug()<< point.x << point.z;
                    qDebug()<< "-------";

                    intimatePol.push_back(point);
                }

                polyline.push_back(intimatePol);
            }

		}

		for(auto p: polylinesSeq[0])
			intimate_seq.push_back(p);
		for(auto p: polylinesSeq[1])
			personal_seq.push_back(p);
		for(auto p: polylinesSeq[2])
			social_seq.push_back(p);
    }


    qDebug()<< "END "<< __FUNCTION__ << intimate_seq.size();
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
        {
			navigation.stopMovingRobot = true;
        }
        else
		{

            navigation.moveRobot = false;
			navigation.stopMovingRobot = false;
		}

        autoMov_checkbox->setEnabled(false);

    }

}


void  SpecificWorker::checkRobotAutoMovState()
{
	qDebug()<<__FUNCTION__;

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

void SpecificWorker::RCISMousePicker_setPick(const Pick &myPick)
{
    navigation.newTarget(QPointF(myPick.x,myPick.z));
}

void SpecificWorker::SocialRules_objectsChanged(const SRObjectSeq &objectsAffordances)
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
    qDebug()<<__FUNCTION__;
	QMutexLocker lockIM(mutex);
	try { worldModel->addEdgeByIdentifiers(nodeid, nodeid, edgeType, attributes); } catch(...){ printf("Couldn't add an edge. Duplicate?\n"); }

	try {
		innerModel.reset(AGMInner::extractInnerModel(worldModel));

        viewer->reloadInnerModel(innerModel);
		navigation.updateInnerModel(innerModel);


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

//    if(!specificWorkerInitialized)
//    {
//        QMutexLocker lockIM(mutex);
//    }

    QMutexLocker lockIM(mutex);

	for (auto modification : modifications)
	{
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
		AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel.get());
	}


}

void SpecificWorker::AGMExecutiveTopic_structuralChange(const RoboCompAGMWorldModel::World &w)
{
	qDebug() << __FUNCTION__;


    QMutexLocker lockIM(mutex);


	try {
		AGMModelConverter::fromIceToInternal(w, worldModel);
		innerModel.reset(AGMInner::extractInnerModel(worldModel));

		viewer->reloadInnerModel(innerModel);
		navigation.updateInnerModel(innerModel);

	} catch(...) { qDebug()<<__FUNCTION__<<"Can't extract an InnerModel from the current model."; }


}

void SpecificWorker::AGMExecutiveTopic_symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
    qDebug()<< __FUNCTION__;

	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);

    if (modification.nodeType == "personalSpace")
    {
        personalSpacesChanged = true;
    }



}

void SpecificWorker::AGMExecutiveTopic_symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications)
{

    qDebug()<< __FUNCTION__;

	QMutexLocker l(mutex);

    for (auto modification : modifications)
    {
        AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);

        for(auto node : modifications)
        {
            if (node.nodeType == "personalSpace")
            {
                personalSpacesChanged = true;
            }

        }

    }

}



bool SpecificWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
//	qDebug() << __FUNCTION__;

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
	qDebug() << __FUNCTION__;

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
