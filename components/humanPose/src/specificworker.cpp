/*    This file is part of RoboComp
*
*    RoboComp is free software: you can redistribute it and/or modify
*
 *    Copyright (C)2018 by YOUR NAME HERE
 *
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

//#ifdef USE_QTGUI
//	innerModelViewer = NULL;
//	osgView = new OsgView(this);
//	osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;
//	osg::Vec3d eye(osg::Vec3(4000.,4000.,-1000.));
//	osg::Vec3d center(osg::Vec3(0.,0.,-0.));
//	osg::Vec3d up(osg::Vec3(0.,1.,0.));
//	tb->setHomePosition(eye, center, up, true);
//	tb->setByMatrix(osg::Matrixf::lookAt(eye,center,up));
//	osgView->setCameraManipulator(tb);
//#endif

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
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

    IDcamera = std::stoi(params["IDcamera"].value);

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


void SpecificWorker::getDataFromAstra()
{

    qDebug()<<"Getting data from Astra";
    list_of_humans.clear();


    try
    {
        PersonList users;
        humantracker_proxy-> getUsersList(users);

        bool facefound = true;

        if(users.size()== 0)
            return;

        //hay alguna persona
        for (auto p:users) //para insertar persona en el modelo tiene que haber cara y cabeza
		{

			Pose3D personpose;
			auto idjoint = p.first; //id esqueleto
			auto faces = facetracking_proxy-> getFaces(); //obtenemos las caras
			auto idperson = getIDgeneric(idjoint,faces);

            if (idperson == -1) { return; }

			jointListType joints_person = p.second.joints;
			getPoseRot(joints_person, personpose);

			if (facefound) //face found se comprueba al obtener el IDgenerico
			{
				personpose.confidence = personpose.confidence + 30;
				facefound = false;
			}

			PersonType person_detected;
			person_detected.id = idperson;
			person_detected.pos = personpose;

			list_of_humans.push_back(person_detected);
		}
    }

    catch(...)
    {
    }


    for(auto human : list_of_humans)
	{
    	qDebug()<<"HUMANO "<< human.id <<" situado en "<< human.pos.x << " " <<human.pos.z;
	}


}

int SpecificWorker::getIDgeneric(int idjoint, RoboCompFaceTracking::Faces faces)
{
    int idperson = -1;
    auto idcam = IDcamera * 100; //Los ids de las personas obtenidas con la camara 1 empezaran por 100
    auto ID = idcam + IDgeneric;

    if (IDjointface.find(idjoint) == IDjointface.end()) //Buscamos en el mapa si el id del esqueleto ya está relacionado con la cara. Si NO:
    {
        joint pointindepth = {};

        if ( humantracker_proxy->getJointDepthPosition(idjoint, "Head", pointindepth)) //obtenemos la posicion de la cabeza en profundidad
        {
			for (auto f:faces)
			{
				if (f.tracking)
				{
					auto rect = QRect(f.boundingbox.posx, f.boundingbox.posy, f.boundingbox.width,f.boundingbox.height ); //para cada cara comprobamos si la cabeza esta contenida en el bounding box de la cara
					if (rect.contains(QPoint(pointindepth[0],pointindepth[1]))) //Si es asi
					{
						if (IDfacegeneric.find(f.id) != IDfacegeneric.end()) //Buscamos en el mapa si el id de la cara ya esta relacionaada con el generico. Si es asi:
						{
							IDjointface[idjoint] = f.id; //Relacionamos el id del joint con el de la cara
						}

						else //Si no esta
						{
							//Antes de asignar generico a cara ,comprobamos si los joints ya tienen generico
							if (IDjointgeneric.find(idjoint) == IDjointgeneric.end()) //Si no hay jointgeneric
							{
								IDfacegeneric[f.id] = ID; //relacionamos id cara con id generico
								IDjointface[idjoint] = f.id; //relacionamos id joint con id cara

								IDjointgeneric[idjoint] = ID; //rel joint con generic

								idperson = IDgeneric; //el id de la persona sera el id generico
								IDgeneric++; //aumentamos en uno el id generico para que no coincidan
							}

							else
							{
								IDfacegeneric[f.id] = IDjointgeneric[idjoint];
								IDjointface[idjoint] = f.id;
								idperson = IDjointgeneric[idjoint];
							}
						}

						break;
					}
				}
				else if (IDfacegeneric.find(f.id) == IDfacegeneric.end())     //Se comprueba si el id de la cara está ya registrado aunque no esté siendo trackeada
					facefound = false;

			}
		}


        if ((faces.size() == 0) or !facefound) //insertar a la persona en el modelo con id person = id generic relacionando joint con generic
        {
            //NO HAY CARAS
            if (IDjointgeneric.find(idjoint) == IDjointgeneric.end())
            {
                IDjointgeneric[idjoint] = ID;
                idperson = ID;
                IDgeneric++;
                backwards = true;
            }

            else //ya existe la relacion joint-generic
                idperson = IDjointgeneric[idjoint];

        }

        facefound = true;
    }

    else //el id ya está registrado accedemos al id generico
    {
        int idface = IDjointface[idjoint];
        idperson = IDfacegeneric[idface];

    }

    return idperson;
}


bool SpecificWorker::getPoseRot (jointListType list, Pose3D &personpose) {

	//////////////////////////////////GETTING POSITION//////////////////////
	int countjoints = 0;
	float newposez = 0;
	float newposex = 0;
	float mediaz;
	float mediax;

	QString name_camera = "camera_astra" ;
//	QString name_camera = "camera_astra" + QString(IDcamera);

	vector<string> tronco = {"Head", "Neck", "ShoulderSpine", "MidSpine", "BaseSpine"};
	for (auto idjoint : tronco)
	{
		if (list.find(idjoint) != list.end()) // found
		{
			auto j = list[idjoint];
			if (j.size() == 3)
			{
				QVec jointinworld = innerModel->transform("world", QVec::vec3(-j[0],0,j[2]), name_camera);
//				qDebug()<<" Found "<<QString::fromStdString(idjoint) <<" x = " << jointinworld.x()<<  " z = " << jointinworld.z() ;
				newposex += jointinworld.x();
				newposez += jointinworld.z();

				countjoints++;
				personpose.confidence = personpose.confidence + 10; //la fiabilidad aumenta en 10 por cada joint del tronco encontrado
			}
		}
//        else qDebug() <<QString::fromStdString(idjoint) <<"Joint "<< QString::fromStdString(idjoint) <<" not found";
	}

	if (countjoints != 0 )
	{
//			qDebug()<<"Numero de joints "<< countjoints;
		mediax = newposex/countjoints;
		mediaz = newposez/countjoints;

		personpose.pos_good = true;
	}
	else
		return false;

	personpose.x = mediax;
	personpose.z = mediaz;


	//////////////////////////////GETTING ROTATION //////////////////////////////////////

	bool leftsh = false;
	bool rightsh = false;

	if ((list.find("LeftShoulder") != list.end()) and (list["LeftShoulder"].size()==3))
	{
		auto j = list["LeftShoulder"];
//        qDebug()<<" Found Left Shoulder" <<" x = " << -j[0]<<  " z = " << j[2] ;
		leftsh = true;

	}
	else {
//        qDebug()<<"LeftShoulder not found";
	}

	if ((list.find("RightShoulder") != list.end()) and (list["LeftShoulder"].size()==3))
	{
		auto j = list["RightShoulder"];
//        qDebug()<<" Found Right Shoulder" <<" x = " << -j[0]<<  " z = " << j[2] ;
		rightsh = true;

	}

//    else qDebug()<<"Right Shoulder not found";

	if (leftsh and rightsh)
	{

		auto l = list["LeftShoulder"];
		QVec joint_left = innerModel->transform("world", QVec::vec3(-l[0],0,l[2]), name_camera);

		auto r = list["RightShoulder"];
		QVec joint_right = innerModel->transform("world", QVec::vec3(-r[0],0,r[2]), name_camera);

		personpose.ry = 3.1415926535 - (atan2(joint_left.z()-joint_right.z(),joint_left.x() - joint_right.x()));


		if (backwards) //puede que este de espaldas.CAMBIAR SI NO ESTA LA CARA LO PONE DE ESPALDAS SI O SI
			personpose.ry = 1.57;

		//cambiar esto, solo vale para una cierta posición de la cámara
		personpose.rot_good = true;
		personpose.confidence = personpose.confidence + 20;
	}


	return true;
}



void SpecificWorker::compute()
{
	QMutexLocker locker(mutex);
	getDataFromAstra();

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
	regenerateInnerModelViewer();
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



void SpecificWorker::regenerateInnerModelViewer()
{
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
		AGMMisc::publishModification(newModel, agmexecutive_proxy, "HumanPose");
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
