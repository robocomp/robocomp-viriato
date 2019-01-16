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

//
//#ifdef USE_QTGUI
//	innerModelViewer = new InnerModelViewer (innerModel, "root", osgView->getRootGroup(), true);
//#endif
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

	if (position_correct)
	{
//	    qDebug()<<"Moving person to "<< pose.x << "," <<pose.z;
		edgeRT.attributes["tx"] = float2str(pose.x);
		edgeRT.attributes["ty"] = "0";
		edgeRT.attributes["tz"] = float2str(pose.z);
	}

	if (rotation_correct)
	{
//        qDebug()<<"Rotating person " <<pose.ry;
		edgeRT.attributes["rx"] = "0";
		edgeRT.attributes["ry"] = float2str(pose.ry);
		edgeRT.attributes["rz"] = "0";
	}

	if(!position_correct and !rotation_correct)
		return;

	try
	{
		AGMMisc::publishEdgeUpdate(edgeRT, agmexecutive_proxy);
	}
	catch(std::exception& e)
	{
		std::cout<<"Exception moving in AGM: "<<e.what()<<std::endl;
	}

	position_correct = false;
	rotation_correct = false;

}


void SpecificWorker::getHumans()
{

	for (int cam = 1; cam < numcameras; cam++)
	{
		try
		{
			PersonList users;
			Faces faces;

			if (cam == 1)
			{
				humantracker1_proxy->getUsersList(users);
				auto faces = facetracking3_proxy-> getFaces();
			}

			if (cam == 2)
			{
				humantracker2_proxy->getUsersList(users);
				auto faces = facetracking4_proxy-> getFaces();
			}

			if(users.size() == 0)
				return;

			for (auto p:users) //para insertar persona en el modelo tiene que haber cara y cabeza
			{
				auto idjoint = p.first; //id esqueleto
				auto idperson = getIDgeneric(idjoint,faces); //modificar para decir qué camara toma los datos (para el transform y el faceTracking)

				if (idperson == -1)
					return;

				//Una vez que se tiene el id de la persona se calcula su posición y rotación
				Pose3D personpose;
				jointListType joints_person = p.second.joints;
				getPoseRot(joints_person, personpose);

				if (cam == 1)
					humans_cam1[idperson] = personpose;
				if (cam == 2)
					humans_cam2[idperson] = personpose;
			}
		}


		catch(...)
		{
			qDebug()<<"Can't connect to camera ", cam;
		}

	}




               PersonList users2;
        humantracker2_proxy-> getUsersList(users2);





}

RoboCompHumanTracker::PersonList SpecificWorker::mixData(RoboCompHumanTracker::PersonList users1, RoboCompHumanTracker::PersonList users2){
    PersonList users;

    if (!(users1.size()== 0) and (users2.size()==0))
        return users1;

    else if((users1.size()== 0) and !(users2.size()==0))
        return users2;

	for (auto p1 : users1)
	{
		Pose3D personpose1;
		auto joints_person1 = p1.second.joints;

		if (getPoseRot(joints_person1, personpose1))
		{
			for (auto p2 : users2)
			{
				Pose3D personpose2;
				auto joints_person2 = p2.second.joints;

				if (getPoseRot(joints_person2, personpose2))
				{
					auto dist = sqrt(((personpose2.x - personpose1.x)*(personpose2.x - personpose1.x))+(personpose2.z - personpose1.z)*(personpose2.z - personpose1.z));

					if (dist > 1000) //BUSCAR UNA DISTANCIA A LA QUE SEAN LA MISMA PERSONA
					{
					    //buscar otra forma de hacerlo que esto es muy feo
						users[p1.first] = p1.second;
						users[p2.first] = p2.second;

						//MAAAAAAAAAAL
						//solo insertar la persona cuando se compruebe que no coincide con ninguna otra 
					}

					else
					{
						qDebug()<<" MISMA PERSONA";
						//cual inserto
					}


				}
			}

		}

	}
        return users;

}

int SpecificWorker::getIDgeneric(int idjoint , RoboCompFaceTracking::Faces faces){

    bool facefound = true;
    int idperson = -1;


    if (IDjointface.find(idjoint) == IDjointface.end()) //Buscamos en el mapa si el id del esqueleto ya está relacionado con la cara. Si NO:
    {
        joint pointindepth = {};

        if (humantracker1_proxy->getJointDepthPosition(idjoint, "Head", pointindepth))//obtenemos la posicion de la cabeza en profundidad
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
                                IDfacegeneric[f.id] = IDgeneric; //relacionamos id cara con id generico
                                IDjointface[idjoint] = f.id; //relacionamos id joint con id cara
                                IDjointgeneric[idjoint] = IDgeneric; //rel joint con generic
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
                IDjointgeneric[idjoint] = IDgeneric;
                idperson = IDgeneric;
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
    
    vector<string> tronco = {"Head", "Neck", "ShoulderSpine", "MidSpine", "BaseSpine"};
    for (auto idjoint : tronco)
    {
        if (list.find(idjoint) != list.end()) // found
        {
			auto j = list[idjoint];
            if (j.size() == 3)
            {
                QVec jointinworld = innerModel->transform("world", QVec::vec3(-j[0],0,j[2]), "camera_astra");
//				qDebug()<<" Found "<<QString::fromStdString(idjoint) <<" x = " << jointinworld.x()<<  " z = " << jointinworld.z() ;
				newposex += jointinworld.x();
				newposez += jointinworld.z();

				countjoints++;
			}
		}
//        else qDebug() <<QString::fromStdString(idjoint) <<"Joint "<< QString::fromStdString(idjoint) <<" not found";
    }

		if (countjoints != 0 )
		{
//			qDebug()<<"Numero de joints "<< countjoints;
			mediax = newposex/countjoints;
			mediaz = newposez/countjoints;

			position_correct = true;
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
        QVec joint_left = innerModel->transform("world", QVec::vec3(-l[0],0,l[2]), "camera_astra");

        auto r = list["RightShoulder"];
        QVec joint_right = innerModel->transform("world", QVec::vec3(-r[0],0,r[2]), "camera_astra");

		personpose.ry = 3.1415926535 - (atan2(joint_left.z()-joint_right.z(),joint_left.x() - joint_right.x()));


        if (backwards) //puede que este de espaldas.CAMBIAR SI NO ESTA LA CARA LO PONE DE ESPALDAS SI O SI
            personpose.ry = 1.57;

        //cambiar esto, solo vale para una cierta posición de la cámara
        rotation_correct = true;
    }

//
//	if (leftsh and !rightsh)
//	{
//        personpose.ry = 0;
//        rotation_correct = true;
//    }
//    //angulo de la camara - pi/2
//    if (!leftsh and rightsh)
//	{
//        personpose.ry = 3.1415926535;
//        rotation_correct = true;
//    }
    //angulo de la camara + pi/2

	return true;
}

void SpecificWorker::compute()
{
	QMutexLocker locker(mutex);

	getHumans();

//#ifdef USE_QTGUI
//    if (innerModelViewer) innerModelViewer->update();
//    osgView->frame();
//#endif
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
//	if (innerModelViewer)
//	{
//		osgView->getRootGroup()->removeChild(innerModelViewer);
//	}
//
//	innerModelViewer = new InnerModelViewer(innerModel, "root", osgView->getRootGroup(), true);
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
