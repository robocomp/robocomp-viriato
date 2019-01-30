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
		IDcamera = std::stoi(params["IDcamera"].value);

		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		std::string innermodel_path = par.value;
		innerModel = new InnerModel(innermodel_path);
	}
	catch(std::exception e) { qFatal("Error reading config params"); }


    Period = 300;
    timer.start(Period);

	return true;
}


void SpecificWorker::getDataFromAstra()
{
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

//			qDebug()<<"ID DE LA PERSONA "<< idperson;

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
			person_detected.IDcamera = IDcamera;


			list_of_humans.push_back(person_detected);

		}

		try
		{
			humanpose_proxy-> obtainHumanPose(list_of_humans);
        }

		catch(...)
		{
		}
    }

    catch(...)
    {
    }



}

int SpecificWorker::getIDgeneric(int idjoint, RoboCompFaceTracking::Faces faces)
{
    if (IDcamera == 0) return -1;

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
				QVec jointinworld = innerModel->transform("world", QVec::vec3(-j[0],0,j[2]), "camera_astra");
//
//                qDebug()<<" Found "<<QString::fromStdString(idjoint) <<" x = " << jointinworld.x()<<  " z = " << jointinworld.z() ;

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

