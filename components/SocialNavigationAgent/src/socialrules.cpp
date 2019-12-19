
#include "socialrules.h"

#define PI M_PI

void SocialRules::initialize(SocialNavigationGaussianPrx socialnavigationgaussian_proxy_,
			     AGMExecutivePrx agmexecutive_proxy_,
			     QMutex *mutex_,
			     robocomp::pathfinder::PathFinder *pathfinder_,
			     AGMModel::SPtr worldModel_,
			     const std::shared_ptr<InnerModel> &innerModel_)

{
	qDebug()<<__FUNCTION__;

	socialnavigationgaussian_proxy = socialnavigationgaussian_proxy_;
	agmexecutive_proxy = agmexecutive_proxy_;
	mux = mutex_;
	pathfinder = pathfinder_;
	innerModel = innerModel_;
	worldModel = worldModel_;


	checkObjectAffordance(false);
}

void SocialRules::innerModelChanged(const std::shared_ptr<InnerModel> &innerModel_)
{
	qDebug()<<__FUNCTION__;
	
	innerModel = innerModel_;
	pathfinder->innerModelChanged(innerModel, totalpersons, intimate_seq, personal_seq, social_seq, object_seq, objectblock_seq);
}


void SocialRules::checkModificationsInModel(AGMModel::SPtr worldModel_)
{

	qDebug()<<__FUNCTION__;
	worldModel = worldModel_;


	updatePeopleInModel();
	checkObjectAffordance(false);

	ApplySocialRules();
}

void SocialRules::updatePeopleInModel()
{
	qDebug()<<__FUNCTION__;

	idselected->clear();
	totalpersons.clear();
	mapIdPersons.clear();

	auto vectorPersons = worldModel->getSymbolsByType("person");

	if (vectorPersons.size() == 0) {
		qDebug() << "No persons found";
	}

	for (auto p: vectorPersons) {
		SNGPerson person;

		auto id = p->identifier;
		AGMModelSymbol::SPtr personParent = worldModel->getParentByLink(id, "RT");
		AGMModelEdge& edgeRT = worldModel->getEdgeByIdentifiers(personParent->identifier, id, "RT");

		person.id = id;
		person.x = str2float(edgeRT.attributes["tx"]);
		person.z = str2float(edgeRT.attributes["tz"]);
		person.angle = str2float(edgeRT.attributes["ry"]);
		cout << "PERSON " << person.id << " x = " << person.x << " z = " << person.z << " rot " << person.angle << endl;

		idselected->addItem(QString::number(person.id));

		mapIdPersons[person.id] = person; //para acceder a la persona teniendo su id
		totalpersons.push_back(person);
    }

	checkInteractions();

}



void SocialRules::checkInteractions()
{
    vector<vector<int32_t>> interactingId;
    interactingpersons.clear();

	qDebug() << __FUNCTION__;

	for (auto p: totalpersons) {
		auto id = p.id;
		AGMModelSymbol::SPtr personAGM = worldModel->getSymbol(id);
		qDebug() << "-------------- COMPROBANDO ENLACES DE LA PERSONA " << id << "-------------- ";
		int32_t pairId = -1;

		for (AGMModelSymbol::iterator edge = personAGM->edgesBegin(worldModel);
			 edge!=personAGM->edgesEnd(worldModel);
			 edge++) {
			if (edge->getLabel()=="interacting") {
				const std::pair<int32_t, int32_t> symbolPair = edge->getSymbolPair();
				qDebug() << "symbolPair.first " << symbolPair.first << "symbolPair.second " << symbolPair.second;
				const string secondType = worldModel->getSymbol(symbolPair.second)->symbolType;

				if (symbolPair.first==id and secondType=="person") {
					pairId = symbolPair.second;
					qDebug() << "INTERACTING" << symbolPair.first << "AND " << symbolPair.second;
					break;
				}
			}
		}
		groupInteractingPeople(id, pairId, interactingId);

	}

    for (auto id_vect: interactingId) {
        SNGPersonSeq persons;
        for (auto id : id_vect) {
            persons.push_back(mapIdPersons[id]);
        }

        interactingpersons.push_back(persons);

    }
}

SNGPolylineSeq SocialRules::ApplySocialRules()
{
    qDebug()<<__FUNCTION__;

    social_seq.clear();
    personal_seq.clear();
    intimate_seq.clear();

	if(!interactingpersons.empty())
	{
		try
		{
			for (auto per: interactingpersons)
			{
				if((per.size() == 1) and (porpulsed))
				{
					seq = socialnavigationgaussian_proxy->  getPassOnRight(per, 0.1, false);
					for (auto s:seq) {social_seq.push_back(s);}
					
					seq = socialnavigationgaussian_proxy-> getPassOnRight(per, 0.4, false);
					for (auto s:seq) {personal_seq.push_back(s);}

					seq = socialnavigationgaussian_proxy-> getPassOnRight(per, 0.8, false);
					for (auto s:seq) {intimate_seq.push_back(s);}

				}
				else
				{
					seq = socialnavigationgaussian_proxy-> getPersonalSpace(per, 0.1, false);
					for (auto s:seq) {social_seq.push_back(s);}

					seq = socialnavigationgaussian_proxy-> getPersonalSpace(per, 0.4, false);
					for (auto s:seq) {personal_seq.push_back(s);}

                    seq = socialnavigationgaussian_proxy-> getPersonalSpace(per, 0.8, false);
                    for (auto s:seq) {intimate_seq.push_back(s);}
				}
			}

			if (followpulsed) followPerson();
			if (accompanypulsed) accompanyPerson();
	
		}

		catch( const Ice::Exception &e)
		{
			std::cout << e << std::endl;
		}
	}

//    qDebug()<<"innerModel changed to pathfinder";
//	pathfinder->innerModelChanged(innerModel, totalpersons, intimate_seq, personal_seq, social_seq, object_seq,objectblock_seq);


	return seq;
}


vector <vector<int32_t>> SocialRules::groupInteractingPeople(int32_t id, int32_t pairId,vector<vector<int32_t>> &interactingId)
{
    vector <int32_t> Ids;
    bool p1found = false;
    bool p2found = false;

    for (auto i : interactingId)
    {
        for (auto v : i)
        {
            if (v == id) p1found = true;
            if (v == pairId) p2found = true;

        }
    }

    if (pairId != -1)
    {
        if (!p1found and !p2found)
        {
            Ids.push_back(id);
            Ids.push_back(pairId);
            interactingId.push_back(Ids);
        }

        else if (p1found and !p2found)
        {
            for (auto &i : interactingId)
            {
                for (auto v : i)
                {
                    if (v == id) i.push_back(pairId);
                }
            }
        }

        else if (!p1found and p2found)
        {
            for (auto &i : interactingId)
            {
                for (auto v : i)
                {
                    if (v == pairId) i.push_back(id);
                }
            }
        }
    }

    else if (!p1found)
    {
        Ids.push_back(id);
        interactingId.push_back(Ids);
    }

    return interactingId;
}


/**
* \brief If the person is in the model it is added to a vector of persons wich is sent to the socialnavigationGaussian
* to model its personal space. 
* The function returns a sequence of polylines.
*/



SNGPolylineSeq SocialRules::calculateGauss(bool draw, float h)
{
	
//	qDebug()<<__FUNCTION__ << "con h = " << h;
	if (!interactingpersons.empty())
	{
		seq.clear();
		for (auto per: interactingpersons)
		{
			seq = socialnavigationgaussian_proxy-> getPersonalSpace(per, h, draw);
		}
		
	}
	return seq;
}


SNGPolylineSeq SocialRules::PassOnRight(bool draw)
{
//	qDebug()<<__FUNCTION__;
	if (!movperson.empty())
	{
		seq.clear();
		seq = socialnavigationgaussian_proxy-> getPassOnRight(movperson, h, draw);
	}
	
	return seq;	
}


void SocialRules::checkObjectAffordance(bool d)
{
//	qDebug()<<__FUNCTION__;

    object_seq.clear();
    objectblock_seq.clear();

	auto vectorObjects = worldModel->getSymbolsByType("object");

    for (auto obj : vectorObjects) {
        ObjectType object;
        auto id = obj->identifier;

        try {
            worldModel->getEdge(obj, obj, "interactive");
        }

        catch (...) {
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
        object.inter_space=str2float(worldModel->getSymbolByIdentifier(id)->getAttribute("inter_space"));
        object.inter_angle=str2float(worldModel->getSymbolByIdentifier(id)->getAttribute("inter_angle"));

        auto affordance = calculateAffordance(object);
        object_seq.push_back(affordance);

		for (AGMModelSymbol::iterator it=obj->edgesBegin(worldModel); it != obj->edgesEnd(worldModel); it++)
		{
			AGMModelEdge edge = *it;
			if(edge->getLabel() == "interacting")
            {
                objectblock_seq.push_back(affordance);
                break;
            }

            else continue;


		}

    }

}


SNGPolyline SocialRules::calculateAffordance(ObjectType obj)
{
    cout << "Entered calculateAffordance"<<endl;
    QPolygonF aff_qp;

    auto left_angle = obj.rot + obj.inter_angle/2;
    auto right_angle = obj.rot - obj.inter_angle/2;

    SNGPolyline polyline;

    SNGPoint2D point1;
    point1.x  = obj.x + obj.width/2;
    point1.z = obj.z;
    polyline.push_back(point1);

    SNGPoint2D point2;
    point2.x  = obj.x - obj.width/2;
    point2.z = obj.z;
    polyline.push_back(point2);

    SNGPoint2D point3;
    point3.x  = obj.x + obj.inter_space*(cos(M_PI_2 - left_angle));
    point3.z = obj.z + obj.inter_space*(sin(M_PI_2 - left_angle));
    polyline.push_back(point3);

    SNGPoint2D point4;
    point4.x  = obj.x + obj.inter_space*(cos(M_PI_2 - right_angle));
    point4.z = obj.z + obj.inter_space*(sin(M_PI_2 - right_angle));
    polyline.push_back(point4);


    return polyline;

}

void SocialRules::checkstate()
{
//    qDebug()<<__FUNCTION__;

    if (follow->checkState() == Qt::CheckState(2) and followpulsed == false)
    	{
			followpulsed = true;
			accompanypulsed = false;
			accompany->setCheckState(Qt::CheckState(0));
            followPerson();

		}

    else if (accompany->checkState() == Qt::CheckState(2) and accompanypulsed == false)
	{
		accompanypulsed = true;
		followpulsed = false;
		follow->setCheckState(Qt::CheckState(0));
        accompanyPerson();
	}

	if (follow->checkState() == Qt::CheckState(0))
	    followpulsed = false;

    if (accompany->checkState() == Qt::CheckState(0))
        accompanypulsed = false;

    if (por->checkState() == Qt::CheckState(2))
        porpulsed = true;
    else
        porpulsed = false;
}

void SocialRules::followPerson()
{
    int32_t id = idselected->currentText().toInt();
    qDebug()<<"Person selected" << id;

    if (id != 0)
    {
        AGMModelSymbol::SPtr personParent = worldModel->getParentByLink(id, "RT");
        AGMModelEdge &edgeRT = worldModel->getEdgeByIdentifiers(personParent->identifier, id, "RT");

        person.x = str2float(edgeRT.attributes["tx"]);
        person.z = str2float(edgeRT.attributes["tz"]);
        person.angle = str2float(edgeRT.attributes["ry"]);
        //person.vel=str2float(edgeRT.attributes["velocity"]);
        person.vel = 0;

        auto angle = M_PI/2 - person.angle;
        pathfinder->go(person.x - 900*cos(angle) , person.z -900*sin(angle));
    }


}


void SocialRules::accompanyPerson()
{
    qDebug() <<__FUNCTION__;
    int32_t id = idselected->currentText().toInt();

    if (id != 0)
    {
        AGMModelSymbol::SPtr personParent = worldModel->getParentByLink(id, "RT");
        AGMModelEdge &edgeRT = worldModel->getEdgeByIdentifiers(personParent->identifier, id, "RT");

        person.x = str2float(edgeRT.attributes["tx"]);
        person.z = str2float(edgeRT.attributes["tz"]);
        person.angle = str2float(edgeRT.attributes["ry"]);
        //person.vel=str2float(edgeRT.attributes["velocity"]);
        person.vel = 0;

        qDebug()<<"Person selected "<< id<< "Pose (x,z) = ( "<<  person.x <<","<<person.z<<")";
        auto angle = M_PI/2 - person.angle;
        pathfinder->go(person.x + 900*sin(angle) , person.z - cos(angle));
    }

}

void SocialRules::goToPerson()
{
	qDebug()<<__FUNCTION__;
	int32_t id = idselected->currentText().toInt();
	qDebug()<<"Person selected" << id;

	if (id != 0)
	{
		AGMModelSymbol::SPtr personParent = worldModel->getParentByLink(id, "RT");
		AGMModelEdge &edgeRT = worldModel->getEdgeByIdentifiers(personParent->identifier, id, "RT");

		person.x = str2float(edgeRT.attributes["tx"]);
		person.z = str2float(edgeRT.attributes["tz"]);
		person.angle = str2float(edgeRT.attributes["ry"]);
		//person.vel=str2float(edgeRT.attributes["velocity"]);
		person.vel = 0;

		auto angle = M_PI/2 - person.angle;
		auto rotation = person.angle + M_PI;
		pathfinder->go_rot(person.x + 1200*cos(angle) , person.z +1200*sin(angle), rotation);
	}

}


void SocialRules::checkRobotmov()
{
	robotSymbolId = worldModel->getIdentifierByType("robot");
	AGMModelSymbol::SPtr robotparent = worldModel->getParentByLink(robotSymbolId, "RT");
	AGMModelEdge &edgeRTrobot  = worldModel->getEdgeByIdentifiers(robotparent->identifier, robotSymbolId, "RT");
	robot.x=str2float(edgeRTrobot.attributes["tx"]);
	robot.z=str2float(edgeRTrobot.attributes["tz"]);
	robot.angle=str2float(edgeRTrobot.attributes["ry"]);


	point.x=robot.x;
	point.z=robot.z;
		 
	if (poserobot.size() == 0)
		poserobot.push_back(point);
	  
	else if ((poserobot[poserobot.size()-1].x != point.x) or (poserobot[poserobot.size()-1].z !=point.z))
	{  
		float  dist=sqrt((point.x - poserobot[poserobot.size()-1].x)*(point.x - poserobot[poserobot.size()-1].x)
				+(point.z - poserobot[poserobot.size()-1].z)*(point.z - poserobot[poserobot.size()-1].z));
		    
		totaldist=totaldist + dist;
		qDebug()<<"Distancia calculada" << dist << "Distancia total" <<totaldist;
	}

	poserobot.push_back(point);

}

void SocialRules::saveData()
{	
	qDebug("Saving in robotpose.txt the robot's pose");
	ofstream file("robotpose.txt", ofstream::out);
	for (auto p:poserobot)
	{
		file<< p.x << " " <<p.z<< endl;
	}
	file.close();

	qDebug("Saving in personpose.txt the human's poses");
	ofstream file2("personpose.txt", ofstream::out);
	for (auto person:totalpersons)
	{
		file2<< person.x << " " <<person.z<<" "<<person.angle<< endl;
	}
	file2.close();	
	poserobot.clear();


	qDebug("Saving intimate polyline");
	ofstream file3("polyline_intimate.txt", ofstream::out);
	for (auto s:intimate_seq)
	{
		for (auto p: s)
			file3<< p.x << " " <<p.z<<" "<< endl;
		file3 << " "<<endl ;
	}

	file3.close();
	
	qDebug("Saving personal polyline");
	ofstream file4("polyline_personal.txt", ofstream::out);
	for (auto s:personal_seq)
	{
		for (auto p: s)
			file4<< p.x << " " <<p.z<<" "<< endl;
        file4 << " "<<endl ;
	}
	file4.close();
	
	qDebug("Saving social polyline");
	ofstream file5("polyline_social.txt", ofstream::out);
	for (auto s:social_seq)
	{
		for (auto p: s)
			file5<< p.x << " " <<p.z<<" "<< endl;
		file5<<" "<<endl;
	}
	file5.close();

	qDebug()<<"Saving in dist.txt the total distance"<<totaldist;
	ofstream file6("dist.txt", ofstream::out);
	file6 <<" Distance " << totaldist << endl;
	totaldist = 0;
	file6.close();
}


bool SocialRules::checkHRI(SNGPerson p, int ind , InnerPtr &i, AGMModel::SPtr w)
{
	worldModel = w;
	bool changes = false;
	/////////////////////Checking if the person is close and looking at the robot
	std::string type = "person";
	std::string name = "person" + std::to_string(ind);

	qDebug()<<QString::fromStdString(type)<<"-"<<QString::fromStdString(name);

	bool looking = false;
	bool close = false;

	mux->lock();
	QVec pose = i->transform(QString::fromStdString(name),"robot");
	mux->unlock();

	float dist = sqrt(pose.x()*pose.x()+pose.z()*pose.z());
	float angle = atan2(pose.x(),pose.z());

	qDebug()<<"pose x"<<pose.x()<<"pose z"<<pose.z();
	qDebug()<<"dist"<<dist<<"angle"<<abs(angle/0.0175);

	if (abs(angle)<20*0.0175)
		looking = true;

	if (dist<2000.0)
		close = true;
	
	///////////////////////Add edge interrupting////////////////////////

	int32_t Idperson = worldModel->getIdentifierByType(type);
	AGMModelSymbol::SPtr person = worldModel->getSymbolByIdentifier(Idperson);
	int32_t Idrobot = worldModel->getIdentifierByType("robot");
	AGMModelSymbol::SPtr robot = worldModel->getSymbolByIdentifier(Idrobot);

	if ((looking==true) and (close==true))
	{
		qDebug()<<"CERCA Y MIRANDO";
		try
		{		
			worldModel->addEdge(person,robot, "interrupting");
			qDebug()<<"SE AÑADE EL ENLACE";
			changes = true;
		}
		catch(...)
		{
			qDebug()<<"EXISTE EL ENLACE";
			changes = false;
		}		
	}	
	
	else 
	{	try
		{
			worldModel->removeEdge(person,robot,"interrupting");
			changes = true;
		}
		catch(...)
		{
			qDebug()<<"NO EXISTE EL ENLACE";
			changes = false;
		}
	}
	return changes;
}



