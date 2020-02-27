
#include "socialrules.h"

#define PI M_PI

void SocialRules::initialize(AGMModel::SPtr worldModel_, SocialNavigationGaussianPrx socialnavigationgaussian_proxy_)
{
    qDebug()<<"Social Rules - "<< __FUNCTION__;

     socialnavigationgaussian_proxy = socialnavigationgaussian_proxy_;
     worldModel = worldModel_;

	checkObjectAffordance(false);
}


SocialRules::retPolylines SocialRules::update(AGMModel::SPtr worldModel_)
{
    qDebug()<<"Social Rules - "<< __FUNCTION__;

	worldModel = worldModel_;
	updatePeopleInModel();

	bool personMoved = peopleChanged();
    bool interactionsChanged = checkInteractions();

	if(personMoved or interactionsChanged or costChanged)
	{
        checkObjectAffordance(false);
        ApplySocialRules();

        costChanged = false;
        return std::make_tuple(true, totalpersons, intimate_seq, personal_seq, social_seq, object_seq, object_lowProbVisited, object_mediumProbVisited, object_highProbVisited, objectblock_seq);
	}

	else return std::make_tuple(false,totalpersons, intimate_seq, personal_seq, social_seq, object_seq, object_lowProbVisited, object_mediumProbVisited, object_highProbVisited, objectblock_seq);
}

void SocialRules::updatePeopleInModel()
{
    qDebug()<<"Social Rules - "<< __FUNCTION__;

	idselect_combobox->clear();
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
		cout << "[FOUND] Person " << person.id << " x = " << person.x << " z = " << person.z << " rot " << person.angle << endl;

		idselect_combobox->addItem(QString::number(person.id));

		mapIdPersons[person.id] = person; //para acceder a la persona teniendo su id
		totalpersons.push_back(person);
    }
}

bool SocialRules::peopleChanged()
{
    qDebug()<<"Social Rules - "<< __FUNCTION__;

	bool changes = false;

    if(prevpersons.size() == totalpersons.size())
    {
        for(int i = 0; i<prevpersons.size(); i++)
        {
            auto prev = prevpersons[i];
            auto curr = totalpersons[i];

            if(prev.id != curr.id or prev.x != curr.x or prev.z != curr.z or prev.angle != curr.angle)
            {
                changes = true;
                break;
            }
        }
    }

    else changes = true;
	prevpersons = totalpersons;

    return changes;

}

bool SocialRules::checkInteractions()
{
    qDebug()<<"Social Rules - "<< __FUNCTION__;

    vector<vector<int32_t>> interactingId;
    interactingpersons.clear();


	for (auto p: totalpersons) {
		auto id = p.id;
		AGMModelSymbol::SPtr personAGM = worldModel->getSymbol(id);
		int32_t pairId = -1;

		for (AGMModelSymbol::iterator edge = personAGM->edgesBegin(worldModel);
			 edge!=personAGM->edgesEnd(worldModel);
			 edge++) {
			if (edge->getLabel()=="interacting") {
				const std::pair<int32_t, int32_t> symbolPair = edge->getSymbolPair();
				const string secondType = worldModel->getSymbol(symbolPair.second)->symbolType;

				if (symbolPair.first==id and secondType=="person") {
					pairId = symbolPair.second;
					qDebug() << "Person " << symbolPair.first << " is interacting with " << symbolPair.second;
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

    if (prevInteractingId == interactingId)
        return false;
    else
    {
        prevInteractingId = interactingId;
        return true;
    }
}

SNGPolylineSeq SocialRules::ApplySocialRules()
{
    qDebug()<<"Social Rules - "<< __FUNCTION__;

    social_seq.clear();
    personal_seq.clear();
    intimate_seq.clear();

    SNGPolylineSeq seq;

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
                    SNGPolylineSeq initmate_result, personal_result, social_result;
                    socialnavigationgaussian_proxy-> getAllPersonalSpaces(per, false, initmate_result, personal_result, social_result);

                    for (auto s:initmate_result) {intimate_seq.push_back(s);}
                    for (auto s:personal_result) {personal_seq.push_back(s);}
                    for (auto s:social_result) {social_seq.push_back(s);}


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

	return seq;
}


vector <vector<int32_t>> SocialRules::groupInteractingPeople(int32_t id, int32_t pairId,vector<vector<int32_t>> &interactingId)
{
    qDebug()<<"Social Rules - "<< __FUNCTION__;

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

void SocialRules::drawGauss()
{
    if (!interactingpersons.empty())
	{
		for (auto per: interactingpersons)
		{
            SNGPolylineSeq initmate_result, personal_result, social_result;
            socialnavigationgaussian_proxy-> getAllPersonalSpaces(per, true, initmate_result, personal_result, social_result);
		}
		
	}
}


SNGPolylineSeq SocialRules::PassOnRight(bool draw)
{
    SNGPolylineSeq seq;

//// qDebug()<<__FUNCTION__;
	if (!movperson.empty())
	{
		seq = socialnavigationgaussian_proxy-> getPassOnRight(movperson, h, draw);
	}
	
	return seq;	
}


void SocialRules::checkObjectAffordance(bool d)
{
    qDebug()<<"Social Rules - "<< __FUNCTION__;

    object_seq.clear();
    object_lowProbVisited.clear();
    object_mediumProbVisited.clear();
    object_highProbVisited.clear();
    objectblock_seq.clear();


    auto vectorObjects = worldModel->getSymbolsByType("object");

    if (vectorObjects.size() != mapIdObjects.size())
    {
        qDebug() << vectorObjects.size() << mapIdObjects.size();
        mapIdObjects.clear();

        for (auto obj : vectorObjects) {

            auto id = obj->identifier;

            try { worldModel->getEdge(obj, obj, "interactive"); }

            catch (const Ice::Exception &e) {
                std::cout <<"Not interactive" <<e.what() << std::endl;
                continue;
            }

            ObjectType object;
            QString imName;

            try
            {
                AGMModelSymbol::SPtr objectParent = worldModel->getParentByLink(id, "RT");
                AGMModelEdge &edgeRT  = worldModel->getEdgeByIdentifiers(objectParent->identifier,id, "RT");

                imName = QString::fromStdString( obj->getAttribute("imName"));
                object.id = id;
                object.x = str2float(edgeRT.attributes["tx"]);
                object.z = str2float(edgeRT.attributes["tz"]);
                object.rot = str2float(edgeRT.attributes["ry"]);

                object.shape = QString::fromStdString(worldModel->getSymbolByIdentifier(id)->getAttribute("shape"));

                object.width=str2float(worldModel->getSymbolByIdentifier(id)->getAttribute("width"));
                object.height=str2float(worldModel->getSymbolByIdentifier(id)->getAttribute("height"));
                object.depth=str2float(worldModel->getSymbolByIdentifier(id)->getAttribute("depth"));
                object.inter_space=str2float(worldModel->getSymbolByIdentifier(id)->getAttribute("inter_space"));
                object.inter_angle=str2float(worldModel->getSymbolByIdentifier(id)->getAttribute("inter_angle"));

                qDebug()<< "[FOUND] Interactive Object"<< imName << object.shape << object.x <<  object.z;

                idobject_combobox->addItem(imName);

                //Defining the affordance

                if (object.shape == "trapezoid")
                    object.affordance = affordanceTrapezoidal(object);

                if (object.shape == "circle")
                    object.affordance = affordanceCircular(object);

                if (object.shape == "rectangle")
                    object.affordance = affordanceRectangular(object);

            }

            catch(const Ice::Exception &e)
            {
                std::cout <<"Error reading symbol attributes -- CHECK INITIAL MODEL SYMBOLIC" <<e.what() << std::endl;

            }


            mapIdObjects[imName] = object;

        }
    }

    for (auto obj : vectorObjects) {

        QString imName = QString::fromStdString( obj->getAttribute("imName"));

        auto cost = mapIdObjects[imName].cost;

        if (cost == 1.5)
            object_seq.push_back(mapIdObjects[imName].affordance);

        if (cost == 2)
            object_lowProbVisited.push_back(mapIdObjects[imName].affordance);

        if (cost == 2.5)
            object_mediumProbVisited.push_back(mapIdObjects[imName].affordance);

        if (cost == 3)
            object_highProbVisited.push_back(mapIdObjects[imName].affordance);

        qDebug()<< __FUNCTION__ << " cost is "<< cost;

        for (AGMModelSymbol::iterator it=obj->edgesBegin(worldModel); it != obj->edgesEnd(worldModel); it++)
        {
            AGMModelEdge edge = *it;
            if(edge->getLabel() == "interacting")
            {
                objectblock_seq.push_back(mapIdObjects[imName].affordance);
                break;
            }

            else continue;

        }
    }


}



SNGPolyline SocialRules::affordanceTrapezoidal(ObjectType obj)
{
    qDebug()<<"Social Rules - "<< __FUNCTION__;

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

SNGPolyline SocialRules::affordanceRectangular(ObjectType obj)
{
    qDebug()<<"Social Rules - "<< __FUNCTION__;

	SNGPolyline polyline;

	SNGPoint2D point1;
	point1.x  = obj.x - obj.width/2 - obj.inter_space;
	point1.z = obj.z - obj.depth/2 -obj.inter_space;
	polyline.push_back(point1);

	SNGPoint2D point2;
	point2.x  = obj.x + obj.width/2 + obj.inter_space;
	point2.z = obj.z - obj.depth/2 -obj.inter_space;
	polyline.push_back(point2);

	SNGPoint2D point3;
	point3.x  = obj.x + obj.width/2 + obj.inter_space;
	point3.z = obj.z + obj.depth/2 +obj.inter_space;
	polyline.push_back(point3);

	SNGPoint2D point4;
	point4.x  = obj.x - obj.width/2 - obj.inter_space;
	point4.z = obj.z + obj.depth/2 +obj.inter_space;
	polyline.push_back(point4);

	return polyline;

}

SNGPolyline SocialRules::affordanceCircular(ObjectType obj)
{
    qDebug()<<"Social Rules - "<< __FUNCTION__;

	SNGPolyline polyline;
    int points = 50;

    float angle_shift = M_PI*2 / points, phi = 0;

    for (int i = 0; i < points; ++i) {
        phi += angle_shift;

        SNGPoint2D point;
        point.x = obj.x + ((obj.width/2 + obj.inter_space)*sin(phi));
        point.z = obj.z +  ((obj.depth/2 + obj.inter_space)*cos(phi));
        polyline.push_back(point);

    }


    FILE *fd = fopen("circularAffordance.txt", "w");
    for (const auto &p: polyline)
    {
        fprintf(fd, "%d %d\n", (int)p.x, (int)p.z);
    }
    fclose(fd);

	return polyline;

}

void SocialRules::affordanceSliderChanged(int value)
{
    float newCost;
    if (value == 1) newCost = 1.5;
    else newCost = 1.5 + (0.5*(value - 1));

    qDebug()<< __FUNCTION__;
    auto imName = idobject_combobox->currentText();
    if (imName != "") {
        qDebug()<< "New cost of " << imName << newCost;
        mapIdObjects[imName].cost = newCost;
    }

    costChanged = true;
}

void SocialRules::affordanceTimeChanged(int step)
{


    auto hours = step / 60;
    auto minutes = step % 60;
    qDebug()<<__FUNCTION__ << step << " -- " << hours << ":" << minutes;

    QTime currentTime = QTime(hours,minutes);


    currentTime_timeEdit->setTime(currentTime);

    for (auto [key,obj] : mapIdObjects)
    {
        if (obj.therapyProgrammed)
        {
            auto before30 = obj.startT.addSecs(-(60*30));
            auto before15 = obj.startT.addSecs(-(60*15));
            auto after15 = obj.endT.addSecs(60*15);
            auto after30 = obj.endT.addSecs(60*30);


            if(obj.startT <= currentTime and currentTime <= obj.endT)
            {
                mapIdObjects[key].cost = 3.0;
                costChanged = true;
            }
            else if ((before15 <= currentTime and currentTime <= obj.startT) or (obj.endT <= currentTime and currentTime <= after15))
            {
                mapIdObjects[key].cost = 2.5;
                costChanged = true;

            }
            else if ((before30 <= currentTime and currentTime <= before15) or (after15 <= currentTime and currentTime <= after30))
            {
                mapIdObjects[key].cost = 2.0;
                costChanged = true;
            }

            else
            {
                mapIdObjects[key].cost = 1.5;
                costChanged = true;
            }
        }
    }







}

void SocialRules::programTherapy()
{
    qDebug()<<__FUNCTION__;
    if(idobject_combobox->currentText() == "")
    {
        qDebug()<< "Please, select object to program the therapy";
        return;
    }

    qDebug() << "Programing therapy with " << idobject_combobox->currentText() << " from " << startTherapy_timeEdit->time()<< " to " << endTherapy_timeEdit->time();

    auto therapy = (idobject_combobox->currentText() +QString ("    ")+ startTherapy_timeEdit->time().toString(Qt::SystemLocaleShortDate)
            + " - " + endTherapy_timeEdit->time().toString(Qt::SystemLocaleShortDate));

    therapies_list->addItem(therapy);

    mapIdObjects[idobject_combobox->currentText()].therapyProgrammed = true;
    mapIdObjects[idobject_combobox->currentText()].startT = startTherapy_timeEdit->time();
    mapIdObjects[idobject_combobox->currentText()].endT = endTherapy_timeEdit->time();

}

void SocialRules::removeTherapy()
{
    auto item_to_delete = therapies_list->currentRow();
    therapies_list->takeItem(item_to_delete);

    mapIdObjects[idobject_combobox->currentText()].therapyProgrammed = false;


}

void SocialRules::checkstate()
{
//    qDebug()<<__FUNCTION__;

    if (follow_checkbox->checkState() == Qt::CheckState(2) and followpulsed == false)
    	{
			followpulsed = true;
			accompanypulsed = false;
			accompany_checkbox->setCheckState(Qt::CheckState(0));
            followPerson();

		}

    else if (accompany_checkbox->checkState() == Qt::CheckState(2) and accompanypulsed == false)
	{
		accompanypulsed = true;
		followpulsed = false;
		follow_checkbox->setCheckState(Qt::CheckState(0));
        accompanyPerson();
	}

	if (follow_checkbox->checkState() == Qt::CheckState(0))
	    followpulsed = false;

    if (accompany_checkbox->checkState() == Qt::CheckState(0))
        accompanypulsed = false;

    if (passonright_checkbox->checkState() == Qt::CheckState(2))
        porpulsed = true;
    else
        porpulsed = false;
}

void SocialRules::followPerson()
{
    int32_t id = idselect_combobox->currentText().toInt();
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
//        pathfinder->go(person.x - 900*cos(angle) , person.z -900*sin(angle));
    }


}


void SocialRules::accompanyPerson()
{
    qDebug() <<__FUNCTION__;
    int32_t id = idselect_combobox->currentText().toInt();

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
//        pathfinder->go(person.x + 900*sin(angle) , person.z - cos(angle));
    }

}

void SocialRules::goToPerson()
{
// qDebug()<<__FUNCTION__;
	int32_t id = idselect_combobox->currentText().toInt();
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
//		pathfinder->go_rot(person.x + 1200*cos(angle) , person.z +1200*sin(angle), rotation);
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
//		qDebug()<<"Distancia calculada" << dist << "Distancia total" <<totaldist;
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

	QVec pose = i->transform(QString::fromStdString(name),"robot");

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




