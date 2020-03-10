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
#include <cppitertools/zip.hpp>

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

	innerModel = std::make_shared<InnerModel>(new InnerModel());
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

    connect(draw_personalSpace_button,SIGNAL(clicked()),this, SLOT(drawPersonalSpace()));
    connect(save_data_button,SIGNAL(clicked()),this, SLOT(recordData()));

    connect(object_slider, SIGNAL (valueChanged(int)),this,SLOT(affordanceSliderChanged(int)));
    connect(currentTime_timeEdit, SIGNAL (timeChanged(const QTime)),this,SLOT(affordanceTimeEditChanged(const QTime)));

    connect(setTherapy_button, SIGNAL (clicked()),this,SLOT(programTherapy()));
    connect(removeT_button, SIGNAL (clicked()),this,SLOT(removeTherapy()));
    connect(currtime_slider, SIGNAL (valueChanged(int)),this,SLOT(affordanceTimeSliderChanged(int)));

    checkObjectAffordance();

	auto timeValue = currtime_slider->value();
	QTime currentTime = QTime(timeValue / 60, timeValue % 60);
	currentTime_timeEdit->setTime(currentTime);

    QString hour = currentTime.toString(Qt::SystemLocaleShortDate);
    for (auto const &[key,obj] : mapIdObjects)
    {
        mapCostsPerHour[hour].push_back(obj.cost);
    }


	this->Period = period;
	timer.start(Period);
	emit this->t_initialize_to_compute();

}

void SpecificWorker::compute()
{

	if (worldModelChanged)
	{
		updatePeopleInModel();
        checkInteractions();
		checkObjectAffordance();
        applySocialRules();

        updatePersonalSpacesInGraph();

        publishPersonalSpaces();
        publishAffordances();

		worldModelChanged = false;
        costChanged = false;
    }

	else if (costChanged)
    {
        publishAffordances();

        costChanged = false;
    }

    checkRobotmov();
}

void SpecificWorker::updatePeopleInModel()
{
	qDebug()<< __FUNCTION__;

	sngPersonSeq.clear();
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

		mapIdPersons[person.id] = person; //para acceder a la persona teniendo su id
		sngPersonSeq.push_back(person);
	}
}


void SpecificWorker::checkInteractions()
{
	qDebug()<< __FUNCTION__;

	vector<vector<int32_t>> interactingId;
	interactingPersonsVec.clear();


	for (auto p: sngPersonSeq) {
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

	for (auto id_vect: interactingId)
	{
		SNGPersonSeq persons;
		for (auto id : id_vect) {
			persons.push_back(mapIdPersons[id]);
		}

		interactingPersonsVec.push_back(persons);
	}
}

vector <vector<int32_t>> SpecificWorker::groupInteractingPeople(int32_t id, int32_t pairId,vector<vector<int32_t>> &interactingId)
{
	qDebug()<< __FUNCTION__;

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

void SpecificWorker::applySocialRules()
{
    qDebug()<< __FUNCTION__;

    socialSpace_seq.clear();
    personalSpace_seq.clear();
    intimateSpace_seq.clear();

    SNGPolylineSeq seq;

    if(!interactingPersonsVec.empty())
    {
        try
        {
            for (auto const &personGroup: interactingPersonsVec) {
                SNGPolylineSeq intimateResult, personalResult, socialResult;

                vector<QPolygonF> intimatePolygon, personalPolygon, socialPolygon;

                socialnavigationgaussian_proxy->getAllPersonalSpaces(personGroup, false, intimateResult,
                        personalResult, socialResult);


                for (auto s:intimateResult) {intimateSpace_seq.push_back(s);}
                for (auto s:personalResult) {personalSpace_seq.push_back(s);}
                for (auto s:socialResult) {socialSpace_seq.push_back(s);}

                arrangePersonalSpaces(personGroup, intimateResult, personalResult, socialResult);
            }

        }

        catch( const Ice::Exception &e)
        {
            std::cout << e << std::endl;
        }
    }

}

void SpecificWorker::arrangePersonalSpaces(SNGPersonSeq personGroup,SNGPolylineSeq intimate,
        SNGPolylineSeq personal, SNGPolylineSeq social)
{
    vector<int> groupIDs;

    for(auto const &person:personGroup)
        groupIDs.push_back(person.id);

    qDebug()<<"Group ID " <<groupIDs;
    qDebug()<< "----";


    for(auto const &person: personGroup)
    {
        vector<int> sharingWith = groupIDs;
        sharingWith.erase(std::remove(sharingWith.begin(), sharingWith.end(), person.id), sharingWith.end());

        qDebug()<< person.id <<" is sharing polylines with " << sharingWith;

        if(sharingWith.size() == 0)
            mapIdSpaces[person.id].spacesSharedWith.clear();
        else
            mapIdSpaces[person.id].spacesSharedWith = sharingWith;


        mapIdSpaces[person.id].intimatePolylines = intimate;
        mapIdSpaces[person.id].personalPolylines = personal;
        mapIdSpaces[person.id].socialPolylines = social;

    }

}

void SpecificWorker::drawPersonalSpace()
{
    if (!interactingPersonsVec.empty())
    {
        for (auto per: interactingPersonsVec)
        {
            SNGPolylineSeq initmate_result, personal_result, social_result;
            socialnavigationgaussian_proxy-> getAllPersonalSpaces(per, true, initmate_result, personal_result, social_result);
        }

    }
}

//---------------------- Objects ----------------------//

void SpecificWorker::checkObjectAffordance()
{
    qDebug()<< __FUNCTION__;

    auto vectorObjects = worldModel->getSymbolsByType("object");

    if (vectorObjects.size() != mapIdObjects.size())
    {
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
                object.imName = obj->getAttribute("imName");
                object.id = id;
                object.x = str2float(edgeRT.attributes["tx"]);
                object.z = str2float(edgeRT.attributes["tz"]);
                object.rot = str2float(edgeRT.attributes["ry"]);

                object.shape = QString::fromStdString(worldModel->getSymbolByIdentifier(id)->getAttribute("shape"));

                object.width = str2float(worldModel->getSymbolByIdentifier(id)->getAttribute("width"));
                object.height = str2float(worldModel->getSymbolByIdentifier(id)->getAttribute("height"));
                object.depth = str2float(worldModel->getSymbolByIdentifier(id)->getAttribute("depth"));
                object.inter_space = str2float(worldModel->getSymbolByIdentifier(id)->getAttribute("inter_space"));
                object.inter_angle = str2float(worldModel->getSymbolByIdentifier(id)->getAttribute("inter_angle"));

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

    for (auto obj : vectorObjects) //check if anyone is interacting with the object
    {
        QString imName = QString::fromStdString( obj->getAttribute("imName"));

        for (AGMModelSymbol::iterator it=obj->edgesBegin(worldModel); it != obj->edgesEnd(worldModel); it++)
        {
            AGMModelEdge edge = *it;
            if(edge->getLabel() == "interacting")
            {
                mapIdObjects[imName].interacting = true;
                break;
            }

            else
            {
                mapIdObjects[imName].interacting = false;
            }
        }
    }


}


SNGPolyline SpecificWorker::affordanceTrapezoidal(ObjectType obj)
{
    qDebug()<< __FUNCTION__;

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

SNGPolyline SpecificWorker::affordanceRectangular(ObjectType obj)
{
    qDebug()<< __FUNCTION__;

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

SNGPolyline SpecificWorker::affordanceCircular(ObjectType obj)
{
    qDebug()<< __FUNCTION__;

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

void SpecificWorker::affordanceSliderChanged(int value)
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

void SpecificWorker::affordanceTimeSliderChanged(int step)
{

    auto hours = step / 60;
    auto minutes = step % 60;

    QTime currentTime = QTime(hours,minutes);
    currentTime_timeEdit->setTime(currentTime);

    for (auto const &[key,obj] : mapIdObjects)
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
            }
            else if ((before15 <= currentTime and currentTime <= obj.startT) or (obj.endT <= currentTime and currentTime <= after15))
            {
                mapIdObjects[key].cost = 2.5;

            }
            else if ((before30 <= currentTime and currentTime <= before15) or (after15 <= currentTime and currentTime <= after30))
            {
                mapIdObjects[key].cost = 2.0;
            }

            else
            {
                mapIdObjects[key].cost = 1.5;
            }

            if(mapIdObjects[key].cost != mapIdObjects[key].prevCost)
                costChanged = true;

            mapIdObjects[key].prevCost = mapIdObjects[key].cost;
        }
    }


    if(minutes % 10 == 0 or minutes % 10 == 5)
    {
        QString hour = currentTime.toString(Qt::SystemLocaleShortDate);
        for (auto const &[key,obj] : mapIdObjects)
        {
            mapCostsPerHour[hour].push_back(obj.cost);
        }
    }



}

void SpecificWorker::affordanceTimeEditChanged(const QTime &time)
{
    auto hours = time.hour();
    auto hoursInMinutes = hours*60;
    auto minutes = time.minute();

    auto totalMinutes = hoursInMinutes + minutes;

    currtime_slider->setValue(totalMinutes);
}

void SpecificWorker::programTherapy()
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

void SpecificWorker::removeTherapy()
{
    auto item_to_delete = therapies_list->currentRow();
    therapies_list->takeItem(item_to_delete);

    mapIdObjects[idobject_combobox->currentText()].therapyProgrammed = false;
}

void SpecificWorker::recordData()
{

    qDebug()<< "Saving in robotpose.txt the robot's pose";
    ofstream file("results/robotpose.txt", ofstream::out);
    for (auto p:poserobot)
    {
        file<< p.x << " " <<p.z<< endl;
    }
    file.close();
    poserobot.clear();

    qDebug()<< "Saving in personpose.txt the human's poses";
    ofstream file2("results/personpose.txt", ofstream::out);
    for (auto person:sngPersonSeq)
    {
        file2<< person.x << " " <<person.z<<" "<<person.angle<< endl;
    }
    file2.close();


    qDebug()<< "Saving intimate polyline";
    ofstream file3("results/polyline_intimate.txt", ofstream::out);
    for (auto s:intimateSpace_seq)
    {
        for (auto p: s)
            file3<< p.x << " " <<p.z<<" "<< endl;
        file3 << " "<<endl ;
    }

    file3.close();

    qDebug()<< "Saving personal polyline";
    ofstream file4("results/polyline_personal.txt", ofstream::out);
    for (auto s:personalSpace_seq)
    {
        for (auto p: s)
            file4<< p.x << " " <<p.z<<" "<< endl;
        file4 << " "<<endl ;
    }
    file4.close();

    qDebug()<< "Saving social polyline";
    ofstream file5("results/polyline_social.txt", ofstream::out);
    for (auto s:socialSpace_seq)
    {
        for (auto p: s)
            file5<< p.x << " " <<p.z<<" "<< endl;
        file5<<" "<<endl;
    }
    file5.close();

    qDebug()<< "-- Saving Affordances --";

    ofstream file7("results/object_normal.txt", ofstream::out);
    ofstream file8("results/object_lowP.txt", ofstream::out);
    ofstream file9("results/object_medP.txt", ofstream::out);
    ofstream file10("results/object_highP.txt", ofstream::out);
    ofstream file11 ("results/objects.txt", ofstream::out);

    std::ofstream costFile("results/costs.csv", ofstream::out);
    vector<string> headers;

    costFile<<"Time ";

    for (auto const &[k,o] : mapIdObjects)
    {
        costFile<< k.toStdString() << " ";

        file11 << o.shape.toStdString() << " " << o.x << " " << o.z << " " << o.rot<< " " << o.width << " " << o.depth << endl;

        if(o.cost == 1.5)
        {
            for (auto p: o.affordance)
                file7 << p.x << " " << p.z << " " << endl;
            file7 <<" "<<endl;
        }

        else if(o.cost == 2.0)
        {
            for (auto p: o.affordance)
                file8 << p.x << " " << p.z << " "<< endl;
            file8 <<" "<<endl;
        }

        else if(o.cost == 2.5)
        {
            for (auto p: o.affordance)
                file9 << p.x << " " << p.z << " " << endl;
            file9 <<" "<<endl;
        }

        else if(o.cost == 3.0)
        {
            for (auto p: o.affordance)
                file10 << p.x << " " << p.z << " " << endl;
            file10 <<" "<<endl;
        }
    }

    costFile<<std::endl;

    for (auto const &[hour, costs] : mapCostsPerHour)
    {

        costFile << hour.toStdString() <<" ";
        for(auto const& c: costs)
        {
            costFile << c <<" ";
        }
        costFile<<std::endl;
    }


    file7.close();
    file8.close();
    file9.close();
    file10.close();
    file11.close();
    costFile.close();

}

void SpecificWorker::checkRobotmov()
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


void SpecificWorker::publishPersonalSpaces()
{
    qDebug() <<__FUNCTION__;
    try
    {
        socialrules_pubproxy->personalSpacesChanged(intimateSpace_seq,personalSpace_seq,socialSpace_seq);

    }
    catch(const Ice::Exception& e)
    {
        qDebug()<< "Can't publish personalSpaces";
    }
}

void SpecificWorker::publishAffordances()
{
    qDebug() <<__FUNCTION__;

    SRObjectSeq objectsToSend;

    for (auto [k,o] : mapIdObjects)
    {
        SRObject object;
        object.name = o.imName;
        object.x = o.x;
        object.z = o.z;
        object.interacting = o.interacting;

        object.cost = o.cost;
        object.affordance = o.affordance;

        objectsToSend.push_back(object);
    }

    try
    {
        socialrules_pubproxy->objectsChanged(objectsToSend);
    }
    catch(const Ice::Exception& e)
    {
        qDebug()<< "Can't publish affordances";
    }

}

void SpecificWorker::updatePersonalSpacesInGraph()
{

    AGMModel::SPtr newModel(new AGMModel(worldModel));
    bool newSymbol = false;

    auto vectorSpacesInGraph = worldModel->getSymbolsByType("personalSpace");

    vector<AGMModelEdge> edgesToPublish;

    for(auto [personID,spaces] : mapIdSpaces)
    {
        std::string type = "personalSpace" ;
        std::string imName = "personalSpace" + std::to_string(personID);

        int spaceSymbolId = -1;

        cout << "Type = "<<type << " " <<"imName = "<<imName<< endl;

        for(auto spaces:vectorSpacesInGraph)
        {
            auto id = spaces->identifier;

            if(newModel->getSymbolByIdentifier(id)->getAttribute("imName") == imName)
            {
                spaceSymbolId = id;
                break;
            }
        }

        if(spaceSymbolId == -1)  //Symbol not found
        {
            qDebug()<< "symbol not found";

            // Symbolic part
            AGMModelSymbol::SPtr personalSpace = newModel->newSymbol("personalSpace");
            int spaceSymbolId = personalSpace->identifier;
            printf("Got personalSpaceSymbolID: %d\n", spaceSymbolId);
            personalSpace->setAttribute("imName", imName);
            personalSpace->setAttribute("imType", "transform");


            vector<string> polylinesStr = {"","",""};
            vector <SNGPolylineSeq> polylinesSeq {spaces.intimatePolylines, spaces.personalPolylines, spaces.socialPolylines};

            for (auto&&[str, polyline] : iter::zip(polylinesStr, polylinesSeq))
            {
                for(auto pol: polyline)
                {
                    for (auto p: pol)
                    {

                        string pointStr = to_string(p.x) + "," + to_string(p.z) + " ";
                        str += pointStr;
                    }
                    str += ";";
                }
            }

//            personalSpace->setAttribute("intimate", polylinesStr[0]);
//            personalSpace->setAttribute("personal", polylinesStr[1]);
//            personalSpace->setAttribute("social", polylinesStr[2]);
//
//            newModel->addEdgeByIdentifiers(personID,spaceSymbolId," ");

//             Geometric part
            std::map<std::string, std::string> edgeRTAtrs;

            edgeRTAtrs["intimate"] = polylinesStr[0];
            edgeRTAtrs["personal"] = polylinesStr[1];
            edgeRTAtrs["social"] = polylinesStr[2];

            newModel->addEdgeByIdentifiers(spaceSymbolId, personID, "RT", edgeRTAtrs);

            newSymbol = true;
        }

        else
        {
            qDebug()<< "Symbol already in model";

            vector<string> polylinesStr = {"","",""};
            vector <SNGPolylineSeq> polylinesSeq {spaces.intimatePolylines, spaces.personalPolylines, spaces.socialPolylines};

            for (auto&&[str, polyline] : iter::zip(polylinesStr, polylinesSeq))
            {
                for(auto pol: polyline)
                {
                    for (auto p: pol)
                    {

                        string pointStr = to_string(p.x) + "," + to_string(p.z) + " ";
                        str += pointStr;
                    }
                    str += ";";
                }
            }

            AGMModelEdge &edgeRT  = worldModel->getEdgeByIdentifiers(spaceSymbolId, personID, "RT");

            edgeRT.attributes["intimate"] = polylinesStr[0];
            edgeRT.attributes["personal"] = polylinesStr[1];
            edgeRT.attributes["social"] = polylinesStr[2];

            edgesToPublish.push_back(edgeRT);

        }

    }

    if(newSymbol)
    {
        try {
            sendModificationProposal(worldModel, newModel);
        }
        catch(...)
        {
            qDebug()<< "Can't send modification proposal";
        }
    }

    else
    {
        for(auto edgeRT : edgesToPublish)
        {
            try
            {
                AGMMisc::publishEdgeUpdate(edgeRT, agmexecutive_proxy);
            }
            catch(std::exception& e)
            {
                std::cout<<"Exception moving in AGM: "<<e.what()<<std::endl;
            }
        }
    }
}


//////////////////////// State machine methods ////////////////////////////////////////

void SpecificWorker::sm_compute()
{
//	std::cout<<"Entered state compute"<<std::endl;
	compute();
}

void SpecificWorker::sm_initialize()
{
//	std::cout<<"Entered initial state initialize"<<std::endl;
}

void SpecificWorker::sm_finalize()
{
//	std::cout<<"Entered final state finalize"<<std::endl;
}





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

void SpecificWorker::AGMExecutiveTopic_edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
//subscribesToCODE
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel.get());

	auto symbol1 = worldModel->getSymbolByIdentifier(modification.a);
	auto symbol2 = worldModel->getSymbolByIdentifier(modification.b);

	if(symbol1.get()->symbolType == "person" or symbol2.get()->symbolType == "person")
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

		auto symbol1 = worldModel->getSymbolByIdentifier(modification.a);
		auto symbol2 = worldModel->getSymbolByIdentifier(modification.b);

		if(symbol1.get()->symbolType == "person" or symbol2.get()->symbolType == "person")
			worldModelChanged = true;
	}

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
	{
        AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
    }


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
		AGMMisc::publishModification(newModel, agmexecutive_proxy, "socialRulesAgentAgent");
	}

	catch(const Ice::Exception& e)
	{
		exit(1);
	}
}