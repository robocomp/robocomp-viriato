
/*
 *    Copyright (C) 2016 by YOUR NAME HERE
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
#include <qt4/QtGui/qdial.h>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	active = false;
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModel = new InnerModel();

	
	setWindowTitle("FakeHumanAgent");
	initializeUI();
//	lastJoystickEvent = QTime::currentTime();
	coordInItem.x = 0;
	coordInItem.y = 0;
	coordInItem.z = 0;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
}

bool SpecificWorker::includeInRCIS(int id, const RoboCompInnerModelManager::Pose3D &pose, std::string meshName)
{
	printf("includeInRCIS begins\n");
	std::string name = "fakeperson" + std::to_string(id);
		
	RoboCompInnerModelManager::meshType mesh;
	mesh.pose.x  = 0;
	mesh.pose.y  = 0;
	mesh.pose.z  = 0;
	mesh.pose.rx = 1.57079632679;
	mesh.pose.ry = 0;
	mesh.pose.rz = 3.1415926535;
	
	mesh.scaleX = mesh.scaleY = mesh.scaleZ = 12;
	mesh.render = 0;
	mesh.meshPath = "/home/robocomp/robocomp/components/robocomp-araceli/models/" + meshName + ".3ds";

	try
	{
		innermodelmanager_proxy->addTransform(name, "static", "root", pose);
		innermodelmanager_proxy->addMesh(name+"_mesh", name, mesh);
	}
	catch (...)
	{
		printf("Can't create fake peson\n");
		return false;
	}
	printf("includeInRCIS ends\n");
	return true;
}

bool SpecificWorker::removeFromRCIS(int id)
{
	printf("removeFromRCIS begins\n");
	try
	{
		std::string name = personMap[id].name;
		std::cout<<"name "<<name<<std::endl;
		innermodelmanager_proxy->removeNode(name);
	}
	catch (std::exception& e)
	{
		std::cout<<"Can't delete fake person "<<e.what()<<std::endl;
		return false;
	}
	printf("removeFromRCIS ends\n");
	return true;
}

int SpecificWorker::includeInAGM(int id,const RoboCompInnerModelManager::Pose3D &pose, std::string mesh)
{
	printf("includeInAGM begins\n");
	
	std::string name = "person" + std::to_string(id);
	std::string imName = "fakeperson" + std::to_string(id);
	int personSymbolId = -1;
	int idx=0;
	while ((personSymbolId = worldModel->getIdentifierByType(name, idx++)) != -1)
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
		printf("Fake person already in the AGM model\n");
		return personSymbolId;
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
	newModel->addEdge(person, personSt, name);
	
	
	//Add person in room ==> It has to be fixed to allow use it in different scenarios 
	if (pose.z < 0 )
	{
		newModel->addEdgeByIdentifiers(person->identifier, 5, "in");
	}else{
		newModel->addEdgeByIdentifiers(person->identifier, 3, "in");
	}

	// Geometric part
	std::map<std::string, std::string> edgeRTAtrs;
	edgeRTAtrs["tx"] = std::to_string(pose.x);
	edgeRTAtrs["ty"] = "0";
	edgeRTAtrs["tz"] = std::to_string(pose.z);
	edgeRTAtrs["rx"] = "0";
	edgeRTAtrs["ry"] = std::to_string(pose.ry);
	edgeRTAtrs["rz"] = "0";
	newModel->addEdgeByIdentifiers(100, person->identifier, "RT", edgeRTAtrs);


	AGMModelSymbol::SPtr personMesh = newModel->newSymbol("mesh_"+name);
	printf("personMesh %d\n", personMesh->identifier);
	personMesh->setAttribute("collidable", "false");
	personMesh->setAttribute("imName", imName + "_Mesh");
	personMesh->setAttribute("imType", "mesh");
	std::string meshPath = "/home/robocomp/robocomp/components/robocomp-araceli/models/" + mesh + ".3ds";
	personMesh->setAttribute("path", meshPath);
	personMesh->setAttribute("render", "NormalRendering");
	personMesh->setAttribute("scalex", "12");
	personMesh->setAttribute("scaley", "12");
	personMesh->setAttribute("scalez", "12");

	edgeRTAtrs["tx"] = "0";
	edgeRTAtrs["ty"] = "0";
	edgeRTAtrs["tz"] = "0";
	edgeRTAtrs["rx"] = "1.570796326794";
	edgeRTAtrs["ry"] = "0";
	edgeRTAtrs["rz"] = "3.1415926535";
	newModel->addEdge(person, personMesh, "RT", edgeRTAtrs);

	while (true)
	{
		if(sendModificationProposal(worldModel, newModel))
		{
			break;
		}
		sleep(1);
	}
	printf("includeInAGM ends\n");
	return personSymbolId;
}

//Change 
void SpecificWorker::changePersonRoom(int personId, int roomId)
{
	std::cout<<"Change room: person "<<personId <<" new room: "<<roomId <<std::endl;
	// Get current roomId
	AGMModelSymbol::SPtr personAGM = worldModel->getSymbol(personId);
	int actualRoomId = -1;
	for (auto edge = personAGM->edgesBegin(worldModel); edge != personAGM->edgesEnd(worldModel); edge++)
	{
		const std::pair<int32_t, int32_t> symbolPair = edge->getSymbolPair();
		if (edge->getLabel() == "in")
		{
			const string secondType = worldModel->getSymbol(symbolPair.second)->symbolType;
			if (symbolPair.first == personId and secondType == "room")
			{
				actualRoomId = symbolPair.second;
				break;
			}
		}
	}
	// Modify IN edge
	try
	{
		AGMModel::SPtr newModel(new AGMModel(worldModel));
		if (actualRoomId != -1)
		{
			std::cout<<"remove edge "<<actualRoomId<<std::endl;			
			newModel->removeEdgeByIdentifiers(personId, actualRoomId, "in");
		}
		newModel->addEdgeByIdentifiers(personId, roomId, "in");
 		sendModificationProposal(worldModel, newModel);
	}
	catch (...)
	{
		printf("Can't update person in room... !!!\n");
	}
}

bool SpecificWorker::removeFromAGM(int id)
{
	printf("removeFromAGM begins\n");
	bool result = false;
	AGMModel::SPtr newModel(new AGMModel(worldModel));
	newModel->removeEdgesRelatedToSymbol(personMap[id].personSymbolId);
	newModel->removeSymbol(personMap[id].personSymbolId);
	while (true)
	{
		try
		{
			sendModificationProposal(worldModel, newModel);
			result = true;
			break;
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
		sleep(1);
	}
	printf("removeFromAGM ends\n");
	return result;
}


/*void SpecificWorker::receivedJoyStickEvent(int value, int type, int number)
{
	printf("*\n");
	fflush(stdout);
	if (type != 2)
		return;
	
	if (number == 0) // rot
	{
		humanRotVel = float(value)/32767.*1.;
		lastJoystickEvent = QTime::currentTime();
	}
	if (number == 1)
	{
		humanAdvVel = float(-value)/32767.*600.;
		lastJoystickEvent = QTime::currentTime();
	}
	printf("*");
	fflush(stdout);
}*/

void SpecificWorker::initializeUI(){
	//Teclado
	//UP
	connect(up,SIGNAL(pressed()),this,SLOT(upP()));
	connect(up,SIGNAL(released()),this,SLOT(upR()));
	//DOWN
	connect(down,SIGNAL(pressed()),this,SLOT(downP()));
	connect(down,SIGNAL(released()),this,SLOT(downR()));
	//RIGHT
	connect (right,SIGNAL(pressed()),this,SLOT(rightP()));
	connect (right,SIGNAL(released()),this,SLOT(rightR()));
	//LEFT
	connect(left,SIGNAL(pressed()),this,SLOT(leftP()));
	connect(left,SIGNAL(released()),this,SLOT(leftR()));
	//GIRO
	connect (giro,SIGNAL(valueChanged(int)),this,SLOT(rotar(int)));
	connect (giro,SIGNAL(sliderPressed()),this,SLOT(giroP()));
	connect (giro,SIGNAL(sliderReleased()),this,SLOT(giroR()));
	//giro->setNotchesVisible(true);
	giro->QAbstractSlider::setMinimum (0);
	giro->QAbstractSlider::setMaximum (360);
	giro->QAbstractSlider::setSliderPosition(0);

	connect(add_pb, SIGNAL(clicked()), this, SLOT(addPerson()));
	connect(del_pb, SIGNAL(clicked()), this, SLOT(delPerson()));
	connect(setPose_pb, SIGNAL(clicked()), this, SLOT(setPose()));
	
	connect(save_pb, SIGNAL(clicked()), this, SLOT(savePoints()));
	connect(load_pb, SIGNAL(clicked()), this, SLOT(loadPoints()));
	//connect(person_cb, SIGNAL(currentIndexChanged(int)), this, SLOT(personChanged(int)));
	connect(interaction_cb, SIGNAL(currentIndexChanged(int)), this, SLOT(interactionChanged(int)));
	connect(autoM_cb, SIGNAL(clicked()),this, SLOT(autoMovement()));
	connect(rinteraction_pb, SIGNAL(clicked()),this, SLOT(removeEdgeAGM()));
	connect(ainteraction_pb, SIGNAL(clicked()),this, SLOT(addInteraction()));
//	connect(point_te, SIGNAL(textChanged()), this, SLOT(pointsChanged()));
	
	//disable interface elements 
	interactionChanged(0);
	updatePersonInterfaz(false);
	rinteraction_pb->setEnabled(interaction_lw->count() > 0);
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

	// Joystick
	/*printf("Creating joystick...\n");
	joystick = new QJoyStick("/dev/input/js0");
	if (!joystick->openQJoy())
	{
		cout << "[" << PROGRAM_NAME << "]: Unable to open device: " << joystick->getDeviceName() << endl;
		return EXIT_FAILURE;
	}
	joystick->start();
	printf("Connecting joystick...\n");
	connect(joystick, SIGNAL(inputEvent(int, int, int)), this, SLOT(receivedJoyStickEvent(int, int, int)));
	
*/

	timer.start(Period);
	
	 
	return true;
}

//UP
void SpecificWorker::upP(){
	coordInItem.x = 0;
	coordInItem.z = HUMANADVVEL;
	moveFlag = true;
}
void SpecificWorker::upR(){
	moveFlag = false;
}
//DOWN
void SpecificWorker::downP(){
	coordInItem.x = 0;
	coordInItem.z = -HUMANADVVEL;
	moveFlag = true;
}
void SpecificWorker::downR(){
	moveFlag = false;
}
//RIGHT
void SpecificWorker::rightP(){
  	coordInItem.x = HUMANADVVEL;
	coordInItem.z = 0;
	moveFlag = true;
}
void SpecificWorker::rightR(){
	moveFlag = false;
}
//LEFT
void SpecificWorker::leftP(){
  	coordInItem.x = -HUMANADVVEL;
	coordInItem.z = 0;
	moveFlag = true;
}
void SpecificWorker::leftR(){
	moveFlag = false;
}
//ROT
void SpecificWorker::rotar(int value){
	coordInItem.x = 0;
	coordInItem.z = 0;
	valorgiro = value * 0.0175;
}
void SpecificWorker::giroP(){
	moveFlag = true;
}
void SpecificWorker::giroR(){ 
	moveFlag = false;
}

void SpecificWorker::addPerson()
{
	qDebug()<<"add clicked";
	RoboCompInnerModelManager::Pose3D pose;
	pose.x = x_sb->value();
	pose.y = 0;
	pose.z = z_sb->value();
	pose.rx = 0.f;
	pose.ry = rot_sb->value();
	pose.rz = 0.f;
	std::string mesh = mesh_cb->currentText().toStdString();
	int id = personMap.size() + 1;
	// avoid inserting same element twice
	while (personMap.find(id) != personMap.end())
		id++;
	
	// Include person in RCIS
	if (includeInRCIS(id, pose, mesh))
	{
		// Include person in AGM
		int personSymbolId = includeInAGM(id, pose, mesh);
		if (personSymbolId != -1)
		{
			TPerson person;
			person.autoMovement = false;
			person.pose = pose;
			person.personSymbolId = personSymbolId;
			person.name = "fakeperson" + std::to_string(id);
			personMap.insert(std::pair<int,TPerson>(personSymbolId,person));
			//include in comboBox
			person_cb->addItem(QString::number(personSymbolId));
			int1_cb->addItem(QString::number(personSymbolId));
			int2_cb->addItem(QString::number(personSymbolId));
			updatePersonInterfaz(true);
		}
	}
}
void SpecificWorker::delPerson()
{
	qDebug()<<"del clicked";
	if (person_cb->currentText() == ""){
		QMessageBox::information(this, "No person selected", "You have to select any person to delete");
	}
	else{
		qDebug()<<"Person selected" << person_cb->currentText();
		int personId = person_cb->currentText().toInt();
		removeFromRCIS(personId);
		removeFromAGM(personId);
		person_cb->removeItem(person_cb->currentIndex());
		int1_cb->removeItem(int1_cb->findText(QString::number(personId)));
		int2_cb->removeItem(int2_cb->findText(QString::number(personId)));
		cleanListWidget(personId);
		personMap.erase(personId);
		updatePersonInterfaz(personMap.size() > 0);
		rinteraction_pb->setEnabled(interaction_lw->count() > 0);
	}
}

//MOVE
void SpecificWorker::move()
{
	if (person_cb->currentText() == ""){
		std::cout<<"No selected person to move"<<std::endl;
	}
	else{
		TPerson *person = &personMap[person_cb->currentText().toInt()];
		movePerson(person, coordInItem, setPoseFlag);
	}
}

void SpecificWorker::movePerson(TPerson *person, RoboCompInnerModelManager::coord3D coordInItem ,bool global)
{
	bool changeRoom = false;
	RoboCompInnerModelManager::coord3D coordInWorld;
	try{
		if (global)
			innermodelmanager_proxy->transform("root", "world", coordInItem, coordInWorld);
		else
			innermodelmanager_proxy->transform("root", person->name, coordInItem, coordInWorld);
	}
	catch (std::exception& e)
	{
		std::cout<<"Exception retrieving transform from RCIS: "<<e.what()<<std::endl;
		return;
	}
	//check room change
	if ((coordInWorld.z < 0 and person->pose.z >= 0) or (coordInWorld.z >= 0 and person->pose.z < 0))
	{
		changeRoom = true;
	}
		
	RoboCompInnerModelManager::Pose3D pose;
	pose.x = coordInWorld.x;
	pose.y = coordInWorld.y;
	pose.z = coordInWorld.z;
	pose.rx = 0;
	pose.ry = valorgiro;
	pose.rz = 0;
	//store new position
	person->pose = pose;
	//move in RCIS
	try{
		innermodelmanager_proxy->setPoseFromParent(person->name, pose);
	}
	catch (std::exception& e)
	{
		std::cout<<"Exception moving person in RCIS: "<<e.what()<<std::endl;
		return;
	}
	//move in AGM
	AGMModelSymbol::SPtr personParent = worldModel->getParentByLink(person->personSymbolId, "RT");
	AGMModelEdge &edgeRT  = worldModel->getEdgeByIdentifiers(personParent->identifier, person->personSymbolId, "RT");
	edgeRT.attributes["tx"] = float2str(coordInWorld.x);
	edgeRT.attributes["ty"] = float2str(coordInWorld.y);
	edgeRT.attributes["tz"] = float2str(coordInWorld.z);
	edgeRT.attributes["rx"] = "0";
	edgeRT.attributes["ry"] = float2str(valorgiro);
	edgeRT.attributes["rz"] = "0";
	try
	{
		AGMMisc::publishEdgeUpdate(edgeRT, agmexecutive_proxy);
	}
	catch(std::exception& e)
	{
		std::cout<<"Exception moving in AGM: "<<e.what()<<std::endl;
	}
	
	//change room when needed
	if(changeRoom)
	{
		int newRoom = 3;
		//TODO ==> reevaluate to make it valid for any world
		if (person->pose.z < 0 ){
			newRoom = 5;
		}
		changePersonRoom(person->personSymbolId, newRoom);
	}
}

void SpecificWorker::compute()
{
	QMutexLocker locker(mutex);
	static int firstTime=true;
	if (firstTime) //retrieve model after initialization
	{
		RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
		structuralChange(w);
		firstTime = false;
		//add robot ID to interaction comboBox
		try{
			robotID = worldModel->getIdentifierByType("robot");
			int2_cb->addItem(QString::number(robotID));
		}catch(...){
			std::cout<<"No robot found in AGM model"<<std::endl;
		}
		
	}
	//static QTime lastCompute = QTime::currentTime();
	
    try
	{
		if (moveFlag or setPoseFlag)
		{
			move();
			setPoseFlag = false;
		}

    }
    catch(...){}

	
	//move person
	for (auto iterator: personMap)
	{
		if (iterator.second.autoMovement)
		{
			RoboCompInnerModelManager::coord3D coordInItem = autoMovePerson(iterator.second);
			movePerson(&iterator.second, coordInItem);
		}
	}
	
	
	

	/*if (lastJoystickEvent.elapsed()  < 3000)
	{
		printf("vel: %f %f\n", humanAdvVel, humanRotVel);
		RoboCompInnerModelManager::coord3D coordInItem;
		coordInItem.x = 0;
		coordInItem.y = 0;
		coordInItem.z = humanAdvVel*0.001*lastCompute.elapsed();
		RoboCompInnerModelManager::coord3D coordInBase;
		printf("transform (%f, %f, %f) to fake person (%f, %f, %f)\n", coordInItem.x, coordInItem.y, coordInItem.z, coordInBase.x, coordInBase.y, coordInBase.z);
		innermodelmanager_proxy->transform("root", "fakeperson",  coordInItem, coordInBase);

		RoboCompInnerModelManager::Pose3D pose;
		humanRot += humanRotVel*0.001*lastCompute.elapsed(); 
		pose.x = coordInBase.x;
		pose.y = coordInBase.y;
		pose.z = coordInBase.z;
		pose.rx = 0;
		pose.ry = humanRot;
		pose.rz = 0;
		innermodelmanager_proxy->setPoseFromParent("fakeperson", pose);

		AGMModelSymbol::SPtr personParent = worldModel->getParentByLink(personSymbolId, "RT");
		AGMModelEdge &edgeRT  = worldModel->getEdgeByIdentifiers(personParent->identifier, personSymbolId, "RT");
		edgeRT.attributes["tx"] = float2str(coordInBase.x);
		edgeRT.attributes["ty"] = float2str(coordInBase.y);
		edgeRT.attributes["tz"] = float2str(coordInBase.z);
		edgeRT.attributes["rx"] = "0";
		edgeRT.attributes["ry"] = float2str(humanRot);
		edgeRT.attributes["rz"] = "0";
		printf("%d----[%f]--->%d\n", personParent->identifier, coordInBase.z, personSymbolId);
		AGMMisc::publishEdgeUpdate(edgeRT, agmexecutive_proxy);
	}
	else
	{
		printf(".");
		fflush(stdout);
	}



	lastCompute = QTime::currentTime();

*/
}

void SpecificWorker::setPose()
{
	if (person_cb->currentText() == ""){
		QMessageBox::information(this, "No person selected", "You have to select any person to change its pose");
	}
	else{
		coordInItem.x = x_sb->value();
		coordInItem.z = z_sb->value();
		valorgiro = rot_sb->value();
		setPoseFlag = true;
	}
}

//***************************
//		AGENT RELATED
//***************************

bool SpecificWorker::reloadConfigAgent()
{
	return true;
}

bool SpecificWorker::activateAgent(const ParameterMap &prs)
{
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
	bool activated = false;
	return setParametersAndPossibleActivation(prs, activated);
}

ParameterMap SpecificWorker::getAgentParameters()
{
	return params;
}

void SpecificWorker::killAgent()
{

}

int SpecificWorker::uptimeAgent()
{
	return 0;
}

bool SpecificWorker::deactivateAgent()
{
	return deactivate();
}

StateStruct SpecificWorker::getAgentState()
{
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
	mutex->lock();
 	AGMModelConverter::fromIceToInternal(w, worldModel);
	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
	mutex->unlock();
}

void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{
	QMutexLocker locker(mutex);
	for (auto modification : modifications)
	{
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
		AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel);
	}
}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel);
}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
}

void SpecificWorker::symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications)
{
	QMutexLocker l(mutex);
	for (auto modification : modifications)
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
}



bool SpecificWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
	QMutexLocker l(mutex);
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
		printf("exception in setParametersAndPossibleActivation\n");
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

bool SpecificWorker::sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel)
{
	bool result = false;
	try
	{	qDebug()<<"Intentando sendModificationProposal";
		AGMMisc::publishModification(newModel, agmexecutive_proxy, "fakeHumanAgentAgent");
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

// save points into file
void SpecificWorker::savePoints()
{
	QString nomFileH = QFileDialog::getSaveFileName(this, tr("File Name"), "",tr("Text file (*.txt)"));
	if (nomFileH != "") {
		QFile fileH(nomFileH);
		if (fileH.open(QFile::WriteOnly)) {
			QTextStream out(&fileH);
			out << point_te->toPlainText();
		}
	}
}
// load points from file
void SpecificWorker::loadPoints()
{
	if (person_cb->currentText() == ""){
		QMessageBox::information(this, "No person selected", "You have to select any person to load points");
	}
	else{
		QString nomFileH = QFileDialog::getOpenFileName(this, tr("File Name"), "",tr("Text file (*.txt)"));
		if (nomFileH != "") {
			QFile fileH(nomFileH);
			if (fileH.open(QFile::ReadOnly)) {
				QTextStream in(&fileH);
				QString line = in.readAll();
				point_te->clear();
				point_te->append(line);
			}
		}
	}
}

// Reload person information
void SpecificWorker::personChanged(int index)
{
	if (person_cb->currentText() == ""){
		QMessageBox::information(this, "No person selected", "You have to select any person to delete");
	}
	else{
		TPerson person = personMap[person_cb->currentText().toInt()];
		point_te->clear();
		for(const auto& s: person.points)
		{
			std::cout << s.x << std::endl;
			//point_te->append(line);
		}
	}
}
//enable/disable person automovement
void SpecificWorker::autoMovement()
{
	TPerson person = personMap[person_cb->currentText().toInt()];
	person.autoMovement = autoM_cb->isChecked();
	
	pointsChanged();
}


void SpecificWorker::pointsChanged()
{
	if (person_cb->currentText() == ""){
		std::cout<<"No selected person to move"<<std::endl;
	}
	else{
		autoM_cb->setChecked(false);
		TPerson person = personMap[person_cb->currentText().toInt()];
		person.points.clear();
		RoboCompInnerModelManager::Pose3D pose;
		QStringList data = point_te->toPlainText().split(QRegExp("[\n]"),QString::SkipEmptyParts);
		for (auto line: data)
		{
			try
			{
				QStringList aux = line.split(QRegExp("[;]"),QString::SkipEmptyParts);
				if (aux.size() == 3)
				{
					pose.x = aux[0].toFloat();
					pose.z = aux[1].toFloat();
					pose.y = 0;
					pose.rx = 0;
					pose.ry = 0;
					pose.rz = 0;
					person.points.push_back(pose);
				}
				else
				{
					std::cout << "Skipping no complete position: " << line.toStdString() << std::endl;
				}
			}catch(...)
			{
				std::cout << "Exception" << std::endl;
			}
		}
	}
}
#define MIN_DISTANCE 100
RoboCompInnerModelManager::coord3D SpecificWorker::autoMovePerson(TPerson person)
{
	std::cout << "Auto move person" << std::endl;
	RoboCompInnerModelManager::coord3D nextPose;
	//compute distance to next point
	QVec current = QVec::vec2(person.points[person.currentPoint].x, person.points[person.currentPoint].z);
	QVec actual = QVec::vec2(person.pose.x, person.pose.z);
	
	QVec next = (current - actual);
	
	
	
	// check if next point is already achieve
	if ( (current - actual).norm2() < MIN_DISTANCE){
		if (person.currentPoint == person.points.size()){
			person.currentPoint = 0;
		}else{
			person.currentPoint++;
		}
	}
	// BASIC_PERIOD 
	return nextPose;
}

//insert one person interaction edge in AGM
void SpecificWorker::addInteraction()
{
	int id1 = -1;
	int id2 = -1;
	std::string edgeName = "";
	QString listEntry;
	
	TInteraction option = string2Interaction(interaction_cb->currentText().toStdString());
	id1 = int1_cb->currentText().toInt();
	switch(option)
	{
		case isBusy:	edgeName = "isBusy";
						id2 = int1_cb->currentText().toInt();
						listEntry = int1_cb->currentText() + QString(" => isBusy");
						break;
		case block:		id2 = robotID;
						edgeName = "block";
						listEntry = int1_cb->currentText() + QString(" => block => ") + QString::number(robotID);
						break;
		case softBlock:	id2 = robotID;
						edgeName = "softBlock";
						listEntry = int1_cb->currentText() + QString(" => softBlock => ") + QString::number(robotID);
						break;
		case interacting:	id2 = int2_cb->currentText().toInt();
							if (id1 == id2)
							{
								std::cout << "Person could not interact with himself" << std::endl;
								QMessageBox::information(this, "Interaction", "Person could not interact with himself");
								return;
							}
							edgeName = "interacting";
							listEntry = int1_cb->currentText() + QString(" => interacting => ") + int2_cb->currentText();
							break;
		case unknown:	std::cout <<"Unknown interaction selected, please check TInteraction valid values" <<std::endl;
						return;
	}
	QList<QListWidgetItem*> list = interaction_lw->findItems(listEntry, Qt::MatchExactly);
	if (list.size() > 0)
	{
		std::cout << "Interaction is already used" << std::endl;
		QMessageBox::information(this, "Interaction", "Interaction is already used");
	}
	else
	{
		AGMModel::SPtr newModel(new AGMModel(worldModel));
		newModel->addEdgeByIdentifiers(id1, id2, edgeName);
		if(sendModificationProposal(worldModel, newModel))
		{
			try{
				AGMModelEdge edge = newModel->getEdgeByIdentifiers(id1, id2, edgeName);
				QListWidgetItem *item = new QListWidgetItem(listEntry);
				item->setData(Qt::UserRole, QVariant::fromValue<AGMModelEdge>(edge));
				interaction_lw->addItem(item);
			}catch(...)
			{
				std::cout << "Error retrieving " << edgeName << " edge from newModel" << std::endl;
			}
		}
		rinteraction_pb->setEnabled(interaction_lw->count() > 0);
	}
}
//Remove edge from AGM
void SpecificWorker::removeEdgeAGM()
{
	std::cout << "Remove edge " << std::endl;
	QListWidgetItem *item = interaction_lw->currentItem();
	if (item != NULL)
	{
		AGMModelEdge edge = item->data(Qt::UserRole).value<AGMModelEdge>();
		AGMModel::SPtr newModel(new AGMModel(worldModel));
		newModel->removeEdge(edge);
		if(sendModificationProposal(worldModel, newModel))
		{
			std::cout << "remove item from list" << std::endl;
			interaction_lw->removeItemWidget(item);
			delete item;
		}
	}
	else{
		QMessageBox::information(this, "Interaction", "No interaction is selected");
	}
	// disable button is no interaction left
	rinteraction_pb->setEnabled(interaction_lw->count() > 0);
}

//remove any entri related with personID (it is used when person is removed)
void SpecificWorker::cleanListWidget(int personID)
{
	QList<QListWidgetItem*> list = interaction_lw->findItems(QString::number(personID), Qt::MatchContains);
	for (auto item: list)
	{
		interaction_lw->removeItemWidget(item);
		delete item;
	}
}

//convert std::string to enum TInteraction
SpecificWorker::TInteraction SpecificWorker::string2Interaction(std::string interaction)
{
	if (interaction == "isBusy") 		return isBusy;
	if (interaction == "interacting") 	return interacting;
	if (interaction == "block") 		return block;
	if (interaction == "softBlock") 	return softBlock;
	return unknown;
}

void SpecificWorker::interactionChanged(int index)
{
	TInteraction option = string2Interaction(interaction_cb->currentText().toStdString());
	switch(option)
	{
		case isBusy:	int2_cb->setEnabled(false);
						break;
		case block:		int2_cb->setCurrentIndex(int2_cb->findText(QString::number(robotID),Qt::MatchExactly));
						int2_cb->setEnabled(false);
						break;
		case softBlock:	int2_cb->setCurrentIndex(int2_cb->findText(QString::number(robotID),Qt::MatchExactly));
						int2_cb->setEnabled(false);
						break;
		case interacting:	int2_cb->setEnabled(true);
							break;
		case unknown:	std::cout <<"Unknown interaction selected, please check TInteraction valid values" <<std::endl;
						break;
	}
}

// enable/disable interface elements (used to disable all element when no person)
void SpecificWorker::updatePersonInterfaz(bool enable)
{
	move_gb->setEnabled(enable);
	interacion_gb->setEnabled(enable);
	pose_gb->setEnabled(enable);
	points_gb->setEnabled(enable);
}
