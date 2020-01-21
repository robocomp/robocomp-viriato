/*
 * Copyright 2017 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */
#ifndef SOCIALRULES_H
#define SOCIALRULES_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <boost/format.hpp>
#include <QObject>
#include <vector>

class SocialRules :public QObject
{
Q_OBJECT
public:
	using InnerPtr = std::shared_ptr<InnerModel>;
	SocialNavigationGaussianPrx socialnavigationgaussian_proxy;

	QComboBox* idselect_combobox;
	QCheckBox* follow_checkbox;
	QCheckBox* accompany_checkbox;
	QCheckBox* passonright_checkbox;

	bool followpulsed = false;
	bool accompanypulsed = false;
	bool porpulsed = false;

	bool first = true;

	float h = 0.1; //umbral

	int32_t robotSymbolId;

	SNGPerson robot;
	SNGPerson person;

	SNGPersonSeq totalpersons, prevpersons = {};
	std::map<int32_t, SNGPerson> mapIdPersons;
	vector <SNGPersonSeq> interactingpersons; //vector de grupos que interactuan

	SNGPersonSeq movperson; //moving person

	struct ObjectType
	{
		QString imName; //Nombre del nodo geometrico en AGM
		int id;
		float x;
		float z;
		float rot;
		float width;
		float inter_space;
		float inter_angle;
	};


	////////////////////////////
	SNGPolylineSeq intimate_seq;
	SNGPolylineSeq personal_seq;
	SNGPolylineSeq social_seq;
	SNGPolylineSeq object_seq;
	SNGPolylineSeq objectblock_seq;

    using retPolylines = std::tuple< bool, SNGPersonSeq, SNGPolylineSeq,SNGPolylineSeq,SNGPolylineSeq,SNGPolylineSeq,SNGPolylineSeq>;

	//PARA GUARDAR LOS DATOS EN UN ARCHIVO
	struct Point {float x;float z;};
	Point point;
	vector <Point> poserobot;

	//PARA GUARDAR LA DISTANCIA RECORRIDA
	float totaldist = 0;

	void initialize(AGMModel::SPtr worldModel_, SocialNavigationGaussianPrx socialnavigationgaussian_proxy_);
	retPolylines update(AGMModel::SPtr worldModel_);

	void updatePeopleInModel();
	bool peopleChanged();
	void checkInteractions();
	vector <vector<int32_t>> groupInteractingPeople(int32_t id, int32_t pairId,vector<vector<int32_t>> &interactingId);

	SNGPolyline calculateAffordance(ObjectType obj);
	SNGPolylineSeq ApplySocialRules();
	void checkRobotmov();
	bool checkHRI(SNGPerson p, int ind, InnerPtr &i, AGMModel::SPtr w);

private:
	AGMModel::SPtr worldModel;



public slots:
    void checkstate();
	void saveData();
	void goToPerson();
    void followPerson();
    void accompanyPerson();
	SNGPolylineSeq calculateGauss(bool draw = true, float h = 0.1);
	SNGPolylineSeq PassOnRight(bool draw = true);
	void checkObjectAffordance(bool d = true);



};


#endif // SOCIALRULES_H
