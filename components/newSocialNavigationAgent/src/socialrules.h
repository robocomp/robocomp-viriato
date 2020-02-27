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
#include <QtCore/QVariant>

#include <QtWidgets/QTimeEdit>
#include <QtWidgets/QListWidget>

class SocialRules :public QObject
{
Q_OBJECT
public:
	using InnerPtr = std::shared_ptr<InnerModel>;
	SocialNavigationGaussianPrx socialnavigationgaussian_proxy;

	QComboBox* idselect_combobox;
	QComboBox* idobject_combobox;
	QCheckBox* follow_checkbox;
	QCheckBox* accompany_checkbox;
	QCheckBox* passonright_checkbox;

    QSlider *object_slider;
    QListWidget *therapies_list;
    QTimeEdit *startTherapy_timeEdit;
    QTimeEdit *endTherapy_timeEdit;
	QTimeEdit *currentTime_timeEdit;



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
	vector<vector<int32_t>> prevInteractingId = {};


	SNGPersonSeq movperson; //moving person

	struct ObjectType
	{
//		QString imName; //Nombre del nodo geometrico en AGM
        int id;
        float x;
        float z;
        float rot;

        float cost = 1.5;
        SNGPolyline affordance;

        QString shape;
        float width;
        float depth;
        float height;
        float inter_space;
        float inter_angle;


        bool therapyProgrammed = false;
        QTime startT;
        QTime endT;

	};

    std::map<QString, ObjectType> mapIdObjects;

    bool costChanged = false;
	////////////////////////////
	SNGPolylineSeq intimate_seq;
	SNGPolylineSeq personal_seq;
	SNGPolylineSeq social_seq;
	SNGPolylineSeq object_seq, object_lowProbVisited, object_mediumProbVisited, object_highProbVisited;
	SNGPolylineSeq objectblock_seq;

    using retPolylines = std::tuple< bool, SNGPersonSeq, SNGPolylineSeq,SNGPolylineSeq,SNGPolylineSeq,SNGPolylineSeq,SNGPolylineSeq,SNGPolylineSeq,SNGPolylineSeq,SNGPolylineSeq>;

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
	bool checkInteractions();
	vector <vector<int32_t>> groupInteractingPeople(int32_t id, int32_t pairId,vector<vector<int32_t>> &interactingId);

	SNGPolyline affordanceTrapezoidal(ObjectType obj);
	SNGPolyline affordanceRectangular(ObjectType obj);
	SNGPolyline affordanceCircular(ObjectType obj);
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
	void drawGauss();
	SNGPolylineSeq PassOnRight(bool draw = true);
	void checkObjectAffordance(bool d = true);

	void affordanceSliderChanged(int value);
	void affordanceTimeChanged(int step);
	void programTherapy();
	void removeTherapy();


};


#endif // SOCIALRULES_H
