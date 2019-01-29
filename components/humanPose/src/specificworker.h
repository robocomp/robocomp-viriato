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

/**
       \brief
       @author authorname
*/


#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT

public:


    humansDetected list_of_humans; //PersonType defined in HumanPose.h


    bool first = true;
    bool facefound = false;

    int IDcamera = 0;

    //----------------- intento de relacionar cara con esqueleto -----------------------//
	typedef map <int,int> relID;

	relID IDjointface; //mapa que relaciona los id procedentes del esqueleto con los de la cara
	relID IDfacegeneric;// rel cara con id generico de humanPose
	relID IDjointgeneric;//rel id joint con id generico

	int IDgeneric = 0;
    bool backwards = false;

    //----------------------------------------------//

	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	void getDataFromAstra();
	bool getPoseRot (jointListType list, Pose3D &personpose);
	int getIDgeneric(int idjoint, RoboCompFaceTracking::Faces faces);


public slots:
	void compute();

private:
	InnerModel *innerModel;

};

#endif
