/*
 *    Copyright (C) 2020 by YOUR NAME HERE
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
#include <grid.h>
#include <controller.h>
#include <navigation.h>


#define USE_QTGUI
#ifdef USE_QTGUI
	#include "innerviewer.h"
#endif

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
    RoboCompLaser::TLaserData  updateLaser();

	#ifdef USE_QTGUI
		using InnerViewerPtr = std::shared_ptr<InnerViewer>;
	#endif
    
public slots:
	void compute();
	int startup_check();
	void initialize(int period);
private:
	std::shared_ptr<InnerModel>innerModel;
    #ifdef USE_QTGUI
        InnerViewerPtr viewer;
    #endif
	bool startup_check_flag;
    Navigation<Grid<>,Controller> navigation;
    std::shared_ptr<RoboCompCommonBehavior::ParameterList> confParams;    
};

#endif
