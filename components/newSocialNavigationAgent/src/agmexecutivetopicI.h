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
#ifndef AGMEXECUTIVETOPIC_H
#define AGMEXECUTIVETOPIC_H

// Ice includes
#include <Ice/Ice.h>
#include <AGMExecutiveTopic.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompAGMExecutiveTopic;

class AGMExecutiveTopicI : public virtual RoboCompAGMExecutiveTopic::AGMExecutiveTopic
{
public:
AGMExecutiveTopicI(GenericWorker *_worker);
	~AGMExecutiveTopicI();

	void edgeUpdated(RoboCompAGMWorldModel::Edge modification, const Ice::Current&);
	void edgesUpdated(RoboCompAGMWorldModel::EdgeSequence modifications, const Ice::Current&);
	void structuralChange(RoboCompAGMWorldModel::World w, const Ice::Current&);
	void symbolUpdated(RoboCompAGMWorldModel::Node modification, const Ice::Current&);
	void symbolsUpdated(RoboCompAGMWorldModel::NodeSequence modifications, const Ice::Current&);

private:

	GenericWorker *worker;

};

#endif
