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
#include "agmexecutivetopicI.h"

AGMExecutiveTopicI::AGMExecutiveTopicI(GenericWorker *_worker)
{
	worker = _worker;
}


AGMExecutiveTopicI::~AGMExecutiveTopicI()
{
}

void AGMExecutiveTopicI::edgeUpdated(RoboCompAGMWorldModel::Edge modification, const Ice::Current&)
{
	worker->AGMExecutiveTopic_edgeUpdated(modification);
}

void AGMExecutiveTopicI::edgesUpdated(RoboCompAGMWorldModel::EdgeSequence modifications, const Ice::Current&)
{
	worker->AGMExecutiveTopic_edgesUpdated(modifications);
}

void AGMExecutiveTopicI::structuralChange(RoboCompAGMWorldModel::World w, const Ice::Current&)
{
	worker->AGMExecutiveTopic_structuralChange(w);
}

void AGMExecutiveTopicI::symbolUpdated(RoboCompAGMWorldModel::Node modification, const Ice::Current&)
{
	worker->AGMExecutiveTopic_symbolUpdated(modification);
}

void AGMExecutiveTopicI::symbolsUpdated(RoboCompAGMWorldModel::NodeSequence modifications, const Ice::Current&)
{
	worker->AGMExecutiveTopic_symbolsUpdated(modifications);
}

