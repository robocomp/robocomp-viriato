#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2020 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.

import sys, Ice, os
from PySide2 import QtWidgets, QtCore

ROBOCOMP = ''
try:
    ROBOCOMP = os.environ['ROBOCOMP']
except KeyError:
    print('$ROBOCOMP environment variable not set, using the default value /opt/robocomp')
    ROBOCOMP = '/opt/robocomp'

Ice.loadSlice("-I ./src/ --all ./src/CommonBehavior.ice")
import RoboCompCommonBehavior

Ice.loadSlice("-I ./src/ --all ./src/AGMCommonBehavior.ice")
import RoboCompAGMCommonBehavior
Ice.loadSlice("-I ./src/ --all ./src/AGMExecutive.ice")
import RoboCompAGMExecutive
Ice.loadSlice("-I ./src/ --all ./src/AGMExecutiveTopic.ice")
import RoboCompAGMExecutiveTopic
Ice.loadSlice("-I ./src/ --all ./src/AGMWorldModel.ice")
import RoboCompAGMWorldModel
Ice.loadSlice("-I ./src/ --all ./src/GenericBase.ice")
import RoboCompGenericBase
Ice.loadSlice("-I ./src/ --all ./src/InnerModelManager.ice")
import RoboCompInnerModelManager
Ice.loadSlice("-I ./src/ --all ./src/Laser.ice")
import RoboCompLaser
Ice.loadSlice("-I ./src/ --all ./src/OmniRobot.ice")
import RoboCompOmniRobot
Ice.loadSlice("-I ./src/ --all ./src/Planning.ice")
import RoboCompPlanning

class NodeInformationSequence(list):
    def __init__(self, iterable=list()):
        super(NodeInformationSequence, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompInnerModelManager.NodeInformation)
        super(NodeInformationSequence, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompInnerModelManager.NodeInformation)
        super(NodeInformationSequence, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompInnerModelManager.NodeInformation)
        super(NodeInformationSequence, self).insert(index, item)

setattr(RoboCompInnerModelManager, "NodeInformationSequence", NodeInformationSequence)

class PointCloudVector(list):
    def __init__(self, iterable=list()):
        super(PointCloudVector, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompInnerModelManager.Colored3DPoint)
        super(PointCloudVector, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompInnerModelManager.Colored3DPoint)
        super(PointCloudVector, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompInnerModelManager.Colored3DPoint)
        super(PointCloudVector, self).insert(index, item)

setattr(RoboCompInnerModelManager, "PointCloudVector", PointCloudVector)

class FloatSeq(list):
    def __init__(self, iterable=list()):
        super(FloatSeq, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, float)
        super(FloatSeq, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, float)
        super(FloatSeq, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, float)
        super(FloatSeq, self).insert(index, item)

setattr(RoboCompInnerModelManager, "FloatSeq", FloatSeq)

class shortVector(list):
    def __init__(self, iterable=list()):
        super(shortVector, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, int)
        super(shortVector, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, int)
        super(shortVector, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, int)
        super(shortVector, self).insert(index, item)

setattr(RoboCompLaser, "shortVector", shortVector)

class TLaserData(list):
    def __init__(self, iterable=list()):
        super(TLaserData, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompLaser.TData)
        super(TLaserData, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompLaser.TData)
        super(TLaserData, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompLaser.TData)
        super(TLaserData, self).insert(index, item)

setattr(RoboCompLaser, "TLaserData", TLaserData)

class StringVector(list):
    def __init__(self, iterable=list()):
        super(StringVector, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, str)
        super(StringVector, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, str)
        super(StringVector, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, str)
        super(StringVector, self).insert(index, item)

setattr(RoboCompPlanning, "StringVector", StringVector)

class ActionSequence(list):
    def __init__(self, iterable=list()):
        super(ActionSequence, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompPlanning.Action)
        super(ActionSequence, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompPlanning.Action)
        super(ActionSequence, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompPlanning.Action)
        super(ActionSequence, self).insert(index, item)

setattr(RoboCompPlanning, "ActionSequence", ActionSequence)


import agmcommonbehaviorI
import agmexecutivetopicI


try:
    from ui_mainUI import *
except:
    print("Can't import UI file. Did you run 'make'?")
    sys.exit(-1)



class GenericWorker(QtWidgets.QWidget):

    kill = QtCore.Signal()

    def __init__(self, mprx):
        super(GenericWorker, self).__init__()

        self.agmexecutive_proxy = mprx["AGMExecutiveProxy"]
        self.innermodelmanager_proxy = mprx["InnerModelManagerProxy"]
        self.laser_proxy = mprx["LaserProxy"]
        self.omnirobot_proxy = mprx["OmniRobotProxy"]

        self.ui = Ui_guiDlg()
        self.ui.setupUi(self)
        self.show()

        self.mutex = QtCore.QMutex(QtCore.QMutex.Recursive)
        self.Period = 30
        self.timer = QtCore.QTimer(self)


    @QtCore.Slot()
    def killYourSelf(self):
        rDebug("Killing myself")
        self.kill.emit()

    # \brief Change compute period
    # @param per Period in ms
    @QtCore.Slot(int)
    def setPeriod(self, p):
        print("Period changed", p)
        self.Period = p
        self.timer.start(self.Period)
