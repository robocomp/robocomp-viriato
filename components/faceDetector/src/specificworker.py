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
#

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from genericworker import *
# AGM related imports
import AGMModelConversion
from AGGL import *


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 2000
        if startup_check:
            self.startup_check()
        else:
            self.timer.start(self.Period)
            self.defaultMachine.start()
            self.destroyed.connect(self.t_compute_to_finalize)

        # Agm related initialization
        self.initAGM()

    def initAGM(self):
        self.worldModel = AGMGraph()
        try:
            w = self.agmexecutive_proxy.getModel()
            self.AGMExecutiveTopic_structuralChange(w)
            # get all the list of persons

        except:
            print("The executive is probably not running, waiting for first AGM model publication...")
        return True

    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        return True




    @QtCore.Slot()
    def compute(self):
        print('SpecificWorker.compute...')
        if self.searchPeopleNearRobot():
            print('Buscar en la camara si estan de espaldas o no')
            pass

        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    def searchPeopleNearRobot(self):
        for link in list(self.worldModel.links):
            if link.linkType == 'is_near':
                type_a = self.worldModel.getNode(link.a).sType
                type_b = self.worldModel.getNode(link.b).sType

                if type_a == 'robot' and type_b == 'person' or type_a == 'person' and type_b == 'robot':
                    print('robot is near person')
                    return True
        return False


    # =============== Slots methods for State Machine ===================
    # ===================================================================

    #
    # sm_initialize
    #
    @QtCore.Slot()
    def sm_initialize(self):
        print("Entered state initialize")
        self.t_initialize_to_compute.emit()
        pass
    

    #
    # sm_compute
    #
    @QtCore.Slot()
    def sm_compute(self):
        print("Entered state compute")
        self.compute()
        pass


    #
    # sm_finalize
    #
    @QtCore.Slot()
    def sm_finalize(self):
        print("Entered state finalize")
        pass

    # =================================================================
    # =================================================================

    # =============== Methods for Component SubscribesTo ================
    # ===================================================================

    #
    # SUBSCRIPTION to edgeUpdated method from AGMExecutiveTopic interface
    #
    def AGMExecutiveTopic_edgeUpdated(self, modification):
    
        #
        # write your CODE here
        #
        pass


    #
    # SUBSCRIPTION to edgesUpdated method from AGMExecutiveTopic interface
    #
    def AGMExecutiveTopic_edgesUpdated(self, modifications):
    
        #
        # write your CODE here
        #
        pass


    #
    # SUBSCRIPTION to selfEdgeAdded method from AGMExecutiveTopic interface
    #
    def AGMExecutiveTopic_selfEdgeAdded(self, nodeid, edgeType, attributes):
    
        #
        # write your CODE here
        #
        pass


    #
    # SUBSCRIPTION to selfEdgeDeleted method from AGMExecutiveTopic interface
    #
    def AGMExecutiveTopic_selfEdgeDeleted(self, nodeid, edgeType):
    
        #
        # write your CODE here
        #
        pass


    #
    # SUBSCRIPTION to structuralChange method from AGMExecutiveTopic interface
    #
    def AGMExecutiveTopic_structuralChange(self, w):
        self.mutex.lock()
        self.worldModel = AGMModelConversion.fromIceToInternal_model(w)
        self.mutex.unlock()



    #
    # SUBSCRIPTION to symbolUpdated method from AGMExecutiveTopic interface
    #
    def AGMExecutiveTopic_symbolUpdated(self, modification):
    
        #
        # write your CODE here
        #
        pass


    #
    # SUBSCRIPTION to symbolsUpdated method from AGMExecutiveTopic interface
    #
    def AGMExecutiveTopic_symbolsUpdated(self, modifications):
    
        #
        # write your CODE here
        #
        pass


    # ===================================================================
    # ===================================================================


    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of activateAgent method from AGMCommonBehavior interface
    #
    def AGMCommonBehavior_activateAgent(self, prs):
        ret = bool()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of deactivateAgent method from AGMCommonBehavior interface
    #
    def AGMCommonBehavior_deactivateAgent(self):
        ret = bool()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of getAgentParameters method from AGMCommonBehavior interface
    #
    def AGMCommonBehavior_getAgentParameters(self):
        ret = RoboCompAGMCommonBehavior.ParameterMap()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of getAgentState method from AGMCommonBehavior interface
    #
    def AGMCommonBehavior_getAgentState(self):
        ret = RoboCompAGMCommonBehavior.StateStruct()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of killAgent method from AGMCommonBehavior interface
    #
    def AGMCommonBehavior_killAgent(self):
    
        #
        # write your CODE here
        #
        pass


    #
    # IMPLEMENTATION of reloadConfigAgent method from AGMCommonBehavior interface
    #
    def AGMCommonBehavior_reloadConfigAgent(self):
        ret = bool()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of setAgentParameters method from AGMCommonBehavior interface
    #
    def AGMCommonBehavior_setAgentParameters(self, prs):
        ret = bool()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of uptimeAgent method from AGMCommonBehavior interface
    #
    def AGMCommonBehavior_uptimeAgent(self):
        ret = int()
        #
        # write your CODE here
        #
        return ret
    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompAGMExecutive you can call this methods:
    # self.agmexecutive_proxy.activate(...)
    # self.agmexecutive_proxy.addSelfEdge(...)
    # self.agmexecutive_proxy.broadcastModel(...)
    # self.agmexecutive_proxy.broadcastPlan(...)
    # self.agmexecutive_proxy.deactivate(...)
    # self.agmexecutive_proxy.delSelfEdge(...)
    # self.agmexecutive_proxy.edgeUpdate(...)
    # self.agmexecutive_proxy.edgesUpdate(...)
    # self.agmexecutive_proxy.getData(...)
    # self.agmexecutive_proxy.getEdge(...)
    # self.agmexecutive_proxy.getModel(...)
    # self.agmexecutive_proxy.getNode(...)
    # self.agmexecutive_proxy.setMission(...)
    # self.agmexecutive_proxy.structuralChangeProposal(...)
    # self.agmexecutive_proxy.symbolUpdate(...)
    # self.agmexecutive_proxy.symbolsUpdate(...)

    ######################
    # From the RoboCompAGMCommonBehavior you can use this types:
    # RoboCompAGMCommonBehavior.StateStruct
    # RoboCompAGMCommonBehavior.Parameter

