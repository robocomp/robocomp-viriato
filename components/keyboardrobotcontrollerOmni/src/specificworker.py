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
import sys, os, Ice, traceback, curses
from PySide2 import *
from genericworker import *

# get the curses screen window
screen = curses.initscr()

# turn off input echoing
curses.noecho()

# respond to keys immediately (don't wait for enter)
curses.cbreak()

# map arrow keys to special values
screen.keypad(True)

ROBOCOMP = ''

try:
    ROBOCOMP = os.environ['ROBOCOMP']
except:
    print('$ROBOCOMP environment variable not set, using the default value /opt/robocomp')
    ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP) < 1:
    print('genericworker.py: ROBOCOMP environment variable not set! Exiting.')
    sys.exit()

preStr = "-I" + ROBOCOMP + "/interfaces/ --all " + ROBOCOMP + "/interfaces/"

Ice.loadSlice(preStr + "DifferentialRobot.ice")
from RoboCompOmniRobot import *


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.advX = 0
        self.advZ = 0
        self.rot = 0
        screen.addstr(0, 0, 'Connected to robot. Use arrows to control speed, space bar to stop ans ''q'' to exit')

        if startup_check:
            self.startup_check()
        else:
            self.timer.start(self.Period)
            self.defaultMachine.start()
            self.destroyed.connect(self.t_compute_to_finalize)

    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        return True

    tt1 = 2000
    tt2 = 2

    @QtCore.Slot()
    def compute(self):
        print('SpecificWorker.compute...')
        try:
            key = screen.getch()

            if key == curses.KEY_UP:
                if self.advZ > self.tt1:
                    self.advZ = self.advZ
                else:
                    self.advZ = self.advZ + 20;
                screen.addstr(5, 0, 'up: ' + '%.2f' % self.advZ + ' : ' + '%.2f' % self.rot)
                self.omnirobot_proxy.setSpeedBase(self.advX, self.advZ, self.rot)
            elif key == curses.KEY_DOWN:
                if self.advZ < -1 * self.tt1:
                    self.advZ = self.advZ
                else:
                    self.advZ = self.advZ - 20;
                screen.addstr(5, 0, 'down: ' + '%.2f' % self.advZ + ' : ' + '%.2f' % self.rot)
                self.omnirobot_proxy.setSpeedBase(self.advX, self.advZ, self.rot)

            elif key == curses.KEY_RIGHT:
                if self.advX > self.tt1:
                    self.advX = self.advX
                else:
                    self.advX = self.advX + 20;
                screen.addstr(5, 0, 'up: ' + '%.2f' % self.advX + ' : ' + '%.2f' % self.rot)
                self.omnirobot_proxy.setSpeedBase(self.advX, self.advZ, self.rot)
            elif key == curses.KEY_LEFT:
                if self.advX < -1 * self.tt1:
                    self.advX = self.advX
                else:
                    self.advX = self.advX - 20;

                screen.addstr(5, 0, 'down: ' + '%.2f' % self.advX + ' : ' + '%.2f' % self.rot)
                self.omnirobot_proxy.setSpeedBase(self.advX, self.advZ, self.rot)

            elif key == curses.KEY_F1:
                if self.rot < -1 * self.tt2:
                    self.rot = self.rot
                else:
                    self.rot = self.rot - 0.1;

                screen.addstr(5, 0, 'left: ' + '%.2f' % self.advZ + ' : ' + '%.2f' % self.rot)
                self.omnirobot_proxy.setSpeedBase(self.advX, self.advZ, self.rot)
            elif key == curses.KEY_F2:
                if self.rot > self.tt2:
                    self.rot = self.rot
                else:
                    self.rot = self.rot + 0.1;

                screen.addstr(5, 0, 'right: ' + '%.2f' % self.advZ + ' : ' + '%.2f' % self.rot)
                self.omnirobot_proxy.setSpeedBase(0, self.advZ, self.rot)
            elif key == ord(' '):
                self.rot = 0
                self.advZ = 0
                self.advX = 0
                screen.addstr(5, 0, 'stop: ' + '%.2f' % self.advZ + ' : ' + '%.2f' % self.rot)
                self.omnirobot_proxy.setSpeedBase(self.advX, self.advZ, self.rot)

            elif key == ord('q'):
                curses.endwin()
                sys.exit()
        except Ice.Exception as e:
            curses.endwin()
            traceback.print_exc()
            print(e)
        return True

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
