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

from genericworker import *
import os, time, queue
from bisect import bisect_left
from os.path import dirname, join, abspath
from pyrep import PyRep
from numpy.linalg import inv
import itertools as it
#from pyrep.robots.mobiles.viriato import Viriato
#from pyrep.robots.mobiles.viriato import Viriato
from pyrep.robots.mobiles.youbot import YouBot
from pyrep.objects.vision_sensor import VisionSensor
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
#from pyrep.objects.shape import Object
from pyrep.objects.joint import Joint
from pytransform3d.transform_manager import TransformManager
import pytransform3d.transformations as pytr
import pytransform3d.rotations as pyrot
import numpy as np
import numpy_indexed as npi
from itertools import zip_longest
import cv2
from threading import Lock
from math import *

class TimeControl:
    def __init__(self, period_):
        self.counter = 0
        self.start = time.time()  # it doesn't exist yet, so initialize it
        self.start_print = time.time()  # it doesn't exist yet, so initialize it
        self.period = period_

    def wait(self):
        elapsed = time.time() - self.start
        if elapsed < self.period:
            time.sleep(self.period - elapsed)
        self.start = time.time()
        self.counter += 1
        if time.time() - self.start_print > 1:
            print("Freq -> ", self.counter)
            self.counter = 0
            self.start_print = time.time()

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
       
    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):

        #SCENE_FILE = '../../etc/autonomy_lab_no_arm_bill.ttt'
        #SCENE_FILE = '../../etc/autonomy_lab_bill.ttt'
        #SCENE_FILE = '../../etc/autonomy_lab_no_arm_no_middle_wall.ttt'
        SCENE_FILE = params["coppelia_file"]
        print(SCENE_FILE)

        self.pr = PyRep()
        self.pr.launch(SCENE_FILE, headless=False)
        self.pr.set_simulation_timestep(0.2)
        self.pr.start()



        #self.robot = Viriato()
        self.robot = YouBot()
        self.robot_object = Shape("youBot")
        #self.robot_left_arm = Shape("viriato_left_arm")
        #self.robot_left_arm_tip = Dummy("viriato_left_arm_tip")
        # self.ViriatoBase_WheelRadius = 76.2  #mm real robot
        self.ViriatoBase_WheelRadius = 44  # mm coppelia
        self.ViriatoBase_DistAxes = 380.
        self.ViriatoBase_AxesLength = 422.
        self.ViriatoBase_Rotation_Factor = 8.1  # it should be (DistAxes + AxesLength) / 2
        self.bState_ant = RoboCompGenericBase.TBaseState()

        self.cameras_write = {}
        self.cameras_read = {}

        self.robot_from_world_ref = np.matrix([[1, 0, 0], [0, 1, 0],[0, 0, 1]])

        # cam = VisionSensor("wall_camera_1")
        # self.cameras["wall_camera_1"] = { "handle": cam,
        #                                             "id": 1,
        #                                             "angle": np.radians(cam.get_perspective_angle()),
        #                                             "width": cam.get_resolution()[0],
        #                                             "height": cam.get_resolution()[1],
        #                                             "depth": 3,
        #                                             "focal": cam.get_resolution()[0]/np.tan(np.radians(cam.get_perspective_angle())),
        #                                             "rgb": np.array(0),
        #                                             "depth": np.ndarray(0) }
        # cam = VisionSensor("camera_2_rgbd_sensor")
        # self.cameras["camera_2_rgbd_sensor"] = {    "handle": cam,
        #                                             "id": 2,
        #                                             "angle": np.radians(cam.get_perspective_angle()),
        #                                             "width": cam.get_resolution()[0],
        #                                             "height": cam.get_resolution()[1],
        #                                             "focal": cam.get_resolution()[0]/np.tan(np.radians(cam.get_perspective_angle())),
        #                                             "rgb": np.array(0),
        #                                             "depth": np.ndarray(0) }
        # cam = VisionSensor("camera_3_rgbd_sensor")
        # self.cameras["camera_3_rgbd_sensor"] = {    "handle": cam,
        #                                             "id": 3,
        #                                             "angle": np.radians(cam.get_perspective_angle()),
        #                                             "width": cam.get_resolution()[0],
        #                                             "height": cam.get_resolution()[1],
        #                                             "focal": cam.get_resolution()[0]/np.tan(np.radians(cam.get_perspective_angle())),
        #                                             "rgb": np.array(0),
        #                                             "depth": np.ndarray(0) }
        #

        # robot head camera
        # self.initialize_cameras()

        # camera tilt motor
        self.viriato_head_camera_pan_joint_name = "viriato_head_camera_pan_joint"
        self.viriato_head_camera_tilt_joint_name = "viriato_head_camera_tilt_joint"
        self.viriato_head_camera_tilt_joint = Joint(self.viriato_head_camera_tilt_joint_name)
        self.viriato_head_camera_pan_joint = Joint(self.viriato_head_camera_pan_joint_name)

        # laser
        self.lasers = {}

        self.ldata_write = []
        self.ldata_read = []

        # Read existing people
        self.people = {}
        if Dummy.exists("Bill"):
            self.people["Bill"] = Dummy("Bill")
        for i in range(0,2):
            name = "Bill#" + str(i)
            if Dummy.exists(name):
                self.people[name] = Dummy(name)

        self.joystick_newdata = []
        self.speed_robot = []
        self.speed_robot_ant = []
        self.last_received_data_time = 0

        # PoseEstimation
        self.tm = TransformManager()
        self.tm.add_transform("origin", "world",
                              pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz([0.0, 0.0, 0.0]),
                                                  [0.0, 0.0, 0.0])
                              )

        pose = self.robot.get_2d_pose()
        # GRID POSE
        self.r2g = pytr.invert_transform(pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz(
                                                [0.0, 0.0, pose[2]]),
                                                  [pose[0]*1000, pose[1]*1000, 0.0]))

        self.tm.add_transform("robot", "grid", self.r2g)

        self.robot_full_pose_write = RoboCompFullPoseEstimation.FullPoseEuler()
        self.robot_full_pose_read = RoboCompFullPoseEstimation.FullPoseEuler()
        self.mutex_pose = Lock()
        self.primera_vez = True
        self.new_coppelia_script_call_available = False



        # ############### UNCOMMENT TO USE mODIFIED VIRIATO ###############
        #
        # # Eye pan motor
        self.eye_motor = Joint("camera_joint")
        self.eye_new_pos = None

        self.hokuyo_front_left_name = "Hokuyo_sensor2"
        cam = VisionSensor(self.hokuyo_front_left_name)
        self.lasers[self.hokuyo_front_left_name] = { "handle": cam,
                                                      "id": 0,
                                                      "angle": np.radians(cam.get_perspective_angle()),
                                                      "width": cam.get_resolution()[0],
                                                     "semiwidth": cam.get_resolution()[0] / 2.0,
                                                      "height": cam.get_resolution()[1],
                                                      "focal": (cam.get_resolution()[0] / 2) / np.tan(
                                                          np.radians(cam.get_perspective_angle() / 2)),
                                                      "rgb": np.array(0),
                                                      "depth": np.ndarray(0),
                                                      "offset_angle": -np.pi/3.0
                                                     }
        self.hokuyo_front_right_name = "Hokuyo_sensor1"
        cam = VisionSensor(self.hokuyo_front_right_name)
        self.lasers[self.hokuyo_front_right_name] = { "handle": cam,
                                                      "id": 0,
                                                      "angle": np.radians(cam.get_perspective_angle()),
                                                      "width": cam.get_resolution()[0],
                                                      "semiwidth": cam.get_resolution()[0]/2.0,
                                                      "height": cam.get_resolution()[1],
                                                      "focal": (cam.get_resolution()[0] / 2) / np.tan(
                                                        np.radians(cam.get_perspective_angle() / 2)),
                                                      "rgb": np.array(0),
                                                      "depth": np.ndarray(0),
                                                      "offset_angle": np.pi / 3.0
                                                    }

        self.top_camera_name = "camera_top"
        cam = VisionSensor(self.top_camera_name)
        self.cameras_write[self.top_camera_name] = { "handle": cam,
                                                     "id": 0,
                                                     "angle": np.radians(cam.get_perspective_angle()),
                                                     "width": cam.get_resolution()[0],
                                                     "height": cam.get_resolution()[1],
                                                     "focalx": (cam.get_resolution()[0] / 2) / np.tan(
                                                        np.radians(cam.get_perspective_angle() / 2.0)),
                                                     "focaly": (cam.get_resolution()[1] / 2) / np.tan(
                                                         np.radians(cam.get_perspective_angle() / 2)),
                                                     "rgb": np.array(0),
                                                     "depth": np.ndarray(0),
                                                     "is_ready": False,
                                                     "is_rgbd": True,
                                                     "rotated": True,
                                                     "has_depth": True
                                                    }
        self.cameras_read = self.cameras_write.copy()

    def initialize_cameras(self):
        print("Initialize camera")
        self.cameras.clear()
        self.cameras.clear()
        cam = VisionSensor("viriato_head_camera_sensor")
        self.cameras["viriato_head_camera_sensor"] = {    "handle": cam,
                                                                "id": 0,
                                                                "angle": np.radians(cam.get_perspective_angle()),
                                                                "width": cam.get_resolution()[0],
                                                                "height": cam.get_resolution()[1],
                                                                "focal": (cam.get_resolution()[0]/2) / np.tan(np.radians(cam.get_perspective_angle()/2.0)),
                                                                "rgb": np.array(0),
                                                                "depth": np.ndarray(0) }



    #@QtCore.Slot()
    def compute(self):
        tc = TimeControl(0.05)  # 50 millis -> 20Hz
        while True:
            # print("TIMESTEEEEEP", self.pr.get_simulation_timestep())
            self.pr.step()
            # self.read_joystick()
            self.move_robot()
            self.read_robot_pose()
            #self.read_robot_arm_tip()

            ############### UNCOMMENT TO USE mODIFIED VIRIATO ###############
            self.move_eye()
            self.read_cameras([self.top_camera_name])
            self.read_laser_raw()



            # self.read_pan_tilt()

            if self.new_coppelia_script_call_available:
                print("Coppelia dummy script sent ", self.coppelia_nose_speed)
                self.pr.script_call("set_velocity@viriato_head_pan_tilt_nose_target", 1, (), self.coppelia_nose_speed, (), '')
                self.new_coppelia_script_call_available = False

            # if self.primera_vez:
            #     self.pr.script_call("set_velocity@viriato_head_pan_tilt_nose_target", 1, (),
            #                         (-0.00064, -0.00024, 0.0), (), '')
            #     self.primera_vez = False

            tc.wait()

    ###########################################
    ### CAMERAS get and publish cameras data
    ###########################################
    def read_cameras(self, camera_names):
        if self.top_camera_name in camera_names:  # RGBD rotated
            cam = self.cameras_write[self.top_camera_name]
            image_float = cam["handle"].capture_rgb()
            image = cv2.normalize(src=image_float, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX,
                                  dtype=cv2.CV_8U)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            depth = cam["handle"].capture_depth(True)
            depth = np.frombuffer(depth, dtype=np.float32).reshape((cam["height"], cam["width"]))
            depth = cv2.rotate(depth, cv2.ROTATE_90_COUNTERCLOCKWISE)
            # we change width and height here to follow the rotation operation
            cam["depth"] = RoboCompCameraRGBDSimple.TDepth(cameraID=cam["id"],
                                                           width=cam["height"],  # cambiados
                                                           height=cam["width"],
                                                           focalx=cam["focaly"],
                                                           focaly=cam["focalx"],
                                                           alivetime=time.time(),
                                                           depthFactor=1.0,
                                                           depth=depth.tobytes())
            cam["rgb"] = RoboCompCameraRGBDSimple.TImage(cameraID=cam["id"],
                                                         width=cam["height"],  # cambiados
                                                         height=cam["width"],
                                                         depth=3,
                                                         focalx=cam["focaly"],
                                                         focaly=cam["focalx"],
                                                         alivetime=time.time(),
                                                         image=image.tobytes(),
                                                         compressed=False)
            cam["is_ready"] = True

        self.cameras_write, self.cameras_read = self.cameras_read, self.cameras_write

#     def read_cameras(self):
#         for name, cam in self.cameras.items():
#             # check resolution change
#             if cam["width"] != cam["handle"].get_resolution()[0] or cam["height"] != cam["handle"].get_resolution()[1]:
#                 print("Resolution changed")
#                 self.initialize_cameras()
#                 break
#
#             image_float = cam["handle"].capture_rgb()
# #            print("len", len(image_float))
#             depth = cam["handle"].capture_depth(True)
#             image = cv2.normalize(src=image_float, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX,
#                                   dtype=cv2.CV_8U)
#             cam["rgb"] = RoboCompCameraRGBDSimple.TImage(cameraID=cam["id"], width=cam["width"], height=cam["height"],
#                                                          depth=3, focalx=cam["focal"], focaly=cam["focal"],
#                                                          alivetime=time.time(), image=image.tobytes())
#             cam["depth"] = RoboCompCameraRGBDSimple.TDepth(cameraID=cam["id"], width=cam["handle"].get_resolution()[0], height=cam["handle"].get_resolution()[1],
#                                                            focalx=cam["focal"], focaly=cam["focal"],
#                                                            alivetime=time.time(), depthFactor=1.0, depth=depth.tobytes())
#
#             try:
#                 self.camerargbdsimplepub_proxy.pushRGBD(cam["rgb"], cam["depth"])
#             except Ice.Exception as e:
#                 print(e)

    ###########################################
    ### LASER get and publish laser data
    ###########################################
    def read_laser_raw(self):
        data = self.pr.script_call("get_depth_data@Hokuyo", 1)
        if len(data[1]) > 0:
            self.hokuyo = Shape("Hokuyo")
            h_pos = self.hokuyo.get_position()
            polar = np.zeros(shape=(int(len(data[1])/3), 2))
            self.ldata_write = []
            for x, y, z in self.grouper(data[1], 3):                      # extract non-intersecting groups of 3
                self.ldata_write.append(RoboCompLaser.TData(-np.arctan2(y, x), np.linalg.norm([x, y])*1000.0))

            del self.ldata_write[-7:]
            del self.ldata_write[:7]

            # if self.ldata_write[0] == 0:
            #    self.ldata_write[0] = 200  # half robot width
            # del self.ldata_write[-3:]
            # del self.ldata_write[:3]
            # for i in range(1, len(self.ldata_write)):
            #    if self.ldata_write[i].dist == 0:
            #        self.ldata_write[i].dist = self.ldata_write[i - 1].dist


            self.ldata_read, self.ldata_write = self.ldata_write, self.ldata_read

            # try:
            #     self.laserpub_proxy.pushLaserData(self.ldata_read)
            # except Ice.Exception as e:
            #     print(e)

    def grouper(self, inputs, n, fillvalue=None):
        iters = [iter(inputs)] * n
        return it.zip_longest(*iters, fillvalue=fillvalue)

    ###########################################
    ###  read and move the robot
    ###########################################
    def move_robot(self):
        if self.speed_robot != self.speed_robot_ant:  # or (isMoving and self.speed_robot == [0,0,0]):
            self.robot.set_base_angular_velocites(self.speed_robot)
            print("Velocities sent to robot:", self.speed_robot)
            self.speed_robot_ant = self.speed_robot

    ###########################################
    ### JOYSITCK read and move the robot
    ###########################################
    def read_joystick(self):
        if self.joystick_newdata: #and (time.time() - self.joystick_newdata[1]) > 0.1:
            adv = 0.0
            rot = 0.0
            side = 0.0
            # pan = 0.0
            # tilt = 0.0
            # head_moves = False

            for x in self.joystick_newdata[0].axes:
                if x.name == "advance":
                    adv = x.value if np.abs(x.value) > 0.1 else 0  # mm/sg
                if x.name == "rotate":
                    rot = x.value if np.abs(x.value) > 0.1 else 0  # rads/sg
                if x.name == "side":
                    side = x.value if np.abs(x.value) > 0.1 else 0
                # if x.name == "pan":
                #     pan = x.value if np.abs(x.value) > 0.01 else 0
                #     head_moves = True
                # if x.name == "tilt":
                #     tilt = x.value if np.abs(x.value) > 0.01 else 0
                #     head_moves = True

            # print("Joystick ", adv, rot, side)
            converted = self.convert_base_speed_to_radians(adv, side, rot)
            # self.robot.set_base_angular_velocites([adv, side, rot])
            # print("CONVERTED * 10", converted * 10)
            # print("CONVERTED * 2", converted * 2)
            self.robot.set_base_angular_velocites(converted)
            print(self.robot_object.get_velocity())
            #
            # if (head_moves):
            #     dummy = Dummy("viriato_head_pan_tilt_nose_target")
            #     pantilt = Dummy("viriato_head_camera_pan_tilt")
            #     pose = dummy.get_position(pantilt)
            #     dummy.set_position([pose[0], pose[1] - pan / 10, pose[2] + tilt / 10], pantilt)

            self.joystick_newdata = None
            self.last_received_data_time = time.time()
        else:
            elapsed = time.time() - self.last_received_data_time
            if elapsed > 2 and elapsed < 3:
                self.robot.set_base_angular_velocites([0, 0, 0])

    ###########################################
    ### ROBOT POSE get and publish robot position
    ###########################################
    def read_robot_pose(self):
        pose = self.robot.get_2d_pose()
        linear_vel, ang_vel = self.robot_object.get_velocity()
        # print("Veld:", linear_vel, ang_vel)

        isMoving = np.abs(linear_vel[0]) > 0.01 or np.abs(linear_vel[1]) > 0.01 or np.abs(ang_vel[2]) > 0.01
        # self.bState = RoboCompGenericBase.TBaseState(x=pose[0] * 1000 ,
        #                                              z=pose[1] * 1000 ,
        #                                              alpha=pose[2],
        #                                              advVx=linear_vel[0] * 1000,
        #                                              advVz=linear_vel[1] * 1000,
        #                                              rotV=ang_vel[2],
        #                                              isMoving=isMoving)
        #
        # print("---------------POSE--------------")
        # print(self.bState)
        # try:        # parameters are adjusted for Coppelia sensibility
        #     if not np.allclose([self.bState.x, self.bState.z], [self.bState_ant.x, self.bState_ant.z], rtol=1e-03, atol=1) \
        #     or not np.isclose(self.bState.alpha, self.bState_ant.alpha, rtol=1e-03, atol=1e-02):
        #         self.robot_full_pose_write, self.robot_full_pose_read = self.robot_full_pose_read, self.robot_full_pose_write
        #         self.bState_ant = self.bState
        #
        #
        # except Ice.Exception as e:
        #     print(e)

        # self.tm.add_transform("world", "robot", pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz
        #                                                             ([0.0, 0.0, pose[2] - np.pi]),
        #                                                             [pose[0] * 1000.0, pose[1] * 1000.0, 0.0]
        #                                                             ))

        t = self.tm.get_transform("robot", "grid")
        angles = pyrot.extrinsic_euler_xyz_from_active_matrix(t[0:3, 0:3])


        aux_position = np.array([pose[0] * 1000, pose[1]* 1000, 1, 1])
        #
        # transformed_position = aux_position * inv(self.robot_from_world_ref)

        trans_pos = pytr.transform(t, aux_position)

        # print("___________________TRANSFORMEEEEEER____________________")
        # print(t)
        # print(angles)
        # print(trans_pos)
        # print(pose)

        self.robot_full_pose_write.x = trans_pos[0]
        self.robot_full_pose_write.y = trans_pos[1]
        self.robot_full_pose_write.z = t[2][3]
        self.robot_full_pose_write.rx = angles[0]
        self.robot_full_pose_write.ry = angles[1]
        # self.robot_full_pose_write.rz = angles[2]
        self.robot_full_pose_write.rz = angles[2] + pose[2]
        self.robot_full_pose_write.vx = linear_vel[0] * 1000.0
        self.robot_full_pose_write.vy = linear_vel[1] * 1000.0
        self.robot_full_pose_write.vz = linear_vel[2] * 1000.0
        self.robot_full_pose_write.vrx = ang_vel[0]
        self.robot_full_pose_write.vry = ang_vel[1]
        self.robot_full_pose_write.vrz = ang_vel[2]

        # swap
        self.mutex_pose.acquire()
        self.robot_full_pose_write, self.robot_full_pose_read = self.robot_full_pose_read, self.robot_full_pose_write
        self.mutex_pose.release()

    ###########################################
    ### ROBOT POSE get and publish robot position
    ###########################################
    # def read_robot_arm_tip(self):
    #     trans = self.robot_left_arm_tip.get_position(self.robot_object)
    #     rot = self.robot_left_arm_tip.get_orientation(self.robot_object)
    #     qt = self.robot_left_arm_tip.get_quaternion(self.robot_object)
    #     linear_vel, ang_vel = self.robot_left_arm_tip.get_velocity()
    #     #print(trans, rot, linear_vel)
    #     try:
    #         isMoving = np.sum(np.abs(linear_vel)) > 0.005 or np.sum(np.abs(ang_vel)) > 0.005
    #         if isMoving:
    #             self.arm_state = RoboCompKinovaArmPub.TArmState(x=trans[0]*1000, y=trans[1]*1000, z=trans[2]*1000,
    #                                                         rx=rot[0], ry=rot[1], rz=rot[2],
    #                                                         qta=qt[0], qtb=qt[1], qtc=qt[2], qtd=qt[3])
    #             self.kinovaarmpub_proxy.newArmState(self.arm_state)
    #     except Ice.Exception as e:
    #         print(e)

    ###########################################
    ### Viriato head camera tilt motor. Command from JointMotor interface
    ############################################
    # def read_pan_tilt(self):
    #     mpan = RoboCompJointMotor.MotorState()
    #     mpan.pos = self.viriato_head_camera_pan_joint.get_joint_position()  # rads
    #     mtilt = RoboCompJointMotor.MotorState()
    #     mtilt.pos = self.viriato_head_camera_tilt_joint.get_joint_position()
    #     motors = dict({self.viriato_head_camera_pan_joint_name: mpan,
    #                    self.viriato_head_camera_tilt_joint_name: mtilt})
    #     try:
    #         #print("Joints: ", motors)
    #         self.jointmotorpub_proxy.motorStates(motors)
    #     except Ice.Exception as e:
    #         print(e)

    def move_eye(self):
        if self.eye_new_pos:
            self.eye_motor.set_joint_position(self.eye_new_pos)  # radians
            self.eye_new_pos = None

    ########################################
    ## General laser computation
    ########################################
    def compute_omni_laser(self, lasers, robot):
        c_data = []
        coor = []
        for laser in lasers:
            semiwidth = laser.get_resolution()[0]/2
            semiangle = np.radians(laser.get_perspective_angle()/2)
            focal = semiwidth/np.tan(semiangle)
            data = laser.capture_depth(in_meters=True)
            m = laser.get_matrix(robot)     # these data should be read first
            # print("m", m)
            # print("m.shape", m.shape)
            if type(m) != list:
                m = [item for sublist in m for item in sublist]
            imat = np.array([[m[0],m[1],m[2],m[3]],[m[4],m[5],m[6],m[7]],[m[8],m[9],m[10],m[11]],[0,0,0,1]])

            for i,d in enumerate(data.T):
                z = d[0]        # min if more than one row in depth image
                vec = np.array([-(i-semiwidth)*z/focal, 0, z, 1])
                res = imat.dot(vec)[:3]       # translate to robot's origin, homogeneous
                c_data.append([np.arctan2(res[0], res[1]), np.linalg.norm(res)])  # add to list in polar coordinates

        # create 360 polar rep
        c_data_np = np.asarray(c_data)
        angles = np.linspace(-np.pi, np.pi, 360)                          # create regular angular values
        positions = np.searchsorted(angles, c_data_np[:,0])               # list of closest position for each laser meas
        ldata = [RoboCompLaser.TData(a, 0) for a in angles]               # create empty 360 angle array
        pos , medians  = npi.group_by(positions).median(c_data_np[:,1])   # group by repeated positions
        for p, m in zip_longest(pos, medians):                            # fill the angles with measures
            ldata[p].dist = int(m*1000)   # to millimeters
        if ldata[0] == 0:
            ldata[0] = 200       #half robot width
        for i in range(1, len(ldata)):
            if ldata[i].dist == 0:
                ldata[i].dist = ldata[i-1].dist

        return ldata

    def convert_base_speed_to_radians(self, adv, side, rot):
        # rot has to be neg so neg rot speeds go clock wise. It is probably a sign in Pyrep forward kinematics
        return [adv / self.ViriatoBase_WheelRadius, side / self.ViriatoBase_WheelRadius, rot * self.ViriatoBase_Rotation_Factor]

    ##################################################################################
    # SUBSCRIPTION to sendData method from JoystickAdapter interface
    ###################################################################################
    def JoystickAdapter_sendData(self, data):
        self.joystick_newdata = [data, time.time()]

    ##################################################################################
    #                       Methods for CameraRGBDSimple
    # ===============================================================================
    #
    # getAll
    #
    def CameraRGBDSimple_getAll(self, camera):
        if camera in self.cameras_read.keys() \
                and self.cameras_read[camera]["is_ready"] \
                and self.cameras_read[camera]["is_rgbd"]:
            return RoboCompCameraRGBDSimple.TRGBD(self.cameras_read[camera]["rgb"], self.cameras_read[camera]["depth"])
        else:
            e = RoboCompCameraRGBDSimple.HardwareFailedException()
            e.what = "No camera found with this name or with depth attributes: " + camera
            raise e

    #
    # getDepth
    #
    def CameraRGBDSimple_getDepth(self, camera):
        if camera in self.cameras_read.keys() \
                and self.cameras_read[camera]["is_ready"] \
                and self.cameras_read[camera]["has_depth"]:
            return self.cameras_read[camera]["depth"]
        else:
            e = RoboCompCameraRGBDSimple.HardwareFailedException()
            e.what = "No camera found with this name or with depth attributes: " + camera
            raise e
    #
    # getImage
    #
    def CameraRGBDSimple_getImage(self, camera):
        if camera in self.cameras_read.keys() and self.cameras_read[camera]["is_ready"]:
            return self.cameras_read[camera]["rgb"]
        else:
            e = RoboCompCameraRGBDSimple.HardwareFailedException()
            e.what = "No camera found with this name: " + camera
            raise e

    #######################################################
    #### Laser
    #######################################################

    #
    # getLaserAndBStateData
    #
    def Laser_getLaserAndBStateData(self):
        bState = RoboCompGenericBase.TBaseState()
        return self.ldata_read, bState

    #
    # getLaserConfData
    #
    def Laser_getLaserConfData(self):
        ret = RoboCompLaser.LaserConfData()
        return ret

    #
    # getLaserData
    #
    def Laser_getLaserData(self):
        return self.ldata_read

    ##############################################
    ## Omnibase
    #############################################

    #
    # correctOdometer
    #
    def OmniRobot_correctOdometer(self, x, z, alpha):
        pass

    #
    # getBasePose
    #
    def OmniRobot_getBasePose(self):
        #
        # implementCODE
        #
        x = self.bState.x
        z = self.bState.z
        alpha = self.bState.alpha
        return [x, z, alpha]

    #
    # getBaseState
    #
    def OmniRobot_getBaseState(self):
        return self.bState

    #
    # resetOdometer
    #
    def OmniRobot_resetOdometer(self):
        pass

    #
    # setOdometer
    #
    def OmniRobot_setOdometer(self, state):
        pass

    #
    # setOdometerPose
    #
    def OmniRobot_setOdometerPose(self, x, z, alpha):
        pass

    #
    # setSpeedBase
    #
    def OmniRobot_setSpeedBase(self, advx, advz, rot):
        #converted = self.convert_base_speed_to_radians(advz, advx, rot)
        # print("SPEEDS OMNI PRE", advx,advz,rot)
        self.speed_robot = self.convert_base_speed_to_radians(advz, advx, rot)
        #self.speed_robot = [advz,advx,rot]
        # print("SPEEDS OMNI",self.speed_robot)
        # print("ROBOT SPEED", self.robot_object.get_velocity())
        #self.robot.set_base_angular_velocites(converted)

    #
    # stopBase
    #
    def OmniRobot_stopBase(self):
        pass

    # DIFFERENTIAL

    def DifferentialRobot_correctOdometer(self, x, z, alpha):
        pass

    #
    # getBasePose
    #
    def DifferentialRobot_getBasePose(self):
        if self.bState:
            x = self.bState.x
            z = self.bState.z
            alpha = self.bState.alpha
            return [x, z, alpha]
        else:
            return RoboCompGenericBase.TBaseState()

    #
    # getBaseState
    #
    def DifferentialRobot_getBaseState(self):
        if self.bState:
            return self.bState
        else:
            return RoboCompGenericBase.TBaseState()

    #
    # resetOdometer
    #
    def DifferentialRobot_resetOdometer(self):
        pass

    #
    # setOdometer
    #
    def DifferentialRobot_setOdometer(self, state):
        pass

    #
    # setOdometerPose
    #
    def DifferentialRobot_setOdometerPose(self, x, z, alpha):
        pass

    #
    # setSpeedBase
    #
    def DifferentialRobot_setSpeedBase(self, advz, rot):
        print("SPEEDS DIFERRENTIAL:", advz, rot)
        self.speed_robot = [advz, 0, rot]

    #
    # stopBase
    #
    def DifferentialRobot_stopBase(self):
        pass


    # ===================================================================
    # CoppeliaUtils
    # ===================================================================
    def CoppeliaUtils_addOrModifyDummy(self, type, name, pose):
        if not Dummy.exists(name):
            dummy = Dummy.create(0.1)
            dummy.set_name(name)
        else:
            dummy = Dummy(name)
            parent_frame_object = None
            if type == RoboCompCoppeliaUtils.TargetTypes.HeadCamera:
                parent_frame_object = Dummy("viriato_head_camera_pan_tilt")          # target in parent's reference system

            # We CHANGE here axis to comply with Coppelia configuration for pan-tilt axis: x -> y; y -> x; z -> -z
            #print("Coppelia sent", name, pose.y/1000, pose.z/1000, -pose.z/1000)
            dummy.set_position([pose.y / 1000., pose.x / 1000., -pose.z / 1000.], parent_frame_object)
            dummy.set_orientation([pose.rx, pose.ry, pose.rz], parent_frame_object)

    def CoppeliaUtils_setDummySpeed(self, type, name, speed):
        if not Dummy.exists(name):
            print("Warning. Attempt to set speed to a non existent dummy", name)
            return
        else:
            # We CHANGE here axis to comply with Coppelia configuration for pan-tilt axis: x -> y; y -> x; z -> -z

            self.new_coppelia_script_call_available = True
            self.coppelia_nose_speed =  (speed.vy/1000, speed.vx/1000, -speed.vz/1000)

    def FullPoseEstimation_getFullPoseEuler(self):
        return self.robot_full_pose_read


    def FullPoseEstimation_getFullPoseMatrix(self):
        t = self.tm.get_transform("origin", "robot")
        m = RoboCompFullPoseEstimation.FullPoseMatrix()
        m.m00 = t[0][0]
        m.m01 = t[0][1]
        m.m02 = t[0][2]
        m.m03 = t[0][3]
        m.m10 = t[1][0]
        m.m11 = t[1][1]
        m.m12 = t[1][2]
        m.m13 = t[1][3]
        m.m20 = t[2][0]
        m.m21 = t[2][1]
        m.m22 = t[2][2]
        m.m23 = t[2][3]
        m.m30 = t[3][0]
        m.m31 = t[3][1]
        m.m32 = t[3][2]
        m.m33 = t[3][3]
        return m

    def FullPoseEstimation_setInitialPose(self, x, y, z, rx, ry, rz):
        pose = self.robot.get_2d_pose()

        self.r2g = pytr.invert_transform(pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz(
                                                [0.0, 0.0, pose[2]]),
                                                  [pose[0]*1000, pose[1]*1000, 0.0]))

        self.tm.add_transform("robot", "grid", self.r2g)





        # self.robot_from_world_ref = np.matrix([[cos(pose[2]), -sin(pose[2]), pose[0] * 1000], [sin(pose[2]), cos(pose[2]), pose[1] * 1000], [0, 0, 1]])
        #
        # print("-------------SET POSE---------------")
        # print(self.robot_from_world_ref)
        #
        # self.tm.add_transform("robot", "world", pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz
        #                                                             ([0.0, 0.0, -(pose[2] - np.pi)]),
        #                                                             [-(pose[0] * 1000.0), -(pose[1] * 1000.0), 0.0]
        #                                                             ))
        # ===================================================================
        # IMPLEMENTATION of getMotorParams method from JointMotorSimple interface
        # ===================================================================

    def JointMotorSimple_getMotorParams(self, motor):
        ret = RoboCompJointMotorSimple.MotorParams()
        return ret

        #
        # IMPLEMENTATION of getMotorState method from JointMotorSimple interface
        #

    def JointMotorSimple_getMotorState(self, motor):
        if motor == "tablet_motor":
            ret = RoboCompJointMotorSimple.MotorState(self.tablet_motor.get_joint_position())  # radians
        elif motor == "eye_motor":
            ret = RoboCompJointMotorSimple.MotorState(self.eye_motor.get_joint_position())  # radians
        return ret

        #
        # IMPLEMENTATION of setPosition method from JointMotorSimple interface
        #

    def JointMotorSimple_setPosition(self, name, goal):
        # print("JointMotorSimple_setPosition: ", name, goal)
        # check position limits -10 to 80
        if name == "tablet":
            self.tablet_new_pos = goal.position
        elif name == "eye_motor":
            self.eye_new_pos = goal.position
        else:
            print("Unknown motor name", name)

        #
        # IMPLEMENTATION of setVelocity method from JointMotorSimple interface
        #

    def JointMotorSimple_setVelocity(self, name, goal):
        pass

        #
        # IMPLEMENTATION of setZeroPos method from JointMotorSimple interface
        #

    def JointMotorSimple_setZeroPos(self, name):

        #
        # write your CODE here
        #
        pass

        # =============== Methods for Component Implements ==================
        #
        # IMPLEMENTATION of getImage method from CameraSimple interface
        #

    def CameraSimple_getImage(self):
        camera = self.tablet_camera_name
        if camera in self.cameras_read.keys() \
                and self.cameras_read[camera]["is_ready"] \
                and not self.cameras_read[camera]["is_rgbd"]:
            return self.cameras_read[camera]["rgb"]
        else:
            e = RoboCompCameraSimple.HardwareFailedException()
            e.what = "No (no RGBD) camera found with this name: " + camera
            raise e

    ######################################################################
   #self.hokuyo_base_front_left_semiangle = np.radians(self.hokuyo_base_front_left.get_perspective_angle()/2)
        #self.hokuyo_base_front_left_semiwidth = self.hokuyo_base_front_left.get_resolution()[0]/2
        #self.hokuyo_base_front_left_focal = self.hokuyo_base_front_left_semiwidth/np.tan(self.hokuyo_base_front_left_semiangle)
     
    # hokuyo_base_front_left_reading = self.hokuyo_base_front_left.capture_depth(in_meters=True)
                # hokuyo_base_front_right_reading = self.hokuyo_base_front_right.capture_depth(in_meters=True)
                # ldata = []
                # for i,d in enumerate(hokuyo_base_front_left_reading.T):
                #     angle = np.arctan2(i-(self.hokuyo_base_front_left_semiwidth), self.hokuyo_base_front_left_focal)
                #     dist = (d[0]/np.abs(np.cos(angle)))*1000
                #     ldata.append(RoboCompLaser.TData(angle-self.hokuyo_base_front_right_semiangle,dist))
                # for i,d in enumerate(hokuyo_base_front_right_reading.T):
                #     angle = np.arctan2(i-(self.hokuyo_base_front_right_semiwidth), self.hokuyo_base_front_right_focal)
                #     dist = (d[0]/np.abs(np.cos(angle)))*1000
                #     ldata.append(RoboCompLaser.TData(angle+self.hokuyo_base_front_right_semiangle,dist))
             
