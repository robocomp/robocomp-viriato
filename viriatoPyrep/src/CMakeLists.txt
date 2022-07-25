cmake_minimum_required(VERSION 2.6)
PROJECT(  viriato_pyrep )

INCLUDE( /opt/robocomp/cmake/robocomp.cmake )

ROBOCOMP_IDSL_TO_ICE( CommonBehavior CameraRGBDSimple CameraSimple CoppeliaUtils DifferentialRobot FullPoseEstimation GenericBase JointMotorSimple JoystickAdapter Laser OmniRobot )
