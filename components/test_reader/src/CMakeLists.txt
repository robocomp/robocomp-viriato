cmake_minimum_required(VERSION 2.6)
PROJECT(  test_reader )

INCLUDE( /opt/robocomp/cmake/robocomp.cmake )

ROBOCOMP_IDSL_TO_ICE( CommonBehavior AGMCommonBehavior AGMExecutive AGMExecutiveTopic AGMWorldModel Planning )
