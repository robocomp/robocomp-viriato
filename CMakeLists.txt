cmake_minimum_required(VERSION 3.16)
project(HumanSceneSim)

set(CMAKE_CXX_STANDARD 14)

include_directories(src)
include_directories(src/HumanSceneSim_autogen)
include_directories(src/HumanSceneSim_autogen/include)

add_executable(HumanSceneSim
    etc/config
    src/HumanSceneSim_autogen/EWIEGA46WW/moc_genericmonitor.cpp
    src/HumanSceneSim_autogen/EWIEGA46WW/moc_genericworker.cpp
    src/HumanSceneSim_autogen/EWIEGA46WW/moc_specificmonitor.cpp
    src/HumanSceneSim_autogen/EWIEGA46WW/moc_specificworker.cpp
    src/HumanSceneSim_autogen/include/moc_sigwatch.cpp
    src/HumanSceneSim_autogen/include/ui_mainUI.h
    src/HumanSceneSim_autogen/moc_predefs.h
    src/HumanSceneSim_autogen/mocs_compilation.cpp
    src/AGMCommonBehavior.cpp
    src/AGMCommonBehavior.h
    src/agmcommonbehaviorI.cpp
    src/agmcommonbehaviorI.h
    src/AGMExecutive.cpp
    src/AGMExecutive.h
    src/AGMExecutiveTopic.cpp
    src/AGMExecutiveTopic.h
    src/agmexecutivetopicI.cpp
    src/agmexecutivetopicI.h
    src/AGMWorldModel.cpp
    src/AGMWorldModel.h
    src/CommonBehavior.cpp
    src/CommonBehavior.h
    src/commonbehaviorI.cpp
    src/commonbehaviorI.h
    src/config.h
    src/genericmonitor.cpp
    src/genericmonitor.h
    src/genericworker.cpp
    src/genericworker.h
    src/InnerModelManager.cpp
    src/InnerModelManager.h
    src/main.cpp
    src/Planning.cpp
    src/Planning.h
    src/specificmonitor.cpp
    src/specificmonitor.h
    src/specificworker.cpp
    src/specificworker.h)
