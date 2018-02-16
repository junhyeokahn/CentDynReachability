cmake_minimum_required(VERSION 2.8.12)
project(CentDynReachability)

## ===========
## CMake Flags
## ===========
set(CMAKE_CXX_FLAGS "-std=c++11")

## ==========
## Dependancy
## ==========
find_package(DART 6.4 REQUIRED COMPONENTS utils-urdf gui-osg CONFIG)

## =================
## Include Directory
## =================
include_directories(${DART_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})
include_directories("${PROJECT_SOURCE_DIR}/RobotModel")
include_directories("${PROJECT_SOURCE_DIR}/ExternalSource")
include_directories("${PROJECT_SOURCE_DIR}/Utils")

add_subdirectory(RobotModel)
add_subdirectory(Utils)

add_executable(CentDynReachability main.cpp
                                   ValkyrieWorldNode.cpp ValkyrieWorldNode.hpp)
target_link_libraries(CentDynReachability dartRobotModel)
target_link_libraries(CentDynReachability ${DART_LIBRARIES}
                                          dartUtils)

## ==================
## Configuration File
## ==================
SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/config.h.cmake ${PROJECT_SOURCE_DIR}/Configuration.h)
