cmake_minimum_required(VERSION 2.8.12)
project(CentDynReachability)

## ===========
## CMake Flags
## ===========
set(CMAKE_CXX_FLAGS "-std=c++11")

## ==========
## Dependancy
## ==========
find_package(DART 7.0.0 REQUIRED COMPONENTS io-urdf CONFIG)

## =================
## Include Directory
## =================
include_directories(${DART_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})
include_directories("${PROJECT_SOURCE_DIR}/RobotModel")
include_directories("${PROJECT_SOURCE_DIR}/ExternalSource")

add_subdirectory(RobotModel)

add_executable(CentDynReachability main.cpp)
target_link_libraries(CentDynReachability dartRobotModel)
target_link_libraries(CentDynReachability ${DART_LIBRARIES})

## ==================
## Configuration File
## ==================
SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/config.h.cmake ${PROJECT_SOURCE_DIR}/Configuration.h)
