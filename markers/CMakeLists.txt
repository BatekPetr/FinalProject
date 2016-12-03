cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(Markers)

SET(RW_ROOT $ENV{RW_ROOT})
MESSAGE("-- RobWork root: " ${RW_ROOT})
SET(RobWork_DIR ${RW_ROOT}/cmake)
FIND_PACKAGE(RobWork REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

find_package(OpenCV REQUIRED)

add_executable(Markers main.cpp marker1.h marker1.cpp)

TARGET_LINK_LIBRARIES(Markers ${OpenCV_LIBS} ${ROBWORKSTUDIO_LIBRARIES})

