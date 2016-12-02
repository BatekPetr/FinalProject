cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(main)


find_package(OpenCV REQUIRED)

add_executable(main main.cpp marker1.h marker1.cpp marker2a.h marker2a.cpp harris.h harris.cpp)

SET(main src/main.cpp)


