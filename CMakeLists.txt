cmake_minimum_required(VERSION 3.2)
project(ProjectWind)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Eigen3 3.2)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(.)

include(cmake/mex.cmake)
mex_setup()

add_mex(drakeMexUtil SHARED drakeMexUtil.cpp)

add_subdirectory(Drake/@QuadWindPlant_numerical)


