cmake_minimum_required(VERSION 3.0)
project(franka_ CXX)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${FRANKA_INCLUDE_DIRS})
include_directories(/home/bionicdl-Mega/libfranka-0.5.0/libfranka/src)
include_directories(include)

add_library(libfranka SHARED
  src/examples_common.cpp
)

target_link_libraries(libfranka Franka::Franka)
add_subdirectory(pybind11)

pybind11_add_module(franka_ src/main.cpp)
target_link_libraries(franka_ PRIVATE libfranka)
