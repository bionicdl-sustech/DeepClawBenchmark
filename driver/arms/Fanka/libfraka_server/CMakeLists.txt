cmake_minimum_required(VERSION 3.0)
PROJECT(server CXX)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${FRANKA_INCLUDE_DIRS})

add_executable(server server.cpp)
target_link_libraries(server Franka::Franka)




