cmake_minimum_required(VERSION 3.16)
project(nether_pathfinder)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -pthread")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g")

file(GLOB_RECURSE SRC src/*)

add_executable(nether_pathfinder ${SRC})