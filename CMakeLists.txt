cmake_minimum_required(VERSION 3.16)
project(nether_pathfinder)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g")

add_executable(nether_pathfinder main.cpp NoiseGeneratorOctaves.h NoiseGeneratorImproved.h Random.h ChunkGeneratorHell.h ChunkPrimer.h ChunkGeneratorHell.cpp NoiseGeneratorOctaves.cpp NoiseGeneratorImproved.cpp)