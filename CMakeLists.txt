cmake_minimum_required(VERSION 3.16)
project(nether_pathfinder)

find_package(JNI COMPONENTS JNI_INCLUDE_DIRS)
include_directories(${JNI_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -pthread -std=c++20")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g")
message("${CMAKE_CXX_COMPILER_VERSION}")

file(GLOB_RECURSE SRC src/*)

add_subdirectory(abseil-cpp)

if (SHARED_LIBRARY)
    add_library(nether_pathfinder SHARED ${SRC})
else()
    add_executable(nether_pathfinder ${SRC})
endif()

#target_link_options(nether_pathfinder PRIVATE "-static-libgcc" "-static-libstdc++")
target_link_libraries(nether_pathfinder absl::flat_hash_map)