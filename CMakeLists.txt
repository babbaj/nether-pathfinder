cmake_minimum_required(VERSION 3.16)
project(nether_pathfinder)

find_package(JNI COMPONENTS JNI_INCLUDE_DIRS)
include_directories(${JNI_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -std=c++20 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
message("${CMAKE_CXX_COMPILER_VERSION}")

message("${CMAKE_EXE_LINKER_FLAGS}")

file(GLOB_RECURSE SRC src/*)

if (SHARED_LIBRARY)
    add_library(nether_pathfinder SHARED ${SRC})
else()
    add_executable(nether_pathfinder ${SRC})
endif()

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    message(FATAL_ERROR "gcc is not supported")
endif()

#target_link_options(nether_pathfinder PRIVATE "-static-libgcc" "-static-libstdc++")
#if (USE_ABSEIL)
    add_subdirectory(abseil-cpp)
    target_link_libraries(nether_pathfinder absl::flat_hash_map)
#endif()