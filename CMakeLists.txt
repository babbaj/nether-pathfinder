cmake_minimum_required(VERSION 3.16)
project(nether_pathfinder)

find_package(JNI COMPONENTS JNI_INCLUDE_DIRS)
include_directories(${JNI_INCLUDE_DIRS})

find_package(PkgConfig) # not required
if(PkgConfig_FOUND)
    pkg_check_modules(GBENCHMARK IMPORTED_TARGET benchmark) # not required
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -std=c++20 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
message("${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")

file(GLOB_RECURSE SRC src/*)
list(REMOVE_ITEM SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/benchmark_main.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")

if (SHARED_LIBRARY)
    add_library(nether_pathfinder SHARED ${SRC})
else()
    add_executable(nether_pathfinder ${SRC} src/main.cpp)
    if(GBENCHMARK_FOUND)
        add_executable(nether_pathfinder_benchmark ${SRC} src/benchmark_main.cpp)
        target_link_libraries(nether_pathfinder_benchmark PkgConfig::GBENCHMARK)
    endif()
endif()

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    message(WARNING "gcc is not recommended")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 13.0)
        message(FATAL_ERROR "requires clang at least clang 13")
    endif()
endif()

#target_link_options(nether_pathfinder PRIVATE "-static-libgcc" "-static-libstdc++")
if (USE_ABSEIL)
    add_subdirectory(abseil-cpp)
    target_link_libraries(nether_pathfinder absl::flat_hash_map)
endif()
