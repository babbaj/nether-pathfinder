cmake_minimum_required(VERSION 3.16)
project(nether_pathfinder)

find_package(JNI COMPONENTS JNI_INCLUDE_DIRS)
include_directories(${JNI_INCLUDE_DIRS})

find_package(PkgConfig) # not required
if(PkgConfig_FOUND)
    pkg_check_modules(GBENCHMARK IMPORTED_TARGET benchmark) # not required
endif()
set(CMAKE_C_COMPILER_TARGET ${PATHFINDER_TARGET})
set(CMAKE_CROSSCOMPILING True)
set(WITH_SANITIZER False)
set(ZLIBNG_ENABLE_TESTS False)
set(ZLIB_ENABLE_TESTS False)
add_subdirectory(zlib-ng)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
message("${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")

file(GLOB_RECURSE SRC src/*)
list(REMOVE_ITEM SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/benchmark_main.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")

if (TESTING)
    add_executable(nether_pathfinder ${SRC} src/main.cpp)
    if(GBENCHMARK_FOUND)
        add_executable(nether_pathfinder_benchmark ${SRC} src/benchmark_main.cpp)
        target_link_libraries(nether_pathfinder_benchmark PkgConfig::GBENCHMARK)
    endif()
else()
    add_executable(baritone_pather ${SRC} src/baritone_pather.cpp)
    target_include_directories(baritone_pather PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/zlib-ng)
    target_link_libraries(baritone_pather PRIVATE zlibstatic)

    add_library(nether_pathfinder SHARED ${SRC})
endif()
target_include_directories(nether_pathfinder PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/zlib-ng)
target_link_libraries(nether_pathfinder PRIVATE zlibstatic)

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
