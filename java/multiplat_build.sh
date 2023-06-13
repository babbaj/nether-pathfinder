#!/usr/bin/env bash

set -e

function do_build {
    mkdir build
    pushd build
    local java_root=$1
    local target=$2
    local output_path=$3

    CXXFLAGS="-target $target" cmake -G Ninja $java_root/.. \
      -DCMAKE_C_COMPILER=$(realpath $java_root/zigcc.sh) -DCMAKE_CXX_COMPILER=$(realpath $java_root/zigcxx.sh) \
      -DSHARED_LIBRARY=True -DCMAKE_BUILD_TYPE=Release

    ninja -j `nproc`

    cp libnether_pathfinder.so ../$output_path
    popd
    rm -rf build
}

do_build $1 x86_64-linux-gnu libnether_pathfinder-x86_64.so
do_build $1 aarch64-linux-gnu libnether_pathfinder-aarch64.so
do_build $1 aarch64-macos-gnu libnether_pathfinder-aarch64.dylib
do_build $1 x86_64-macos-gnu libnether_pathfinder-x86_64.dylib
do_build $1 x86_64-windows-gnu nether_pathfinder-x86_64.dll