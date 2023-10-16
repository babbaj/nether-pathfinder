#!/usr/bin/env bash

set -e

function do_build {
    mkdir build
    pushd build
    local java_root=$1
    local target=$2
    local output_path=$3
    echo "Building $target..."
    rm -rf ~/.cache/zig
    CXXFLAGS="-target $target" cmake -G Ninja $java_root/.. \
      -DPATHFINDER_TARGET=$target \
      -DCMAKE_C_COMPILER=$(realpath $java_root/zigcc.sh) -DCMAKE_CXX_COMPILER=$(realpath $java_root/zigcxx.sh) \
      -DCMAKE_AR=$(realpath $java_root/zigar.sh) \
      -DCMAKE_RANLIB=$(realpath $java_root/zigranlib.sh) \
      -DCMAKE_BUILD_TYPE=Release

    ninja -j `nproc`

    cp libnether_pathfinder.* ../$output_path
    popd
    rm -rf build
}

do_build $1 x86_64-linux-gnu libnether_pathfinder-x86_64.so
do_build $1 aarch64-linux-gnu libnether_pathfinder-aarch64.so
do_build $1 x86_64-macos-gnu libnether_pathfinder-x86_64.dylib
do_build $1 aarch64-macos-gnu libnether_pathfinder-aarch64.dylib
do_build $1 x86_64-windows-gnu nether_pathfinder-x86_64.dll
do_build $1 aarch64-windows-gnu nether_pathfinder-aarch64.dll
