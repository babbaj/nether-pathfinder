#include <iostream>
#include <chrono>
#include <fstream>
#include <cstring>
#include <bitset>

#include "ChunkGeneratorHell.h"
#include "PathFinder.h"
#include "Refiner.h"
#include "baritone.h"

template<size_t Bits>
std::array<char, Bits / 8> bitsetToBytes(const std::bitset<Bits>& bitSet) {
    std::array<char, Bits / 8> out;
    for (int i = 0; i < Bits / 8; i++) {
        char byte = 0;
        for (int b = 0; b < 8; b++) {
            byte |= bitSet[(i * 8) + b] << b;
        }
        out[i] = byte;
    }
    return out;
}

static int getBlockIndex(int x, int y, int z) {
    return x << 12  | z << 8  | y;
}

void writeChunk(const char* fileName, const Chunk& chunk) {
    std::fstream out(fileName, std::ios::out);
    std::bitset<65536> bits;
    for (int x = 0; x < 16; x++) {
        for (int z = 0; z < 16; z++) {
            for (int y = 0; y < 128; y++) {
                bits[getBlockIndex(x, y, z)] = chunk.isSolid(x, y, z);
            }
        }
    }

    std::array bytes = bitsetToBytes(bits);
    out.write(&bytes[0], bytes.size());
}

template<typename T>
void write(std::ofstream& out, T x) {
    char buf[sizeof(T)];
    memcpy(&buf[0], (char*)&x, sizeof(T));
    std::reverse(std::begin(buf), std::end(buf));
    out.write(buf, sizeof(buf));
}

void writeBreadCrumbFile(const char* fileName, const Path& path) {
    std::ofstream out(fileName, std::ios::out | std::ios::binary);
    write(out, 1); // 1 trail
    {
        write(out, -1);
        write(out, (int)path.blocks.size());
        for (const BlockPos& pos : path.blocks) {
            write(out, (double)pos.x);
            write(out, (double)pos.y);
            write(out, (double)pos.z);
        }
    }
    out.close();
}

double totalLength(const Path& path) {
    const auto& blocks = path.blocks;
    double sumDist = 0;
    for (auto it = blocks.begin() + 1; it != blocks.end(); it++) {
        const BlockPos& behind = *(it - 1);
        const BlockPos& pos = *it;
        const double pythag = pos.distanceTo(behind);
        sumDist += pythag;
    }
    return sumDist;
}

void printSizes(const Path& path) {
    int x16 = 0, x8 = 0, x4 = 0, x2 = 0, x1 = 0;
    for (const auto& nodePtr : path.nodes) {
        const PathNode& node = *nodePtr;
        const Size size = node.pos.size;

        switch (size) {
            case Size::X16: x16++; break;
            case Size::X8: x8++; break;
            case Size::X4: x4++; break;
            case Size::X2: x2++; break;
            case Size::X1: x1++; break;
        }
    }

    std::cout << "X16 = " << x16 << '\n';
    std::cout << "X8 = " << x8 << '\n';
    std::cout << "X4 = " << x4 << '\n';
    std::cout << "X2 = " << x2 << '\n';
    std::cout << "X1 = " << x1 << '\n';
    const PathNode& last = *path.nodes.back();
    auto print = [](auto& node) {
        switch (node.pos.size) {
            case Size::X16: std::cout << "X16"; break;
            case Size::X8: std::cout << "X8"; break;
            case Size::X4: std::cout << "X4"; break;
            case Size::X2: std::cout << "X2"; break;
            case Size::X1: std::cout << "X1"; break;
        }
        std::cout << '\n';
    };


    std::cout << "last node = ";
    print(last);
    std::cout << '\n';
    std::cout << "Length = " << totalLength(path) << '\n';

}

int main(int argc, char** argv) {
    constexpr auto seed = 146008555100680;

    [[maybe_unused]] constexpr BlockPos ONE_MIL = {1000072, 64, -121};
    [[maybe_unused]] constexpr BlockPos ONE_HUNDRED_K = {100000, 50, 0};
    [[maybe_unused]] constexpr BlockPos TEN_K = {10000, 64, 0};
    [[maybe_unused]] constexpr BlockPos ONE_K = {1000, 64, 0};
    //findPath({0, 40, 0}, ONE_HUNDRED_K, generator, false);
    //return 0;

    /*auto uwu = std::chrono::steady_clock::now();
    auto firstIteration =  findPath({0, 40, 0}, ONE_MIL, generator, cache, false); // fill the cache
    auto owo = std::chrono::steady_clock::now();
    auto nyaa = std::chrono::duration_cast<std::chrono::milliseconds>(owo - uwu).count();
    std::cout << "first iteration took " << nyaa / 1000.0 << "s " << std::endl;*/

    Context ctx{seed, {}};
    auto t1 = std::chrono::steady_clock::now();
    auto realStart = findAir<Size::X4>(ctx, {0, 50, 0});
    auto realGoal = findAir<Size::X4>(ctx, TEN_K);
    //std::optional<Path> path = findPathSegment(ctx, realStart, realGoal, true, 10000, true).value();
    std::optional<Path> path = findPathFull(ctx, realStart.absolutePosCenter(), realGoal.absolutePosCenter()).value();
    auto t2 = std::chrono::steady_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

    std::cout << "Finding path took " << duration / 1000.0 << "s " << std::endl;
    if (path.has_value()) {
        std::cout << "Path has " << path->blocks.size() << " blocks and " << path->nodes.size() << " nodes\n";
        const auto& endPos = path->getEndPos();
        std::cout << "start = " << "{" << path->start.x << ", " << path->start.y << ", " << path->start.z << "} end = " << "{" << endPos.x << ", " << endPos.y << ", " << endPos.z << "}\n";

        std::cout<< "refining..." << std::endl;
        auto t1 = std::chrono::steady_clock::now();
        auto refined = refine(ctx, path->blocks);
        auto t2 = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
        std::cout << "refining took " << duration / 1000.0 << "s " << std::endl;
        std::cout << "done refining" << std::endl;
        std::cout << "refined path has " << refined.size() << std::endl;
        //writeBreadCrumbFile("test", *path);
        printSizes(*path);
    } else {
        std::cout << "No path :-(\n";
    }
}
