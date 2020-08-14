#include <iostream>
#include <chrono>
#include <fstream>
#include <cstring>

#include "ChunkGeneratorHell.h"
#include "PathFinder.h"

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

void writeChunk(const char* fileName, const ChunkPrimer& chunk) {
    std::fstream out(fileName, std::ios::out);
    std::array bytes = bitsetToBytes(chunk.data);
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

int main(int argc, char** argv) {
    auto now = std::chrono::system_clock::now();
    now.time_since_epoch().count();
    auto millis = std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch().count();
    std::cout << "now = " << now.time_since_epoch().count()  << "  millis = " << millis << '\n';

    constexpr auto seed = 146008555100680;
    auto generator = ChunkGeneratorHell::fromSeed(seed);
    /*auto pool = ParallelExecutor<3>{};

    auto t1 = std::chrono::steady_clock::now();
    auto chunk = generator.generateChunk(0, 0, pool);
    auto t2 = std::chrono::steady_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

    std::cout << "Generating chunk took " << duration << "us " << std::endl;

    writeChunk("testchunk", chunk);*/

    auto t1 = std::chrono::steady_clock::now();
    constexpr BlockPos ONE_MIL = {1000072, 64, -121};
    constexpr BlockPos ONE_HUNDRED_K = {100000, 50, 0};
    constexpr BlockPos TEN_K = {10000, 64, 0};
    constexpr BlockPos ONE_K = {1000, 64, 0};
    std::optional<Path> path = findPath({0, 40, 0}, ONE_HUNDRED_K, generator);
    auto t2 = std::chrono::steady_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

    std::cout << "Finding path took " << duration / 1000.0 << "s " << std::endl;
    if (path.has_value()) {
        std::cout << "Path has " << path->blocks.size() << " blocks and " << path->nodes.size() << " nodes\n";
        const auto& endPos = path->getEndPos();
        std::cout << "start = " << "{" << path->start.x << ", " << path->start.y << ", " << path->start.z << "} end = " << "{" << endPos.x << ", " << endPos.y << ", " << endPos.z << "}\n";

        writeBreadCrumbFile("test", *path);
    } else {
        std::cout << "No path :-(\n";
    }
}
