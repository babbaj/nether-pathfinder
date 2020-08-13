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
        write(out, (int)path.path.size());
        for (const BlockPos& pos : path.path) {
            write(out, (double)pos.x);
            write(out, (double)pos.y);
            write(out, (double)pos.z);
        }
    }
    out.close();
}

int main(int argc, char** argv) {
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
    std::optional<Path> path = findPath({335597, 32, 108802}, {335511, 53, 107812}, generator);
    auto t2 = std::chrono::steady_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

    std::cout << "Finding path took " << duration / 1000.0 << "s " << std::endl;
    std::cout << "Path has " << path->path.size() << " blocks and " << path->nodes.size() << " nodes\n";
    std::cout << "start = " << "{" << path->start.x << ", " << path->start.y << ", " << path->start.z << "} end = " << "{" << path->goal.x << ", " << path->goal.y << ", " << path->goal.z << "}\n";

    std::cout << (path.has_value() ? "Found a path!\n" : "no path :-(\n") << '\n';
    if (path) {
        writeBreadCrumbFile("test", *path);
    }
}
