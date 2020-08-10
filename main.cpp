#include <iostream>
#include <chrono>
#include <fstream>

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
    std::optional<Path> path = findPath({-67, 67, -31}, {103, 84, -177}, generator);
    auto t2 = std::chrono::steady_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count();

    //std::cout << "Finding path took " << duration << "s " << std::endl;
    std::cout << "Path has " << path->path.size() << " blocks and " << path->nodes.size() << " nodes\n";

    std::cout << (path.has_value() ? "Found a path!\n" : "no path :-(\n") << '\n';
}
