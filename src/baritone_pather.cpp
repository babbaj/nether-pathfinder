#include <iostream>
#include <fstream>
#include <cstring>
#include <bitset>
#include <charconv>

#include "ChunkGeneratorHell.h"
#include "PathFinder.h"
#include "baritone.h"

template<typename T>
void write(std::ofstream& out, T x) {
    char buf[sizeof(T)];
    memcpy(&buf[0], (char*)&x, sizeof(T));
    std::reverse(std::begin(buf), std::end(buf));
    out.write(buf, sizeof(buf));
}

void writePath(const char* fileName, const Path& path) {
    std::ofstream out(fileName, std::ios::out | std::ios::binary);
    for (const BlockPos& pos : path.blocks) {
        write(out, pos.x);
        write(out, pos.y);
        write(out, pos.z);
    }
    out.close();
}

int parseInt(const char* str) {
    int x;
    auto [ptr, ec] = std::from_chars(str, str + strlen(str), x);
    if (ec == std::errc{}) {
        return x;
    } else {
        std::cerr << "Failed to parse \"" << str << "\" as int" << std::endl;
        exit(1);
    }
}

int main(int argc, const char** argv) {
    if (argc < 5 + 1) {
        std::cout << "usage: " << argv[0] << " <cache path> <x1> <z1> <x2> <z2>" << std::endl;
        return 1;
    }
    const char* cache = argv[1];
    BlockPos start {
            parseInt(argv[2]),
            64,
            parseInt(argv[3])
    };
    BlockPos end {
            parseInt(argv[4]),
            64,
            parseInt(argv[5])
    };

    Context ctx{0, Dimension::OVERWORLD, std::string{cache}};
    auto realStart = findAir<Size::X4>(ctx, start);
    auto realGoal = findAir<Size::X4>(ctx, end);
    std::optional<Path> path = findPathFull(ctx, realStart, realGoal, 10000).value();
    if (!path.has_value()) {
        std::cerr << "failed to find path" << std::endl;
        return 1;
    }
    std::cout << "Writing path" << std::endl;
    writePath("out.path", path.value());
}
