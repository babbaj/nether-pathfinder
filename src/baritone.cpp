#include <cstdio>
#include <filesystem>
#include <iostream>
#define WITH_GZFILEOP 1
#include <zlib-ng.h>

#include "baritone.h"

int32_t beInt(std::span<const int8_t> span) {
    if (span.size() < 4) [[unlikely]] {
        throw std::range_error{"failed to read int"};
    }
    int32_t out = static_cast<int32_t>(span[0]) << 24
            | static_cast<int32_t>(span[1]) << 16
            | static_cast<int32_t>(span[2]) << 8
            | static_cast<int32_t>(span[3]);
    return out;
}

int8_t byte(std::span<const int8_t> span) {
    if (span.empty()) [[unlikely]] {
        throw std::range_error{"failed to read byte"};
    }
    return span[0];
}

template<size_t N>
std::array<int8_t, N> decomp(gzFile file) {
    std::array<int8_t, N> out{};
    if (zng_gzread(file, out.data(), N) < N) {
        throw std::range_error{"not enough data"};
    }
    return out;
}

int positionIndex(int x, int y, int z) {
    return (x << 1) | (z << 5) | (y << 9);
}

int8_t get2Bits(size_t i, std::span<const int8_t> data) {
    int8_t byte = data[i / 8];
    return (byte >> (6 - (i % 8))) & 0b11;
}

void parseAndInsertChunk(cache_t& cache, int chunkX, int chunkZ, std::span<const int8_t> data) {
    auto [it, inserted] = cache.try_emplace(ChunkPos{chunkX, chunkZ});
    if (inserted) {
        auto chunk = std::make_unique<Chunk>();
        chunk->isFromJava = true;
        for (int y = 0; y < 128; y++) {
            for (int z = 0; z < 16; z++) {
                for (int x = 0; x < 16; x++) {
                    auto idx = positionIndex(x, y, z);
                    auto bits = get2Bits(idx, data);
                    chunk->setBlock(x, y, z, bits != 0);
                }
            }
        }
        it->second = std::move(chunk);
    }
}

void parseBaritoneRegion(cache_t& cache, RegionPos regionPos, gzFile data) {
    int magic = beInt(decomp<4>(data));
    if (magic != 456022911) {
        puts("Bad magic");
        std::terminate();
    }
    for (int x = 0; x < 32; x++) {
        for (int z = 0; z < 32; z++) {
            const int8_t present = decomp<1>(data)[0];
            if (present == 1) {
                // DimensionType.height for the nether is 256 and that is what is used for serializing to disk
                constexpr auto chunkSizeBytes = (2 * 16 * 16 * 256) / 8;

                parseAndInsertChunk(cache, x + 32 * regionPos.x, z + 32 * regionPos.z, decomp<chunkSizeBytes>(data));
            }
        }
    }
    zng_gzclose_r(data);
}

std::optional<gzFile> openRegionFile(std::string_view dir, RegionPos pos) {
    auto fileName = std::string{"r."} + std::to_string(pos.x) + "." + std::to_string(pos.z) + ".bcr";
    auto path = std::filesystem::path{dir} / fileName;
    gzFile file = zng_gzopen(path.string().c_str(), "rb");
    if (!file) {
        return {};
    }
    zng_gzbuffer(file, 32768); // same as baritone
    return {file};
}
