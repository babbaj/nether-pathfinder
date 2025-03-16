#include <cstdio>
#include <filesystem>
#include <iostream>
#define WITH_GZFILEOP 1
#include <zlib-ng.h>

#include "baritone.h"
#include "Allocator.h"

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

void parseAndInsertChunk(Allocator<Chunk>& chunkAllocator, cache_t& cache, int chunkX, int chunkZ, int height, std::span<const int8_t> data) {
    auto [it, inserted] = cache.try_emplace(ChunkPos{chunkX, chunkZ});
    if (inserted) {
        auto chunk = chunkAllocator.allocate();
        for (int y = 0; y < height; y++) {
            bool foundBlockInPage = false;
            for (int z = 0; z < 16; z++) {
                for (int x = 0; x < 16; x++) {
                    auto idx = positionIndex(x, y, z);
                    auto bits = get2Bits(idx, data);
                    if (y % 128 == 0) foundBlockInPage = false;
                    // Don't do unnecessary writes because they may trigger a page to be unnecessarily allocated.
                    // Also try to be branch predictor friendly (too lazy to verify if this is any different than branching on bits != 0 every time).
                    foundBlockInPage |= bits != 0;
                    if (foundBlockInPage) {
                        chunk->setBlock(x, y, z, bits != 0);
                    }
                }
            }
        }
        it->second = {ChunkState::FROM_JAVA, chunk};
    }
}

void parseBaritoneRegion(Allocator<Chunk>& allocator, cache_t& cache, RegionPos regionPos, gzFile data, Dimension dim) {
    try {
        int magic = beInt(decomp<4>(data));
        if (magic != 456022911) {
            std::cerr << "Bad magic for baritone region " << regionPos.x << "," << regionPos.z << std::endl;
            goto close;
        }
        for (int x = 0; x < 32; x++) {
            for (int z = 0; z < 32; z++) {
                const int8_t present = decomp<1>(data)[0];
                if (present == 1) {
                    switch (dim) {
                        case Dimension::Overworld:
                            static constexpr size_t chunkSizeOverworld = (2 * 16 * 16 * 384) / 8;
                            parseAndInsertChunk(allocator, cache, x + 32 * regionPos.x, z + 32 * regionPos.z, 384, decomp<chunkSizeOverworld>(data));
                            break;
                        case Dimension::Nether:
                        case Dimension::End:
                            static constexpr size_t chunkSize = (2 * 16 * 16 * 256) / 8;
                            parseAndInsertChunk(allocator, cache, x + 32 * regionPos.x, z + 32 * regionPos.z, 256, decomp<chunkSize>(data));
                            break;
                    }
                }
            }
        }
    } catch(...) {
        std::cerr << "exception thrown parsing " << regionPos.x << "," << regionPos.z << std::endl;
    }
    close:
    zng_gzclose_r(data);
}

std::optional<std::tuple<gzFile, Dimension>> openRegionFile(std::string_view dir, RegionPos pos) {
    Dimension dim;
    auto cacheParent = std::filesystem::path{dir}.parent_path().filename().string();
    if (cacheParent == "the_nether_128") {
        dim = Dimension::Nether;
    } else if (cacheParent == "overworld_384") {
        dim = Dimension::Overworld;
    } else if (cacheParent == "the_end_256") {
        dim = Dimension::End;
    } else {
        return {};
    }

    auto fileName = std::string{"r."} + std::to_string(pos.x) + "." + std::to_string(pos.z) + ".bcr";
    auto path = std::filesystem::path{dir} / fileName;
    gzFile file = zng_gzopen(path.string().c_str(), "rb");
    if (!file) {
        return {};
    }
    zng_gzbuffer(file, 32768); // same as baritone
    return {{file, dim}};
}
