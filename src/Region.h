#pragma once
#include "Utils.h"
#include "Chunk.h"
#include "ChunkGen.h"

enum class Dimension : uint8_t {
    OVERWORLD = 0
    ,NETHER = 1
    ,END = 2
};
constexpr int DIMENSION_MIN_Y[] = {-64, 0, 0};
constexpr int DIMENSION_Y_SIZE[] = {384, 256, 256};

struct Region {
    std::array<std::array<Chunk, 32>, 32> chunks{};
    std::array<std::array<ChunkState, 32>, 32> states{};
    Dimension dimension;

public:
    static std::unique_ptr<Region, std::function<void(Region*)>> allocate();
    [[nodiscard]] Chunk& getChunk(int x, int z) { return chunks[x & 31][z & 31]; }
    [[nodiscard]] const ChunkState getChunkState(int x, int z) const { return states[x & 31][z & 31]; }
    void setChunkState(int x, int z, ChunkState state) { states[x & 31][z & 31] = state; }
};


using region_cache_t = map_t<RegionPos , std::unique_ptr<Region, std::function<void(Region*)>>>;
using region_lock_t = std::mutex;

struct RegionCache {
    region_lock_t lock;
    region_cache_t cache;
    Region& getRegion(RegionPos pos);
    Region& getRegion(int x, int z);
    Region& getRegion_locked(RegionPos pos);
    const int size() const { return this->cache.size(); };
};
