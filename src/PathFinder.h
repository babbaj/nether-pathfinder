#pragma once

#include <vector>
#include <optional>
#include <unordered_set>

#include <jni.h>

#include "Utils.h"
#include "ChunkGeneratorHell.h"
#include "PathNode.h"
#include "ChunkGen.h"
#include "Allocator.h"

enum class FakeChunkMode {
    GENERATE = 0
    ,AIR = 1
    ,SOLID = 2
};

struct Path {
    enum class Type {
        SEGMENT,
        FINISHED
    };

    Type type;
    BlockPos start;
    BlockPos goal; // where the path wants to go, not necessarily where it ends
    std::vector<BlockPos> blocks;
    std::vector<std::unique_ptr<PathNode>> nodes;
    cache_t chunkCache;

    [[nodiscard]] const BlockPos& getEndPos() const {
        // This should basically never be empty
        return !blocks.empty() ? blocks.back() : this->start;
    }
};

struct Context {
    ChunkGeneratorHell generator;
    std::optional<std::string> baritoneCache;
    std::mutex cacheMutex;
    std::unique_ptr<Allocator<Chunk>> chunkAllocator;
    cache_t chunkCache;
    ParallelExecutor<4> topExecutor;
    std::array<ChunkGenExec, 4> executors;
    std::atomic_flag cancelFlag;
    std::unordered_set<RegionPos> checkedRegions;
    int maxHeight;
    Dimension dimension;


    explicit Context(int64_t seed, std::optional<std::string>&& cacheDir, Dimension dim, int maxHeight, bool pageAllocator):
        generator(ChunkGeneratorHell::fromSeed(seed)), baritoneCache(cacheDir), maxHeight(maxHeight), dimension(dim)
        {
            if (maxHeight <= 0 || maxHeight > 384) {
                throw std::range_error("bad max height");
            }
            if (pageAllocator && getPageSize() == 4096) {
                chunkAllocator = std::make_unique<PageAllocator<Chunk>>();
            } else {
                chunkAllocator = std::make_unique<Allocator<Chunk>>();
            }
        }
    explicit Context(int64_t seed, Dimension dim, int maxHeight, bool pageAllocator): Context(seed, std::nullopt, dim, maxHeight, pageAllocator) {}
    explicit Context(int64_t seed, std::string&& cacheDir, Dimension dim, int maxHeight, bool pageAllocator): Context(seed, std::optional{cacheDir}, dim, maxHeight, pageAllocator) {}
    ~Context() {
        // useless optimization
        if (!chunkAllocator->auto_frees_on_destroy()) {
            for (auto &p: chunkCache) {
                if (p.second.second) {
                    chunkAllocator->free(p.second.second);
                }
            }
        }
    }
};

// long name but I do not care
// simply calls getRealChunkOrDefault or getOrGenChunk depending on mode
const Chunk& getRealChunkFromCacheOrFakeChunkMaybeGen(Context& ctx, ChunkGenExec& executor, const ChunkPos& pos, FakeChunkMode mode);
// gets from cache, or generates and inserts into cache
const Chunk& getOrGenChunk(Context& ctx, ChunkGenExec& executor, const ChunkPos& pos);
const Chunk& getRealChunkOrDefault(Context& ctx, const ChunkPos& pos, bool solid);

std::optional<Path> findPathFull(Context& ctx, const NodePos& start, const NodePos& goal, double fakeChunkCost);
std::optional<Path> findPathSegment(Context& ctx, const NodePos& start, const NodePos& goal, bool x4Min, int failTimeoutMs, bool airIfFake, double fakeChunkCost);

template<Size size>
NodePos findAir(Context& ctx, const BlockPos& start1x);
