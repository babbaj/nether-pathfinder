#pragma once

#include <vector>
#include <optional>

#include "Utils.h"
#include "ChunkGeneratorHell.h"
#include "PathNode.h"
#include "ChunkGen.h"

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
    cache_t chunkCache;
    ParallelExecutor<4> topExecutor;
    std::array<ChunkGenExec, 4> executors;
    std::atomic_flag cancelFlag;

    explicit Context(int64_t seed): generator(ChunkGeneratorHell::fromSeed(seed)) {}
};

std::optional<Path> findPathFull(Context& ctx, const BlockPos& start, const BlockPos& goal);

template<Size minSize>
std::optional<Path> findPathSegment(Context& ctx, const BlockPos& start, const BlockPos& goal);

extern template std::optional<Path> findPathSegment<Size::X2>(Context& ctx, const BlockPos& start, const BlockPos& goal);
extern template std::optional<Path> findPathSegment<Size::X4>(Context& ctx, const BlockPos& start, const BlockPos& goal);

BlockPos findAir(const BlockPos& pos, const ChunkGeneratorHell& gen);