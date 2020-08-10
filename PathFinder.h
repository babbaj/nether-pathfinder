#pragma once

#include <vector>
#include <optional>

#include "Utils.h"
#include "ChunkGeneratorHell.h"
#include "PathNode.h"

struct Path {
    BlockPos start;
    BlockPos goal;
    std::vector<BlockPos> path;
    std::vector<std::unique_ptr<PathNode>> nodes;
};


std::optional<Path> findPath(const BlockPos& start, const BlockPos& goal, const ChunkGeneratorHell& gen);