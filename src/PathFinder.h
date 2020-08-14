#pragma once

#include <vector>
#include <optional>

#include "Utils.h"
#include "ChunkGeneratorHell.h"
#include "PathNode.h"

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

    [[nodiscard]] const BlockPos& getEndPos() const {
        // This should basically never be empty
        return !nodes.empty() ? nodes.back()->pos : this->start;
    }
};


std::optional<Path> findPath(const BlockPos& start, const BlockPos& goal, const ChunkGeneratorHell& gen);