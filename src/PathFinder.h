#pragma once

#include <vector>
#include <optional>
#include <variant>

#include "Utils.h"
#include "ChunkGeneratorHell.h"
#include "PathNode.h"

namespace impl {
    
}

template<typename>
struct NodeTypeForBlockType;
template<>
struct NodeTypeForBlockType<BlockPos> {
    using type = PathNode3D;
};
template<>
struct NodeTypeForBlockType<Pos2D> {
    using type = PathNode2D;
};

template<typename BlockType>
struct Path {
    enum class Type {
        SEGMENT,
        FINISHED
    };

    Type type;
    BlockPos start;
    BlockPos goal; // where the path wants to go, not necessarily where it ends
    std::vector<BlockType> blocks;

    std::vector<std::unique_ptr<typename NodeTypeForBlockType<BlockType>::type>> nodes;

    [[nodiscard]] const BlockPos& getEndPos() const {
        // This should basically never be empty
        return !blocks.empty() ? blocks.back() : this->start;
    }
};

using Path3D = Path<BlockPos>;
using Path2D = Path<Pos2D>;


std::optional<Path3D> findPath(const BlockPos& start, const BlockPos& goal, const ChunkGeneratorHell& gen);

std::optional<Path2D> findLavaPath(const Pos2D& start, const Pos2D& goal, const ChunkGeneratorHell& gen);