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

enum class PathTypeEnum {
    SEGMENT,
    FINISHED
};

template<typename BlockType>
struct Path {
    using block_type = BlockType;

    PathTypeEnum type;
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

// PathFinder.cpp
extern std::optional<Path3D> findPath0(const BlockPos& start, const BlockPos& goal, ChunkGeneratorHell& gen);

// LavaPathFinder.cpp
extern std::optional<Path2D> findPath0(const Pos2D& start, const Pos2D& goal, ChunkGeneratorHell& gen);

inline bool isInBounds(const BlockPos& pos) {
    return pos.y >= 0 && pos.y < 128;
}

template<typename PathType>
void appendPath(PathType& path, PathType&& segment) {
    path.blocks.insert(path.blocks.end(), segment.blocks.begin(), segment.blocks.end());
    path.nodes.insert(path.nodes.end(), std::move_iterator{segment.nodes.begin()}, std::move_iterator{segment.nodes.end()});
}

template<typename PathType>
PathType splicePaths(std::vector<PathType>&& paths) {
    Path path = std::move(paths.at(0));

    std::for_each(paths.begin() + 1, paths.end(), [&path](PathType& segment) {
        appendPath(path, std::move(segment));
    });

    return path;
}

// TODO: pass findPath0 impl as argument?
template<typename PathType>
std::optional<PathType> findPath(const typename PathType::block_type& start, const typename PathType::block_type& goal, ChunkGeneratorHell& gen) {
    if constexpr (std::is_same_v<PathType, Path3D>) {
        if (!isInBounds(start)) throw "troll";
    }

    std::vector<Path3D> segments;

    while (true) {
        const BlockPos lastPathEnd = !segments.empty() ? segments.back().getEndPos() : start;
        std::optional path = findPath0(lastPathEnd, goal, gen);
        if (!path.has_value()) {
            break;
        } else {
            const bool finished = path->type == PathTypeEnum::FINISHED;
            segments.push_back(std::move(*path));
            if (finished) break;
        }
    }

    if (!segments.empty()) {
        return splicePaths(std::move(segments));
    } else {
        return std::nullopt;
    }
}