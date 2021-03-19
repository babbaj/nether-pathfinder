#pragma once

#include <vector>
#include <optional>
#include <variant>

#include "Utils.h"
#include "ChunkGeneratorHell.h"
#include "PathNode.h"

#include "absl/container/flat_hash_map.h"


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
    using block_type = BlockType; // BlockPos/Pos2D
    using node_type = typename NodeTypeForBlockType<BlockType>::type; // PathNode3D/PathNode2D

    PathTypeEnum type;
    BlockType start;
    BlockType goal; // where the path wants to go, not necessarily where it ends
    std::vector<BlockType> blocks;

    std::vector<std::unique_ptr<node_type>> nodes;

    [[nodiscard]] const BlockType& getEndPos() const {
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

    std::vector<PathType> segments;

    while (true) {
        const auto lastPathEnd = !segments.empty() ? segments.back().getEndPos() : start;
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

namespace impl {
    template<typename K, typename V>
    using map_t = absl::flat_hash_map<K, V>;

    inline Chunk& getOrGenChunk(map_t<ChunkPos, std::unique_ptr<Chunk>>& cache, const ChunkPos& pos, ChunkGeneratorHell& generator) {
        auto it = cache.find(pos);
        if (it != cache.end()) {
            return *it->second;
        } else {
            std::unique_ptr ptr = std::make_unique<Chunk>();
            auto& chunk = *ptr;
            generator.generateChunk(pos.x, pos.z, *ptr);
            cache.emplace(pos, std::move(ptr));
            return chunk;
        }
    }
}