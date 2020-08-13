#include "PathFinder.h"
#include "PathNode.h"
#include "BinaryHeapOpenSet.h"
#include "ParallelExecutor.h"

#include <unordered_map>
#include <memory>
#include <array>
#include <iostream>
#include <algorithm>

#include "absl/container/flat_hash_map.h"


template<typename K, typename V>
//using map_t = std::unordered_map<K, V>;
using map_t = absl::flat_hash_map<K, V>;

// never returns null
PathNode* getNodeAtPosition(map_t<BlockPos, std::unique_ptr<PathNode>>& map, const BlockPos& pos, const BlockPos& goal) {
    auto iter = map.find(pos);
    if (iter == map.end()) {
        auto node = std::make_unique<PathNode>(pos, goal);
        auto it = map.emplace(pos, std::move(node));
        assert(it.second);
        return it.first->second.get();
    } else {
        return iter->second.get();
    }
}

bool closeEnough(const PathNode& node, const BlockPos& goal) {
    return node.pos.distanceTo(goal) < 5;
}

Path createPath(map_t<BlockPos, std::unique_ptr<PathNode>>& map, const PathNode* start, const PathNode* end, const BlockPos& startPos, const BlockPos& goal) {
    std::vector<std::unique_ptr<PathNode>> tempNodes;
    std::vector<BlockPos> tempPath;

    const PathNode* current = end;
    while (current != nullptr) {
        auto& uniqueptr = map.at(current->pos);
        tempNodes.push_back(std::move(uniqueptr));
        tempPath.push_back(current->pos);
        current = current->previous;
    }

    //auto nodes = decltype(tempNodes)(tempPath.rbegin(), tempPath.rend());
    auto nodes = decltype(tempNodes){}; nodes.reserve(tempNodes.size());
    std::move(tempNodes.rbegin(), tempNodes.rend(), std::back_inserter(nodes));

    auto path = decltype(tempPath){}; path.reserve(tempPath.size());
    std::move(tempPath.rbegin(), tempPath.rend(), std::back_inserter(path));

    return Path {
        startPos,
        goal,
        std::move(path),
        std::move(nodes)
    };
}

std::array<BlockPos, 6> getNeighbors(const BlockPos& pos) {
    return {pos.up(), pos.down(), pos.north(), pos.south(), pos.east(), pos.west()};
}

ChunkPrimer& getOrGenChunk(map_t<ChunkPos, std::unique_ptr<ChunkPrimer>>& cache, const ChunkPos& pos, const ChunkGeneratorHell& generator, ParallelExecutor<3>& executor) {
    auto it = cache.find(pos);
    if (it != cache.end()) {
        return *it->second;
    } else {
        std::unique_ptr ptr = std::make_unique<ChunkPrimer>();
        auto& chunk = *ptr;
        generator.generateChunk(pos.x, pos.z, *ptr, executor);
        cache.emplace(pos, std::move(ptr));
        return chunk;
    }
}

std::optional<Path> findPath(const BlockPos& start, const BlockPos& goal, const ChunkGeneratorHell& gen) {
    std::cout << "distance = " << start.distanceTo(goal) << '\n';

    map_t<ChunkPos, std::unique_ptr<ChunkPrimer>> chunkCache;
    map_t<BlockPos, std::unique_ptr<PathNode>> map;
    BinaryHeapOpenSet openSet;
    ParallelExecutor<3> executor;

    PathNode* const startNode = getNodeAtPosition(map, start, goal);
    startNode->cost = 0;
    startNode->combinedCost = startNode->estimatedCostToGoal;
    openSet.insert(startNode);

    PathNode* bestSoFar = startNode;
    double bestHeuristicSoFar = startNode->estimatedCostToGoal;

    auto t1 = std::chrono::steady_clock::now();
    while (!openSet.isEmpty()) {
        PathNode* currentNode = openSet.removeLowest();

        if (closeEnough(*currentNode, goal)) {
            std::cout << "chunkCache size = " << chunkCache.size() << '\n';
            std::cout << "openSet size = " << openSet.getSize() << '\n';
            std::cout << "map size = " << map.size() << '\n';
            return createPath(map, startNode, currentNode, start, goal);
        }

        const auto pos = currentNode->pos;
        const auto cpos = pos.toChunkPos();
        const ChunkPrimer& currentChunk = getOrGenChunk(chunkCache, cpos, gen, executor);
        for (auto neighbors = getNeighbors(pos); const BlockPos& neighborBlock : neighbors) {
            // avoid unnecessary lookups
            const ChunkPrimer& neighborChunk = neighborBlock.toChunkPos() == cpos
                ? currentChunk : getOrGenChunk(chunkCache, neighborBlock.toChunkPos(), gen, executor);

            if (neighborChunk.isSolid(neighborBlock.toChunkLocal())) {
                continue;
            }

            PathNode* neighborNode = getNodeAtPosition(map, neighborBlock, goal);
            constexpr double cost = 1; // cost to move 1 block is 1
            const double tentativeCost = currentNode->cost + cost;
            constexpr double MIN_IMPROVEMENT = 0.01;
            if (neighborNode->cost - tentativeCost > MIN_IMPROVEMENT) {
                neighborNode->previous = currentNode;
                neighborNode->cost = tentativeCost;
                neighborNode->combinedCost = tentativeCost + neighborNode->estimatedCostToGoal;

                if (neighborNode->isOpen()) {
                    openSet.update(neighborNode);
                } else {
                    openSet.insert(neighborNode);//dont double count, dont insert into open set if it's already there
                }

                const double heuristic = neighborNode->estimatedCostToGoal + neighborNode->cost;
                if (bestHeuristicSoFar - heuristic > MIN_IMPROVEMENT) {
                    bestHeuristicSoFar = heuristic;
                    bestSoFar = neighborNode;
                }
            }
        }


    }
    std::cout << "Open set getSize: " << openSet.getSize() << '\n';
    std::cout << "PathNode map getSize: " << map.size() << '\n';
    // TODO: return bestSoFar if <100 blocks?

    return std::nullopt; // :sob:
}