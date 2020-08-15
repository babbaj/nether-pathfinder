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
#include "absl/container/node_hash_map.h"


template<typename K, typename V>
//using map_t = std::unordered_map<K, V>;
using map_t = absl::flat_hash_map<K, V>;
//using map_t = absl::node_hash_map<K, V>;

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


Path createPath(map_t<BlockPos, std::unique_ptr<PathNode>>& map, const PathNode* start, const PathNode* end, const BlockPos& startPos, const BlockPos& goal, Path::Type pathType) {
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
        pathType,
        startPos,
        goal,
        std::move(path),
        std::move(nodes)
    };
}

std::array<BlockPos, 6> getNeighbors(const BlockPos& pos) {
    return {pos.up(), pos.down(), pos.north(), pos.south(), pos.east(), pos.west()};
}

Chunk& getOrGenChunk(map_t<ChunkPos, std::unique_ptr<Chunk>>& cache, const ChunkPos& pos, const ChunkGeneratorHell& generator, ParallelExecutor<3>& executor) {
    auto it = cache.find(pos);
    if (it != cache.end()) {
        return *it->second;
    } else {
        std::unique_ptr ptr = std::make_unique<Chunk>();
        auto& chunk = *ptr;
        generator.generateChunk(pos.x, pos.z, *ptr, executor);
        cache.emplace(pos, std::move(ptr));
        return chunk;
    }
}

constexpr double MIN_DIST_PATH = 5; // might want to increase this

std::optional<Path> bestPathSoFar(map_t<BlockPos, std::unique_ptr<PathNode>>& map, const PathNode* start, const PathNode* end, const BlockPos& startPos, const BlockPos& goal) {
    const double distSq = startPos.distanceToSq(end->pos);

    if (distSq > MIN_DIST_PATH * MIN_DIST_PATH) {
        //return createPath(map, start, end, startPos, goal, Path::Type::SEGMENT);
    } else {
        std::cout << "Path took too long and got nowhere\n";
        auto [x, y, z] = end->pos;
        std::cout << "(Path ended at {" << x << ", " << y << ", " << z << "})\n";
        //return std::nullopt;
    }

    // assume there's always a way
    return createPath(map, start, end, startPos, goal, Path::Type::SEGMENT);
}

std::optional<Path> findPath0(const BlockPos& start, const BlockPos& goal, const ChunkGeneratorHell& gen, ParallelExecutor<3>& executor) {

    std::cout << "distance = " << start.distanceTo(goal) << '\n';

    map_t<ChunkPos, std::unique_ptr<Chunk>> chunkCache;
    map_t<BlockPos, std::unique_ptr<PathNode>> map;
    BinaryHeapOpenSet openSet;

    PathNode* const startNode = getNodeAtPosition(map, start, goal);
    startNode->cost = 0;
    startNode->combinedCost = startNode->estimatedCostToGoal;
    openSet.insert(startNode);

    PathNode* bestSoFar = startNode;
    double bestHeuristicSoFar = startNode->estimatedCostToGoal;

    using namespace std::chrono_literals;
    const auto now = std::chrono::system_clock::now();
    const auto primaryTimeoutTime = now + 500ms;
    const auto failureTimeout = now + 1min;

    bool failing = true;
    int numNodes = 0;
    while (!openSet.isEmpty()) {
        constexpr int timeCheckInterval = 1 << 6;
        if ((numNodes & (timeCheckInterval - 1)) == 0) { // only call this once every 64 nodes
            auto now = std::chrono::system_clock::now();

            if (now >= failureTimeout || (!failing && now >= primaryTimeoutTime)) {
                break;
            }
        }

        PathNode* currentNode = openSet.removeLowest();

        if (currentNode->pos == goal) {
            std::cout << "chunkCache size = " << chunkCache.size() << '\n';
            std::cout << "openSet size = " << openSet.getSize() << '\n';
            std::cout << "map size = " << map.size() << '\n';
            std::cout << '\n';
            return createPath(map, startNode, currentNode, start, goal, Path::Type::FINISHED);
        }

        const auto pos = currentNode->pos;
        const auto cpos = pos.toChunkPos();
        const Chunk& currentChunk = getOrGenChunk(chunkCache, cpos, gen, executor);
        for (auto neighbors = getNeighbors(pos); const BlockPos& neighborBlock : neighbors) {
            // TODO: make sure neighborBlock is in bounds
            // avoid unnecessary lookups
            const Chunk& neighborChunk = neighborBlock.toChunkPos() == cpos
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
                    if (failing && start.distanceToSq(neighborBlock) > MIN_DIST_PATH * MIN_DIST_PATH) {
                        failing = false;
                    }
                }
            }
        }


    }

    auto [x, y, z] = bestSoFar->pos;
    std::cout << "Best position = {" << x << ", " << y << ", " << z << "}\n";
    std::cout << "failing = " << failing << '\n';
    std::cout << "Open set getSize: " << openSet.getSize() << '\n';
    std::cout << "PathNode map size: " << map.size() << '\n';
    std::cout << "chunk cache size: " << chunkCache.size() << '\n';
    std::cout << '\n';

    return bestPathSoFar(map, startNode, bestSoFar, start, goal);
}

void appendPath(Path& path, Path&& segment) {
    path.blocks.insert(path.blocks.end(), segment.blocks.begin(), segment.blocks.end());
    path.nodes.insert(path.nodes.end(), std::move_iterator{segment.nodes.begin()}, std::move_iterator{segment.nodes.end()});
}

Path splicePaths(std::vector<Path>&& paths) {
    Path path = std::move(paths.at(0));

    std::for_each(paths.begin() + 1, paths.end(), [&path](Path& segment) {
        appendPath(path, std::move(segment));
    });

    return path;
}

std::optional<Path> findPath(const BlockPos& start, const BlockPos& goal, const ChunkGeneratorHell& gen) {
    ParallelExecutor<3> executor;

    std::vector<Path> segments;

    while (true) {
        const BlockPos lastPathEnd = !segments.empty() ? segments.back().getEndPos() : start;
        std::optional path = findPath0(lastPathEnd, goal, gen, executor);
        if (!path.has_value()) {
            break;
        } else {
            const bool finished = path->type == Path::Type::FINISHED;
            // TODO: print current position
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