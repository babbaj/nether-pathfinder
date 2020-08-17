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

struct ChunkProvider {
    map_t<ChunkPos, std::unique_ptr<Chunk>>& cache;
    const ChunkGeneratorHell& generator;
    ParallelExecutor<3>& executor;
};

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


// TODO: take a ChunkProvider
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

// This is called inside of a big neighbor cube and returns the 4 sub cubes that are adjacent to the original cube.
// The face argument is relative to the original cube.
// This size argument is the size of the sub cubes.
std::array<BlockPos, 4> neighborCubes(const BlockPos& origin, Face face, int size) {
    // origin leans towards north/west/down

    switch (face) {
        case Face::UP: {
            const BlockPos& corner = origin;
            const BlockPos& oppositeCorner = origin.east(size).south(size);
            return {corner, corner.east(size), corner.south(size), oppositeCorner};
        }
        case Face::DOWN: {
            const BlockPos corner = origin.up(size);
            const BlockPos& oppositeCorner = origin.east(size).south(size);
            return {corner, corner.east(size), corner.south(size), oppositeCorner};
        }
        case Face::NORTH: {
            const BlockPos& corner = origin.south(size);
            const BlockPos oppositeCorner = corner.east(size).up(size);
            return {corner, corner.east(size), corner.up(size), oppositeCorner};
        }
        case Face::SOUTH: {
            const BlockPos& corner = origin;
            const BlockPos oppositeCorner = corner.east(size).up(size);
            return {corner, corner.east(size), corner.up(size), oppositeCorner};
        }
        case Face::EAST: {
            const BlockPos& corner = origin;
            const BlockPos oppositeCorner = corner.south(size).up(size);
            return {corner, corner.south(size), corner.up(size), oppositeCorner};
        }
        case Face::WEST: {
            const BlockPos& corner = origin.east(size);
            const BlockPos oppositeCorner = corner.east(size).up(size);
            return {corner, corner.east(size), corner.up(size), oppositeCorner};
        }
    }
}

// face is relative to the original cube
void forEachNeighborInCube(const Chunk& chunk, const NodePos& neighborNode, const Face face, auto callback) {
    const auto [nodeX, nodeY, nodeZ] = neighborNode.pos;
    const auto size = neighborNode.size;

    // TODO: reduce copy/pasting
    switch (size) {
        case Size::X16: {
            if (chunk.isX16Empty(nodeY)) {
                callback(neighborNode);
            } else {
                // x/y only ever use the first 4 bits
                const int alignedX = 0;
                const int alignedY = nodeY & ~15;
                const int alignedZ = 0;
                const std::array subCubes = neighborCubes({alignedX, alignedY, alignedZ}, face, 8);
                for (const BlockPos& subCube : subCubes) {
                    forEachNeighbor0(chunk, NodePos{Size::X8, subCube}, face, callback);
                }
            }
            break;
        }
        case Size::X8: {
            if (chunk.isX8Empty(nodeX, nodeY, nodeZ)) {
                callback(neighborNode);
            } else {
                const int alignedX = nodeX & ~7;
                const int alignedY = nodeY & ~7;
                const int alignedZ = nodeZ & ~7;
                const std::array subCubes = neighborCubes({alignedX, alignedY, alignedZ}, face, 4);
                for (const BlockPos& subCube : subCubes) {
                    forEachNeighbor0(chunk, NodePos{Size::X8, subCube}, face, callback);
                }
            }
            break;
        }
        case Size::X4: {
            if (chunk.isX4Empty(nodeX, nodeY, nodeZ)) {
                callback(neighborNode);
            } else {
                const int alignedX = nodeX & ~3;
                const int alignedY = nodeY & ~3;
                const int alignedZ = nodeZ & ~3;
                const std::array subCubes = neighborCubes({alignedX, alignedY, alignedZ}, face, 2);
                for (const BlockPos& subCube : subCubes) {
                    forEachNeighbor0(chunk, NodePos{Size::X8, subCube}, face, callback);
                }
            }
            break;
        }
        case Size::X2: {
            if (chunk.isX2Empty(nodeX, nodeY, nodeZ)) {
                callback(neighborNode);
            } else {
                const int alignedX = nodeX & ~1;
                const int alignedY = nodeY & ~1;
                const int alignedZ = nodeZ & ~1;
                const std::array subCubes = neighborCubes({alignedX, alignedY, alignedZ}, face, 1);
                for (const BlockPos& subCube : subCubes) {
                    forEachNeighbor0(chunk, NodePos{Size::X8, subCube}, face, callback);
                }
            }
            break;
        }
        case Size::X1: {
            if (chunk.isX1Empty(nodeX, nodeY, nodeZ)) {
                callback(neighborNode);
            }
            break;
        }
    }
}

int getSize(Size sizeEnum) {
    switch (sizeEnum) {
        case X16: return 16;
        case X8: return 8;
        case X4: return 4;
        case X2: return 2;
        case X1: return 1;
    }
}

void forEachNeighbor(ChunkProvider chunks, const PathNode& node, auto callback) {
    const auto[size, pos] = node.pos0;
    const ChunkPos cpos = pos.toChunkPos();
    const Chunk& currentChunk = getOrGenChunk(chunks.cache, cpos, chunks.generator, chunks.executor);

    for (const enum Face face : ALL_FACES) {
        const NodePos neighborNodePos = {size, pos.offset(face, getSize(size))};
        const ChunkPos neighborCpos = neighborNodePos.pos.toChunkPos();
        const Chunk& chunk = neighborCpos == cpos
            ? currentChunk : getOrGenChunk(chunks.cache, neighborCpos, chunks.generator, chunks.executor);

        forEachNeighborInCube(chunk, neighborNodePos, face, callback);
    }
}

std::array<BlockPos, 6> getNeighbors(const BlockPos& pos) {
    return {pos.up(), pos.down(), pos.north(), pos.south(), pos.east(), pos.west()};
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
        const auto pos0 = currentNode->pos0;
        const auto cpos = pos.toChunkPos();
        const Chunk& currentChunk = getOrGenChunk(chunkCache, cpos, gen, executor);

        for (const enum Face face : ALL_FACES) {
            const BlockPos neighborCubePos = pos0.pos.offset(face, pos0.size); // approximate position, good enough to query the cube
            const Chunk& neighborChunk = neighborCubePos.toChunkPos() == cpos
                ? currentChunk : getOrGenChunk(chunkCache, neighborCubePos.toChunkPos(), gen, executor);

            switch (pos0.size) {

            }
        }

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