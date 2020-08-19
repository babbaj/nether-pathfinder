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
PathNode* getNodeAtPosition(map_t<NodePos, std::unique_ptr<PathNode>>& map, const NodePos& pos, const BlockPos& goal) {
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


Path createPath(map_t<NodePos, std::unique_ptr<PathNode>>& map, const PathNode* start, const PathNode* end, const BlockPos& startPos, const BlockPos& goal, Path::Type pathType) {
    std::vector<std::unique_ptr<PathNode>> tempNodes;
    std::vector<BlockPos> tempPath;

    const PathNode* current = end;
    while (current) {
        auto& uniqueptr = map.at(current->pos);
        tempNodes.push_back(std::move(uniqueptr));
        tempPath.push_back(current->pos.absolutePosCenter());
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
// origin leans towards north/west/down
template<Face face>
inline std::array<BlockPos, 4> neighborCubes(const BlockPos& origin, int size) {
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
            const BlockPos oppositeCorner = corner.south(size).up(size);
            return {corner, corner.south(size), corner.up(size), oppositeCorner};
        }
    }
}


// face is relative to the original cube
template<Face face, Size size>
void forEachNeighborInCube(const Chunk& chunk, const NodePos& neighborNode, auto callback) {
    // I'm pretty sure this is already aligned
    const auto [nodeX, nodeY, nodeZ] = neighborNode.absolutePosZero();
    const auto [chunkX, chunkY, chunkZ] = neighborNode.absolutePosZero().toChunkLocal();

    switch (size) {
        case Size::X16: {
            if (chunk.isX16Empty(chunkY)) {
                callback(neighborNode);
                return;
            }
            break;
        }
        case Size::X8: {
            if (chunk.isX8Empty(chunkX, chunkY, chunkZ)) {
                callback(neighborNode);
                return;
            }
            break;
        }
        case Size::X4: {
            if (chunk.isX4Empty(chunkX, chunkY, chunkZ)) {
                callback(neighborNode);
                return;
            }
            break;
        }
        case Size::X2: {
            if (chunk.isX2Empty(chunkX, chunkY, chunkZ)) {
                callback(neighborNode);
                return;
            }
            break;
        }
        case Size::X1: {
            if (chunk.isX1Empty(chunkX, chunkY, chunkZ)) {
                callback(neighborNode);
            }
            return; // intentional return and not break
        }
    }
    if constexpr (size != Size::X1) {
        const auto mask = getSize(size) - 1;
        const int alignedX = nodeX & ~mask;
        const int alignedY = nodeY & ~mask;
        const int alignedZ = nodeZ & ~mask;
        constexpr auto nextSize = static_cast<Size>(static_cast<int>(size) - 1);
        const std::array subCubes = neighborCubes<face>({alignedX, alignedY, alignedZ}, getSize(nextSize));
        for (const BlockPos& subCube : subCubes) {
            forEachNeighborInCube<face, nextSize>(chunk, NodePos{nextSize, subCube}, callback);
        }
    }
}



template<Face face>
void growThenIterate(const Chunk& chunk, const NodePos& pos, auto callback) {
    const auto bpos = pos.absolutePosZero();

    switch (pos.size) {
        case Size::X1:
            if (!chunk.isX2Empty(bpos.x, bpos.y, bpos.z)) {
                forEachNeighborInCube<face, Size::X1>(chunk, pos, callback);
                return;
            }
        case Size::X2: 
            if (!chunk.isX4Empty(bpos.x, bpos.y, bpos.z)) {
                forEachNeighborInCube<face, Size::X2>(chunk, NodePos{Size::X2, bpos}, callback);
                return;
            }
        case Size::X4:
            if (!chunk.isX8Empty(bpos.x, bpos.y, bpos.z)) {
                forEachNeighborInCube<face, Size::X4>(chunk, NodePos{Size::X4, bpos}, callback);
                return;
            }
        case Size::X8:
            if (!chunk.isX16Empty(bpos.y)) {
                forEachNeighborInCube<face, Size::X8>(chunk, NodePos{Size::X8, bpos}, callback);
                return;
            }
        case Size::X16:
            forEachNeighborInCube<face, Size::X16>(chunk, NodePos{Size::X16, bpos}, callback);
    }
}

bool isInBounds(const BlockPos& pos) {
    return pos.y >= 0 && pos.y < 128;
}

template<Face face>
void expandNode(ChunkProvider chunks, Size size, const BlockPos& bpos, const ChunkPos& cpos, auto callback) {
    const NodePos neighborNodePos {size, bpos.offset(face, getSize(size))};
    if (!isInBounds(neighborNodePos.absolutePosZero())) return;

    const ChunkPos neighborCpos = neighborNodePos.absolutePosZero().toChunkPos();
    const Chunk& chunk = getOrGenChunk(chunks.cache, neighborCpos, chunks.generator, chunks.executor);

    growThenIterate<face>(chunk, neighborNodePos, callback);
}

void forEachNeighbor(ChunkProvider chunks, const PathNode& node, auto callback) {
    const auto size = node.pos.size;
    const auto bpos = node.pos.absolutePosZero();

    const ChunkPos cpos = bpos.toChunkPos();
    expandNode<Face::UP>(chunks, size, bpos, cpos, callback);
    expandNode<Face::DOWN>(chunks, size, bpos, cpos, callback);
    expandNode<Face::NORTH>(chunks, size, bpos, cpos, callback);
    expandNode<Face::SOUTH>(chunks, size, bpos, cpos, callback);
    expandNode<Face::EAST>(chunks, size, bpos, cpos, callback);
    expandNode<Face::WEST>(chunks, size, bpos, cpos, callback);
}

[[deprecated]] std::array<BlockPos, 6> getNeighbors(const BlockPos& pos) {
    return {pos.up(), pos.down(), pos.north(), pos.south(), pos.east(), pos.west()};
}


constexpr double MIN_DIST_PATH = 5; // might want to increase this

std::optional<Path> bestPathSoFar(map_t<NodePos, std::unique_ptr<PathNode>>& map, const PathNode* start, const PathNode* end, const BlockPos& startPos, const BlockPos& goal) {
    const double distSq = startPos.distanceToSq(end->pos.absolutePosCenter());

    if (distSq > MIN_DIST_PATH * MIN_DIST_PATH) {
        //return createPath(map, start, end, startPos, goal, Path::Type::SEGMENT);
    } else {
        std::cout << "Path took too long and got nowhere\n";
        auto [x, y, z] = end->pos.absolutePosCenter();
        std::cout << "(Path ended at {" << x << ", " << y << ", " << z << "})\n";
        //return std::nullopt;
    }

    // assume there's always a way
    return createPath(map, start, end, startPos, goal, Path::Type::SEGMENT);
}

bool inGoal(const NodePos& node, const BlockPos& goal) {
    // TODO: test if goal is in cube
    return node.absolutePosCenter().distanceToSq(goal) <= 10 * 10;
}

std::optional<Path> findPath0(const BlockPos& start, const BlockPos& goal, const ChunkGeneratorHell& gen, ParallelExecutor<3>& executor) {
    std::cout << "distance = " << start.distanceTo(goal) << '\n';

    map_t<ChunkPos, std::unique_ptr<Chunk>> chunkCache;
    map_t<NodePos, std::unique_ptr<PathNode>> map;
    BinaryHeapOpenSet openSet;

    PathNode* const startNode = getNodeAtPosition(map, NodePos{Size::X1, start}, goal);
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

        // TODO: get the right sub cube
        if (inGoal(currentNode->pos, goal)) {
            std::cout << "chunkCache size = " << chunkCache.size() << '\n';
            std::cout << "openSet size = " << openSet.getSize() << '\n';
            std::cout << "map size = " << map.size() << '\n';
            std::cout << '\n';
            return createPath(map, startNode, currentNode, start, goal, Path::Type::FINISHED);
        }

        forEachNeighbor({chunkCache, gen, executor}, *currentNode, [&](const NodePos& neighborPos) {
            const auto& block = neighborPos.absolutePosZero();

            PathNode* neighborNode = getNodeAtPosition(map, neighborPos, goal);
            auto sqrtSize = [](Size sz) { return sqrt(getSize(sz)); };
            const double cost = 1;//sqrtSize(neighborNode->pos.size);//getSize(neighborNode->pos.size);
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
                    // TODO: consider the size of the node
                    if (failing && start.distanceToSq(neighborPos.absolutePosCenter()) > MIN_DIST_PATH * MIN_DIST_PATH) {
                        failing = false;
                    }
                }
            }
        });
    }

    auto [x, y, z] = bestSoFar->pos.absolutePosCenter();
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