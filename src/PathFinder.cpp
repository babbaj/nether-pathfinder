#include "PathFinder.h"
#include "BinaryHeapOpenSet.h"

#include <unordered_map>
#include <memory>
#include <array>
#include <iostream>
#include <algorithm>

#include "absl/container/flat_hash_map.h"
#include "absl/container/node_hash_map.h"


template<typename K, typename V>
using map_t = absl::flat_hash_map<K, V>;

struct ChunkProvider {
    map_t<ChunkPos, std::unique_ptr<Chunk>>& cache;
    ChunkGeneratorHell& generator;
};

// never returns null
PathNode3D* getNodeAtPosition(map_t<NodePos, std::unique_ptr<PathNode3D>>& map, const NodePos& pos, const BlockPos& goal) {
    auto iter = map.find(pos);
    if (iter == map.end()) {
        auto node = std::make_unique<PathNode3D>(pos, goal);
        auto it = map.emplace(pos, std::move(node));
        assert(it.second);
        return it.first->second.get();
    } else {
        return iter->second.get();
    }
}


Path3D createPath(map_t<NodePos, std::unique_ptr<PathNode3D>>& map, const PathNode3D* start, const PathNode3D* end, const BlockPos& startPos, const BlockPos& goal, PathTypeEnum pathType) {
    std::vector<std::unique_ptr<PathNode3D>> tempNodes;
    std::vector<BlockPos> tempPath;

    const PathNode3D* current = end;
    while (current) {
        auto& uniqueptr = map.at(current->pos);
        tempNodes.push_back(std::move(uniqueptr));
        tempPath.push_back(current->pos.absolutePosCenter());
        current = static_cast<PathNode3D*>(current->previous);
    }

    //auto nodes = decltype(tempNodes)(tempPath.rbegin(), tempPath.rend());
    auto nodes = decltype(tempNodes){}; nodes.reserve(tempNodes.size());
    std::move(tempNodes.rbegin(), tempNodes.rend(), std::back_inserter(nodes));

    auto path = decltype(tempPath){}; path.reserve(tempPath.size());
    std::move(tempPath.rbegin(), tempPath.rend(), std::back_inserter(path));

    return Path3D {
        pathType,
        startPos,
        goal,
        std::move(path),
        std::move(nodes)
    };
}


// TODO: take a ChunkProvider
Chunk& getOrGenChunk(map_t<ChunkPos, std::unique_ptr<Chunk>>& cache, const ChunkPos& pos, ChunkGeneratorHell& generator) {
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

// This is called inside of a big neighbor cube and returns the 4 sub cubes that are adjacent to the original cube.
// The face argument is relative to the original cube.
// This size argument is the size of the sub cubes.
// origin leans towards north/west/down
template<Face face, Size sz>
std::array<BlockPos, 4> neighborCubes(const BlockPos& origin) {
    // origin leans towards north/west/down
    constexpr auto size = getSize(sz);
    switch (face) {
        case Face::UP: {
            const BlockPos& corner = origin;
            const BlockPos oppositeCorner = origin.east(size).south(size);
            return {corner, corner.east(size), corner.south(size), oppositeCorner};
        }
        case Face::DOWN: {
            const BlockPos corner = origin.up(size);
            const BlockPos oppositeCorner = origin.east(size).south(size);
            return {corner, corner.east(size), corner.south(size), oppositeCorner};
        }
        case Face::NORTH: {
            const BlockPos corner = origin.south(size);
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
            const BlockPos corner = origin.east(size);
            const BlockPos oppositeCorner = corner.south(size).up(size);
            return {corner, corner.south(size), corner.up(size), oppositeCorner};
        }
    }
}


// face is relative to the original cube
template<Face face, Size size, bool sizeChange>
void forEachNeighborInCube(const Chunk& chunk, const NodePos& neighborNode, auto& callback) {
    if constexpr (sizeChange) {
        callback(neighborNode);
        return;
    }
    const auto pos = neighborNode.absolutePosZero();
    if (chunk.isEmpty<size>(pos.x, pos.y, pos.z)) {
        callback(neighborNode);
        return;
    }
    if constexpr (size != Size::X1) {
        constexpr auto nextSize = static_cast<Size>(static_cast<int>(size) - 1);
        // Don't shrink cubes to X1 because they suck and make the path try to squeeze through small areas
        // TODO: allow this to be configurable?
        if constexpr (nextSize == Size::X1) return;
        const std::array subCubes = neighborCubes<face, nextSize>(pos);
        for (const BlockPos& subCube : subCubes) {
            forEachNeighborInCube<face, nextSize, false>(chunk, NodePos{nextSize, subCube}, callback);
        }
    }
}


template<Face face, Size originalSize>
void growThenIterateInner(const Chunk& chunk, const NodePos& pos, auto& callback) {
    const auto bpos = pos.absolutePosZero();

    switch (originalSize) {
        case Size::X1:
            if (!chunk.isEmpty<Size::X2>(bpos.x, bpos.y, bpos.z)) {
                forEachNeighborInCube<face, Size::X1, originalSize != Size::X1>(chunk, pos, callback);
                return;
            }
        case Size::X2: 
            if (!chunk.isEmpty<Size::X4>(bpos.x, bpos.y, bpos.z)) {
                forEachNeighborInCube<face, Size::X2, originalSize != Size::X2>(chunk, NodePos{Size::X2, bpos}, callback);
                return;
            }
        case Size::X4:
            if (!chunk.isEmpty<Size::X8>(bpos.x, bpos.y, bpos.z)) {
                forEachNeighborInCube<face, Size::X4, originalSize != Size::X4>(chunk, NodePos{Size::X4, bpos}, callback);
                return;
            }
        case Size::X8:
            if (!chunk.isEmpty<Size::X16>(bpos.x, bpos.y, bpos.z)) {
                forEachNeighborInCube<face, Size::X8, originalSize != Size::X8>(chunk, NodePos{Size::X8, bpos}, callback);
                return;
            }
        case Size::X16:
            forEachNeighborInCube<face, Size::X16, originalSize != Size::X16>(chunk, NodePos{Size::X16, bpos}, callback);
    }
}


template<Face face>
void growThenIterateOuter(const Chunk& chunk, const NodePos& pos, auto& callback) {
    #define CASE(sz) case sz: growThenIterateInner<face, sz>(chunk, pos, callback); return;
    switch (pos.size) {
        CASE(Size::X1)
        CASE(Size::X2)
        CASE(Size::X4)
        CASE(Size::X8)
        CASE(Size::X16)
    }
    #undef CASE
}


void forEachNeighbor(ChunkProvider chunks, const NodePos& pos, auto callback) {
    const auto size = pos.size;
    const auto bpos = pos.absolutePosZero();

    const ChunkPos cpos = bpos.toChunkPos();
    const Chunk& currentChunk = getOrGenChunk(chunks.cache, cpos, chunks.generator);

    [&]<size_t... I>(std::index_sequence<I...>) {
        ([&] {
            constexpr Face face = ALL_FACES[I];
            const NodePos neighborNodePos {size, bpos.offset(face, getSize(size))};
            const BlockPos origin = neighborNodePos.absolutePosZero();
            if constexpr (face == Face::UP || face == Face::DOWN) {
                if (!isInBounds(origin)) return;
            }
            const ChunkPos neighborCpos = origin.toChunkPos();
            const Chunk& chunk = neighborCpos == cpos
                ? currentChunk : getOrGenChunk(chunks.cache, neighborCpos, chunks.generator);

            growThenIterateOuter<face>(chunk, neighborNodePos, callback);
        }(), ...);
    }(std::make_index_sequence<ALL_FACES.size()>{});
}


constexpr double MIN_DIST_PATH = 5; // might want to increase this

std::optional<Path3D> bestPathSoFar(map_t<NodePos, std::unique_ptr<PathNode3D>>& map, const PathNode3D* start, const PathNode3D* end, const BlockPos& startPos, const BlockPos& goal) {
    const double distSq = startPos.distanceToSq(end->pos.absolutePosCenter());

    if (distSq > MIN_DIST_PATH * MIN_DIST_PATH) {
        return createPath(map, start, end, startPos, goal, PathTypeEnum::SEGMENT);
    } else {
        std::cout << "Path took too long and got nowhere\n";
        auto [x, y, z] = end->pos.absolutePosCenter();
        std::cout << "(Path ended at {" << x << ", " << y << ", " << z << "})\n";
        return std::nullopt;
    }

    // assume there's always a way
    //return createPath(map, start, end, startPos, goal, Path::Type::SEGMENT);
}

bool inGoal(const NodePos& node, const BlockPos& goal) {
    // TODO: test if goal is in cube
    return node.absolutePosCenter().distanceToSq(goal) <= 16 * 16;
}

std::optional<Path3D> findPath0(const BlockPos& start, const BlockPos& goal, ChunkGeneratorHell& gen) {
    std::cout << "distance = " << start.distanceTo(goal) << '\n';

    map_t<ChunkPos, std::unique_ptr<Chunk>> chunkCache;
    map_t<NodePos, std::unique_ptr<PathNode3D>> map;
    BinaryHeapOpenSet openSet;

    PathNode3D* const startNode = getNodeAtPosition(map, NodePos{Size::X1, start}, goal);
    startNode->cost = 0;
    startNode->combinedCost = startNode->estimatedCostToGoal;
    openSet.insert(startNode);

    PathNode3D* bestSoFar = startNode;
    double bestHeuristicSoFar = startNode->estimatedCostToGoal;

    using namespace std::chrono_literals;
    const auto startTime = std::chrono::system_clock::now();
    const auto primaryTimeoutTime = startTime + 500ms;
    const auto failureTimeout = startTime + 30s;

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

        auto* currentNode = static_cast<PathNode3D*>(openSet.removeLowest());

        // TODO: get the right sub cube
        if (inGoal(currentNode->pos, goal)) {
            std::cout << "chunkCache size = " << chunkCache.size() << '\n';
            std::cout << "openSet size = " << openSet.getSize() << '\n';
            std::cout << "map size = " << map.size() << '\n';
            std::cout << '\n';
            return createPath(map, startNode, currentNode, start, goal, PathTypeEnum::FINISHED);
        }

        forEachNeighbor({chunkCache, gen}, currentNode->pos, [&](const NodePos& neighborPos) {
            PathNode3D* neighborNode = getNodeAtPosition(map, neighborPos, goal);
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

                const double heuristic = neighborNode->combinedCost;
                if (bestHeuristicSoFar - heuristic > MIN_IMPROVEMENT) {
                    bestHeuristicSoFar = heuristic;
                    bestSoFar = neighborNode;

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
