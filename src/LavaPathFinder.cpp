#include "PathFinder.h"
#include "BinaryHeapOpenSet.h"
#include "ChunkSlice.h"

#include "absl/container/flat_hash_map.h"
#include "absl/container/node_hash_map.h"

using namespace impl;

template<typename K, typename V>
using node_map = absl::node_hash_map<K, V>;
template<typename K, typename V>
using flat_map = absl::flat_hash_map<K, V>;

using ChunkMap = node_map<ChunkPos, ChunkSlice>;

struct ChunkProvider {
    ChunkMap& cache;
    ChunkGeneratorHell& generator;
};

using NodeP = NodePos<Pos2D>;

// never returns null
PathNode2D* getNodeAtPosition(flat_map<NodeP, std::unique_ptr<PathNode2D>>& map, const NodeP& pos, const Pos2D& goal) {
    auto iter = map.find(pos);
    if (iter == map.end()) {
        auto node = std::make_unique<PathNode2D>(pos, goal);
        auto it = map.emplace(pos, std::move(node));
        assert(it.second);
        return it.first->second.get();
    } else {
        return iter->second.get();
    }
}

// temporary
ChunkSlice sliceChunk(const Chunk& chunk, int y) {
    ChunkSlice out{};
    for (int x = 0; x < 16; x++) {
        for (int z = 0; z < 16; z++) {
            out.setBlock(x, z, chunk.isSolid(x, y, z));
        }
    }
    std::array<std::array<bool, 16>, 16> real{};
    std::array<std::array<bool, 16>, 16> slice{};
    for (int x = 0; x < 16; x++) {
        for (int z = 0; z < 16; z++) {
            //out.setBlock(x, z, chunk.isSolid(x, y, z));
            real[x][z] = !chunk.isSolid(x, y, z);
            slice[x][z] = out.x1Empty(x, z);
        }
    }
    bool equal = real == slice;

    return out;
}

ChunkSlice& getOrGenChunk(ChunkMap& cache, const ChunkPos& pos, ChunkGeneratorHell& generator) {
    auto it = cache.find(pos);
    if (it != cache.end()) {
        return it->second;
    } else {
        auto chunk = generator.generateChunk(pos.x, pos.z);
        auto slice = sliceChunk(chunk, 32);
        return cache.emplace(pos, slice).first->second;
    }
}

Path2D createPath(flat_map<NodeP, std::unique_ptr<PathNode2D>>& map, const PathNode2D* start, const PathNode2D* end, const Pos2D& startPos, const Pos2D& goal, PathTypeEnum pathType) {
    std::vector<std::unique_ptr<PathNode2D>> tempNodes;
    std::vector<Pos2D> tempPath;

    const PathNode2D* current = end;
    while (current) {
        auto& uniqueptr = map.at(current->pos);
        tempNodes.push_back(std::move(uniqueptr));
        tempPath.push_back(current->pos.absolutePosCenter());
        current = static_cast<PathNode2D*>(current->previous);
    }

    //auto nodes = decltype(tempNodes)(tempPath.rbegin(), tempPath.rend());
    auto nodes = decltype(tempNodes){}; nodes.reserve(tempNodes.size());
    std::move(tempNodes.rbegin(), tempNodes.rend(), std::back_inserter(nodes));

    auto path = decltype(tempPath){}; path.reserve(tempPath.size());
    std::move(tempPath.rbegin(), tempPath.rend(), std::back_inserter(path));

    return Path2D {
            pathType,
            startPos,
            goal,
            std::move(path),
            std::move(nodes)
    };
}

constexpr double MIN_DIST_PATH = 5; // might want to increase this

std::optional<Path2D> bestPathSoFar(flat_map<NodeP, std::unique_ptr<PathNode2D>>& map, const PathNode2D* start, const PathNode2D* end, const Pos2D& startPos, const Pos2D& goal) {
    const double distSq = startPos.distanceToSq(end->pos.absolutePosCenter());

    if (distSq > MIN_DIST_PATH * MIN_DIST_PATH) {
        return createPath(map, start, end, startPos, goal, PathTypeEnum::SEGMENT);
    } else {
        std::cout << "Path took too long and got nowhere\n";
        auto [x, z] = end->pos.absolutePosCenter();
        std::cout << "(Path ended at {" << x << ", " << z << "})\n";
        return std::nullopt;
    }

    // assume there's always a way
    //return createPath(map, start, end, startPos, goal, PathTypeEnum::FINISHED);
}

template<Face face, Size sz> requires (face != Face::UP && face != Face::DOWN)
std::array<Pos2D, 2> neighborSquares(const Pos2D& origin) {
    // origin leans towards north/west
    constexpr auto size = getSize(sz);
    switch (face) {
        case Face::NORTH: {
            const Pos2D corner = origin.south(size);
            const Pos2D oppositeCorner = corner.east(size);
            return {corner, oppositeCorner};
        }
        case Face::SOUTH: {
            const Pos2D& corner = origin;
            const Pos2D oppositeCorner = corner.east(size);
            return {corner, oppositeCorner};
        }
        case Face::EAST: {
            const Pos2D& corner = origin;
            const Pos2D oppositeCorner = corner.south(size);
            return {corner, oppositeCorner};
        }
        case Face::WEST: {
            const Pos2D corner = origin.east(size);
            const Pos2D oppositeCorner = corner.south(size);
            return {corner, oppositeCorner};
        }
    }
}

// face is relative to the original square
template<Face face, Size size, bool sizeChange>
void forEachNeighborInSquare(const ChunkSlice& chunk, const NodeP& neighborNode, auto& callback) {
    if constexpr (sizeChange) {
        callback(neighborNode);
        return;
    }
    const auto pos = neighborNode.absolutePosZero();
    if (chunk.isEmpty<size>(pos.x, pos.z)) {
        callback(neighborNode);
    } else {
        if constexpr (size != Size::X1) {
            constexpr auto nextSize = static_cast<Size>(static_cast<int>(size) - 1);

            const std::array subSquares = neighborSquares<face, nextSize>(pos);
            for (const Pos2D &subSquare : subSquares) {
                forEachNeighborInSquare<face, nextSize, false>(chunk, NodeP{nextSize, subSquare}, callback);
            }
        }
    }
}

template<Face face, Size originalSize>
void growThenIterateInner(const ChunkSlice& chunk, const NodeP& pos, auto& callback) {
    const auto pos2d = pos.absolutePosZero();

    switch (originalSize) {
        case Size::X1:
            if (!chunk.isEmpty<Size::X2>(pos2d.x, pos2d.z)) {
                forEachNeighborInSquare<face, Size::X1, originalSize != Size::X1>(chunk, pos, callback);
                return;
            }
        case Size::X2:
            if (!chunk.isEmpty<Size::X4>(pos2d.x, pos2d.z)) {
                forEachNeighborInSquare<face, Size::X2, originalSize != Size::X2>(chunk, NodeP{Size::X2, pos2d}, callback);
                return;
            }
        case Size::X4:
            if (!chunk.isEmpty<Size::X8>(pos2d.x, pos2d.z)) {
                forEachNeighborInSquare<face, Size::X4, originalSize != Size::X4>(chunk, NodeP{Size::X4, pos2d}, callback);
                return;
            }
        case Size::X8:
            if (!chunk.isEmpty<Size::X16>(pos2d.x, pos2d.z)) {
                forEachNeighborInSquare<face, Size::X8, originalSize != Size::X8>(chunk, NodeP{Size::X8, pos2d}, callback);
                return;
            }
        case Size::X16:
            forEachNeighborInSquare<face, Size::X16, originalSize != Size::X16>(chunk, NodeP{Size::X16, pos2d}, callback);
    }
}

template<Face face>
void growThenIterateOuter(const ChunkSlice& chunk, const NodeP& pos, auto& callback) {
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

void forEachNeighbor(ChunkProvider chunks, const NodeP& pos, auto callback) {
    const Size size = pos.size;
    const Pos2D pos2d = pos.absolutePosZero();

    const ChunkPos cpos = pos2d.toChunkPos();
    const ChunkSlice& currentChunk = getOrGenChunk(chunks.cache, cpos, chunks.generator);

    [&]<size_t... I>(std::index_sequence<I...>) {
        ([&] {
            constexpr Face face = HORIZONTAL_FACES[I];
            const NodeP neighborNode {size, pos2d.offset(face, getSize(size))};
            const Pos2D origin = neighborNode.absolutePosZero();

            const ChunkPos neighborCpos = origin.toChunkPos();
            const ChunkSlice& chunk = neighborCpos == cpos
                                 ? currentChunk : getOrGenChunk(chunks.cache, neighborCpos, chunks.generator);

            growThenIterateOuter<face>(chunk, neighborNode, callback);
        }(), ...);
    }(std::make_index_sequence<HORIZONTAL_FACES.size()>{});
}

bool inGoal(const NodeP& node, const Pos2D& goal) {
    // TODO: test if goal is in cube
    return node.absolutePosCenter().distanceToSq(goal) <= 16 * 16;
}

std::optional<Path2D> findPath0(const Pos2D& start, const Pos2D& goal, ChunkGeneratorHell& gen) {
    std::cout << "distance = " << start.distanceTo(goal) << '\n';

    ChunkMap chunkCache;
    flat_map<NodeP, std::unique_ptr<PathNode2D>> map;
    BinaryHeapOpenSet openSet;

    PathNode2D* const startNode = getNodeAtPosition(map, NodeP{Size::X1, start}, goal);
    startNode->cost = 0;
    startNode->combinedCost = startNode->estimatedCostToGoal;
    openSet.insert(startNode);

    PathNode2D* bestSoFar = startNode;
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
                std::cout << "break;\n";
            }
        }

        auto* currentNode = static_cast<PathNode2D*>(openSet.removeLowest());
        numNodes++;
        //std::cout << "yay {" << currentNode->pos.x << ", " << currentNode->pos.z << "}\n";

        if (inGoal(currentNode->pos, goal)) {
            std::cout << "chunkCache size = " << chunkCache.size() << '\n';
            std::cout << "openSet size = " << openSet.getSize() << '\n';
            std::cout << "map size = " << map.size() << '\n';
            std::cout << '\n';
            return createPath(map, startNode, currentNode, start, goal, PathTypeEnum::FINISHED);
        }

        forEachNeighbor({chunkCache, gen}, currentNode->pos, [&](const NodeP& neighborPos) {
            PathNode2D* neighborNode = getNodeAtPosition(map, neighborPos, goal);
            constexpr double cost = 1;
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

    auto [x, z] = bestSoFar->pos.absolutePosCenter();
    std::cout << "Best position = {" << x << ", " << z << "}\n";
    std::cout << "failing = " << failing << '\n';
    std::cout << "Open set getSize: " << openSet.getSize() << '\n';
    std::cout << "PathNode map size: " << map.size() << '\n';
    std::cout << "chunk cache size: " << chunkCache.size() << '\n';
    std::cout << '\n';

    return bestPathSoFar(map, startNode, bestSoFar, start, goal);
}