#include "PathFinder.h"
#include "BinaryHeapOpenSet.h"
#include "ChunkGen.h"

#include <memory>
#include <array>
#include <iostream>
#include <algorithm>
#include <functional>
#include <queue>
#include <unordered_set>

constexpr bool VERBOSE = false;

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


Path createPath(map_t<NodePos, std::unique_ptr<PathNode>>& map, const PathNode* start, const PathNode* end,
                const BlockPos& startPos, const BlockPos& goal, Path::Type pathType) {
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
    auto nodes = decltype(tempNodes){};
    nodes.reserve(tempNodes.size());
    std::move(tempNodes.rbegin(), tempNodes.rend(), std::back_inserter(nodes));

    auto path = decltype(tempPath){};
    path.reserve(tempPath.size());
    std::move(tempPath.rbegin(), tempPath.rend(), std::back_inserter(path));

    return Path{
            pathType,
            startPos,
            goal,
            std::move(path),
            std::move(nodes)
    };
}

const Chunk& getOrGenChunk(Context& ctx, ChunkGenExec& executor, const ChunkPos& pos, bool airIfFake) {
    ctx.cacheMutex.lock();
    auto it = ctx.chunkCache.find(pos);
    if (it != ctx.chunkCache.end()) {
        auto& chunk = *it->second;
        ctx.cacheMutex.unlock();
        if (!chunk.isFromJava && airIfFake) {
            return AIR_CHUNK;
        }
        return *it->second;
    } else if (airIfFake) {
        ctx.cacheMutex.unlock();
        return AIR_CHUNK;
    } else {
        ctx.cacheMutex.unlock();
        std::unique_ptr ptr = std::make_unique<Chunk>();
        auto& chunk = *ptr;
        ctx.generator.generateChunk(pos.x, pos.z, *ptr, executor);
        ctx.cacheMutex.lock();
        ctx.chunkCache.emplace(pos, std::move(ptr));
        ctx.cacheMutex.unlock();
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
    constexpr auto size = width(sz);
    switch (face) {
        case Face::UP: {
            const BlockPos& corner = origin;
            const BlockPos oppositeCorner = origin.east(size).south(size);
            return {corner, corner.east(size), corner.south(size), oppositeCorner};
        }
        case Face::DOWN: {
            const BlockPos corner = origin.up(size);
            const BlockPos oppositeCorner = corner.east(size).south(size);
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
template<Face face, Size size, bool sizeChange, Size minSize>
void forEachNeighborInCube(const Chunk& chunk, const NodePos& neighborNode, auto& callback) {
    if constexpr (sizeChange) {
        callback(neighborNode);
        return;
    }
    const auto pos = neighborNode.absolutePosZero();
    const auto chunkLocal = pos.toChunkLocal();
    if (chunk.isEmpty<size>(chunkLocal.x, chunkLocal.y, chunkLocal.z)) {
        callback(neighborNode);
        return;
    }
    if constexpr (size != Size::X1) {
        constexpr auto nextSize = static_cast<Size>(static_cast<int>(size) - 1);
        // Don't shrink cubes to X1 because they suck and make the path try to squeeze through small areas
        // TODO: allow this to be configurable?
        if constexpr (nextSize < minSize) return;
        const std::array subCubes = neighborCubes<face, nextSize>(pos);
        for (const BlockPos& subCube : subCubes) {
            forEachNeighborInCube<face, nextSize, false, minSize>(chunk, NodePos{nextSize, subCube}, callback);
        }
    }
}


template<Face face, Size originalSize, Size minSize>
void growThenIterateInner(const Chunk& chunk, const NodePos& pos, auto& callback) {
    const auto bpos = pos.absolutePosZero();
    const auto chunkLocal = bpos.toChunkLocal();

    switch (originalSize) {
        case Size::X1:
            if (!chunk.isEmpty<Size::X2>(chunkLocal.x, chunkLocal.y, chunkLocal.z)) {
                forEachNeighborInCube<face, Size::X1, originalSize != Size::X1, minSize>(chunk, pos, callback);
                return;
            }
            [[fallthrough]];
        case Size::X2:
            if (!chunk.isEmpty<Size::X4>(chunkLocal.x, chunkLocal.y, chunkLocal.z)) {
                forEachNeighborInCube<face, Size::X2, originalSize != Size::X2, minSize>(chunk, NodePos{Size::X2, bpos},callback);
                return;
            }
            [[fallthrough]];
        case Size::X4:
            if (!chunk.isEmpty<Size::X8>(chunkLocal.x, chunkLocal.y, chunkLocal.z)) {
                forEachNeighborInCube<face, Size::X4, originalSize != Size::X4, minSize>(chunk, NodePos{Size::X4, bpos},callback);
                return;
            }
            [[fallthrough]];
        case Size::X8:
            if (!chunk.isEmpty<Size::X16>(chunkLocal.x, chunkLocal.y, chunkLocal.z)) {
                forEachNeighborInCube<face, Size::X8, originalSize != Size::X8, minSize>(chunk, NodePos{Size::X8, bpos},callback);
                return;
            }
            [[fallthrough]];
        case Size::X16:
            forEachNeighborInCube<face, Size::X16, originalSize != Size::X16, minSize>(chunk, NodePos{Size::X16, bpos},callback);
    }
}


template<Face face, Size minSize>
void growThenIterateOuter(const Chunk& chunk, const NodePos& pos, auto& callback) {
#define CASE(sz) case sz: growThenIterateInner<face, sz, minSize>(chunk, pos, callback); return;
    switch (pos.size) {
        CASE(Size::X1)
        CASE(Size::X2)
        CASE(Size::X4)
        CASE(Size::X8)
        CASE(Size::X16)
    }
#undef CASE
}

bool isInBounds(const BlockPos& pos) {
    return pos.y >= 0 && pos.y < 128;
}

constexpr double MIN_DIST_PATH = 5; // might want to increase this

std::optional<Path>
bestPathSoFar(map_t<NodePos, std::unique_ptr<PathNode>>& map, const PathNode* start, const PathNode* end,
              const BlockPos& startPos, const BlockPos& goal) {
    const double distSq = startPos.distanceToSq(end->pos.absolutePosCenter());

    if (distSq > MIN_DIST_PATH * MIN_DIST_PATH) {
        return createPath(map, start, end, startPos, goal, Path::Type::SEGMENT);
    } else {
        if (VERBOSE) {
            std::cout << "Path took too long and got nowhere\n";
            auto[x, y, z] = end->pos.absolutePosCenter();
            std::cout << "(Path ended at {" << x << ", " << y << ", " << z << "})\n";
        }
        return std::nullopt;
    }

    // assume there's always a way
    //return createPath(map, start, end, startPos, goal, Path::Type::SEGMENT);
}

bool closeToGoal(const NodePos& node, const BlockPos& goal) {
    return node.absolutePosCenter().distanceToSq(goal) <= 16 * 16;
}

bool inGoal(const NodePos& node, const BlockPos& goal) {
    [[likely]] if (!closeToGoal(node, goal)) return false;
    auto c1 = node.absolutePosZero();
    auto c2 = c1 + width(node.size);
    return goal.x >= c1.x && goal.x <= c2.x &&
            goal.y >= c1.y && goal.y <= c2.y &&
            goal.z >= c1.z && goal.z <= c2.z;
}

std::atomic_flag cancelFlag;

std::optional<Path> findPathSegment(Context& ctx, const NodePos& start, const NodePos& goal, bool x4Min, int timeoutMs) {
    const auto goalCenter = goal.absolutePosCenter();
    const auto startCenter = start.absolutePosCenter();
    if (VERBOSE) std::cout << "distance = " << start.absolutePosCenter().distanceTo(goalCenter) << '\n';

    map_t<NodePos, std::unique_ptr<PathNode>> map;
    map_t<ChunkPos, bool> doneFull;
    BinaryHeapOpenSet openSet;

    PathNode* const startNode = getNodeAtPosition(map, start, goal.absolutePosZero());
    startNode->cost = 0;
    startNode->combinedCost = startNode->estimatedCostToGoal;
    openSet.insert(startNode);
    getOrGenChunk(ctx, ctx.executors[0], start.absolutePosZero().toChunkPos());

    PathNode* bestSoFar = startNode;
    double bestHeuristicSoFar = startNode->estimatedCostToGoal;

    using namespace std::chrono_literals;
    const auto startTime = std::chrono::system_clock::now();
    const auto primaryTimeoutTime = startTime + 500ms;
    const auto timeout = timeoutMs != 0 ? std::chrono::milliseconds{timeoutMs} : 30s;
    const auto failureTimeout = startTime + timeout;

    bool failing = true;
    int numNodes = 0;
    while (!openSet.isEmpty()) {
        constexpr int timeCheckInterval = 1 << 6;
        if ((numNodes & (timeCheckInterval - 1)) == 0) { // only call this once every 64 nodes
            auto now = std::chrono::system_clock::now();

            if (now >= failureTimeout || (!failing && now >= primaryTimeoutTime)) {
                break;
            } else if (cancelFlag.test()) {
                return {};
            }
        }

        PathNode* currentNode = openSet.removeLowest();

        if (inGoal(currentNode->pos, goal.absolutePosCenter())) {
            if (VERBOSE) {
                std::cout << "chunkCache size = " << ctx.chunkCache.size() << '\n';
                std::cout << "openSet size = " << openSet.getSize() << '\n';
                std::cout << "map size = " << map.size() << '\n';
                std::cout << '\n';
            }
            return createPath(map, startNode, currentNode, startCenter, goalCenter, Path::Type::FINISHED);
        }
        const auto pos = currentNode->pos;
        const auto size = pos.size;
        const auto bpos = pos.absolutePosZero();
        const ChunkPos cpos = bpos.toChunkPos();
        const Chunk& currentChunk = *ctx.chunkCache[cpos];
        const ChunkPos cposNorth = bpos.north(16).toChunkPos();
        const ChunkPos cposSouth = bpos.south(16).toChunkPos();
        const ChunkPos cposEast = bpos.east(16).toChunkPos();
        const ChunkPos cposWest = bpos.west(16).toChunkPos();
        if (!doneFull.contains(cpos)) {
            ctx.topExecutor.compute(
                    [&] {
                        return getOrGenChunk(ctx, ctx.executors[0], cposNorth);
                    },
                    [&] {
                        return getOrGenChunk(ctx, ctx.executors[1], cposSouth);
                    },
                    [&] {
                        return getOrGenChunk(ctx, ctx.executors[2], cposEast);
                    },
                    [&] {
                        return getOrGenChunk(ctx, ctx.executors[3], cposWest);
                    }
            );
            doneFull.emplace(cpos, true);
        }

        auto callback = [&](const NodePos& neighborPos) {
            PathNode* neighborNode = getNodeAtPosition(map, neighborPos, goalCenter);
            //auto sqrtSize = [](Size sz) { return sqrt(width(sz)); };
            const double cost = 1;//sqrtSize(neighborNode->pos.size);//width(neighborNode->pos.size);
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

                    if (failing &&
                            startCenter.distanceToSq(neighborPos.absolutePosCenter()) > MIN_DIST_PATH * MIN_DIST_PATH) {
                        failing = false;
                    }
                }
            }
        };

        [&]<size_t... I>(std::index_sequence<I...>) {
            ([&] {
                constexpr Face face = ALL_FACES[I];
                const NodePos neighborNodePos{size, bpos.offset(face, width(size))};
                const BlockPos origin = neighborNodePos.absolutePosZero();
                if constexpr (face == Face::UP || face == Face::DOWN) {
                    if (!isInBounds(origin)) return;
                }
                const ChunkPos neighborCpos = origin.toChunkPos();
                const Chunk& chunk =
                        face == Face::UP || face == Face::DOWN ? currentChunk :
                        face == Face::NORTH ? neighborCpos == cpos ? currentChunk : *ctx.chunkCache[cposNorth] :
                        face == Face::SOUTH ? neighborCpos == cpos ? currentChunk : *ctx.chunkCache[cposSouth] :
                        face == Face::EAST ? neighborCpos == cpos ? currentChunk : *ctx.chunkCache[cposEast] :
                        /* face == Face::WEST */ neighborCpos == cpos ? currentChunk : *ctx.chunkCache[cposWest];

                // 1x only
                if (/*fine*/ false) {
                    if (!chunk.isSolid(neighborNodePos.absolutePosZero())) {
                        callback(neighborNodePos);
                    }
                } else {
                    if (x4Min) {
                        growThenIterateOuter<face, Size::X4>(chunk, neighborNodePos, callback);
                    } else {
                        growThenIterateOuter<face, Size::X2>(chunk, neighborNodePos, callback);
                    }
                }
            }(), ...);
        }(std::make_index_sequence<ALL_FACES.size()>{});
    }

    auto[x, y, z] = bestSoFar->pos.absolutePosCenter();
    if (VERBOSE) {
        std::cout << "Best position = {" << x << ", " << y << ", " << z << "}\n";
        std::cout << "failing = " << failing << '\n';
        std::cout << "Open set width: " << openSet.getSize() << '\n';
        std::cout << "PathNode map size: " << map.size() << '\n';
        std::cout << "chunk cache size: " << ctx.chunkCache.size() << '\n';
        std::cout << '\n';
    }

    return bestPathSoFar(map, startNode, bestSoFar, startCenter, goalCenter);
}

const Chunk& getChunkNoMutex(const BlockPos& pos, const ChunkGeneratorHell& gen, ChunkGenExec& exec, map_t<ChunkPos, std::unique_ptr<Chunk>>& cache) {
    const ChunkPos chunkPos = pos.toChunkPos();
    auto it = cache.find(chunkPos);
    if (it != cache.end()) {
        return *it->second;
    } else {
        std::unique_ptr ptr = std::make_unique<Chunk>();
        auto& chunk = *ptr;
        gen.generateChunk(chunkPos.x, chunkPos.z, *ptr, exec);
        cache.emplace(chunkPos, std::move(ptr));
        return chunk;
    }
}

template<Size size>
NodePos findAir(Context& ctx, const BlockPos& start1x) {
    auto start = NodePos{size, start1x};
    auto queue = std::queue<NodePos>{};
    auto visited = std::unordered_set<NodePos>{};
    queue.push(start);
    visited.insert(start);
    if (!isInBounds(start1x)) goto retard;

    while (!queue.empty()) {
        const NodePos node = queue.front();
        const auto blockPos = node.absolutePosZero();
        queue.pop();
        if (isInBounds(node.absolutePosZero())) {
            const auto& chunk = getChunkNoMutex(blockPos, ctx.generator, ctx.executors[0], ctx.chunkCache);
            if (chunk.isEmpty<size>(blockPos.x & 15, blockPos.y, blockPos.z & 15)) {
                return node;
            }
            auto push = [&](NodePos pos) {
                if (visited.emplace(pos).second) queue.push(pos);
            };
            constexpr auto w = width(size);
            push(NodePos{size, blockPos.west(w)});
            push(NodePos{size, blockPos.east(w)});
            push(NodePos{size, blockPos.north(w)});
            push(NodePos{size, blockPos.south(w)});
            push(NodePos{size, blockPos.up(w)});
            push(NodePos{size, blockPos.down(w)});
        }
    }
    // shouldn't be possible to exit the while loop
    retard:
    std::cerr << "retard" << std::endl;
    exit(1);
}

template NodePos findAir<Size::X2>(Context& ctx, const BlockPos& start1x);
template NodePos findAir<Size::X4>(Context& ctx, const BlockPos& start1x);


void appendPath(Path& path, Path&& segment) {
    path.blocks.insert(path.blocks.end(), segment.blocks.begin(), segment.blocks.end());
    path.nodes.insert(path.nodes.end(), std::move_iterator{segment.nodes.begin()},
                      std::move_iterator{segment.nodes.end()});
}

Path splicePaths(std::vector<Path>&& paths) {
    Path path = std::move(paths.at(0));

    std::for_each(paths.begin() + 1, paths.end(), [&path](Path& segment) {
        appendPath(path, std::move(segment));
    });

    return path;
}

std::optional<Path> findPathFull(Context& ctx, const BlockPos& start, const BlockPos& goal) {
    if (!isInBounds(start)) throw "troll";

    ParallelExecutor<4> topExecutor;
    std::array<ChunkGenExec, 4> executors;
    std::vector<Path> segments;

    // we can't pathfind through solid blocks
    const auto realStart = findAir<Size::X2>(ctx, start);
    const auto realGoal = findAir<Size::X2>(ctx, goal);

    while (true) {
        const NodePos lastPathEnd = !segments.empty() ? NodePos{Size::X2, segments.back().getEndPos()} : realStart;
        std::optional path = findPathSegment(ctx, lastPathEnd, realGoal, false, 0);
        if (!path.has_value()) {
            if (cancelFlag.test()) {
                cancelFlag.clear();
                return std::nullopt;
            } else {
                break;
            }
        } else {
            const bool finished = path->type == Path::Type::FINISHED;
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
