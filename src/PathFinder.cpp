#include "PathFinder.h"
#include "BinaryHeapOpenSet.h"
#include "ChunkGen.h"
#include "baritone.h"

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

const Chunk& getRealChunkFromCacheOrFakeChunkMaybeGen(Context& ctx, ChunkGenExec& executor, const ChunkPos& pos, FakeChunkMode mode) {
    if (mode == FakeChunkMode::GENERATE) {
        return getOrGenChunk(ctx, executor, pos);
    }
    return getRealChunkOrDefault(ctx, pos, mode == FakeChunkMode::SOLID);
}

const Chunk& getOrGenChunk(Context& ctx, ChunkGenExec& executor, const ChunkPos& pos) {
    ctx.cacheMutex.lock();
    auto it = ctx.chunkCache.find(pos);
    if (it != ctx.chunkCache.end()) {
        Chunk* chunk = it->second.second;
        ctx.cacheMutex.unlock();
        return *chunk;
    } else {
        Chunk* chunk = ctx.chunkAllocator->allocate();
        ctx.cacheMutex.unlock();
        ctx.generator.generateChunk(pos.x, pos.z, *chunk, executor);
        ctx.cacheMutex.lock();
        ctx.chunkCache.emplace(pos, std::pair{ChunkState::FAKE, chunk});
        ctx.cacheMutex.unlock();
        return *chunk;
    }
}

const Chunk& getRealChunkOrDefault(Context& ctx, const ChunkPos& pos, bool solid) {
    auto it = ctx.chunkCache.find(pos);
    if (it != ctx.chunkCache.end()) {
        auto& [state, chunk] = it->second;
        if (state != ChunkState::FROM_JAVA) return solid ? SOLID_CHUNK : AIR_CHUNK;
        return *chunk;
    } else {
        return solid ? SOLID_CHUNK : AIR_CHUNK;
    }
}

std::pair<ChunkState, const Chunk&> getChunkOrAir(Context& ctx, const ChunkPos& pos) {
    auto it = ctx.chunkCache.find(pos);
    if (it != ctx.chunkCache.end()) {
        auto& [state, chunk] = it->second;
        return {state, *chunk};
    } else {
        return {ChunkState::FAKE, AIR_CHUNK};
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
void forEachNeighborInCube(const Chunk& chunk, ChunkState state, const NodePos& neighborNode, auto& callback) {
    if constexpr (sizeChange) {
        callback(neighborNode, chunk, state);
        return;
    }
    const auto pos = neighborNode.absolutePosZero();
    const auto chunkLocal = pos.toChunkLocal();
    if (chunk.isEmpty<size>(chunkLocal.x, chunkLocal.y, chunkLocal.z)) {
        callback(neighborNode, chunk, state);
        return;
    }
    if constexpr (size != Size::X1) {
        constexpr auto nextSize = static_cast<Size>(static_cast<int>(size) - 1);
        // Don't shrink cubes to X1 because they suck and make the path try to squeeze through small areas
        // TODO: allow this to be configurable?
        if constexpr (nextSize < minSize) return;
        const std::array subCubes = neighborCubes<face, nextSize>(pos);
        for (const BlockPos& subCube : subCubes) {
            forEachNeighborInCube<face, nextSize, false, minSize>(chunk, state, NodePos{nextSize, subCube}, callback);
        }
    }
}


template<Face face, Size originalSize, Size minSize>
void growThenIterateInner(const Chunk& chunk, ChunkState state, const NodePos& pos, auto& callback) {
    const auto bpos = pos.absolutePosZero();
    const auto chunkLocal = bpos.toChunkLocal();

    switch (originalSize) {
        case Size::X1:
            if (!chunk.isEmpty<Size::X2>(chunkLocal.x, chunkLocal.y, chunkLocal.z)) {
                forEachNeighborInCube<face, Size::X1, originalSize != Size::X1, minSize>(chunk, state, pos, callback);
                return;
            }
            [[fallthrough]];
        case Size::X2:
            if (!chunk.isEmpty<Size::X4>(chunkLocal.x, chunkLocal.y, chunkLocal.z)) {
                forEachNeighborInCube<face, Size::X2, originalSize != Size::X2, minSize>(chunk, state, NodePos{Size::X2, bpos},callback);
                return;
            }
            [[fallthrough]];
        case Size::X4:
            if (!chunk.isEmpty<Size::X8>(chunkLocal.x, chunkLocal.y, chunkLocal.z)) {
                forEachNeighborInCube<face, Size::X4, originalSize != Size::X4, minSize>(chunk, state, NodePos{Size::X4, bpos},callback);
                return;
            }
            [[fallthrough]];
        case Size::X8:
            if (!chunk.isEmpty<Size::X16>(chunkLocal.x, chunkLocal.y, chunkLocal.z)) {
                forEachNeighborInCube<face, Size::X8, originalSize != Size::X8, minSize>(chunk, state, NodePos{Size::X8, bpos},callback);
                return;
            }
            [[fallthrough]];
        case Size::X16:
            forEachNeighborInCube<face, Size::X16, originalSize != Size::X16, minSize>(chunk, state, NodePos{Size::X16, bpos},callback);
    }
}


template<Face face, Size minSize>
void growThenIterateOuter(const Chunk& chunk, ChunkState state, const NodePos& pos, auto& callback) {
#define CASE(sz) case sz: growThenIterateInner<face, sz, minSize>(chunk, state, pos, callback); return;
    switch (pos.size) {
        CASE(Size::X1)
        CASE(Size::X2)
        CASE(Size::X4)
        CASE(Size::X8)
        CASE(Size::X16)
    }
#undef CASE
}

bool isInBounds(int worldHeight, const BlockPos& pos) {
    return pos.y >= 0 && pos.y < worldHeight;
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


std::chrono::milliseconds tryLoadRegionNative(Context& ctx, ChunkPos pos) {
    auto regionPos = RegionPos{pos.x >> 5, pos.z >> 5};
    if (ctx.baritoneCache.has_value() && ctx.checkedRegions.insert(regionPos).second) {
        // only measure when actually reading a file because there might be overhead
        auto t1 = std::chrono::steady_clock::now();
        auto file = openRegionFile(ctx.baritoneCache.value(), regionPos);
        if (!file) {
            return {};
        }

        auto [data, dim] = file.value();
        parseBaritoneRegion(*ctx.chunkAllocator, ctx.chunkCache, regionPos, data, dim);

        auto t2 = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
        return duration;
        //std::cout << "Loading region took " << duration.count() << "ms" << std::endl;
    }
    return {};
}

std::atomic_flag cancelFlag;

std::optional<Path> findPathSegment(Context& ctx, const NodePos& start, const NodePos& goal, bool x4Min, int timeoutMs, bool airIfFake, double fakeChunkCost) {
    const auto fakeChunkMode = airIfFake ? FakeChunkMode::AIR : FakeChunkMode::GENERATE;
    const auto goalCenter = goal.absolutePosCenter();
    const auto startCenter = start.absolutePosCenter();
    if (VERBOSE) std::cout << "distance = " << start.absolutePosCenter().distanceTo(goalCenter) << '\n';

    map_t<NodePos, std::unique_ptr<PathNode>> map;
    map_t<ChunkPos, bool> doneFull;
    BinaryHeapOpenSet openSet;

    PathNode* const startNode = getNodeAtPosition(map, start, goal.absolutePosZero());
    tryLoadRegionNative(ctx, start.absolutePosZero().toChunkPos());
    startNode->cost = 0;
    startNode->combinedCost = startNode->estimatedCostToGoal;
    openSet.insert(startNode);
    getRealChunkFromCacheOrFakeChunkMaybeGen(ctx, ctx.executors[0], start.absolutePosZero().toChunkPos(), fakeChunkMode);

    PathNode* bestSoFar = startNode;
    double bestHeuristicSoFar = startNode->estimatedCostToGoal;

    using namespace std::chrono_literals;
    const auto startTime = std::chrono::system_clock::now();
    const auto primaryTimeoutTime = startTime + 500ms;
    const auto timeout = timeoutMs != 0 ? std::chrono::milliseconds{timeoutMs} : 30s;
    const auto failureTimeout = startTime + timeout;
    std::chrono::milliseconds timeDoingIO{};

    bool failing = true;
    int numNodes = 0;
    int fakeChunkVisits = 0; // if this gets too high we return
    while (!openSet.isEmpty()) {
        constexpr int timeCheckInterval = 1 << 6;
        if ((numNodes & (timeCheckInterval - 1)) == 0) { // only call this once every 64 nodes
            auto now = std::chrono::system_clock::now() - timeDoingIO;

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
        const std::pair currentChunk = getChunkOrAir(ctx, cpos);
        if (currentChunk.first != ChunkState::FROM_JAVA) {
            fakeChunkVisits++;
        } else {
            fakeChunkVisits = 0;
        }
        if (fakeChunkVisits >= 100 && airIfFake) {
            return bestPathSoFar(map, startNode, bestSoFar, startCenter, goalCenter);
        }
        const ChunkPos cposNorth = bpos.north(16).toChunkPos();
        const ChunkPos cposSouth = bpos.south(16).toChunkPos();
        const ChunkPos cposEast = bpos.east(16).toChunkPos();
        const ChunkPos cposWest = bpos.west(16).toChunkPos();
        if (!airIfFake && !doneFull.contains(cpos)) {
            ctx.topExecutor.compute(
                    [&] {
                        return getRealChunkFromCacheOrFakeChunkMaybeGen(ctx, ctx.executors[0], cposNorth, fakeChunkMode);
                    },
                    [&] {
                        return getRealChunkFromCacheOrFakeChunkMaybeGen(ctx, ctx.executors[1], cposSouth, fakeChunkMode);
                    },
                    [&] {
                        return getRealChunkFromCacheOrFakeChunkMaybeGen(ctx, ctx.executors[2], cposEast, fakeChunkMode);
                    },
                    [&] {
                        return getRealChunkFromCacheOrFakeChunkMaybeGen(ctx, ctx.executors[3], cposWest, fakeChunkMode);
                    }
            );
            doneFull.emplace(cpos, true);
        }

        auto callback = [&](const NodePos& neighborPos, const Chunk& chunk, ChunkState state) {
            PathNode* neighborNode = getNodeAtPosition(map, neighborPos, goalCenter);
            const double cost = state == ChunkState::FROM_JAVA ? 1 : fakeChunkCost;
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
                    if (!isInBounds(ctx.maxHeight, origin)) return;
                }
                const ChunkPos neighborCpos = origin.toChunkPos();
                timeDoingIO += tryLoadRegionNative(ctx, neighborCpos);
                const auto [state, chunk] =
                        face == Face::UP || face == Face::DOWN ? currentChunk :
                        face == Face::NORTH ? neighborCpos == cpos ? currentChunk : getChunkOrAir(ctx, cposNorth) :
                        face == Face::SOUTH ? neighborCpos == cpos ? currentChunk : getChunkOrAir(ctx, cposSouth) :
                        face == Face::EAST ? neighborCpos == cpos ? currentChunk : getChunkOrAir(ctx, cposEast) :
                        /* face == Face::WEST */ neighborCpos == cpos ? currentChunk : getChunkOrAir(ctx, cposWest);

                // 1x only
                if (/*fine*/ false) {
                    if (!chunk.isSolid(neighborNodePos.absolutePosZero())) {
                        callback(neighborNodePos, chunk, state);
                    }
                } else {
                    if (x4Min) {
                        growThenIterateOuter<face, Size::X4>(chunk, state, neighborNodePos, callback);
                    } else {
                        growThenIterateOuter<face, Size::X2>(chunk, state, neighborNodePos, callback);
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

// TODO: fix this lol
const Chunk& getChunkNoMutex(const BlockPos& pos, const ChunkGeneratorHell& gen, ChunkGenExec& exec, cache_t& cache, Allocator<Chunk>& allocator) {
    const ChunkPos chunkPos = pos.toChunkPos();
    auto it = cache.find(chunkPos);
    if (it != cache.end()) {
        return *it->second.second;
    } else {
        Chunk* ptr = allocator.allocate();
        auto& chunk = *ptr;
        gen.generateChunk(chunkPos.x, chunkPos.z, *ptr, exec);
        cache.emplace(chunkPos, std::pair{ChunkState::FAKE, ptr});
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
    if (!isInBounds(ctx.maxHeight, start1x)) goto retard;

    while (!queue.empty()) {
        const NodePos node = queue.front();
        const auto blockPos = node.absolutePosZero();
        queue.pop();
        if (isInBounds(ctx.maxHeight, node.absolutePosZero())) {
            const auto& chunk = getChunkNoMutex(blockPos, ctx.generator, ctx.executors[0], ctx.chunkCache, *ctx.chunkAllocator);
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

std::optional<Path> findPathFull(Context& ctx, const NodePos& start, const NodePos& goal, double fakeChunkCost) {
    if (!isInBounds(ctx.maxHeight, start.absolutePosCenter())) throw "troll";

    std::vector<Path> segments;

    while (true) {
        const NodePos lastPathEnd = !segments.empty() ? NodePos{Size::X2, segments.back().getEndPos()} : start;
        std::optional path = findPathSegment(ctx, lastPathEnd, goal, true, 0, false, fakeChunkCost);
        if (!path.has_value()) {
            if (cancelFlag.test()) {
                cancelFlag.clear();
                return std::nullopt;
            } else {
                break;
            }
        } else {
            const bool finished = path->type == Path::Type::FINISHED;
            auto endCpos = path->blocks.end()->toChunkPos();
            const auto distSqBlocks = (200 / 16) * (200 / 16);
            const auto distSq = distSqBlocks;
            std::erase_if(ctx.chunkCache, [&](const auto& item) {
                const auto cpos = item.first;
                bool out = cpos.distanceToSq({endCpos.x, endCpos.z}) > distSq;
                if (out) ctx.chunkAllocator->free(item.second.second);
                return out;
            });

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
