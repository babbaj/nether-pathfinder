#include "PathFinder.h"
#include "BinaryHeapOpenSet.h"

#include "absl/container/flat_hash_map.h"

using namespace impl;

// never returns null
PathNode2D* getNodeAtPosition(map_t<Pos2D, std::unique_ptr<PathNode2D>>& map, const Pos2D& pos, const Pos2D& goal) {
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

Path2D createPath(map_t<Pos2D, std::unique_ptr<PathNode2D>>& map, const PathNode2D* start, const PathNode2D* end, const Pos2D& startPos, const Pos2D& goal, PathTypeEnum pathType) {
    std::vector<std::unique_ptr<PathNode2D>> tempNodes;
    std::vector<Pos2D> tempPath;

    const PathNode2D* current = end;
    while (current) {
        auto& uniqueptr = map.at(current->pos);
        tempNodes.push_back(std::move(uniqueptr));
        tempPath.push_back(current->pos);
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

std::optional<Path2D> bestPathSoFar(map_t<Pos2D, std::unique_ptr<PathNode2D>>& map, const PathNode2D* start, const PathNode2D* end, const Pos2D& startPos, const Pos2D& goal) {
    const double distSq = startPos.distanceToSq(end->pos);

    if (distSq > MIN_DIST_PATH * MIN_DIST_PATH) {
        return createPath(map, start, end, startPos, goal, PathTypeEnum::SEGMENT);
    } else {
        std::cout << "Path took too long and got nowhere\n";
        auto [x, z] = end->pos;
        std::cout << "(Path ended at {" << x << ", " << z << "})\n";
        return std::nullopt;
    }

    // assume there's always a way
    //return createPath(map, start, end, startPos, goal, Path::Type::SEGMENT);
}

struct ChunkProvider {
    map_t<ChunkPos, std::unique_ptr<Chunk>>& cache;
    ChunkGeneratorHell& generator;
};

void forEachNeighbor(ChunkProvider chunks, const Pos2D& pos, auto callback) {
    const ChunkPos cpos = pos.toChunkPos();
    const Chunk& currentChunk = getOrGenChunk(chunks.cache, cpos, chunks.generator);

    [&]<size_t... I>(std::index_sequence<I...>) {
        ([&] {
            constexpr Face face = HORIZONAL_FACES[I];
            const Pos2D neighbor = pos.offset(face);

            const ChunkPos neighborCpos = neighbor.toChunkPos();
            const Chunk& chunk = neighborCpos == cpos
                                 ? currentChunk : getOrGenChunk(chunks.cache, neighborCpos, chunks.generator);
            if (!chunk.isSolid(neighbor.x, 33, neighbor.z)) {
                callback(neighbor);
                //std::cout << "Callback  {" << neighbor.x << ", " << neighbor.z << "}\n";
            } else {
                //std::cout << "Solid at {" << neighbor.x << ", " << neighbor.z << "}\n";
            }
        }(), ...);
    }(std::make_index_sequence<HORIZONAL_FACES.size()>{});
}


std::optional<Path2D> findPath0(const Pos2D& start, const Pos2D& goal, ChunkGeneratorHell& gen) {
    std::cout << "distance = " << start.distanceTo(goal) << '\n';

    map_t<ChunkPos, std::unique_ptr<Chunk>> chunkCache;
    map_t<Pos2D, std::unique_ptr<PathNode2D>> map;
    BinaryHeapOpenSet openSet;

    PathNode2D* const startNode = getNodeAtPosition(map, start, goal);
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
            }
        }

        auto* currentNode = static_cast<PathNode2D*>(openSet.removeLowest());
        numNodes++;
        std::cout << "yay {" << currentNode->pos.x << ", " << currentNode->pos.z << "}\n";

        // TODO: get the right sub cube
        if (currentNode->pos == goal) {
            std::cout << "chunkCache size = " << chunkCache.size() << '\n';
            std::cout << "openSet size = " << openSet.getSize() << '\n';
            std::cout << "map size = " << map.size() << '\n';
            std::cout << '\n';
            return createPath(map, startNode, currentNode, start, goal, PathTypeEnum::FINISHED);
        }

        forEachNeighbor({chunkCache, gen}, currentNode->pos, [&](const Pos2D& neighborPos) {
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

                    if (failing && start.distanceToSq(neighborPos) > MIN_DIST_PATH * MIN_DIST_PATH) {
                        failing = false;
                    }
                }
            }
        });
    }

    auto [x, z] = bestSoFar->pos;
    std::cout << "Best position = {" << x << ", " << z << "}\n";
    std::cout << "failing = " << failing << '\n';
    std::cout << "Open set getSize: " << openSet.getSize() << '\n';
    std::cout << "PathNode map size: " << map.size() << '\n';
    std::cout << "chunk cache size: " << chunkCache.size() << '\n';
    std::cout << '\n';

    return bestPathSoFar(map, startNode, bestSoFar, start, goal);
}