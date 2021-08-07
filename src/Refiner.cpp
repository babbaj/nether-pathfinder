#include "Refiner.h"
#include "BinaryHeapOpenSet.h"

#include <unordered_map>
#include <memory>
#include <array>
#include <iostream>
#include <algorithm>
#include <functional>

#include "absl/container/flat_hash_map.h"
#include "absl/container/node_hash_map.h"


template<typename K, typename V>
using map_t = absl::flat_hash_map<K, V>;


PathOutcome findPath0(const BlockPos& start, const BlockPos& goal, const ChunkGeneratorHell& gen) {

    map_t<ChunkPos, std::unique_ptr<Chunk>> chunkCache;
    map_t<NodePos, std::unique_ptr<PathNode>> map;
    BinaryHeapOpenSet openSet;

    PathNode* const startNode = getNodeAtPosition(map, NodePos{Size::X1, start}, goal);
    startNode->cost = 0;
    startNode->combinedCost = startNode->estimatedCostToGoal;
    openSet.insert(startNode);

    while (!openSet.isEmpty()) {

        PathNode* curr = openSet.removeLowest();

        if (currentNode->pos == goal) {
            return yay;
        }

        [&]<size_t... I>(std::index_sequence<I...>) {
            ([&] {
                constexpr Face face = ALL_FACES[I];
                const NodePos neighborNodePos{size, bpos.offset(face, getSize(size))};
                const BlockPos origin = neighborNodePos.absolutePosZero();
                if constexpr (face == Face::UP || face == Face::DOWN) {
                    if (!isInBounds(origin)) return;
                }
                const ChunkPos neighborCpos = origin.toChunkPos();
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

                    const double heuristic = neighborNode->combinedCost;
                    if (bestHeuristicSoFar - heuristic > MIN_IMPROVEMENT) {
                        bestHeuristicSoFar = heuristic;
                        bestSoFar = neighborNode;

                        if (failing &&
                            start.distanceToSq(neighborPos.absolutePosCenter()) > MIN_DIST_PATH * MIN_DIST_PATH) {
                            failing = false;
                        }
                    }
                }
            }(), ...);
        }(std::make_index_sequence<ALL_FACES.size()>{});
    }

    auto[x, y, z] = bestSoFar->pos.absolutePosCenter();
    std::cout << "Best position = {" << x << ", " << y << ", " << z << "}\n";
    std::cout << "failing = " << failing << '\n';
    std::cout << "Open set getSize: " << openSet.getSize() << '\n';
    std::cout << "PathNode map size: " << map.size() << '\n';
    std::cout << "chunk cache size: " << chunkCache.size() << '\n';
    std::cout << '\n';

    return {bestPathSoFar(map, startNode, bestSoFar, start, goal), std::move(chunkCache)};
}