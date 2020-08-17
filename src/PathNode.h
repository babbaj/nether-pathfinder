#pragma once

#include <cmath>
#include <array>

#include "Utils.h"


enum Size : int {
    X1,
    X2,
    X4,
    X8,
    X16
};


struct NodePos {
    const Size size;
    const BlockPos pos; // not guaranteed to be aligned
};

struct PathNode {
    static constexpr double COST_INF = 1000000.0; // probably don't need this
    static constexpr double COST_ONE = 1.0;

    const BlockPos pos;
    const NodePos pos0{};

    const double estimatedCostToGoal;
    double cost = COST_INF;
    double combinedCost = 0;

    PathNode* previous = nullptr;
    int heapPosition = -1;

    explicit PathNode(const BlockPos& pos, const BlockPos& goal): pos(pos), estimatedCostToGoal(heuristic(pos, goal)) {}

    [[nodiscard]] bool isOpen() const {
        return this->heapPosition != -1;
    }

private:
    static int manhattan(const BlockPos& a, const BlockPos& b) {
        return (abs((a.x - b.x)) + abs((a.y - b.y)) + abs((a.z - b.z))) * 2;
    }

    static double heuristic(const BlockPos& pos, const BlockPos& goal) {
        return manhattan(pos, goal) * 1.1 + pos.distanceTo(goal) * 0.001;
    }
};