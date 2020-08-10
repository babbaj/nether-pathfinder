#pragma once

#include <cmath>

#include "Utils.h"

struct PathNode {
    static constexpr double COST_INF = 1000000.0; // probably don't need this
    static constexpr double COST_ONE = 1.0;

    const BlockPos pos;

    const double estimatedCostToGoal;
    double cost = COST_INF;
    double combinedCost = 0;

    PathNode* previous = nullptr;
    int heapPosition = -1;

    explicit PathNode(const BlockPos& pos, const BlockPos& goal): pos(pos), estimatedCostToGoal(heuristic(pos, goal)) {}

    bool isOpen() const {
        return this->heapPosition != -1;
    }

private:
    static double heuristic(const BlockPos& pos, const BlockPos& goal) {
        return pos.distanceTo(goal);
    }
};