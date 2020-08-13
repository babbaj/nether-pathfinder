#pragma once

#include <cmath>

#include "Utils.h"

//constexpr int64_t COST_SCALE = 1000;
//constexpr int64_t COST_INF = 1000000 * COST_SCALE;/*1000000.0*/;
//constexpr int64_t COST_ONE = 1 * COST_SCALE;


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