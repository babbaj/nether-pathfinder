#pragma once

#include <cmath>
#include <array>

#include "Utils.h"

struct NodePos {
    friend struct std::hash<NodePos>;

    const Size size;
private:
    const BlockPos pos; // absolute pos / size
public:
    explicit NodePos(Size enumSize, const BlockPos& approxPosition): size(enumSize), pos(approxPosition >> shiftFor(enumSize)) { }

    BlockPos absolutePosZero() const {
        return this->pos << shiftFor(this->size);
    }
    BlockPos absolutePosCenter() const {
        const auto sz = width(this->size);
        return (this->pos << shiftFor(this->size)) + (sz / 2);
        // this seems to make the line go through blocks, must be broken?
        //const auto zero = this->absolutePosZero();
        //int off_x = sz / 2;
        //int off_z = sz / 2;
        //if (zero.x < 0) {
        //    off_x = -off_x;
        //}
        //if (zero.z < 0) {
        //    off_z = -off_z;
        //}
        //return {zero.x + off_x, zero.y, zero.z + off_z};
    }

    constexpr friend bool operator==(const NodePos& a, const NodePos& b) {
        return a.pos == b.pos && a.size == b.size;
    }
};


namespace std {
    template<>
    struct hash<NodePos> {
        size_t operator()(const NodePos& pos) const {
            const auto& [x, y, z] = pos.pos;
            size_t hash = 3241;
            hash = 6406146L * hash + static_cast<int>(pos.size);
            hash = 3457689L * hash + x;
            hash = 8734625L * hash + y;
            hash = 2873465L * hash + z;
            return hash;
        }
    };
}

struct PathNode {
    static constexpr double COST_INF = 1000000.0; // probably don't need this
    static constexpr double COST_ONE = 1.0;

    const NodePos pos;

    const double estimatedCostToGoal;
    double cost = COST_INF;
    double combinedCost = 0;

    PathNode* previous = nullptr;
    int heapPosition = -1;

    explicit PathNode(const NodePos& pos, const BlockPos& goal): pos(pos), estimatedCostToGoal(heuristic(pos, goal)) {}

    [[nodiscard]] bool isOpen() const {
        return this->heapPosition != -1;
    }

private:
    [[maybe_unused]] static int manhattan(const BlockPos& a, const BlockPos& b) {
        return abs(a.x - b.x) + abs(a.z - b.z);
    }

    [[maybe_unused]] static double octile(const BlockPos& a, const BlockPos& b) {
        auto dx = (double) std::abs(a.x - b.x);
        auto dz = (double) std::abs(a.z - b.z);
        return (dx + dz) + (M_SQRT2 - 2) * std::min(dx, dz);
    }

    // https://rdrr.io/cran/LearnClust/src/R/octileDistance.details.R
    // seems to work as well euclidean but im sus
    static double heuristic(const NodePos& pos, const BlockPos& goal) {
        const auto center = pos.absolutePosCenter();
        // the original heuristic (turns out it sucks)
        //return manhattan(center, goal) * 0.7 + center.distanceTo(goal) * 0.001 - (width(pos.size) * 4);
        //return octile(center, goal) - (width(pos.size) * 4);
        return center.distanceTo(goal) - (width(pos.size) * 4);
    }
};