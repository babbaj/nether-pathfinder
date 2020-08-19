#pragma once

#include <cmath>
#include <array>

#include "Utils.h"


enum class Size : int {
    X1,
    X2,
    X4,
    X8,
    X16
};

constexpr int shiftFor(Size size) {
    return static_cast<int>(size);
}

constexpr int getSize(Size sizeEnum) {
    return 1 << shiftFor(sizeEnum);
}

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
        const auto sz = getSize(this->size);
        return (this->pos << shiftFor(this->size)) + (sz / 2);
    }

    // TODO: function that returns the real center instead of block aligned center?

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
    static int manhattan(const BlockPos& a, const BlockPos& b) {
        return abs(a.x - b.x) + abs(a.z - b.z);
    }

    static double heuristic(const NodePos& pos, const BlockPos& goal) {
        const auto bpos = pos.absolutePosCenter();
        return manhattan(bpos, goal) * 0.7 + bpos.distanceTo(goal) * 0.001 - (getSize(pos.size) * 4);
    }
};