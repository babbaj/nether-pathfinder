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

constexpr int getSize(Size sizeEnum) {
    switch (sizeEnum) {
        case Size::X16: return 16;
        case Size::X8: return 8;
        case Size::X4: return 4;
        case Size::X2: return 2;
        case Size::X1: return 1;
    }
}

struct NodePos {
    friend struct std::hash<NodePos>;

    const Size size;
private:
    const BlockPos pos; // absolute pos / size
public:
    explicit NodePos(Size enumSize, const BlockPos& approxPosition): size(enumSize), pos(approxPosition / getSize(enumSize)) { }

    BlockPos absolutePosZero() const {
        return this->pos * getSize(this->size);
    }
    BlockPos absolutePosCenter() const {
        const auto sz = getSize(this->size);
        return (this->pos * sz) + sz / 2;
    }

    // TODO: function that returns the real center instead of block aligned center

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

    explicit PathNode(const NodePos& pos, const BlockPos& goal): pos(pos), estimatedCostToGoal(heuristic(pos.absolutePosCenter(), goal)) {}

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