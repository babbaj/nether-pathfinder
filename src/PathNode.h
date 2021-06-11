#pragma once

#include <cmath>
#include <array>

#include "Utils.h"

constexpr int shiftFor(Size size) {
    return static_cast<int>(size);
}

constexpr int getSize(Size sizeEnum) {
    return 1 << shiftFor(sizeEnum);
}

template<typename PosType>
struct NodePos {
    friend struct std::hash<NodePos>;

    const Size size;
private:
    const PosType pos; // absolute pos / size
public:
    explicit NodePos(Size enumSize, const PosType& approxPosition): size(enumSize), pos(approxPosition >> shiftFor(enumSize)) { }

    [[nodiscard]] PosType absolutePosZero() const {
        return this->pos << shiftFor(this->size);
    }
    [[nodiscard]] PosType absolutePosCenter() const {
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
    struct hash<NodePos<BlockPos>> {
        size_t operator()(const NodePos<BlockPos>& pos) const {
            const auto& [x, y, z] = pos.pos;
            size_t hash = 3241;
            hash = 6406146L * hash + static_cast<int>(pos.size);
            hash = 3457689L * hash + x;
            hash = 8734625L * hash + y;
            hash = 2873465L * hash + z;
            return hash;
        }
    };

    template<>
    struct hash<NodePos<Pos2D>> {
        size_t operator()(const NodePos<Pos2D>& pos) const {
            const auto& [x, z] = pos.pos;
            size_t hash = 3241;
            hash = 6406146L * hash + static_cast<int>(pos.size);
            hash = 3457689L * hash + x;
            hash = 2873465L * hash + z;
            return hash;
        }
    };
}

struct PathNode {
    static constexpr double COST_INF = 1000000.0; // probably don't need this
    static constexpr double COST_ONE = 1.0;

    const double estimatedCostToGoal;
    double cost = COST_INF;
    double combinedCost = 0;

    PathNode* previous = nullptr;
    int heapPosition = -1;

protected:
    explicit PathNode(double cost): estimatedCostToGoal(cost) {}
public:

    [[nodiscard]] bool isOpen() const {
        return this->heapPosition != -1;
    }

};

struct PathNode3D : PathNode {
    using pos_type = NodePos<BlockPos>;
    const NodePos<BlockPos> pos;

    explicit PathNode3D(const NodePos<BlockPos>& posIn, const BlockPos& goal): PathNode(heuristic(posIn, goal)), pos(posIn) {}

private:
    static int manhattan(const BlockPos& a, const BlockPos& b) {
        return abs(a.x - b.x) + abs(a.z - b.z);
    }

    static double heuristic(const NodePos<BlockPos>& pos, const BlockPos& goal) {
        const auto bpos = pos.absolutePosCenter();
        return manhattan(bpos, goal) * 0.7 + bpos.distanceTo(goal) * 0.001 - (getSize(pos.size) * 4);
    }
};

struct PathNode2D : PathNode {
    using pos_type = NodePos<Pos2D>;
    const NodePos<Pos2D> pos;

    explicit PathNode2D(const NodePos<Pos2D>& posIn, const Pos2D& goal): PathNode(heuristic(posIn, goal)), pos(posIn) {}

private:
    static int manhattan(const Pos2D& a, const Pos2D& b) {
        return abs(a.x - b.x) + abs(a.z - b.z);
    }

    static double heuristic(const NodePos<Pos2D>& pos, const Pos2D& goal) {
        const auto center = pos.absolutePosCenter();
        ///return manhattan(center, goal) * 1.1 + center.distanceTo(goal) * 0.001;
        return manhattan(center, goal) * 10 + center.distanceTo(goal) * 0.001 - (getSize(pos.size) * 4);
    }
};