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
        const auto zero = this->absolutePosZero();
        // d normally stands for difference but im adding lol
        int dx = sz / 2;
        int dz = sz / 2;
        if (zero.x < 0) {
            dx = -dx;
        }
        if (zero.z < 0) {
            dz = -dz;
        }
        return {zero.x + dx, zero.y, zero.z + dz};
    }
    Vec3 absolutePosCenterPrecise() const {
        const auto sz = (double) width(this->size);
        const auto zero = this->absolutePosZero();
        double dx = (sz / 2);
        double dz = (sz / 2);
        if (zero.x < 0) {
            dx = -dx;
        }
        if (zero.z < 0) {
            dz = -dz;
        }
        return {zero.x + dx, (double) zero.y, zero.z + dz};
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
    static int manhattan(const BlockPos& a, const BlockPos& b) {
        return abs(a.x - b.x) + abs(a.z - b.z);
    }
    static double octile(const BlockPos& a, const BlockPos& b) {
        double D = 1;
        double D2 = M_SQRT2;
        auto dx = (double) std::abs(b.x - a.x);
        auto dy = (double) std::abs(b.y - a.y);
        return D * (dx + dy) + (D2 - 2 * D) * std::min(dx, dy);
    }

    static double heuristic(const NodePos& pos, const BlockPos& goal) {
        const auto center = pos.absolutePosCenter();
        ///return manhattan(center, goal) * 0.7 + center.distanceTo(goal) * 0.001 - (width(pos.size) * 4);
        return octile(center, goal) - (width(pos.size) * 4);
    }
};