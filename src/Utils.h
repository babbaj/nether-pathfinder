#pragma once

#include <cmath>
#include <functional>

struct ChunkPos {
    int x, z;
};

struct BlockPos {
    int x;
    int y;
    int z;

    [[nodiscard]] ChunkPos toChunkPos() const {
        return {this->x >> 4, this->z >> 4};
    }

    [[nodiscard]] BlockPos toChunkLocal() const {
        return {this->x & 15, this->y, this->z & 15};
    }

    [[nodiscard]] double distanceToSq(const BlockPos& pos) const {
        const double dx = pos.x - this->x;
        const double dy = pos.y - this->y;
        const double dz = pos.z - this->z;

        return (dx * dx) + (dy * dy) + (dz * dz);
    }

    [[nodiscard]] double distanceTo(const BlockPos& pos) const {
        return sqrt(this->distanceToSq(pos));
    }

    [[nodiscard]] BlockPos up(int n = 1) const {
        return {this->x, this->y + n, this->z};
    }

    [[nodiscard]] BlockPos down(int n = 1) const {
        return {this->x, this->y - n, this->z};
    }

    [[nodiscard]] BlockPos east(int n = 1) const {
        return {this->x + n, this->y, this->z};
    }

    [[nodiscard]] BlockPos west(int n = 1) const {
        return {this->x - n, this->y, this->z};
    }

    [[nodiscard]] BlockPos north(int n = 1) const {
        return {this->x, this->y, this->z - n};
    }

    [[nodiscard]] BlockPos south(int n = 1) const {
        return {this->x, this->y, this->z + n};
    }
};

constexpr bool operator==(const BlockPos& a, const BlockPos& b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

constexpr bool operator!=(const BlockPos& a, const BlockPos& b) {
    return !(a == b);
}

constexpr bool operator==(const ChunkPos& a, const ChunkPos& b) {
    return a.x == b.x && a.z == b.z;
}

constexpr bool operator!=(const ChunkPos& a, const ChunkPos& b) {
    return !(a == b);
}


namespace std {
    template<>
    struct hash<BlockPos> {
        size_t operator()(const BlockPos& pos) const {
            size_t hash = 3241;
            hash = 3457689L * hash + pos.x;
            hash = 8734625L * hash + pos.y;
            hash = 2873465L * hash + pos.z;
            return hash;
        }
    };

    template<>
    struct hash<ChunkPos> {
        size_t operator()(const ChunkPos& pos) const {
            size_t hash = 3241;
            hash = 3457689L * hash + pos.x;
            hash = 2873465L * hash + pos.z;
            return hash;
        }
    };
}