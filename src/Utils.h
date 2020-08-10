#pragma once

#include <cmath>



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

    [[nodiscard]] double distanceTo(const BlockPos& pos) const {
        const int dx = pos.x - this->x;
        const int dy = pos.y - this->y;
        const int dz = pos.z - this->z;

        return sqrt((dx * dx) + (dy * dy) + (dz * dz));
    }

    [[nodiscard]] BlockPos up() const {
        return {this->x, this->y + 1, this->z};
    }

    [[nodiscard]] BlockPos down() const {
        return {this->x, this->y - 1, this->z};
    }

    [[nodiscard]] BlockPos east() const {
        return {this->x + 1, this->y, this->z};
    }

    [[nodiscard]] BlockPos west() const {
        return {this->x - 1, this->y, this->z};
    }

    [[nodiscard]] BlockPos north() const {
        return {this->x, this->y, this->z - 1};
    }

    [[nodiscard]] BlockPos south() const {
        return {this->x, this->y, this->z + 1};
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