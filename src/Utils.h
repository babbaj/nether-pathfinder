#pragma once

#include <cmath>
#include <functional>

enum Face {
    UP,
    DOWN,
    NORTH,
    SOUTH,
    EAST,
    WEST
};
constexpr std::array<Face, 6> ALL_FACES {Face::UP, Face::DOWN, Face::NORTH, Face::SOUTH, Face::EAST, Face::WEST};

enum class Size : int {
    X1,
    X2,
    X4,
    X8,
    X16
};

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

    /*template<unsigned Res> requires (16 % Res == 0 && Res <= 16)
    [[nodiscard]] constexpr BlockPos toChunkLocal() const {
        constexpr int xzMask = 0xFu >> (Res - 1);
        static_assert(xzMask <= 0xF && xzMask >= 1);
        return {this->x & xzMask, static_cast<int>(this->y / Res), this->z & xzMask};
    }*/

    [[nodiscard]] constexpr BlockPos toChunkLocal() const {
        return {this->x & 0xF, this->y, this->z & 0xF};
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

    [[nodiscard]] BlockPos offset(Face face, int n = 1) const {
        switch (face) {
            case Face::UP: return up(n);
            case Face::DOWN: return down(n);
            case Face::NORTH: return north(n);
            case Face::SOUTH: return south(n);
            case Face::EAST: return east(n);
            case Face::WEST: return west(n);
        }
    }

    BlockPos up(int n = 1) const {
        return {this->x, this->y + n, this->z};
    }

    BlockPos down(int n = 1) const {
        return {this->x, this->y - n, this->z};
    }

    BlockPos east(int n = 1) const {
        return {this->x + n, this->y, this->z};
    }

    BlockPos west(int n = 1) const {
        return {this->x - n, this->y, this->z};
    }

    BlockPos north(int n = 1) const {
        return {this->x, this->y, this->z - n};
    }

    BlockPos south(int n = 1) const {
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

constexpr BlockPos operator/(const BlockPos& pos, int n) {
    return {pos.x / n, pos.y / n, pos.z / n};
}

constexpr BlockPos operator*(const BlockPos& pos, int n) {
    return {pos.x * n, pos.y * n, pos.z * n};
}

constexpr BlockPos operator+(const BlockPos& pos, int n) {
    return {pos.x + n, pos.y + n, pos.z + n};
}

constexpr BlockPos operator-(const BlockPos& pos, int n) {
    return {pos.x - n, pos.y - n, pos.z - n};
}

constexpr BlockPos operator>>(const BlockPos& pos, int n) {
    return {pos.x >> n, pos.y >> n, pos.z >> n};
}


constexpr BlockPos operator<<(const BlockPos& pos, int n) {
    return {pos.x << n, pos.y << n, pos.z << n};
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