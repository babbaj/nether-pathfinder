#pragma once

#include "Utils.h"

#include <array>
#include <cstring>

using x2_t = uint8_t;
using x4_t = std::array<x2_t, 8>;
using x8_t = std::array<x4_t, 8>;
using x16_t = std::array<x8_t, 8>;

template<typename T> requires (sizeof(T) >= sizeof(int64_t))
inline bool isEmpty(const T& input) {
    constexpr auto arrSize = sizeof(T) / sizeof(int64_t);
    int64_t nums[arrSize];
    memcpy(&nums[0], &input, sizeof(nums));

    return [&nums]<size_t... I>(std::index_sequence<I...>) {
        return (nums[I] | ...) == 0;
    }(std::make_index_sequence<arrSize>{});
}

struct Chunk {
private:
    std::array<x16_t, 8> data{};
    std::array<bool, 8> x16Empty{};

#define CHUNK_GETBIT(x, y, z)                   \
    auto& x2 = data                             \
    [x16Index(y)] /* x16 */                     \
    [x8Index(x, y, z)]  /* x8 */                \
    [x4Index(x, y, z)]  /* x4 */                \
    [x2Index(x, y, z)]; /* x2 */                \
    uint8_t bit = bitIndex(x, y, z); /* bit */

public:
    static constexpr int x16Index(int y) {
        return y >> 4;
    }

    static constexpr int x8Index(int x, int y, int z) {
        return ((x & 8) >> 1) | ((y & 8) >> 2) | ((z & 8) >> 3);
    }

    static constexpr int x4Index(int x, int y, int z) {
        return ((x & 4)) | ((y & 4) >> 1) | ((z & 4) >> 2);
    }

    static constexpr int x2Index(int x, int y, int z) {
        return ((x & 2) << 1) | ((y & 2)) | ((z & 2) >> 1);
    }

    static constexpr int bitIndex(int x, int y, int z) {
        return ((x & 1) << 2) | ((y & 1) << 1) | ((z & 1));
    }

private:
    bool getBit(int x, int y, int z) const {
        CHUNK_GETBIT(x, y, z);

        return (x2 >> bit) & 1;
    }

    void setBit(int x, int y, int z, bool b) {
        CHUNK_GETBIT(x, y, z);

         if (b) {
             x2 |= (1u << bit);
         } else {
             x2 &= ~(1u << bit);
         }
    }

#undef CHUNK_GETBIT

public:
    void calcEmptyX16() {
        for (int i = 0; i < 8; i++) {
            x16Empty[i] = ::isEmpty(data[i]);
        }
    }

    const x16_t& getX16(int y) const {
        return data[x16Index(y)];
    }

    const x8_t& getX8(int x, int y, int z) const {
        return getX16(y)[x8Index(x, y, z)];
    }

    const x4_t& getX4(int x, int y, int z) const {
        return getX8(x, y, z)[x4Index(x, y, z)];
    }

    const x2_t& getX2(int x, int y, int z) const {
        return getX4(x, y, z)[x2Index(x, y, z)];
    }

    template<Size>
    bool isEmpty(int x, int y, int z) const = delete;

    void setBlock(int x, int y, int z, bool solid) {
        setBit(x, y, z, solid);
    }

    bool isSolid(const BlockPos& pos) const {
        return isSolid(pos.x, pos.y, pos.z);
    }

    bool isSolid(int x, int y, int z) const {
        return getBit(x, y, z) == 1;
    }
};

template<>
inline bool Chunk::isEmpty<Size::X16>(int, int y, int) const {
    return x16Empty[x16Index(y)];
}

template<>
inline bool Chunk::isEmpty<Size::X8>(int x, int y, int z) const {
    const x8_t& x8 = getX8(x, y, z);
    return ::isEmpty(x8);
}

template<>
inline bool Chunk::isEmpty<Size::X4>(int x, int y, int z) const {
    const x4_t& x4 = getX4(x, y, z);
    return ::isEmpty(x4);
}

template<>
inline bool Chunk::isEmpty<Size::X2>(int x, int y, int z) const {
    return getX2(x, y, z) == 0;
}

template<>
inline bool Chunk::isEmpty<Size::X1>(int x, int y, int z) const {
    return getBit(x, y, z) == 0;
}