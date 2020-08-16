#pragma once

#include "Utils.h"

#include <array>

using x2 = uint8_t;
using x4 = std::array<x2, 8>;
using x8 = std::array<x4, 8>;
using x16 = std::array<x8, 8>;

struct Chunk {
private:
    std::array<x16, 8> data{};

#define CHUNK_GETBIT                                               \
    auto& x2 = data                                                \
    [y >> 4] /* x16 */                                             \
    [((x & 8) >> 3) | ((y & 8) >> 2) | ((z & 8) >> 1)]  /* x8 */   \
    [((x & 4) >> 2) | ((y & 4) >> 1) | ((z & 4))     ]  /* x4 */   \
    [((x & 2) >> 1) | ((y & 2))      | ((z & 2) << 1)]; /* x2 */   \
    uint8_t bit =                                                  \
     ((x & 1))      | ((y & 1) << 1) | ((z & 1) << 2); /* bit */


    static constexpr int x16Index(int y) {
        return y >> 4;
    }

    static constexpr int x8Index(int x, int y, int z) {
        return ((x & 8) >> 3) | ((y & 8) >> 2) | ((z & 8) >> 1);
    }

    static constexpr int x4Index(int x, int y, int z) {
        return ((x & 4) >> 2) | ((y & 4) >> 1) | ((z & 4));
    }

    static constexpr int x2Index(int x, int y, int z) {
        return ((x & 2) >> 1) | ((y & 2)) | ((z & 2) << 1);
    }

    static constexpr int bitIndex(int x, int y, int z) {
        return ((x & 1)) | ((y & 1) << 1) | ((z & 1) << 2);
    }



    bool getBit(int x, int y, int z) const {
        CHUNK_GETBIT;

        return (x2 >> bit) & 1;
    }

    void setBit(int x, int y, int z, bool b) {
        CHUNK_GETBIT;

         if (b) {
             x2 |= (1u << bit);
         } else {
             x2 &= ~(1u << bit);
         }
    }

public:
    void setBlock(int x, int y, int z, bool solid) {
        setBit(x, y, z, solid);
    }

    bool isSolid(const BlockPos& pos) const {
        return isSolid(pos.x, pos.y, pos.z);
    }

    bool isSolid(int x, int y, int z) const {
        return getBit(x, y, z);
    }
};