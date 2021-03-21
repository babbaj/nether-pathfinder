#pragma once

#include "Utils.h"

#include <immintrin.h>

// x16 = 4 x8  (1)
// x8 = 4 x4   (4)
// x4 = 4 x2   (16)
// x2 = 4 bits (64)
// x1 = 1 bit  (256)

struct ChunkSlice {
    uint8_t data[32]; // 256 bits

    // x bits always on the right
    static int x8Index(int x, int z) {
        return ((x & 0b1000) >> 3) | ((z & 0b1000) >> 2);
    }

    static int x4Index(int x, int z) {
        return ((x & 0b100) >> 2) | ((z & 0b100) >> 1);
    }

    static int x2Index(int x, int z) {
        return ((x & 0b10) >> 1) | ((z & 0b10));
    }

    static int x1Index(int x, int z) {
        return (x & 1) | ((z & 1) << 1);
    }

    uint64_t getX8(int x, int z) const {
        const auto x8offset = x8Index(x, z) * sizeof(uint64_t);
        uint64_t x8;
        memcpy(&x8, this->data + x8offset, sizeof(x8));
        return x8;
    }

    uint16_t getX4(int x, int z) const {
        const auto x8offset = x8Index(x, z) * sizeof(uint64_t);
        const auto x4offset = x8offset + (x4Index(x, z) * sizeof(uint16_t));
        uint16_t x4;
        memcpy(&x4, this->data + x4offset, sizeof(x4));
        return x4;
    }

    uint8_t getX2(int x, int z) const {
        const auto x4 = getX4(x, z);
        const auto x2Idx = x2Index(x, z);
        return x4 >> (x2Idx * 4);
    }


    bool isFullyEmpty() const {
        const __m256i reg = _mm256_loadu_si256((__m256i*) this->data);
        return _mm256_testz_si256(reg, reg) == 0;
    }

    bool x8Empty(int x, int z) const {
        return this->getX8(x, z) == 0;
    }

    bool x4Empty(int x, int z) const {
        return this->getX4(x, z) == 0;
    }

    bool x2Empty(int x, int z) const {
        return this->getX2(x, z) == 0;
    }

    bool x1Empty(int x, int z) const {
        const uint8_t x2 = getX2(x, z);
        const auto bit = x1Index(x, z);
        return ((x2 >> bit) & 1) == 0;
    }

    template<Size>
    bool isEmpty(int x, int z) const = delete;

    void setBlock(int x, int z, bool solid) {
        const auto x8offset = x8Index(x, z) * sizeof(uint64_t);
        const auto x4offset = x8offset + (x4Index(x, z) * sizeof(uint16_t));
        uint16_t x4;
        memcpy(&x4, this->data + x4offset, sizeof(x4));
        const auto x2Idx = x2Index(x, z);
        const uint8_t x2 = x4 >> (x2Idx * 4);
        const auto bit = x1Index(x, z);
        uint8_t newX2 = x2;
        if (solid) {
            newX2 |= 1 << bit;
        } else {
            newX2 &= ~(1 << bit);
        }
        uint16_t newX4 = x4 & ~(0xF << (x2Idx * 4));
        newX4 |= static_cast<uint16_t>(newX2) << (x2Idx * 4);
        memcpy(this->data + x4offset, &newX4, sizeof(newX4));
    }
};

template<>
inline bool ChunkSlice::isEmpty<Size::X16>(int, int) const {
    return this->isFullyEmpty();
}

template<>
inline bool ChunkSlice::isEmpty<Size::X8>(int x, int z) const {
    return this->x8Empty(x, z);
}

template<>
inline bool ChunkSlice::isEmpty<Size::X4>(int x, int z) const {
    return this->x4Empty(x, z);
}

template<>
inline bool ChunkSlice::isEmpty<Size::X2>(int x, int z) const {
    return this->x2Empty(x, z);
}

template<>
inline bool ChunkSlice::isEmpty<Size::X1>(int x, int z) const {
    return this->x1Empty(x, z);
}