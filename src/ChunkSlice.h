#pragma once

#include "Utils.h"

#include <immintrin.h>

// x16 = 4 x8  (1)
// x8 = 4 x4   (4)
// x4 = 4 x2   (16)
// x2 = 4 bits (64)
// x1 = 1 bit  (256)

struct ChunkSlice {
    uint8_t data[64]; // 256 bits

    // x bits always on the right
    static int x8Index(int x, int z) {
        return ((x & 0b1000) >> 3) | ((z & 0b1000) >> 2);
    }

    static int x4Index(int x, int z) {
        return ((x & 0b100) >> 2) | ((z & 0b100) >> 1);
    }

    static int x2Index(int x, int z) {
        return ((x & 0b10)) | ((z & 0b10) >> 1);
    }

    static int x1Index(int x, int z) {
        return (x & 1) | ((z & 1) << 1);
    }

    uint64_t getX8(int x, int z) const {
        const auto x8offset = x8Index(x, z) * 16;
        uint64_t x8;
        memcpy(&x8, this->data + x8offset, sizeof(x8));
        return x8;
    }

    uint16_t getX4(int x, int z) const {
        const auto x8offset = x8Index(x, z) * 16;
        const auto x4offset = x8offset + (x4Index(x, z) * sizeof(uint16_t));
        uint16_t x4;
        memcpy(&x4, this->data + x4offset, sizeof(x4));
        return x4;
    }

    uint8_t getX2(int x, int z) const {
        const auto x4 = getX4(x, z);
        const auto x2Idx = x2Index(x, z);
        return x4 << (x2Idx * 4);
    }


    bool isEmpty() const {
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
        return ((x2 << bit) & 1) == 0;
    }

    template<Size>
    bool IsEmpty(int x, int z) const = delete;

};

template<>
inline bool ChunkSlice::IsEmpty<Size::X16>(int, int) const {
    return this->isEmpty();
}

template<>
inline bool ChunkSlice::IsEmpty<Size::X8>(int x, int z) const {
    return this->x8Empty(x, z);
}

template<>
inline bool ChunkSlice::IsEmpty<Size::X4>(int x, int z) const {
    return this->x4Empty(x, z);
}

template<>
inline bool ChunkSlice::IsEmpty<Size::X2>(int x, int z) const {
    return this->x2Empty(x, z);
}

template<>
inline bool ChunkSlice::IsEmpty<Size::X1>(int x, int z) const {
    return this->x1Empty(x, z);
}