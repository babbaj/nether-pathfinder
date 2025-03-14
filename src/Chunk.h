#pragma once

#include "Utils.h"

#include <array>
#include <cstring>

using x2_t = uint8_t;
using x4_t = std::array<x2_t, 8>;
using x8_t = std::array<x4_t, 8>;
using x16_t = std::array<x8_t, 8>;

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

constexpr auto makeX2Index() {
    std::array<std::array<std::array<uint16_t, 192>, 8>, 8> out{};

    for (int x = 0; x < 16; x += 2) {
        for (int z = 0; z < 16; z += 2) {
            for (int y = 0; y < 384; y+= 2) {
                const auto idx = (x16Index(y) * sizeof(x16_t))
                        + (x8Index(x, y, z) * sizeof(x8_t))
                        + (x4Index(x, y, z) * sizeof(x4_t))
                        + (x2Index(x, y, z) * sizeof(x2_t));
                out[x / 2][z / 2][y / 2] = idx;
            }
        }
    }
    return out;
}
constexpr auto X2_INDEX = makeX2Index();

template<typename T> requires (sizeof(T) >= sizeof(int64_t) && sizeof(T) <= 32 && sizeof(T) % sizeof(int64_t) == 0)
inline bool isEmpty(const T& input) {
    constexpr auto arrSize = sizeof(T) / sizeof(int64_t);
    auto* nums = reinterpret_cast<const int64_t*>(&input);

    int64_t out = 0;
    for (int i = 0; i < arrSize; i++) {
        out |= nums[i];
    }
    return out == 0;
}

// https://gcc.godbolt.org/z/nsvzefd6r
template<typename T> requires (sizeof(T) > 32 && sizeof(T) % 32 == 0)
inline bool isEmpty(const T& input) {
    using block = std::array<std::byte, 32>; // size of avx2 register (256 bits)
    constexpr auto arrSize = sizeof(T) / sizeof(block);
    auto* blocks = reinterpret_cast<const block*>(&input);

    for (int i = 0; i < arrSize; i++) {
        block data = blocks[i];
        if (!isEmpty(data)) return false;
    }
    return true;
}

enum class ChunkState : uint8_t {
    FROM_JAVA = 0
    ,FAKE = 1 // could be generated or just air
};

enum class Dimension {
    Overworld
    ,Nether
    ,End
};

inline int dimensionHeight(Dimension dim) {
    return dim == Dimension::Overworld ? 384 : 256;
}

struct Chunk {
    std::array<x16_t, 24> data;
    //std::array<bool, 8> x16Empty{};
private:

#define CHUNK_GETBIT(x, y, z)                   \
    auto& x2 = data                             \
    [x16Index(y)] /* x16 */                     \
    [x8Index(x, y, z)]  /* x8 */                \
    [x4Index(x, y, z)]  /* x4 */                \
    [x2Index(x, y, z)]; /* x2 */                \
    uint8_t bit = bitIndex(x, y, z); /* bit */
private:
    bool getBit(int x, int y, int z) const {
        CHUNK_GETBIT(x, y, z);

        return (x2 >> bit) & 1;
    }

#undef CHUNK_GETBIT

public:
    const x16_t& getX16(int y) const {
        return data[x16Index(y)];
    }

    const x8_t& getX8(int x, int y, int z) const {
        return getX16(y)[x8Index(x, y, z)];
    }

    const x4_t& getX4(int x, int y, int z) const {
        return getX8(x, y, z)[x4Index(x, y, z)];
    }

    // TODO: this doesn't modulo the input args like the other functions
    const x2_t& getX2(int x, int y, int z) const {
        auto x2Idx = X2_INDEX[x/2][z/2][y/2];
        auto* asX2Array = reinterpret_cast<const x2_t*>(this->data.data());
        return asX2Array[x2Idx];
    }

    x2_t& getX2(int x, int y, int z) {
        auto x2Idx = X2_INDEX[x/2][z/2][y/2];
        auto* asX2Array = reinterpret_cast<x2_t*>(this->data.data());
        return asX2Array[x2Idx];
    }

    // for benchmarking
    const x2_t& getX2Old(int x, int y, int z) const {
        return getX4(x, y, z)[x2Index(x, y, z)];
    }

    template<Size>
    bool isEmpty(int x, int y, int z) const = delete;

    // for benchmarking (this actually compiled to cmov)
    void setBlockOld(int x, int y, int z, bool solid) {
        auto& x2 = getX2(x, y, z);
        const uint8_t bit = bitIndex(x, y, z);
        if (solid) {
            x2 |= (1u << bit);
        } else {
            x2 &= ~(1u << bit);
        }
    }

    void setBlock(int x, int y, int z, bool solid) {
        auto& x2 = getX2(x, y, z);
        const uint8_t bit = bitIndex(x, y, z);
        // for some reason this compiled to a bit less code
        x2 = solid ? (x2 | (1u << bit)) : (x2 & ~(1u << bit));
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
    return ::isEmpty(data[x16Index(y)]);
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

static const Chunk AIR_CHUNK alignas(4096) = [] {
    Chunk out{};
    return out;
}();

static const Chunk SOLID_CHUNK alignas(4096) = [] {
    Chunk out{};
    memset(&out.data, 0xFF, sizeof(out.data));
    return out;
}();
