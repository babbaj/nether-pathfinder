#pragma once

#include <cstdint>
#include <cassert>

// java.util.Random
struct Random {
private:
    uint64_t seed;


    static constexpr int64_t multiplier = 0x5DEECE66DL;
    static constexpr int64_t addend = 0xBL;
    static constexpr int64_t mask = (1uLL << 48u) - 1;

    static constexpr double DOUBLE_UNIT = 0x1.0p-53; // 1.0 / (1L << 53)

    static constexpr int64_t initialScramble(int64_t seed) {
        return (seed ^ multiplier) & mask;
    }

    template<typename T>
    constexpr static bool compareAndSet(T& value, T expected, T update) {
        if (value == expected) {
            value = update;
            return true;
        } else {
            return false;
        }
    }

    constexpr int next(int bits) {
        uint64_t oldseed, nextseed;
        uint64_t& seed = this->seed;
        do {
            oldseed = seed;
            nextseed = (oldseed * multiplier + addend) & mask;
        } while (!compareAndSet(this->seed, oldseed, nextseed));
        return (int)(nextseed >> (48 - bits)); // I hope this is a valid >>>
    }

public:
    constexpr explicit Random(uint64_t seed): seed(initialScramble(seed)) {}

    constexpr double nextDouble() {
        return (((int64_t)(next(26)) << 27) + next(27)) * DOUBLE_UNIT;
    }

    constexpr int nextInt(int bound) {
        assert(bound > 0);

        int r = next(31);
        int m = bound - 1;
        if ((bound & m) == 0)  // i.e., bound is a power of 2
            r = (int)((bound * (int64_t)r) >> 31);
        else {
            for (int u = r;
                 u - (r = u % bound) + m < 0;
                 u = next(31))
                ;
        }
        return r;
    }

    constexpr void setSeed(int64_t newSeed) {
        this->seed = initialScramble(newSeed);
    }
};
