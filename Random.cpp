#include "Random.h"

#include <cassert>

constexpr int64_t multiplier = 0x5DEECE66DL;
constexpr int64_t addend = 0xBL;
constexpr int64_t mask = (1uLL << 48u) - 1;

constexpr double DOUBLE_UNIT = 0x1.0p-53; // 1.0 / (1L << 53)

static constexpr int64_t initialScramble(int64_t seed) {
    return (seed ^ multiplier) & mask;
}


Random::Random(uint64_t seed): seed(initialScramble(seed)) {}

/*int Random::next(int bits) {
    if (bits < 1) { bits = 1; }
    else if (bits > 32) { bits = 32; }
    seed = (seed * 0x5deece66d + 0xb);
    seed &= ((1LLU << 48u) - 1);
    return (int64_t) (seed >> (48u - bits));
}*/
template<typename T>
bool compareAndSet(T& value, T expected, T update) {
    if (value == expected) {
        value = update;
        return true;
    } else {
        return false;
    }
}

int Random::next(int bits) {
    uint64_t oldseed, nextseed;
    uint64_t& seed = this->seed;
    do {
        oldseed = seed;
        nextseed = (oldseed * multiplier + addend) & mask;
    } while (!compareAndSet(this->seed, oldseed, nextseed));
    return (int)(nextseed >> (48 - bits)); // I hope this is a valid >>>
}

void Random::setSeed(int64_t newSeed) {
    this->seed = initialScramble(newSeed);
}

double Random::nextDouble() {
    return (((int64_t)(next(26)) << 27) + next(27)) * DOUBLE_UNIT;
}

/*int Random::nextInt() {
    return next(31);
}*/

int Random::nextInt(int bound) {
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

    /*if ((bound & (bound - 1)) == 0) {
        return (long signed) ((bound * (unsigned long long) next(31)) >> 31u);
    }
    long signed bits = next(31);
    long signed val = bits % bound;
    while ((bits - val + bound - 1) < 0) {
        bits = next(31);
        val = bits % bound;
    }
    return val;*/
}