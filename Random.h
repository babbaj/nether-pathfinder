#pragma once

#include <cstdint>
#include <cassert>

// java.util.Random
struct Random {
private:
    uint64_t seed;

    int next(int bits);
public:
    explicit Random(uint64_t seed);
    double nextDouble();
    //int nextInt();
    int nextInt(int bound);
    void setSeed(int64_t newSeed);
};
