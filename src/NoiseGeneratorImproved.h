#pragma once

#include <array>
#include <cstdio>

#include "Random.h"


// This only effects the random number generator in the constructor
struct NoiseGeneratorImproved {
    std::array<int, 512> permutations{};
    // must be declared in this order
    const double xCoord;
    const double yCoord;
    const double zCoord;

    explicit NoiseGeneratorImproved(Random& random):
        xCoord(random.nextDouble() * 256.0),  yCoord(random.nextDouble() * 256.0), zCoord(random.nextDouble() * 256.0)
        {
            for (int i = 0; i < 256; i++) {
                permutations[i] = i;
            }

            for (int l = 0; l < 256; ++l)
            {
                int j = random.nextInt(256 - l) + l;
                int k = permutations[l];
                permutations[l] = permutations[j];
                permutations[j] = k;
                permutations[l + 256] = permutations[l];
            }
        }


    void populateNoiseArray(double* noiseArray, double xOffset, double yOffset, double zOffset, int xSize, int ySize, int zSize, double xScale, double yScale, double zScale, double noiseScale) const;
};