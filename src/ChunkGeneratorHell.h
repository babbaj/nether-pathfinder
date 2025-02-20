#pragma once

#include <cmath>
#include <numbers>
#include <chrono>
#include <iostream>

#include "NoiseGeneratorOctaves.h"
#include "Chunk.h"
#include "ParallelExecutor.h"
#include "ChunkGen.h"
#include "Region.h"

struct ChunkGeneratorHell {
public:
    // These must be declared in the right order
    NoiseGeneratorOctaves<16> lperlinNoise1;
    NoiseGeneratorOctaves<16> lperlinNoise2;
    NoiseGeneratorOctaves<8>  perlinNoise1;

    void prepareHeights(int x, int z, Chunk& primer, ChunkGenExec& threadPool) const;

    // buffer may be null
    template<int xSize, int ySize, int zSize>
    std::array<double, xSize * ySize * zSize> getHeights(int xOffset, int yOffset, int zOffset, ChunkGenExec& threadPool) const;
public:

    static ChunkGeneratorHell fromSeed(uint64_t seed) {
        Random rand{seed};

        return ChunkGeneratorHell {
            decltype(lperlinNoise1)(rand),
            decltype(lperlinNoise2)(rand),
            decltype(perlinNoise1)(rand)
        };
    }

    void generateChunk(int x, int z, Region& region, ChunkGenExec& threadPool) const;
    Chunk generateChunk(int x, int z, ChunkGenExec& threadPool) const;
};

// This is only instantiated once
template<int xSize, int ySize, int zSize>
std::array<double, xSize * ySize * zSize> ChunkGeneratorHell::getHeights(int xOffset, int yOffset, int zOffset, ChunkGenExec& threadPool) const {
    std::array<double, xSize * ySize * zSize> buffer{};

    //auto noiseData4 = this->scaleNoise.generateNoiseOctaves<xSize,     1, zSize>(xOffset, yOffset, zOffset, 1.0, 0.0, 1.0);

    /*auto pnr =      this->perlinNoise1.generateNoiseOctaves<xSize, ySize, zSize>(xOffset, yOffset, zOffset, 8.555150000000001, 34.2206, 8.555150000000001); // 55us
    auto ar =      this->lperlinNoise1.generateNoiseOctaves<xSize, ySize, zSize>(xOffset, yOffset, zOffset, 684.412, 2053.236, 684.412); // 105us
    auto br =      this->lperlinNoise2.generateNoiseOctaves<xSize, ySize, zSize>(xOffset, yOffset, zOffset, 684.412, 2053.236, 684.412); // 105us*/

    auto [pnr, ar, br] = threadPool.compute(
        [=, this] {
            return this->perlinNoise1.generateNoiseOctaves<xSize, ySize, zSize>(xOffset, yOffset, zOffset, 8.555150000000001, 34.2206, 8.555150000000001);
        },
        [=, this] {
            return this->lperlinNoise1.generateNoiseOctaves<xSize, ySize, zSize>(xOffset, yOffset, zOffset, 684.412, 2053.236, 684.412);
        },
        [=, this] {
            return this->lperlinNoise2.generateNoiseOctaves<xSize, ySize, zSize>(xOffset, yOffset, zOffset, 684.412, 2053.236, 684.412);
        }
    );

    int i = 0;
    double adouble[ySize];

    for (int j = 0; j < ySize; ++j)
    {
        adouble[j] = std::cos((double)j * std::numbers::pi * 6.0 / (double)ySize) * 2.0;
        double d2 = (double)j;

        if (j > ySize / 2)
        {
            d2 = (double)(ySize - 1 - j);
        }

        if (d2 < 4.0)
        {
            d2 = 4.0 - d2;
            adouble[j] -= d2 * d2 * d2 * 10.0;
        }
    }

    for (int l = 0; l < xSize; ++l)
    {
        for (int i1 = 0; i1 < zSize; ++i1)
        {
            for (int k = 0; k < ySize; ++k)
            {
                const double d4 = adouble[k];
                const double d5 = ar[i] / 512.0;
                const double d6 = br[i] / 512.0;
                const double d7 = (pnr[i] / 10.0 + 1.0) / 2.0;
                double d8;

                if (d7 < 0.0)
                {
                    d8 = d5;
                }
                else if (d7 > 1.0)
                {
                    d8 = d6;
                }
                else
                {
                    d8 = d5 + (d6 - d5) * d7;
                }

                d8 = d8 - d4;

                if (k > ySize - 4)
                {
                    double d9 = (double)((float)(k - (ySize - 4)) / 3.0F);
                    d8 = d8 * (1.0 - d9) + -10.0 * d9;
                }

                if ((double)k < 0.0)
                {
                    double d10 = (0.0 - (double)k) / 4.0;
                    d10 = std::clamp(d10, 0.0, 1.0);
                    d8 = d8 * (1.0 - d10) + -10.0 * d10;
                }

                buffer[i] = d8;
                ++i;
            }
        }
    }

    return buffer;
}