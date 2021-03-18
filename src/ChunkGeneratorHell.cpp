#include "ChunkGeneratorHell.h"

#include <cmath>
#include <cassert>
#include <chrono>

void ChunkGeneratorHell::generateChunk(int x, int z, Chunk& chunkprimer) {
    prepareHeights(x, z, chunkprimer);
    chunkprimer.calcEmptyX16();
}

Chunk ChunkGeneratorHell::generateChunk(int x, int z) {
    Chunk chunkprimer{};
    prepareHeights(x, z, chunkprimer);
    chunkprimer.calcEmptyX16();

    return chunkprimer;
}

void ChunkGeneratorHell::prepareHeights(int x, int z, Chunk& primer) {
    std::array buffer = this->getHeights<5, 17, 5>(x * 4, 0, z * 4);

    constexpr auto j = 64 / 2 + 1; // 64 = sea level

    for (int j1 = 0; j1 < 4; ++j1)
    {
        for (int k1 = 0; k1 < 4; ++k1)
        {
            for (int l1 = 0; l1 < 16; ++l1)
            {
                double d1 = buffer[((j1 + 0) * 5 + k1 + 0) * 17 + l1 + 0];
                double d2 = buffer[((j1 + 0) * 5 + k1 + 1) * 17 + l1 + 0];
                double d3 = buffer[((j1 + 1) * 5 + k1 + 0) * 17 + l1 + 0];
                double d4 = buffer[((j1 + 1) * 5 + k1 + 1) * 17 + l1 + 0];
                const double d5 = (buffer[((j1 + 0) * 5 + k1 + 0) * 17 + l1 + 1] - d1) * 0.125;
                const double d6 = (buffer[((j1 + 0) * 5 + k1 + 1) * 17 + l1 + 1] - d2) * 0.125;
                const double d7 = (buffer[((j1 + 1) * 5 + k1 + 0) * 17 + l1 + 1] - d3) * 0.125;
                const double d8 = (buffer[((j1 + 1) * 5 + k1 + 1) * 17 + l1 + 1] - d4) * 0.125;

                for (int i2 = 0; i2 < 8; ++i2)
                {
                    double d10 = d1;
                    double d11 = d2;
                    double d12 = (d3 - d1) * 0.25;
                    double d13 = (d4 - d2) * 0.25;

                    for (int j2 = 0; j2 < 4; ++j2)
                    {
                        double d15 = d10;
                        double d16 = (d11 - d10) * 0.25;

                        for (int k2 = 0; k2 < 4; ++k2)
                        {
                            bool iblockstate = false;
                            if (l1 * 8 + i2 < j)
                            {
                                iblockstate = true; // LAVA
                            }

                            if (d15 > 0.0)
                            {
                                iblockstate = true; // NETHERRACK
                            }
                            const int l2 = j2 + j1 * 4;
                            const int i3 = i2 + l1 * 8;
                            const int j3 = k2 + k1 * 4;
                            // bitset defaults to air so writing 0 to it is pointless
                            if (iblockstate) primer.setBlock(l2, i3, j3, iblockstate);
                            d15 += d16;
                        }

                        d10 += d12;
                        d11 += d13;
                    }

                    d1 += d5;
                    d2 += d6;
                    d3 += d7;
                    d4 += d8;
                }
            }
        }
    }
}


