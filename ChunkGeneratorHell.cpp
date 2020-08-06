#include "ChunkGeneratorHell.h"

#include <cmath>
#include <cassert>

ChunkPrimer ChunkGeneratorHell::generateChunk(int x, int z) {
    this->rand.setSeed((int64_t)x * 341873128712L + (int64_t)z * 132897987541L);
    ChunkPrimer chunkprimer{};
    prepareHeights(x, z, chunkprimer);
    //buildSurfaces(x, z, chunkprimer);

    return chunkprimer;
}

void ChunkGeneratorHell::prepareHeights(int sizeX, int sizeZ, ChunkPrimer& primer) {
    this->buffer = this->getHeights(&this->buffer, sizeX * 4, 0, sizeZ * 4, 5, 17, 5);

    constexpr auto j = 64 / 2 + 1; // 64 = sea level

    for (int j1 = 0; j1 < 4; ++j1)
    {
        for (int k1 = 0; k1 < 4; ++k1)
        {
            for (int l1 = 0; l1 < 16; ++l1)
            {
                double d1 = this->buffer[((j1 + 0) * 5 + k1 + 0) * 17 + l1 + 0];
                double d2 = this->buffer[((j1 + 0) * 5 + k1 + 1) * 17 + l1 + 0];
                double d3 = this->buffer[((j1 + 1) * 5 + k1 + 0) * 17 + l1 + 0];
                double d4 = this->buffer[((j1 + 1) * 5 + k1 + 1) * 17 + l1 + 0];
                double d5 = (this->buffer[((j1 + 0) * 5 + k1 + 0) * 17 + l1 + 1] - d1) * 0.125;
                double d6 = (this->buffer[((j1 + 0) * 5 + k1 + 1) * 17 + l1 + 1] - d2) * 0.125;
                double d7 = (this->buffer[((j1 + 1) * 5 + k1 + 0) * 17 + l1 + 1] - d3) * 0.125;
                double d8 = (this->buffer[((j1 + 1) * 5 + k1 + 1) * 17 + l1 + 1] - d4) * 0.125;

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
                                iblockstate = false; // LAVA
                            }

                            if (d15 > 0.0)
                            {
                                iblockstate = true; // NETHERRACK
                            }
                            int l2 = j2 + j1 * 4;
                            int i3 = i2 + l1 * 8;
                            int j3 = k2 + k1 * 4;
                            primer.setBlock(l2, i3, j3, iblockstate);
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


// I hope this is right
// TODO: this is probably not necessary
void ChunkGeneratorHell::buildSurfaces(int x, int z, ChunkPrimer& primer) {
    constexpr int i = /*this->world.getSeaLevel()*/64 + 1;
    //this->slowsandNoise = this->slowsandGravelNoiseGen.generateNoiseOctaves(this->slowsandNoise, p_185937_1_ * 16, p_185937_2_ * 16, 0, 16, 16, 1, 0.03125, 0.03125, 1.0);
    //this->gravelNoise = this->slowsandGravelNoiseGen.generateNoiseOctaves(this->gravelNoise, p_185937_1_ * 16, 109, p_185937_2_ * 16, 16, 1, 16, 0.03125, 1.0, 0.03125);
    this->depthBuffer = this->netherrackExculsivityNoiseGen.generateNoiseOctaves(&this->depthBuffer, x * 16, z * 16, 0, 16, 16, 1, 0.0625, 0.0625, 0.0625);

    for (int j = 0; j < 16; ++j)
    {
        for (int k = 0; k < 16; ++k)
        {
            /*bool flag = this->slowsandNoise[j + k * 16] +*/ this->rand.nextDouble() /* * 0.2 > 0.0*/;
            /*bool flag1 = this->gravelNoise[j + k * 16] +*/ this->rand.nextDouble() /* * 0.2 > 0.0*/;
            int l = (int)(this->depthBuffer[j + k * 16] / 3.0 + 3.0 + this->rand.nextDouble() * 0.25);
            int i1 = -1;
            bool iblockstate = true;
            bool iblockstate1 = true;

            for (int j1 = 127; j1 >= 0; --j1)
            {
                if (j1 < 127 - this->rand.nextInt(5) && j1 > this->rand.nextInt(5))
                {
                    bool iblockstate2 = primer.isSolid(k, j1, j);

                    if (iblockstate2)
                    {
                        if (iblockstate2)
                        {
                            if (i1 == -1)
                            {
                                if (l <= 0)
                                {
                                    iblockstate = false;
                                    iblockstate1 = true;
                                }
                                else if (j1 >= i - 4 && j1 <= i + 1)
                                {
                                    iblockstate = true;
                                    iblockstate1 = true;
                                }

                                if (j1 < i && !iblockstate)
                                {
                                    iblockstate = true; // LAVA
                                }

                                i1 = l;

                                if (j1 >= i - 1)
                                {
                                    primer.setBlock(k, j1, j, iblockstate);
                                }
                                else
                                {
                                    primer.setBlock(k, j1, j, iblockstate1);
                                }
                            }
                            else if (i1 > 0)
                            {
                                --i1;
                                primer.setBlock(k, j1, j, iblockstate1);
                            }
                        }
                    }
                    else
                    {
                        i1 = -1;
                    }
                }
                else
                {
                    primer.setBlock(k, j1, j, true); // BEDROCK
                }
            }
        }
    }
}

std::vector<double> ChunkGeneratorHell::getHeights(std::vector<double>* bufferIn, int xOffset, int yOffset, int zOffset, int xSize, int ySize, int zSize) {
    std::vector<double> buffer;
    if (bufferIn && !bufferIn->empty()) {
        buffer = std::move(*bufferIn);
    } else {
        buffer.resize(xSize * ySize * zSize);
    }

    this->noiseData4 = this->scaleNoise.generateNoiseOctaves(&this->noiseData4, xOffset, yOffset, zOffset, xSize, 1, zSize, 1.0, 0.0, 1.0);
    this->dr =         this->depthNoise.generateNoiseOctaves(&this->dr,         xOffset, yOffset, zOffset, xSize, 1, zSize, 100.0, 0.0, 100.0);
    this->pnr =      this->perlinNoise1.generateNoiseOctaves(&this->pnr,        xOffset, yOffset, zOffset, xSize, ySize,   zSize, 8.555150000000001, 34.2206, 8.555150000000001);
    this->ar =      this->lperlinNoise1.generateNoiseOctaves(&this->ar,         xOffset, yOffset, zOffset, xSize, ySize,   zSize, 684.412, 2053.236, 684.412);
    this->br =      this->lperlinNoise2.generateNoiseOctaves(&this->br,         xOffset, yOffset, zOffset, xSize, ySize,   zSize, 684.412, 2053.236, 684.412);
    int i = 0;
    // 256 allocated but only ySize used
    double adouble[256]{}; assert(ySize <= 256);

    for (int j = 0; j < ySize; ++j)
    {
        adouble[j] = std::cos((double)j * M_PI * 6.0 / (double)ySize) * 2.0;
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
                double d4 = adouble[k];
                double d5 = this->ar[i] / 512.0;
                double d6 = this->br[i] / 512.0;
                double d7 = (this->pnr[i] / 10.0 + 1.0) / 2.0;
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