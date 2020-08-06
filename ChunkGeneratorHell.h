#pragma once

#include <vector>

#include "NoiseGeneratorOctaves.h"
#include "ChunkPrimer.h"

struct ChunkGeneratorHell {
private:
    // must be declared first
    Random rand;

    // not sure if the size changes
    std::vector<double> depthBuffer;
    std::vector<double> buffer;

    //these must be declared in the right order
    NoiseGeneratorOctaves<16> lperlinNoise1;
    NoiseGeneratorOctaves<16> lperlinNoise2;
    NoiseGeneratorOctaves<8>  perlinNoise1;

    NoiseGeneratorOctaves<4> slowsandGravelNoiseGen;
    NoiseGeneratorOctaves<4> netherrackExculsivityNoiseGen;

    NoiseGeneratorOctaves<10> scaleNoise;
    NoiseGeneratorOctaves<16> depthNoise;

    // these can probably be optimized later
    std::vector<double> pnr;
    std::vector<double> ar;
    std::vector<double> br;
    std::vector<double> noiseData4;
    std::vector<double> dr;

    void prepareHeights(int p_185936_1_, int p_185936_2_, ChunkPrimer& primer);

    void buildSurfaces(int x, int z, ChunkPrimer& primer);

    // buffer may be null
    std::vector<double> getHeights(std::vector<double>* buffer, int xOffset, int yOffset, int zOffset, int xSize, int ySize, int zSize);
public:
    explicit ChunkGeneratorHell(uint64_t seed)
    : rand(seed),
    lperlinNoise1(rand),
    lperlinNoise2(rand),
    perlinNoise1(rand),
    slowsandGravelNoiseGen(rand),
    netherrackExculsivityNoiseGen(rand),
      scaleNoise(rand),
      depthNoise(rand) {}

    ChunkPrimer generateChunk(int x, int z);
};