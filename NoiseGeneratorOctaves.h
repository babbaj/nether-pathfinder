#pragma once

#include <array>
#include <vector>

#include "NoiseGeneratorImproved.h"
#include "Random.h"

template<size_t Octaves>
struct NoiseGeneratorOctaves;

struct NoiseGeneratorOctavesBase {
    const size_t octaves;
    NoiseGeneratorImproved* const generators;

    template<size_t Octaves>
    explicit NoiseGeneratorOctavesBase(NoiseGeneratorOctaves<Octaves>&);

    std::vector<double> generateNoiseOctaves(std::vector<double>* noiseArray, int xOffset, int yOffset, int zOffset, int xSize, int ySize, int zSize, double xScale, double yScale, double zScale);
};

template<size_t Octaves>
struct NoiseGeneratorOctaves : NoiseGeneratorOctavesBase {
    explicit NoiseGeneratorOctaves(Random& seed): NoiseGeneratorOctavesBase{*this}, generatorCollection(createGeneratorArray(seed)) {}

    friend NoiseGeneratorOctavesBase;
private:
    std::array<NoiseGeneratorImproved, Octaves> generatorCollection;

    static auto createGeneratorArray(Random& rand) {
        return [&rand]<size_t... Is>(std::index_sequence<Is...>) -> std::array<NoiseGeneratorImproved, Octaves> {
            return { ((void)Is, NoiseGeneratorImproved(rand))... };
        }(std::make_index_sequence<Octaves>{});
    }
};

template<size_t Octaves>
NoiseGeneratorOctavesBase::NoiseGeneratorOctavesBase(NoiseGeneratorOctaves<Octaves>& noiseGen):
    octaves(Octaves), generators(&noiseGen.generatorCollection[0]) {};