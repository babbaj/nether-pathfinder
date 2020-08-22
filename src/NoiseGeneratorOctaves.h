#pragma once

#include <array>

#include "NoiseGeneratorImproved.h"
#include "Random.h"
#include "ParallelExecutor.h"

template<size_t Octaves>
struct NoiseGeneratorOctaves;

struct NoiseGeneratorOctavesBase {
    const size_t octaves;
    NoiseGeneratorImproved* const generators;

    template<size_t Octaves>
    explicit NoiseGeneratorOctavesBase(NoiseGeneratorOctaves<Octaves>&);


private:
    [[deprecated]] void generateNoiseOctaves0(double* noiseArray, int xOffset, int yOffset, int zOffset, int xSize, int ySize, int zSize, double xScale, double yScale, double zScale) const;
};

template<size_t Octaves>
struct NoiseGeneratorOctaves : NoiseGeneratorOctavesBase {
    explicit NoiseGeneratorOctaves(Random& seed): NoiseGeneratorOctavesBase{*this}, generatorCollection(createGeneratorArray(seed)) {}

    friend NoiseGeneratorOctavesBase;

    template<int xSize, int ySize, int zSize>
    std::array<double, xSize * ySize * zSize> generateNoiseOctaves(int xOffset, int yOffset, int zOffset, double xScale, double yScale, double zScale) const;

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



inline int64_t lfloor(double value) {
    int64_t i = (int64_t)value;
    return value < (double)i ? i - 1L : i;
}

template<typename T, typename...>
struct firstType {
    using type = T;
};

template<typename T, typename... Us>
constexpr bool allSame = (... && std::is_same_v<T, Us>);

template<typename... Arr> requires (allSame<Arr...>)
auto sumArrays(const Arr&... arrays) {
    using array_type = typename firstType<Arr...>::type;
    constexpr size_t size = std::tuple_size_v<array_type>;

    array_type result;
    for (int i = 0; i < size; i++) {
        result[i] = (arrays[i] + ...);
    }
    return result;
}



template<size_t Octaves>
template<int xSize, int ySize, int zSize>
std::array<double, xSize * ySize * zSize> NoiseGeneratorOctaves<Octaves>::generateNoiseOctaves(int xOffset, int yOffset, int zOffset, double xScale, double yScale, double zScale) const {
    using return_type = std::array<double, xSize * ySize * zSize>;

    constexpr auto OCTAVES_PER_THREAD = 4;
    constexpr auto NumThreads = Octaves / OCTAVES_PER_THREAD;
    static thread_local ParallelExecutor<NumThreads> executor;

    static_assert(Octaves % OCTAVES_PER_THREAD == 0);

    // TODO: optimize captures?
    auto octaveFn = [&](const int index, const int iterations) -> return_type {
        return_type noiseArray{};
        for (int i = index; i < index + iterations; i++) {
            const double d3 = 1.0 / static_cast<double>(1u << i);

            double d0 = (double) xOffset * d3 * xScale;
            const double d1 = (double) yOffset * d3 * yScale;
            double d2 = (double) zOffset * d3 * zScale;
            int64_t k = lfloor(d0);
            int64_t l = lfloor(d2);
            d0 = d0 - (double) k;
            d2 = d2 - (double) l;
            k = k % 16777216L;
            l = l % 16777216L;
            d0 = d0 + (double) k;
            d2 = d2 + (double) l;

            this->generatorCollection[i].populateNoiseArray(&noiseArray[0], d0, d1, d2, xSize, ySize, zSize, xScale * d3, yScale * d3, zScale * d3, d3);
        }
        return noiseArray;
    };


    if constexpr (Octaves == OCTAVES_PER_THREAD) {
        return octaveFn(0, Octaves);
    } else {
        std::tuple resultTuple =
        [&]<size_t... I>(std::index_sequence<I...>) {
            return executor.compute([&] { return octaveFn(I * OCTAVES_PER_THREAD, OCTAVES_PER_THREAD); }...);
        }(std::make_index_sequence<NumThreads>{});

        return_type result = std::apply([](const auto&... arrays) { return sumArrays(arrays...); }, resultTuple);
        return result;
    }
}