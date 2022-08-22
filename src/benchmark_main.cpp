#include <random>
#include <array>

#include <benchmark/benchmark.h>

#include "ChunkGeneratorHell.h"
#include "PathFinder.h"
#include "Refiner.h"

constexpr auto seed = 146008555100680;
const auto generator = ChunkGeneratorHell::fromSeed(seed);
const auto chunkZero = [] {
    ChunkGenExec exec;
    return generator.generateChunk(1000, 0, exec);
}();

static void BM_testGetx2(benchmark::State& state) {
    const auto chunk = chunkZero;
    benchmark::DoNotOptimize(chunk);

    for (auto _ : state) {
        for (int x = 0; x < 16; x++) {
            for (int z = 0; z < 16; z++) {
                for (int y = 0; y < 128; y++) {
                    auto& x2 = chunk.getX2(x, y, z);
                    benchmark::DoNotOptimize(x2);
                }
            }
        }
    }
}


static void BM_testOldGetx2(benchmark::State& state) {
    const auto chunk = chunkZero;
    benchmark::DoNotOptimize(chunk);

    for (auto _ : state) {
        for (int x = 0; x < 16; x++) {
            for (int z = 0; z < 16; z++) {
                for (int y = 0; y < 128; y++) {
                    auto& x2 = chunk.getX2Old(x, y, z);
                    benchmark::DoNotOptimize(x2);
                }
            }
        }
    }
}


bool randomBool() {
    static std::mt19937 gen{std::random_device{}()};
    static auto distrib = [] { return std::uniform_int_distribution<>(0,1)(gen); };
    return distrib();
}


static void BM_testOldSetBlock(benchmark::State& state) {
    auto chunk = chunkZero;
    benchmark::DoNotOptimize(chunk);
    std::array<bool, 16 * 16 * 128> randArray{};
    for (auto& b : randArray) {
        b = randomBool();
    }

    for (auto _ : state) {
        for (int x = 0; x < 16; x++) {
            for (int z = 0; z < 16; z++) {
                for (int y = 0; y < 128; y++) {
                    chunk.setBlockOld(x, y, z, randArray[(y * 128) + (z * 16) + x]);
                    benchmark::ClobberMemory();
                }
            }
        }
        benchmark::DoNotOptimize(chunk);
    }
}
static void BM_testSetBlock(benchmark::State& state) {
    auto chunk = chunkZero;
    benchmark::DoNotOptimize(chunk);
    std::array<bool, 16 * 16 * 128> randArray{};
    for (auto& b : randArray) {
        b = randomBool();
    }

    for (auto _ : state) {
        for (int x = 0; x < 16; x++) {
            for (int z = 0; z < 16; z++) {
                for (int y = 0; y < 128; y++) {
                    chunk.setBlock(x, y, z, randArray[(y * 128) + (z * 16) + x]);
                }
            }
        }
        benchmark::DoNotOptimize(chunk);
    }
}

static void BM_testGenChunk(benchmark::State& state) {
    ChunkGenExec exec;

    for (auto _ : state) {
        for (int x = 0; x < 100; x++) {
            for (int z = 0; z < 100; z++) {
                auto chunk = generator.generateChunk(x, z, exec);
                benchmark::DoNotOptimize(chunk);
            }
        }
    }
}

static void BM_testPathFind(benchmark::State& state) {
    ChunkGenExec exec;

    for (auto _ : state) {
        cache_t cache;
        auto path = findPath({0, 40, 0}, {(int)state.range(0), 64, (int)state.range(0)}, generator, cache, false);
        benchmark::DoNotOptimize(path);
    }
}

static void BM_testParallelExecutor(benchmark::State& state) {
    ChunkGenExec exec;

    for (auto _ : state) {
        auto result = exec.compute([] { return 1;}, [] { return 2;}, [] { return 3;});
        benchmark::DoNotOptimize(result);
    }
}

static void BM_testChunkisX16Empty(benchmark::State& state) {
    std::vector<Chunk> chunks;
    ChunkGenExec exec;
    for (int i = 0; i < 10000; i++) {
        chunks.emplace_back(generator.generateChunk(i * 16, i * 16, exec));
    }

    for (auto _ : state) {
        for (const auto& chunk : chunks) {
            benchmark::DoNotOptimize(isEmpty(chunk.getX16(64)));
            //benchmark::DoNotOptimize(chunk.isEmpty<Size::X16>(0, 64, 0));
        }
    }
}

BENCHMARK(BM_testGetx2);
BENCHMARK(BM_testOldGetx2);
BENCHMARK(BM_testSetBlock);
BENCHMARK(BM_testOldSetBlock);
BENCHMARK(BM_testParallelExecutor);
BENCHMARK(BM_testChunkisX16Empty);
//BENCHMARK(BM_testPathFind)->Range(1000, 128000)->RangeMultiplier(2)->Unit(benchmark::kSecond);
//BENCHMARK(BM_testGenChunk)->Iterations(10)->Complexity()->Unit(benchmark::kMicrosecond);

BENCHMARK_MAIN();