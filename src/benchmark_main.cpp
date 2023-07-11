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
    for (auto _ : state) {
        Context ctx{seed};
        auto path = findPathFull(ctx, {0, 40, 0}, {(int)state.range(0), 64, (int)state.range(0)});
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

double randomDouble() {
    static std::mt19937 gen{std::random_device{}()};
    static auto distrib = [] { return std::uniform_real_distribution<double>(0, 10000)(gen); };
    return distrib();
}

void BM_generateNoiseOctaves(benchmark::State& state) {
    for (auto _ : state) {
        generator.lperlinNoise1.generateNoiseOctaves<5, 17, 5>(0, 0, 0, 684.412, 2053.236, 684.412);
    }
}

void BM_testDirectChunkInit(benchmark::State& state) {
    Context ctx{seed};
    Chunk source = ctx.generator.generateChunk(69, 420, ctx.executors[0]);
    bool input[BLOCKS_IN_CHUNK]{};
    auto* asX2 = (uint8_t*) &source.data;
    for (int i = 0; i < BLOCKS_IN_CHUNK / 8; i++) {
        auto x2 = asX2[i];
        for (int j = 0; j < 8; j++) {
            input[i * 8 + j] = ((x2 >> j) & 1);
        }
    }
    benchmark::DoNotOptimize(input);
    for (auto _ : state) {
        Chunk chunk;
        for (int y = 0; y < 128; y++) {
            for (int x = 0; x < 16; x++) {
                for (int z = 0; z < 16; z++) {
                    auto b = input[X2_INDEX[x/2][z/2][y/2] + bitIndex(x, y, z)];
                    chunk.setBlock(x, y, z, b);
                }
            }
        }
        benchmark::DoNotOptimize(chunk);
    }
}


void BM_testFastChunkInit(benchmark::State& state) {
    Context ctx{seed};
    Chunk source = ctx.generator.generateChunk(69, 420, ctx.executors[0]);
    uint8_t input[BLOCKS_IN_CHUNK]{};
    auto* asX2 = (uint8_t*) &source.data;
    for (int i = 0; i < BLOCKS_IN_CHUNK / 8; i++) {
        auto x2 = asX2[i];
        for (int j = 0; j < 8; j++) {
            input[i * 8 + j] = ((x2 >> (7 - j)) & 1);
        }
    }
    benchmark::DoNotOptimize(input);

    for (auto _ : state) {
        Chunk chunk;
        unpackedToPackedChunk(chunk, input);
        benchmark::DoNotOptimize(chunk);

        /*if (std::memcmp(&chunk, &source, sizeof(Chunk)) != 0) {
            std::cout << "different :sob:" << std::endl;
            exit(1);
        }*/
    }
}

//BENCHMARK(BM_testGetx2);
//BENCHMARK(BM_testOldGetx2);
//BENCHMARK(BM_testSetBlock);
//BENCHMARK(BM_testOldSetBlock);
//BENCHMARK(BM_testParallelExecutor);
//BENCHMARK(BM_testChunkisX16Empty);
//BENCHMARK(BM_testMaxNoAttribute);
//BENCHMARK(BM_testMax);
//BENCHMARK(BM_testPathFind)->Range(1000, 128000)->RangeMultiplier(2)->Unit(benchmark::kSecond);
//BENCHMARK(BM_testGenChunk)/*->Iterations(10)*/->Unit(benchmark::kMicrosecond);
//BENCHMARK(BM_generateNoiseOctaves)/*->Iterations(10)*/->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_testDirectChunkInit);
BENCHMARK(BM_testFastChunkInit);

BENCHMARK_MAIN();
