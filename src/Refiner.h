#pragma once

#include <vector>
#include "Utils.h"
#include "ChunkGeneratorHell.h"
#include "ChunkGen.h"

std::vector<BlockPos> refine(const std::vector<BlockPos>& path, const ChunkGeneratorHell& gen, cache_t& cache);