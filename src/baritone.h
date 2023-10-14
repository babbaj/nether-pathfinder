#pragma once

#include <fstream>
#include <vector>
#include <optional>
#include <span>

#include "ChunkGen.h"
#include "Chunk.h"


void parseBaritoneRegion(cache_t& cache, RegionPos, std::span<const int8_t> data);
std::optional<std::vector<int8_t>> readRegionFile(std::string_view dir, RegionPos pos);
