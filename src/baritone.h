#pragma once

#include <fstream>
#include <optional>
#include <span>

#include "ChunkGen.h"
#include "Chunk.h"


void parseBaritoneRegion(cache_t& cache, std::span<const char> data);
