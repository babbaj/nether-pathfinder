#pragma once

#include <fstream>
#include <vector>
#include <optional>
#include <span>
#define WITH_GZFILEOP 1
#include <zlib-ng.h>


#include "ChunkGen.h"
#include "Chunk.h"


void parseBaritoneRegion(cache_t& cache, RegionPos, gzFile data);
std::optional<gzFile> openRegionFile(std::string_view dir, RegionPos pos);
