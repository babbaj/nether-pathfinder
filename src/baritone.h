#pragma once

#include <fstream>
#include <vector>
#include <optional>
#include <span>
#define WITH_GZFILEOP 1
#include <zlib-ng.h>


#include "ChunkGen.h"
#include "Chunk.h"
#include "Allocator.h"

void parseBaritoneRegion(Allocator<Chunk>&, cache_t& cache, RegionPos regionPos, gzFile data, Dimension dim);
std::optional<std::tuple<gzFile, Dimension>> openRegionFile(std::string_view dir, RegionPos pos);
