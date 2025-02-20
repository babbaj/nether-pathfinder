#pragma once

#include <fstream>
#include <vector>
#include <optional>
#include <span>
#define WITH_GZFILEOP 1
#include <zlib-ng.h>


#include "ChunkGen.h"
#include "Chunk.h"
#include "Region.h"


void parseBaritoneRegion(RegionCache& cache, Dimension dim, RegionPos regionPos, gzFile data);
std::optional<gzFile> openRegionFile(std::string_view dir, Dimension dim,  RegionPos pos);
