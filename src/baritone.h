#pragma once

#include <fstream>
#include <vector>
#include <optional>
#include <span>


#include "ChunkGen.h"
#include "Chunk.h"


void parseBaritoneRegion(cache_t& cache, RegionPos, std::span<const int8_t> data);
jbyteArray readRegionFileChunks(JNIEnv* env, std::string_view dir, RegionPos pos);
