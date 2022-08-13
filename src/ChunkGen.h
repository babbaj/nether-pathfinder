#pragma once

#include <unordered_map>
#include <memory>
#include "Utils.h"
#include "Chunk.h"

using cache_t = std::unordered_map<ChunkPos, std::unique_ptr<Chunk>>;
