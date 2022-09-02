#pragma once

#include <unordered_map>
#include <memory>
#include "Utils.h"
#include "Chunk.h"
#include "ParallelExecutor.h"

#if __has_include("absl/container/flat_hash_map.h")
#include "absl/container/flat_hash_map.h"
    #include "absl/container/node_hash_map.h"
#endif

template<typename K, typename V>
#if __has_include("absl/container/flat_hash_map.h")
    using map_t = absl::flat_hash_map<K, V>;
#else
using map_t = std::unordered_map<K, V>;
#endif

using cache_t = map_t<ChunkPos, std::unique_ptr<Chunk>>;

using ChunkGenExec = ParallelExecutor<3>;