#pragma once

#include <vector>
#include <variant>

#include "Utils.h"
#include "ChunkGeneratorHell.h"
#include "ChunkGen.h"

enum class Plane {
    YZ,
    XZ,
    XY
};

// we intersected with a solid block
struct Hit {
    Vec3 where;
};
// line of sight confirmed
struct Finished {};
// we got to the other side of an x16 without hitting anything
struct Miss {
    Plane exitPlane;
};
using RaytraceResult = std::variant<Hit, Finished, Miss>;

RaytraceResult raytrace(const Vec3& from, const Vec3& to, bool airIfFakeChunk, const ChunkGeneratorHell& gen, ChunkGenExec& exec, cache_t& cache);

std::vector<BlockPos> refine(const std::vector<BlockPos>& path, const ChunkGeneratorHell& gen, cache_t& cache);