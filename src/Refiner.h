#pragma once

#include <vector>
#include <variant>

#include "PathFinder.h"
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

RaytraceResult raytrace(Context& ctx, const Vec3& from, const Vec3& to, bool airIfFakeChunk);

std::vector<BlockPos> refine(Context& ctx, const std::vector<BlockPos>& path);
