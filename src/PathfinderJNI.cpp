#include <algorithm>

#include <jni.h>

#include "ChunkGeneratorHell.h"
#include "PathFinder.h"
#include "Refiner.h"

jlong packBlockPos(const BlockPos& pos) {
    constexpr jint NUM_X_BITS = 26;//1 + MathHelper.log2(MathHelper.smallestEncompassingPowerOfTwo(30000000));
    constexpr jint NUM_Z_BITS = NUM_X_BITS;
    constexpr jint NUM_Y_BITS = 64 - NUM_X_BITS - NUM_Z_BITS;
    constexpr jint Y_SHIFT = 0 + NUM_Z_BITS;
    constexpr jint X_SHIFT = Y_SHIFT + NUM_Y_BITS;
    constexpr jlong X_MASK = (1L << NUM_X_BITS) - 1L;
    constexpr jlong Y_MASK = (1L << NUM_Y_BITS) - 1L;
    constexpr jlong Z_MASK = (1L << NUM_Z_BITS) - 1L;

    return ((jlong)pos.x & X_MASK) << X_SHIFT | ((jlong)pos.y & Y_MASK) << Y_SHIFT | ((jlong)pos.z & Z_MASK) << 0;
}

bool inBounds(int y) {
    return y >= 0 && y < 128;
}

extern "C" {
    JNIEXPORT jlongArray JNICALL Java_com_babbaj_pathfinder_PathFinder_pathFind(JNIEnv* env, jclass clazz, jlong seed, jboolean fine, jboolean raytrace, jint x1, jint y1, jint z1, jint x2, jint y2, jint z2) {
        if (!inBounds(y1) || !inBounds(y2)) {
            return nullptr; // TODO: throw exception
        }
        cancelFlag.clear();
        auto generator = ChunkGeneratorHell::fromSeed(seed);
        std::optional<Path> path = findPath({x1, y1, z1}, {x2, y2, z2}, generator, fine);

        if (path) {
            const std::vector<BlockPos>& ugly = path->blocks;
            std::vector<jlong> packed;
            if (raytrace) {
                auto blocks = refine(ugly, generator, path->chunkCache);
                packed.reserve(blocks.size());
                std::transform(blocks.begin(), blocks.end(), std::back_inserter(packed), packBlockPos);
            } else {
                packed.reserve(ugly.size());
                std::transform(ugly.begin(), ugly.end(), std::back_inserter(packed), packBlockPos);
            }

            const auto len = packed.size();
            jlongArray array = env->NewLongArray(len);
            env->SetLongArrayRegion(array, 0, len, packed.data());

            return array;
        } else {
            return nullptr;
        }
    }

    JNIEXPORT jboolean JNICALL Java_com_babbaj_pathfinder_PathFinder_cancel(JNIEnv* env, jclass clazz) {
        return cancelFlag.test_and_set();
    }
}