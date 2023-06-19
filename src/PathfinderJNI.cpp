#include <algorithm>
#include <bit>

#include <jni.h>

#include "ChunkGeneratorHell.h"
#include "PathFinder.h"
#include "Refiner.h"

constexpr jint NUM_X_BITS = 26;//1 + MathHelper.log2(MathHelper.smallestEncompassingPowerOfTwo(30000000));
constexpr jint NUM_Z_BITS = NUM_X_BITS;
constexpr jint NUM_Y_BITS = 64 - NUM_X_BITS - NUM_Z_BITS;
constexpr jint Y_SHIFT = 0 + NUM_Z_BITS;
constexpr jint X_SHIFT = Y_SHIFT + NUM_Y_BITS;
constexpr jlong X_MASK = (1L << NUM_X_BITS) - 1L;
constexpr jlong Y_MASK = (1L << NUM_Y_BITS) - 1L;
constexpr jlong Z_MASK = (1L << NUM_Z_BITS) - 1L;

jlong packBlockPos(const BlockPos& pos) {
    return ((jlong)pos.x & X_MASK) << X_SHIFT | ((jlong)pos.y & Y_MASK) << Y_SHIFT | ((jlong)pos.z & Z_MASK) << 0;
}

BlockPos unpackBlockPos(jlong packed) {
    return {
            (jint) ((packed >> X_SHIFT) & X_MASK),
            (jint) ((packed >> Y_SHIFT) & Y_MASK),
            (jint) ((packed) & Z_MASK)
    };
}

bool inBounds(int y) {
    return y >= 0 && y < 128;
}

void throwException(JNIEnv* env, const char* msg) {
    jclass exception = env->FindClass("java/lang/IllegalArgumentException");
    env->ThrowNew(exception, msg);
}

struct State {
    jclass      pathSegmentClass{};
    jmethodID   pathSegmentCtor{};
} state;

extern "C" {
#if defined(_WIN32)
#define EXPORT __declspec(dllexport) JNIEXPORT
#else
#define EXPORT JNIEXPORT
#endif

    EXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void* reserved) {
        JNIEnv* env;
        if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_8) != JNI_OK) {
            return JNI_EVERSION;
        }

        jclass cls = env->FindClass("dev/babbaj/pathfinder/PathSegment");
        if (!cls) return JNI_ERR;

        jmethodID ctor = env->GetMethodID(cls, "<init>", "(Z[J)V");
        if (!ctor) return JNI_ERR;

        state = {
            .pathSegmentClass = (jclass) env->NewGlobalRef(cls),
            .pathSegmentCtor  = ctor
        };

        return JNI_VERSION_1_8;
    }

    EXPORT void JNICALL JNI_OnUnload(JavaVM* vm, void* reserved) {
        JNIEnv* env;
        if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_8) != JNI_OK) {
            return;
        }

        if (state.pathSegmentClass) {
            env->DeleteGlobalRef(state.pathSegmentClass);
        }
    }

    EXPORT jlong JNICALL Java_dev_babbaj_pathfinder_NetherPathfinder_newContext(JNIEnv* env, jclass, jlong seed) {
        return reinterpret_cast<jlong>(new Context{seed});
    }

    EXPORT void JNICALL Java_dev_babbaj_pathfinder_NetherPathfinder_freeContext(JNIEnv* env, jclass, Context* ctx) {
        delete ctx;
    }

    EXPORT void JNICALL Java_dev_babbaj_pathfinder_NetherPathfinder_insertChunkData(JNIEnv* env, jclass, Context* ctx, jint chunkX, jint chunkZ, jbooleanArray input) {
        jboolean isCopy{};
        constexpr auto blocksInChunk = 16 * 16 * 128;
        if (auto len = env->GetArrayLength(input); len != blocksInChunk) {
            throwException(env, "input is not 32768 elements");
            return;
        }
        jboolean* data = env->GetBooleanArrayElements(input, &isCopy);
        auto chunk_ptr = std::make_unique<Chunk>();
        for (int i = 0; i < blocksInChunk; i++) {
            auto x = (i >> 0) & 0xF;
            auto z = (i >> 4) & 0xF;
            auto y = (i >> 8) & 0x7F;
            chunk_ptr->setBlock(x, y, z, data[i]);
        }
        chunk_ptr->isFromJava = true;
        env->ReleaseBooleanArrayElements(input, data, JNI_ABORT);

        ctx->chunkCache.insert_or_assign(ChunkPos{chunkX, chunkZ}, std::move(chunk_ptr));
    }

    EXPORT jobject JNICALL Java_dev_babbaj_pathfinder_NetherPathfinder_pathFind(JNIEnv* env, jclass, Context* ctx, jint x1, jint y1, jint z1, jint x2, jint y2, jint z2, jboolean x4Min, jint timeoutMs) {
        if (!inBounds(y1) || !inBounds(y2)) {
            throwException(env, "Invalid y1 or y2");
            return nullptr;
        }
        ctx->cancelFlag.clear();
        std::optional<Path> path = findPathSegment(*ctx, {x1, y1, z1}, {x2, y2, z2}, x4Min, timeoutMs);
        if (!path) return nullptr;

        std::vector<jlong> packed;
        packed.reserve(path->blocks.size());
        std::transform(path->blocks.begin(), path->blocks.end(), std::back_inserter(packed), packBlockPos);

        const auto len = (jint) packed.size();
        jlongArray array = env->NewLongArray(len);
        env->SetLongArrayRegion(array, 0, len, packed.data());

        jobject object = env->NewObject(
            state.pathSegmentClass,
            state.pathSegmentCtor,
            // args
            path->type == Path::Type::FINISHED,
            array
        );
        return object;
    }

    EXPORT jboolean JNICALL Java_dev_babbaj_pathfinder_NetherPathfinder_cancel(JNIEnv* env, jclass clazz, Context* ctx) {
        return ctx->cancelFlag.test_and_set();
    }

    EXPORT jlongArray JNICALL Java_dev_babbaj_pathfinder_NetherPathfinder_refinePath(JNIEnv* env, jclass, Context* ctx, jlongArray packed) {
        auto packedLen = env->GetArrayLength(packed);
        jboolean isCopy{};
        jlong* pointer = env->GetLongArrayElements(packed, &isCopy);
        std::vector<BlockPos> unpacked;
        unpacked.reserve(packedLen);
        std::transform(pointer, pointer + packedLen, std::back_inserter(unpacked), unpackBlockPos);
        env->ReleaseLongArrayElements(packed, pointer, JNI_ABORT);

        auto refined = refine(unpacked, ctx->generator, ctx->chunkCache);
        jlongArray out = env->NewLongArray((jint) refined.size());
        jlong* outPointer = env->GetLongArrayElements(out, &isCopy);
        for (int i = 0; i < refined.size(); i++) {
            outPointer[i] = packBlockPos(refined[i]);
        }
        env->ReleaseLongArrayElements(out, outPointer, 0);
        return out;
    }

    EXPORT void JNICALL Java_dev_babbaj_pathfinder_NetherPathfinder_raytrace0(JNIEnv* env, jclass, Context* ctx, jboolean assumeFakeChunksAreAir, jint inputs, jdoubleArray startArr, jdoubleArray endArr, jbooleanArray hitsOut, jdoubleArray hitPosOut) {
        jboolean isCopy{};
        jdouble* startPtr = env->GetDoubleArrayElements(startArr, &isCopy);
        jdouble * endPtr = env->GetDoubleArrayElements(endArr, &isCopy);
        jboolean* hitsOutPtr = env->GetBooleanArrayElements(hitsOut, &isCopy);
        jdouble* hitPosOutPtr = hitPosOut != nullptr ? env->GetDoubleArrayElements(hitPosOut, &isCopy) : nullptr;
        for (int i = 0; i < inputs; i++) {
            auto& start = reinterpret_cast<const Vec3*>(startPtr)[i];
            auto& end = reinterpret_cast<const Vec3*>(endPtr)[i];
            const std::variant result = raytrace(start, end, assumeFakeChunksAreAir, ctx->generator, ctx->executors[0], ctx->chunkCache);
            auto* hit = std::get_if<Hit>(&result);
            if (hit) {
                hitsOutPtr[i] = true;
                if (hitPosOutPtr != nullptr) {
                    hitPosOutPtr[i * 3] = hit->where.x;
                    hitPosOutPtr[i * 3 + 1] = hit->where.y;
                    hitPosOutPtr[i * 3 + 2] = hit->where.z;
                }
            }
        }
        env->ReleaseDoubleArrayElements(startArr, startPtr, JNI_ABORT);
        env->ReleaseDoubleArrayElements(endArr, endPtr, JNI_ABORT);
        env->ReleaseBooleanArrayElements(hitsOut, hitsOutPtr, 0);
        if (hitPosOut) {
            env->ReleaseDoubleArrayElements(hitPosOut, hitPosOutPtr, 0);
        }
    }
}
