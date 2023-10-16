#include <filesystem>
#include <iostream>
#include <jni.h>

#include "baritone.h"


int8_t byte(std::span<const int8_t> span) {
    if (span.empty()) [[unlikely]] {
        throw std::range_error{"failed to read byte"};
    }
    return span[0];
}

int positionIndex(int x, int y, int z) {
    return (x << 1) | (z << 5) | (y << 9);
}

int8_t get2Bits(size_t i, std::span<const int8_t> data) {
    int8_t byte = data[i / 8];
    return (byte >> (6 - (i % 8))) & 0b11;
}

void parseAndInsertChunk(cache_t& cache, int chunkX, int chunkZ, std::span<const int8_t> data) {
    auto [it, inserted] = cache.try_emplace(ChunkPos{chunkX, chunkZ});
    if (inserted) {
        auto chunk = std::make_unique<Chunk>();
        chunk->isFromJava = true;
        for (int y = 0; y < 128; y++) {
            for (int z = 0; z < 16; z++) {
                for (int x = 0; x < 16; x++) {
                    auto idx = positionIndex(x, y, z);
                    auto bits = get2Bits(idx, data);
                    chunk->setBlock(x, y, z, bits != 0);
                }
            }
        }
        it->second = std::move(chunk);
    }
}

void parseBaritoneRegion(cache_t& cache, RegionPos regionPos, std::span<const int8_t> data) {
    for (int x = 0; x < 32; x++) {
        for (int z = 0; z < 32; z++) {
            const int8_t present = byte(data);
            data = data.subspan<1>();
            if (present == 1) {
                // DimensionType.height for the nether is 256 and that is what is used for serializing to disk
                constexpr auto chunkSizeBytes = (2 * 16 * 16 * 256) / 8;
                if (data.size() < chunkSizeBytes) {
                    std::cerr << "not enough bytes for chunk" << std::endl;
                    exit(69);
                }
                parseAndInsertChunk(cache, x + 32 * regionPos.x, z + 32 * regionPos.z, data.subspan(0, chunkSizeBytes));
                data = data.subspan<chunkSizeBytes>();
            }
        }
    }

}

jbyteArray callReadRegionChunks(JNIEnv* env, const char* file) {
    thread_local static jclass clazz = env->FindClass("dev/babbaj/pathfinder/NetherPathfinder");
    if (!clazz) {
        std::cerr << "No NetherPathfinder class wtf" << std::endl;
        exit(69);
    }
    thread_local static jmethodID method = env->GetStaticMethodID(clazz, "readRegionChunks", "(Ljava/lang/String;)[B");
    if (!method) {
        std::cerr << "No readRegionChunks wtf" << std::endl;
        exit(69);
    }
    jstring fileStr = env->NewStringUTF(file);
    auto bytes = (jbyteArray) env->CallStaticObjectMethod(clazz, method, fileStr);
    env->DeleteLocalRef(fileStr);

    if (env->ExceptionCheck()) {
        env->ExceptionDescribe();
        env->ExceptionClear();
    }
    return bytes;
}

jbyteArray readRegionFileChunks(JNIEnv* env, std::string_view dir, RegionPos pos) {
    auto fileName = std::string{"r."} + std::to_string(pos.x) + "." + std::to_string(pos.z) + ".bcr";
    auto path = std::filesystem::path{dir} / fileName;

    return callReadRegionChunks(env, path.string().c_str());
}
