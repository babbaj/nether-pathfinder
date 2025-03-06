#include "Allocator.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/mman.h>
#endif

void* alignToPoolSize(void* base) {
    return reinterpret_cast<void*>((reinterpret_cast<uintptr_t>(base) + POOL_SIZE - 1) & ~(POOL_SIZE - 1));
}

#ifdef _WIN32
std::pair<void*, void*> alloc_pool() {
    void* original = VirtualAlloc(nullptr, POOL_SIZE * 2, MEM_RESERVE, PAGE_READWRITE);
    if (!original) {
        return {nullptr, nullptr};
    }

    void* aligned = alignToPoolSize(original);
    void* committed = VirtualAlloc(aligned, POOL_SIZE, MEM_COMMIT, PAGE_READWRITE);
    if (!committed) {
        VirtualFree(original, 0, MEM_RELEASE);
        return {nullptr, nullptr};
    }

    return {aligned, original};
}

void free_pool(void* ptr) {
    VirtualFree(ptr, 0, MEM_RELEASE);
}

void decommit(void* ptr, size_t len) {
    // can probably just MEM_RESET
    VirtualFree(ptr, len, MEM_DECOMMIT);
}
#else
std::pair<void*, void*> alloc_pool() {
    void* original = mmap(nullptr, POOL_SIZE * 2, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (original == MAP_FAILED) {
        return {nullptr, nullptr};
    }
    void* aligned = alignToPoolSize(original);
    return {aligned, original};
}

void free_pool(void* ptr) {
    munmap(ptr, POOL_SIZE * 2);
}

void decommit(void* ptr, size_t len) {
    madvise(ptr, len, MADV_DONTNEED);
    //mmap(ptr, len, PROT_NONE, MAP_FIXED, -1, 0);
}
#endif
