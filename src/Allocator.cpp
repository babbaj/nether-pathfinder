#include "Allocator.h"


#ifdef _WIN32
#include <windows.h>
#else
#include <sys/mman.h>
#endif

#ifdef _WIN32
void* alloc_pool() {
    return VirtualAlloc(nullptr, POOL_SIZE, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);
}

void free_pool(void* ptr) {
    VirtualFree(ptr, 0, MEM_RELEASE);
}
#else
void* alloc_pool() {
    return mmap(nullptr, POOL_SIZE, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
}

void free_pool(void* ptr) {
    munmap(ptr, POOL_SIZE);
}
#endif

