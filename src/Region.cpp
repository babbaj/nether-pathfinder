#include "Region.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/mman.h>
#include <unistd.h>
#endif


void freeRegion(Region* region) {
#ifdef _WIN32
    VirtualFree(region, 0, MEM_RELEASE);
#else
    munmap(region, sizeof(Region));
#endif
}

std::unique_ptr<Region, std::function<void(Region*)>> Region::allocate() {
    size_t size = sizeof(Region);

#ifdef _WIN32
    auto* region = static_cast<Region*>(VirtualAlloc(nullptr, size, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE));
    if (!region) {
        throw std::bad_alloc();
    }
#else
    auto* region = static_cast<Region*>(mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0));
        if (region == MAP_FAILED) {
            throw std::bad_alloc();
        }
#endif

    return std::unique_ptr<Region, std::function<void(Region*)>>(region, freeRegion);
}

Region& RegionCache::getRegion(RegionPos pos) {
    std::lock_guard<std::mutex> guard(this->lock);
    return getRegion_locked(pos);
}
Region& RegionCache::getRegion(int x, int z) {
    std::lock_guard<std::mutex> guard(this->lock);
    return getRegion_locked({x,z});
}

// Only call this directly if you are already holding the lock or are certain it is unnecessary
Region& RegionCache::getRegion_locked(RegionPos pos) {
    auto it = cache.find(pos);
    if (it == cache.end()) {
        auto [it, success] = cache.emplace(pos, Region::allocate());
        return *it->second.get();
    }
    return *it->second.get();
}