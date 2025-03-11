#include "Allocator.h"

#ifdef _WIN32
#include <windows.h>
#include <atomic>
#else
#include <sys/mman.h>
#include <unistd.h>
#endif

void* alignToPoolSize(void* base) {
    return reinterpret_cast<void*>((reinterpret_cast<uintptr_t>(base) + POOL_SIZE - 1) & ~(POOL_SIZE - 1));
}

size_t getPageSize() {
#ifdef _WIN32
    SYSTEM_INFO si;
    GetSystemInfo(&si);
    return si.dwPageSize;
#else
    return sysconf(_SC_PAGESIZE);
#endif
}

#ifdef _WIN32
// in case multiple threads want to change the global pool list at the same time but this does not need to be held to read it
std::mutex pool_mutate_mutex;
std::shared_ptr<std::vector<void*>> all_pools = std::make_shared<std::vector<void*>>();

bool is_pool_pointer(void* ptr) {
    auto pools = all_pools;
    return std::find(pools->rbegin(), pools->rend(), (void*) (((uintptr_t) ptr) & POOL_PTR_MASK)) != pools->rend();
}

LONG page_handler(PEXCEPTION_POINTERS ptr) {
    PEXCEPTION_RECORD record = ptr->ExceptionRecord;
    if (record->ExceptionCode == EXCEPTION_ACCESS_VIOLATION) {
        void* fault_address = (void*)(record->ExceptionInformation[1]);
        if (is_pool_pointer(fault_address)) {
            void* page_aligned = (void*)((uintptr_t)fault_address & ~(4096 - 1));
            if (VirtualAlloc(page_aligned, 4096, MEM_COMMIT, PAGE_READWRITE)) {
                return EXCEPTION_CONTINUE_EXECUTION;
            }
        }
    }
    return EXCEPTION_CONTINUE_SEARCH;
}

std::atomic_flag handler_added;
void init_page_handler() {
    if (!handler_added.test_and_set()) {
        AddVectoredExceptionHandler(0, PVECTORED_EXCEPTION_HANDLER(page_handler));
    }
}

void add_pool_global(void* pool) {
    std::lock_guard lock{pool_mutate_mutex};
    std::vector<void*> new_pools;
    new_pools.reserve(all_pools->size() + 1);
    std::copy(all_pools->begin(), all_pools->end(), std::back_inserter(new_pools));
    new_pools.push_back(pool);
    all_pools = std::make_shared<std::vector<void*>>(std::move(new_pools));
}

void remove_pools_global(std::span<void*> pools) {
    std::lock_guard lock{pool_mutate_mutex};
    auto& old_pools = *all_pools;
    std::vector<void*> new_pools;
    new_pools.reserve(old_pools.size());
    for (auto p : old_pools) {
        if (std::find(pools.begin(), pools.end(), p) == pools.end()) {
            new_pools.push_back(p);
        }
    }
    all_pools = std::make_shared<std::vector<void*>>(new_pools);
}

std::pair<void*, void*> alloc_pool() {
    void* original = VirtualAlloc(nullptr, POOL_SIZE * 2, MEM_RESERVE, PAGE_NOACCESS);
    if (!original) {
        return {nullptr, nullptr};
    }

    void* aligned = alignToPoolSize(original);
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

// not needed on linux
void add_pool_global(void*) {}
void remove_pools_global(std::span<void*>) {}
void init_page_handler() {}
#endif
