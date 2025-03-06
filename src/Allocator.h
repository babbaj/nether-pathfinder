#pragma once
#include <vector>
#include <utility>
#include <unordered_map>
#include <cstdint>
#include <memory>
#include <cstring>
#include <iostream>

constexpr size_t POOL_SIZE = 4096 * 2048; // 8 MiB (682 chunks)
constexpr uintptr_t POOL_PTR_MASK = ~(POOL_SIZE - 1);

template<typename T>
struct Value {
    alignas(T) char buf[sizeof(T)];
};

template<typename T>
struct Pool {
    size_t next;
    size_t frees;
    Value<T>* elements;
    void* originalPointer;
};

template<typename T>
constexpr size_t pool_max_elements() {
    return POOL_SIZE / sizeof(T);
}

std::pair<void*, void*> alloc_pool();
void free_pool(void* ptr);
void decommit(void* ptr, size_t len);

template<typename T>
struct Allocator {
    // pointer to elements -> index in pools vector
    std::unordered_map<uintptr_t, size_t> poolByPointer;
    std::vector<Pool<T>> pools;

    Allocator() = default;
    Allocator(const Allocator&) = delete;
    Allocator(Allocator&& other) = default;
    ~Allocator() {
        for (auto& p : pools) {
            if (p.elements) {
                free_pool(p.elements);
            }
        }
    }

    template<typename... Args>
    T* allocate_init(Args&&... args) {
        void* ptr = allocate0();
        return new (ptr) T(std::forward<Args>(args)...);
    }

    T* allocate() requires std::is_trivial_v<T> {
        void* ptr = allocate0();
        return new (ptr) T;
    }

    void* allocate0() {
        if (!pools.empty()) {
            Pool<T>& p = pools.back();
            if (p.next < pool_max_elements<T>()) {
                auto out = &p.elements[p.next];
                p.next++;
                return out;
            }
        }

        auto [elems, rawPointer] = alloc_pool();
        auto pool = Pool<T> {
            1,
            0,
            reinterpret_cast<Value<T>*>(elems),
            rawPointer
        };
        pools.push_back(pool);
        auto base = reinterpret_cast<uintptr_t>(elems) & POOL_PTR_MASK;
        auto [it, inserted] = poolByPointer.emplace(base, pools.size() - 1);
        if (!inserted) {
            std::cout << "pool at " << base << " already exists in poolByPointer" << std::endl;
            std::terminate();
        }
        return &reinterpret_cast<Value<T>*>(elems)[0];
    }

    void free(T* ptr) {
        std::destroy_at(ptr);
        decommit(ptr, sizeof(T));
        auto upperBits = reinterpret_cast<uintptr_t>(ptr) & POOL_PTR_MASK;
        auto it = poolByPointer.find(upperBits);
        if (it != poolByPointer.end()) {
            Pool<T>& pool = pools[it->second];
            if (!pool.elements || pool.frees == pool_max_elements<T>()) {
                puts("[nether-pathfinder] this pool has already been freed");
                std::terminate();
            }
            pool.frees++;
            if (pool.frees == pool_max_elements<T>()) {
                free_pool(pool.originalPointer);
                poolByPointer.erase(it);
                pool.elements = nullptr;
            }
        } else {
            puts("[nether-pathfinder] no pool associated with this pointer");
            std::terminate();
        }
    }
};
