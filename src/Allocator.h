#pragma once
#include <vector>
#include <utility>
#include <unordered_map>
#include <cstdint>
#include <memory>
#include <cstring>

constexpr size_t POOL_SIZE = 4096 * 2048; // 8 MiB (682 chunks)
constexpr uintptr_t POOL_PTR_MASK = ~(POOL_SIZE - 1);

template<typename T>
struct Value {
    alignas(T) char buf[sizeof(T)];

    T* as() {
        return std::launder((T*) buf);
    }
};

template<typename T>
struct Pool {
    size_t next;
    size_t frees;
    Value<T>* elements;
};

template<typename T>
constexpr size_t pool_max_elements() {
    return POOL_SIZE / sizeof(T);
}

void* alloc_pool();
void free_pool(void* ptr);

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
        return (T*) allocate0();
    }

    void* allocate0() {
        if (!pools.empty()) {
            Pool<T>& p = pools.back();
            if (p.next < pool_max_elements<T>()) {
                auto out = &p.elements[p.next];
                p.next++;
                return out->as();
            }
        }

        auto* elems = (Value<T>*) alloc_pool();
        auto pool = Pool<T> {
            0,
            0,
            elems
        };
        pools.push_back(pool);
        return elems[0].as();
    }

    void free(T* ptr) {
        std::destroy_at(ptr);
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
                free_pool(pool.elements);
                pool.elements = nullptr;
            }
        } else {
            puts("[nether-pathfinder] no pool associated with this pointer");
            std::terminate();
        }
    }
};
