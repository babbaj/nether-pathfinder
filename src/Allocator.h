#pragma once
#include <vector>
#include <utility>
#include <unordered_map>
#include <cstdint>
#include <memory>
#include <cstring>
#include <iostream>
#include <span>
#include <algorithm>

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

void add_pool_global(void*);
void remove_pools_global(std::span<void*> pools);
void init_page_handler();

size_t getPageSize();

// this doesn't really need to be generic it's only ever gonna be used for chunks lol
template<typename T>
struct Allocator {
    virtual ~Allocator() = default;

    virtual T* allocate() {
        return new T{};
    }

    virtual void free(T* ptr) {
        delete ptr;
    }

    virtual bool auto_frees_on_destroy() {
        return false;
    }
};

template<typename T> requires (sizeof(T) % 4096 == 0)
struct PageAllocator : Allocator<T> {
    // pointer to elements -> index in pools vector
    std::unordered_map<uintptr_t, size_t> poolByPointer;
    std::vector<Pool<T>> pools;

    explicit PageAllocator() {
        init_page_handler();
    }
    PageAllocator(const PageAllocator&) = delete;
    PageAllocator(PageAllocator&& other) = default;
    ~PageAllocator() override {
        std::vector<void*> toRemove;
        for (auto& p : pools) {
            if (p.originalPointer) {
                free_pool(p.originalPointer);
            }
            toRemove.push_back(p.elements);
        }
        remove_pools_global(toRemove);
    }

    template<typename... Args>
    T* allocate_init(Args&&... args) {
        void* ptr = allocate0();
        return new (ptr) T(std::forward<Args>(args)...);
    }

    T* allocate() override {
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
        add_pool_global(pool.elements);
        auto base = reinterpret_cast<uintptr_t>(elems) & POOL_PTR_MASK;
        auto [it, inserted] = poolByPointer.emplace(base, pools.size() - 1);
        if (!inserted) {
            std::cout << "pool at " << base << " already exists in poolByPointer" << std::endl;
            std::terminate();
        }
        return &reinterpret_cast<Value<T>*>(elems)[0];
    }

    void free(T* ptr) override {
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
                remove_pools_global({(void**) &pool.elements, 1});
                poolByPointer.erase(it);
                pool.elements = nullptr;
                pool.originalPointer = nullptr;
            }
        } else {
            puts("[nether-pathfinder] no pool associated with this pointer");
            std::terminate();
        }
    }

    bool auto_frees_on_destroy() override {
        return std::is_trivially_destructible_v<T>;
    }
};
