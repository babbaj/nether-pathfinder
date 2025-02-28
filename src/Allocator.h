#pragma once
#include <vector>
#include <utility>
#include <unordered_map>
#include <cstdint>
#include <memory>
#include <cstring>

constexpr size_t POOL_SIZE = 4096 * 2048; // 8 MiB (682 chunks)

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
    Value<T>* elements;
};

template<typename T>
constexpr size_t pool_max_elements() {
    return POOL_SIZE / sizeof(T);
}

template<typename T>
struct Allocator {
    std::vector<void*> freeList;
    std::vector<Pool<T>> pools;

    Allocator() = default;
    Allocator(const Allocator&) = delete;
    Allocator(Allocator&& other) = default;
    ~Allocator() {
        for (auto& p : pools) {
            operator delete[] (p.elements, std::align_val_t{4096});
        }
    }

    template<typename... Args>
    T* allocate_init(Args&&... args) {
        void* ptr = allocate0();
        return new (ptr) T(std::forward<Args>(args)...);
    }

    T* allocate(bool reuse) requires std::is_trivial_v<T> {
        return (T*) allocate0(reuse);
    }

    void* allocate0(bool reuse) {
        if (reuse && !freeList.empty()) {
            auto out = freeList.back();
            freeList.pop_back();
            return out;
        }
        if (!pools.empty()) {
            Pool<T>& p = pools.back();
            if (p.next < pool_max_elements<T>()) {
                auto out = &p.elements[p.next];
                p.next++;
                return out->as();
            }
        }

        auto* elems = new (std::align_val_t{4096}) Value<T>[pool_max_elements<T>()];
        auto pool = Pool<T> {
            0,
            elems
        };
        pools.push_back(pool);
        return elems[0].as();
    }

    void free(T* ptr) {
        std::destroy_at(ptr);
        freeList.push_back(ptr);
    }
};
