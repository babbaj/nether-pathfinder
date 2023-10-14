#pragma once

#include <cstdio>
#include <thread>
#include <functional>
#include <array>
#include <tuple>
#include <condition_variable>
#include <mutex>
#include <type_traits>


struct Worker {
    std::condition_variable& condition;
    std::mutex& mutex;
    std::function<void()> task;
    std::thread thread;
    std::atomic_bool stopRequest;

    Worker(std::condition_variable& cv, std::mutex& m): condition(cv), mutex(m), thread([this] {
        while (true) {
            std::unique_lock lock(this->mutex);
            condition.wait(lock, [this] { return stopRequest.load(std::memory_order_acquire) || static_cast<bool>(this->task); });
            if (stopRequest.load(std::memory_order_acquire)) return;

            this->task();
            this->task = nullptr;
        }
    }) {}

    void stop() {
        stopRequest.store(true, std::memory_order_release);
    }

    void join() {
        thread.join();
    }
};

#if 0
template<int Threads>
struct ParallelExecutor {
    std::condition_variable condition_variable;
    std::mutex mutex;
    std::array<Worker, Threads - 1> workers = std::apply([&](auto... uwu) {
        return std::array{(uwu, Worker{condition_variable, mutex})...};
    }, std::array<char, Threads -1>{});

    template<typename... Fn> requires (sizeof...(Fn) == Threads)
    __attribute__((noinline)) auto compute(Fn&&... tasks) {
        // Indexing parameter packs is aids
        std::tuple args = std::make_tuple(std::forward<Fn>(tasks)...);

        std::tuple<std::invoke_result_t<Fn>...> results;
        std::atomic_int counter = 0;

        [&]<size_t... I>(std::index_sequence<I...>) {
            std::lock_guard lock(mutex);
            ([&] {
                Worker& worker = workers[I];
                auto& r = std::get<I>(results);
                worker.task = [&] {
                    r = std::get<I>(args)();
                    // signal that we are finished
                    counter++;
                };
            }(), ...);
        }(std::make_index_sequence<Threads - 1>{});
        this->condition_variable.notify_all();
        // take advantage of the calling tread
        std::get<Threads - 1>(results) = std::get<Threads - 1>(args)();
        counter++;

        //while (counter != Threads); // wait
        while (counter.load(std::memory_order_acquire) != Threads);

        return results;
    }

    ~ParallelExecutor() {
        for (auto& w : workers) {
            w.stop();
        }
        this->condition_variable.notify_all();
        for (auto& w : workers) {
            w.join();
        }
    }
};

#else
template<int Threads>
struct ParallelExecutor {
    template<typename... Fn> requires (sizeof...(Fn) == Threads)
    auto compute(Fn&&... tasks) {
        return std::tuple{tasks()...};
    }
};
#endif
