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
    std::condition_variable_any condition;
    std::function<void()> task;
    std::mutex mutex;
    std::jthread thread;

    Worker(): thread([this](std::stop_token stoken) {
        std::stop_callback cb{stoken, [&] { condition.notify_one(); }};

        while (true) {
            std::unique_lock lock(this->mutex);
            condition.wait(lock, stoken,[this, &stoken] { return stoken.stop_requested() || static_cast<bool>(this->task); });

            if (stoken.stop_requested()) return;

            this->task();
            this->task = nullptr;
        }
    }) {}
};

// promise/future is bloated
struct WorkTracker {
    std::atomic_int counter = 0;
    std::condition_variable cv;
};

template<int Tasks>
struct ParallelExecutor {
    std::array<Worker, Tasks> workers{};

    template<typename... Fn> requires (sizeof...(Fn) == Tasks)
    auto compute(Fn&&... tasks) {
        // Indexing parameter packs is aids
        std::tuple args = std::make_tuple(std::forward<Fn>(tasks)...);

        std::tuple<std::invoke_result_t<Fn>...> results;
        WorkTracker tracker{};

        [&]<size_t... I>(std::index_sequence<I...>) {
            ([&] {
                Worker& worker = workers[I];

                auto& r = std::get<I>(results);
                {
                    std::lock_guard lock(worker.mutex);
                    worker.task = [&] {
                        r = std::get<I>(args)();

                        // signal that we are finished
                        {
                            //std::unique_lock lock(tracker.mutex);
                            tracker.counter++;
                        }
                        tracker.cv.notify_one();
                    };
                }

                // wake up babe we have more work for you!
                worker.condition.notify_one();
            }(), ...);
        }(std::make_index_sequence<Tasks>{});

        // for some time this is just randomly freezing
        //{
        //    std::unique_lock lock(tracker.mutex);
        //    tracker.cv.wait(lock, [&] { return tracker.counter == Tasks; });
        //}
        while (true) {
            //std::unique_lock lock(tracker.mutex);
            if (tracker.counter == Tasks) break;
        }

        return results;
    }
};

using ChunkGenExec = ParallelExecutor<3>;
