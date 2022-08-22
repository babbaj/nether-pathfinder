#pragma once

#include <cstdio>
#include <thread>
#include <functional>
#include <array>
#include <tuple>
#include <condition_variable>
#include <mutex>
#include <type_traits>

#ifndef _LIBCPP_VERSION
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

template<int Tasks>
struct ParallelExecutor {
    static constexpr bool IsActuallyParallel = true;
    std::array<Worker, Tasks> workers{};

    template<typename... Fn> requires (sizeof...(Fn) == Tasks)
    auto compute(Fn&&... tasks) {
        // Indexing parameter packs is aids
        std::tuple args = std::make_tuple(std::forward<Fn>(tasks)...);

        std::tuple<std::invoke_result_t<Fn>...> results;
        std::atomic_int counter = 0;

        [&]<size_t... I>(std::index_sequence<I...>) {
            ([&] {
                Worker& worker = workers[I];

                auto& r = std::get<I>(results);
                {
                    std::lock_guard lock(worker.mutex);
                    worker.task = [&] {
                        r = std::get<I>(args)();

                        // signal that we are finished
                        counter++;
                    };
                }

                // wake up babe we have more work for you!
                worker.condition.notify_one();
            }(), ...);
        }(std::make_index_sequence<Tasks - 1>{});
        // take advantage of the calling tread
        std::get<Tasks - 1>(results) = std::get<Tasks - 1>(args)();
        counter++;

        while (counter != Tasks); // wait

        return results;
    }
};
#else
  template<int Tasks>
  struct ParallelExecutor {
      static constexpr bool IsActuallyParallel = false;

      template<typename... Fn>
      auto compute(Fn&&... tasks) {
          return std::make_tuple(tasks()...);
      }
  };
#endif

using ChunkGenExec = ParallelExecutor<3>;
