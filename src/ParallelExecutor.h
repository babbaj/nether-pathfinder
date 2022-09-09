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
        while (true) {
            std::unique_lock lock(this->mutex);
            condition.wait(lock, stoken, [this] { return static_cast<bool>(this->task); });

            if (stoken.stop_requested()) return;

            this->task();
            this->task = nullptr;
        }
    }) {}
};

template<int Threads>
struct ParallelExecutor {
    std::array<Worker, Threads - 1> workers{};

    template<typename... Fn> requires (sizeof...(Fn) == Threads)
    __attribute__((noinline)) auto compute(Fn&&... tasks) {
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
        }(std::make_index_sequence<Threads - 1>{});
        // take advantage of the calling tread
        std::get<Threads - 1>(results) = std::get<Threads - 1>(args)();
        counter++;

        while (counter != Threads); // wait

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