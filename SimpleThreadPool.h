#pragma once

#include <cstdio>
#include <thread>
#include <future>
#include <array>
#include <tuple>
#include <condition_variable>
#include <mutex>
#include <type_traits>

struct Worker {
    std::condition_variable condition;
    std::function<void()> task;
    std::mutex mutex;
    std::thread thread;

    Worker(): thread([this] {
        while (true) {
            std::unique_lock lock(this->mutex);
            this->condition.wait(lock, [this] { return static_cast<bool>(this->task); });

            this->task();
            this->task = nullptr;
        }
    }) {}
};

// promise/future is bloated
struct Signal {
    enum Status {
        READY, NOT_READY
    };
    Status status = NOT_READY;
    std::condition_variable condition;
};

template<int Threads>
struct ThreadPool {
    std::array<Worker, Threads> workers;
    // TODO: destructor

    template<typename... Fn> requires (sizeof...(Fn) == Threads)
    auto enqueue(Fn&&... tasks) {
        std::tuple args = std::make_tuple(std::forward<Fn>(tasks)...);

        std::tuple<std::invoke_result_t<Fn>...> results;
        std::array<Signal, Threads> signals;

        [&]<size_t... I>(std::index_sequence<I...>) {
            ([&] {
                Worker& worker = workers[I];
                Signal& wait = signals[I];

                auto& r = std::get<I>(results);
                {
                    std::lock_guard lock(worker.mutex);
                    worker.task = [&] {
                        r = std::get<I>(args)();

                        // signal that we are finished
                        wait.condition.notify_one();
                        wait.status = Signal::Status::READY;
                    };
                }

                worker.condition.notify_one();
            }(), ...);
        }(std::make_index_sequence<Threads>{});

        for (int i = 0; i < Threads; i++) {
            std::unique_lock lock(workers[i].mutex);
            Signal& signal = signals[i];
            signal.condition.wait(lock, [&] { return signal.status == Signal::Status::READY; });
        }

        return results;
    }
};