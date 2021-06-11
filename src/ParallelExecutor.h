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
    std::condition_variable condition;
    std::function<void()> task;
    std::mutex mutex;
    std::thread thread;
    bool stop = false;

    Worker(): thread([this] {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(3s);
        while (true) {
            std::unique_lock lock(this->mutex);
            this->condition.wait(lock, [this] { return this->stop || static_cast<bool>(this->task); });
            if (this->stop && !this->task) return;

            this->task();
            this->task = nullptr;
        }
    }) {}

    // Stopping and joining sequentially isn't the most efficient but doesn't matter
    ~Worker() {
        {
            std::lock_guard lock(mutex);
            this->stop = true;
        }
        this->condition.notify_all();
        this->thread.join();
    }
};

// promise/future is bloated
struct Signal {
    enum Status {
        READY, NOT_READY
    };
    Status status = NOT_READY;
    std::condition_variable condition;
};

template<int Tasks>
struct ParallelExecutor {
private:
    static constexpr int Threads = Tasks - 1;
public:
    std::array<Worker, Threads> workers;

    template<typename... Fn> requires (sizeof...(Fn) == Tasks)
    auto compute(Fn&&... tasks) {
        // Indexing parameter packs is aids
        std::tuple args = std::make_tuple(std::forward<Fn>(tasks)...);

        std::tuple<std::invoke_result_t<Fn>...> results;
        std::array<Signal, Threads> signals;

        // invoke all but last task on separate threads
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
                        wait.status = Signal::Status::READY;
                        wait.condition.notify_one();
                    };
                }

                worker.condition.notify_one();
            }(), ...);
        }(std::make_index_sequence<Threads>{});

        std::get<Tasks - 1>(results) = std::get<Tasks - 1>(args)(); // invoke last task on same thread

        for (int i = 0; i < Threads; i++) {
            std::unique_lock lock(workers[i].mutex);
            Signal& signal = signals[i];
            signal.condition.wait(lock, [&] { return signal.status == Signal::Status::READY; });
        }

        return results;
    }
};