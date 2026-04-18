/**
 * @file test_threadsafequeue_stress.cpp
 * @brief Concurrent stress tests for ThreadSafeQueue.
 *
 * These tests exercise the queue under multi-threaded contention to verify
 * correctness of the mutex/condition_variable synchronisation under load.
 */

#include "common/ThreadSafeQueue.hpp"
#include "src/tests/support/test_common.hpp"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <thread>
#include <vector>

using solar::ThreadSafeQueue;

TEST_CASE(ThreadSafeQueue_concurrent_producers_and_consumer_no_data_loss) {
    // Multiple producers push unique values. A single consumer collects them.
    // After joining all threads, the total consumed count must equal the total
    // pushed count — no items may be silently lost.
    ThreadSafeQueue<int> q;

    constexpr int kProducers = 4;
    constexpr int kItemsPerProducer = 500;

    std::atomic<int> push_count{0};
    std::atomic<int> pop_count{0};
    std::atomic<bool> producers_done{false};

    // Producer threads — each pushes kItemsPerProducer unique values.
    std::vector<std::thread> producers;
    for (int p = 0; p < kProducers; ++p) {
        producers.emplace_back([&, p] {
            for (int i = 0; i < kItemsPerProducer; ++i) {
                const int value = p * kItemsPerProducer + i;
                while (!q.push(value)) {
                    std::this_thread::yield();
                }
                push_count.fetch_add(1, std::memory_order_relaxed);
            }
        });
    }

    // Consumer thread — drains items until producers are done and queue is empty.
    std::thread consumer([&] {
        while (true) {
            const auto item = q.try_pop();
            if (item.has_value()) {
                pop_count.fetch_add(1, std::memory_order_relaxed);
            } else if (producers_done.load(std::memory_order_acquire)) {
                // Drain any remaining items after producers finish.
                while (true) {
                    const auto tail = q.try_pop();
                    if (!tail.has_value()) break;
                    pop_count.fetch_add(1, std::memory_order_relaxed);
                }
                break;
            } else {
                std::this_thread::yield();
            }
        }
    });

    for (auto& t : producers) {
        t.join();
    }
    producers_done.store(true, std::memory_order_release);
    consumer.join();

    REQUIRE(push_count.load() == kProducers * kItemsPerProducer);
    REQUIRE(pop_count.load() == push_count.load());
}

TEST_CASE(ThreadSafeQueue_concurrent_wait_pop_wakes_all_consumers) {
    // Multiple consumer threads block on wait_pop(). Each pushed item must
    // wake exactly one consumer. After stopping the queue, all consumers
    // must unblock and exit cleanly.
    ThreadSafeQueue<int> q;

    constexpr int kConsumers = 4;
    constexpr int kItems = 20;

    std::atomic<int> total_received{0};
    std::vector<std::thread> consumers;

    for (int c = 0; c < kConsumers; ++c) {
        consumers.emplace_back([&] {
            while (true) {
                const auto item = q.wait_pop();
                if (!item.has_value()) break;
                total_received.fetch_add(1, std::memory_order_relaxed);
            }
        });
    }

    // Push items — each one wakes exactly one blocked consumer.
    for (int i = 0; i < kItems; ++i) {
        REQUIRE(q.push(i));
    }

    // Give consumers time to process.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Stop the queue — all blocked consumers must unblock.
    q.stop();

    for (auto& t : consumers) {
        t.join();
    }

    REQUIRE(total_received.load() == kItems);
}

TEST_CASE(ThreadSafeQueue_push_latest_under_contention_keeps_freshest) {
    // Under contention, push_latest() must always keep the most recently
    // pushed value when the queue is full. After contention stops, the
    // consumer must see only recent values.
    ThreadSafeQueue<int> q(1);

    constexpr int kIterations = 1000;

    std::atomic<bool> done{false};
    std::atomic<int> last_seen{-1};

    // Consumer drains continuously.
    std::thread consumer([&] {
        while (!done.load(std::memory_order_acquire)) {
            const auto item = q.try_pop();
            if (item.has_value()) {
                last_seen.store(*item, std::memory_order_relaxed);
            }
            std::this_thread::yield();
        }
        // Final drain.
        while (true) {
            const auto item = q.try_pop();
            if (!item.has_value()) break;
            last_seen.store(*item, std::memory_order_relaxed);
        }
    });

    // Producer pushes increasing values with push_latest().
    for (int i = 0; i < kIterations; ++i) {
        q.push_latest(i);
    }

    done.store(true, std::memory_order_release);
    consumer.join();

    // The consumer must have seen at least one value, and the last value
    // seen must be from the later portion of the sequence — not an early
    // stale value that sat in the queue while newer values were dropped.
    REQUIRE(last_seen.load() >= 0);
    REQUIRE(last_seen.load() >= kIterations / 2);
}