/**
 * @file test_threadsafequeue.cpp
 * @brief Unit tests for ThreadSafeQueue.
 */

#include "common/ThreadSafeQueue.hpp"
#include "src/tests/support/test_common.hpp"

#include <atomic>
#include <thread>

using solar::ThreadSafeQueue;

TEST_CASE(ThreadSafeQueue_FIFO_basic) {
    ThreadSafeQueue<int> q;

    REQUIRE(q.push(1));
    REQUIRE(q.push(2));
    REQUIRE(q.push(3));

    const auto a = q.try_pop();
    const auto b = q.try_pop();
    const auto c = q.try_pop();
    const auto d = q.try_pop();

    REQUIRE(a.has_value());
    REQUIRE(b.has_value());
    REQUIRE(c.has_value());
    REQUIRE(!d.has_value());

    REQUIRE(*a == 1);
    REQUIRE(*b == 2);
    REQUIRE(*c == 3);
}

TEST_CASE(ThreadSafeQueue_bounded_push_strict_rejects_when_full) {
    ThreadSafeQueue<int> q(2);

    REQUIRE(q.push(10));
    REQUIRE(q.push(20));
    REQUIRE(!q.push(30));

    const auto a = q.try_pop();
    const auto b = q.try_pop();
    const auto c = q.try_pop();

    REQUIRE(a.has_value());
    REQUIRE(b.has_value());
    REQUIRE(!c.has_value());

    REQUIRE(*a == 10);
    REQUIRE(*b == 20);
}

TEST_CASE(ThreadSafeQueue_bounded_push_latest_drops_oldest) {
    ThreadSafeQueue<int> q(2);

    REQUIRE(q.push(10));
    REQUIRE(q.push(20));
    REQUIRE(q.push_latest(30));

    const auto a = q.try_pop();
    const auto b = q.try_pop();
    const auto c = q.try_pop();

    REQUIRE(a.has_value());
    REQUIRE(b.has_value());
    REQUIRE(!c.has_value());

    REQUIRE(*a == 20);
    REQUIRE(*b == 30);
}

TEST_CASE(ThreadSafeQueue_wait_pop_blocks_then_wakes) {
    ThreadSafeQueue<int> q;

    // The consumer signals readiness before entering wait_pop() so the
    // producer can confirm the blocking path has been reached before pushing.
    std::atomic<bool> consumer_ready{false};
    int value = 0;
    bool got_value = false;

    std::thread consumer([&] {
        consumer_ready.store(true, std::memory_order_release);
        const auto item = q.wait_pop();
        if (item.has_value()) {
            value = *item;
            got_value = true;
        }
    });

    // Yield until the consumer thread has advanced past its readiness store.
    // If push() happens to race the wait_pop() entry, wait_pop() returns
    // immediately with the pushed value, which is still a valid test outcome.
    while (!consumer_ready.load(std::memory_order_acquire)) {
        std::this_thread::yield();
    }

    REQUIRE(q.push(42));
    consumer.join();

    REQUIRE(got_value);
    REQUIRE(value == 42);
}

TEST_CASE(ThreadSafeQueue_stop_unblocks_waiters_and_returns_nullopt_when_empty) {
    ThreadSafeQueue<int> q;

    std::atomic<bool> consumer_ready{false};
    bool got_nullopt = false;

    std::thread consumer([&] {
        consumer_ready.store(true, std::memory_order_release);
        const auto item = q.wait_pop();
        got_nullopt = !item.has_value();
    });

    // Yield until the consumer has started and is about to block.
    while (!consumer_ready.load(std::memory_order_acquire)) {
        std::this_thread::yield();
    }

    q.stop();
    consumer.join();

    REQUIRE(got_nullopt);
}
