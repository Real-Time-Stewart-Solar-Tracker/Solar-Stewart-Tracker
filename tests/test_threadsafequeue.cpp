#include "test_common.hpp"

#include "common/ThreadSafeQueue.hpp"

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

using solar::ThreadSafeQueue;

TEST(ThreadSafeQueue_FIFO_basic) {
    ThreadSafeQueue<int> q; // unbounded

    REQUIRE(q.push(1));
    REQUIRE(q.push(2));
    REQUIRE(q.push(3));

    auto a = q.try_pop();
    auto b = q.try_pop();
    auto c = q.try_pop();
    auto d = q.try_pop();

    REQUIRE(a.has_value() && *a == 1);
    REQUIRE(b.has_value() && *b == 2);
    REQUIRE(c.has_value() && *c == 3);
    REQUIRE(!d.has_value());
}

TEST(ThreadSafeQueue_bounded_push_strict_rejects_when_full) {
    ThreadSafeQueue<int> q(2);

    REQUIRE(q.push(10));
    REQUIRE(q.push(20));
    REQUIRE(!q.push(30)); // strict push rejects when full

    auto a = q.try_pop();
    auto b = q.try_pop();
    REQUIRE(a.has_value() && *a == 10);
    REQUIRE(b.has_value() && *b == 20);
}

TEST(ThreadSafeQueue_bounded_push_latest_drops_oldest) {
    ThreadSafeQueue<int> q(2);

    REQUIRE(q.push_latest(1));
    REQUIRE(q.push_latest(2));
    REQUIRE(q.push_latest(3)); // should drop 1, keep 2,3

    auto a = q.try_pop();
    auto b = q.try_pop();
    REQUIRE(a.has_value() && *a == 2);
    REQUIRE(b.has_value() && *b == 3);
}

TEST(ThreadSafeQueue_wait_pop_blocks_then_wakes) {
    ThreadSafeQueue<int> q(1);

    std::atomic<bool> got{false};
    int value = 0;

    std::thread t([&] {
        auto v = q.wait_pop();
        REQUIRE(v.has_value());
        value = *v;
        got.store(true);
    });

    // ensure waiter is blocked briefly
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    REQUIRE(!got.load());

    REQUIRE(q.push(42));

    // wait for thread to finish
    t.join();
    REQUIRE(got.load());
    REQUIRE(value == 42);
}

TEST(ThreadSafeQueue_stop_unblocks_waiters_and_returns_nullopt_when_empty) {
    ThreadSafeQueue<int> q(1);

    std::atomic<bool> finished{false};
    std::atomic<bool> returnedNull{false};

    std::thread t([&] {
        auto v = q.wait_pop(); // should unblock after stop()
        returnedNull.store(!v.has_value());
        finished.store(true);
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    REQUIRE(!finished.load());

    q.stop(); // should wake all waiters

    t.join();
    REQUIRE(finished.load());
    REQUIRE(returnedNull.load());
}