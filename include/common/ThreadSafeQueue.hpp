#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <utility>

namespace solar {

/**
 * ThreadSafeQueue<T>
 *
 * A blocking, thread-safe queue designed for event-driven systems.
 *
 * Properties:
 * - Producers call push(...)
 * - Consumers call wait_pop(...) which blocks until:
 *     - an item arrives, or
 *     - stop is requested
 * - stop() wakes all waiting threads and causes wait_pop to return std::nullopt
 *
 * No polling and no sleep-based timing are used.
 */
template <typename T>
class ThreadSafeQueue {
public:
    ThreadSafeQueue() = default;

    ThreadSafeQueue(const ThreadSafeQueue&) = delete;
    ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;

    ~ThreadSafeQueue() { stop(); }

    // Push by copy
    void push(const T& item) {
        {
            std::lock_guard<std::mutex> lock(m_);
            if (stopped_) return; // ignore pushes after stop
            q_.push_back(item);
        }
        cv_.notify_one();
    }

    // Push by move
    void push(T&& item) {
        {
            std::lock_guard<std::mutex> lock(m_);
            if (stopped_) return;
            q_.push_back(std::move(item));
        }
        cv_.notify_one();
    }

    /**
     * wait_pop()
     * Blocks until an item is available or stop() is called.
     * Returns:
     *  - std::optional<T> with value if an item was popped
     *  - std::nullopt if stopped and queue is empty
     */
    std::optional<T> wait_pop() {
        std::unique_lock<std::mutex> lock(m_);
        cv_.wait(lock, [&] { return stopped_ || !q_.empty(); });

        if (q_.empty()) {
            // If stopped_ is true and queue is empty, exit cleanly.
            return std::nullopt;
        }

        T item = std::move(q_.front());
        q_.pop_front();
        return item;
    }

    /**
     * try_pop()
     * Non-blocking: returns immediately.
     */
    std::optional<T> try_pop() {
        std::lock_guard<std::mutex> lock(m_);
        if (q_.empty()) return std::nullopt;

        T item = std::move(q_.front());
        q_.pop_front();
        return item;
    }

    /**
     * stop()
     * Requests stop and wakes any waiting consumer threads.
     * After stop:
     * - wait_pop will return remaining items until empty, then std::nullopt
     * - pushes are ignored
     */
    void stop() {
        {
            std::lock_guard<std::mutex> lock(m_);
            stopped_ = true;
        }
        cv_.notify_all();
    }

    /**
     * clear()
     * Removes all queued items (does not change stopped state).
     */
    void clear() {
        std::lock_guard<std::mutex> lock(m_);
        q_.clear();
    }

    /**
     * size()
     * Returns current queue size (snapshot).
     */
    std::size_t size() const {
        std::lock_guard<std::mutex> lock(m_);
        return q_.size();
    }

    /**
     * stopped()
     * Returns whether stop has been requested.
     */
    bool stopped() const {
        std::lock_guard<std::mutex> lock(m_);
        return stopped_;
    }

private:
    mutable std::mutex m_;
    std::condition_variable cv_;
    std::deque<T> q_;
    bool stopped_{false};
};

} // namespace solar