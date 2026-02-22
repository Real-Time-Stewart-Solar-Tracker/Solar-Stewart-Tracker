#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <utility>

namespace solar {

/**
 * @brief Blocking, thread-safe queue for event-driven systems.
 *
 * Provides safe multi-producer / multi-consumer semantics.
 *
 * Lifecycle semantics:
 * - stop(): prevents further pushes and wakes all waiting threads.
 * - After stop(): remaining items may be drained.
 * - Once stopped and empty, wait_pop() returns std::nullopt.
 * - reset(): re-enables queue (useful for tests or controlled restarts).
 *
 * No polling or sleep-based timing is used.
 *
 * @tparam T Type of elements stored in the queue.
 */
template <typename T>
class ThreadSafeQueue {
public:
    /// @brief Default constructor.
    ThreadSafeQueue() = default;

    ThreadSafeQueue(const ThreadSafeQueue&) = delete;
    ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;

    /// @brief Destructor automatically calls stop().
    ~ThreadSafeQueue() { stop(); }

    /**
     * @brief Push an item by copy.
     * @param item Item to enqueue.
     * @return false if the queue is stopped and the item was not queued.
     */
    bool push(const T& item) {
        {
            std::lock_guard<std::mutex> lock(m_);
            if (stopped_) return false;
            q_.push_back(item);
        }
        cv_.notify_one();
        return true;
    }

    /**
     * @brief Push an item by move.
     * @param item Item to enqueue.
     * @return false if the queue is stopped and the item was not queued.
     */
    bool push(T&& item) {
        {
            std::lock_guard<std::mutex> lock(m_);
            if (stopped_) return false;
            q_.push_back(std::move(item));
        }
        cv_.notify_one();
        return true;
    }

    /**
     * @brief Block until an item is available or the queue is stopped.
     * @return Popped value, or std::nullopt if stopped and empty.
     */
    std::optional<T> wait_pop() {
        std::unique_lock<std::mutex> lock(m_);
        cv_.wait(lock, [&] { return stopped_ || !q_.empty(); });

        if (q_.empty()) {
            return std::nullopt;
        }

        T item = std::move(q_.front());
        q_.pop_front();
        return item;
    }

    /**
     * @brief Attempt to pop an item without blocking.
     * @return Popped value, or std::nullopt if empty.
     */
    std::optional<T> try_pop() {
        std::lock_guard<std::mutex> lock(m_);
        if (q_.empty()) return std::nullopt;

        T item = std::move(q_.front());
        q_.pop_front();
        return item;
    }

    /**
     * @brief Stop the queue and wake all waiting threads.
     *
     * Prevents further pushes. Remaining items may still be drained.
     */
    void stop() {
        {
            std::lock_guard<std::mutex> lock(m_);
            stopped_ = true;
        }
        cv_.notify_all();
    }

    /// @brief Alias for stop().
    void close() { stop(); }

    /**
     * @brief Re-enable the queue after stop().
     *
     * Does not clear existing items.
     */
    void reset() {
        std::lock_guard<std::mutex> lock(m_);
        stopped_ = false;
    }

    /**
     * @brief Remove all queued items.
     *
     * Does not modify the stopped state.
     */
    void clear() {
        std::lock_guard<std::mutex> lock(m_);
        q_.clear();
    }

    /// @brief Get current number of queued items.
    std::size_t size() const {
        std::lock_guard<std::mutex> lock(m_);
        return q_.size();
    }

    /// @brief Check whether the queue has been stopped.
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