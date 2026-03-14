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
 * Multi-producer / multi-consumer.
 *
 * Lifecycle semantics:
 * - stop(): prevents further pushes and wakes all waiting threads.
 * - After stop(): remaining items may be drained.
 * - Once stopped and empty, wait_pop() returns std::nullopt.
 * - reset(): re-enables queue (useful for tests or controlled restarts).
 *
 * Realtime support:
 * - Optional bounded capacity (0 = unbounded).
 * - push_latest(): if full, drops oldest item so newest is kept.
 *
 * No polling or sleep-based timing is used.
 */
template <typename T>
class ThreadSafeQueue {
public:
    /// @brief Default constructor (unbounded).
    ThreadSafeQueue() = default;

    /// @brief Construct queue with optional capacity (0 = unbounded).
    explicit ThreadSafeQueue(std::size_t capacity)
        : capacity_(capacity) {}

    ThreadSafeQueue(const ThreadSafeQueue&) = delete;
    ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;

    /// @brief Destructor automatically calls stop().
    ~ThreadSafeQueue() { stop(); }

    /**
     * @brief Push by copy (strict when bounded).
     * @return false if stopped or (bounded and full).
     */
    bool push(const T& item) {
        {
            std::lock_guard<std::mutex> lock(m_);
            if (stopped_) return false;
            if (capacity_ > 0 && q_.size() >= capacity_) return false;
            q_.push_back(item);
        }
        cv_.notify_one();
        return true;
    }

    /**
     * @brief Push by move (strict when bounded).
     * @return false if stopped or (bounded and full).
     */
    bool push(T&& item) {
        {
            std::lock_guard<std::mutex> lock(m_);
            if (stopped_) return false;
            if (capacity_ > 0 && q_.size() >= capacity_) return false;
            q_.push_back(std::move(item));
        }
        cv_.notify_one();
        return true;
    }

    /**
     * @brief Push keeping the latest element when bounded.
     *
     * If bounded and full, drops the oldest item so the newest is enqueued.
     * This is ideal for realtime pipelines where freshness > completeness.
     *
     * @return false if stopped.
     */
    bool push_latest(T item) {
        {
            std::lock_guard<std::mutex> lock(m_);
            if (stopped_) return false;

            if (capacity_ > 0 && q_.size() >= capacity_) {
                q_.pop_front(); // drop oldest
            }
            q_.push_back(std::move(item));
        }
        cv_.notify_one();
        return true;
    }

    /**
     * @brief Block until an item is available or the queue is stopped.
     * @return item, or std::nullopt if stopped and empty.
     */
    std::optional<T> wait_pop() {
        std::unique_lock<std::mutex> lock(m_);
        cv_.wait(lock, [&] { return stopped_ || !q_.empty(); });

        if (q_.empty()) return std::nullopt;

        T item = std::move(q_.front());
        q_.pop_front();
        return item;
    }

    /**
     * @brief Non-blocking pop.
     * @return item, or std::nullopt if empty.
     */
    std::optional<T> try_pop() {
        std::lock_guard<std::mutex> lock(m_);
        if (q_.empty()) return std::nullopt;

        T item = std::move(q_.front());
        q_.pop_front();
        return item;
    }

    /**
     * @brief Stop queue and wake all waiting threads.
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
     * @brief Re-enable queue after stop().
     *
     * Does not clear existing items.
     */
    void reset() {
        std::lock_guard<std::mutex> lock(m_);
        stopped_ = false;
    }

    /**
     * @brief Clear all queued items (does not change stopped state).
     */
    void clear() {
        std::lock_guard<std::mutex> lock(m_);
        q_.clear();
    }

    /// @brief Current number of queued items.
    std::size_t size() const {
        std::lock_guard<std::mutex> lock(m_);
        return q_.size();
    }

    /// @brief True if stop() has been called.
    bool stopped() const {
        std::lock_guard<std::mutex> lock(m_);
        return stopped_;
    }

private:
    mutable std::mutex m_;
    std::condition_variable cv_;
    std::deque<T> q_;
    std::size_t capacity_{0}; // 0 = unbounded
    bool stopped_{false};
};

} // namespace solar