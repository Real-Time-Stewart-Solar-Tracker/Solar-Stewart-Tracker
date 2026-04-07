#pragma once

/**
 * @file ThreadSafeQueue.hpp
 * @brief Blocking thread-safe queue with optional bounded freshest-data behaviour.
 *
 * This queue is intended for event-driven userspace realtime pipelines where:
 * - worker threads block until data arrives
 * - stale backlog can be harmful
 * - stop() must wake blocked threads cleanly
 *
 * Supported policies:
 * - strict bounded push via push(): reject when full
 * - freshest-data bounded push via push_latest(): drop oldest when full
 */

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <utility>

namespace solar {

/**
 * @brief Blocking thread-safe queue.
 *
 * @tparam T Stored item type.
 */
template <typename T>
class ThreadSafeQueue {
public:
    /**
     * @brief Construct an unbounded queue.
     */
    ThreadSafeQueue() = default;

    /**
     * @brief Construct a queue with optional capacity.
     *
     * @param capacity Maximum queue size. 0 means unbounded.
     */
    explicit ThreadSafeQueue(std::size_t capacity)
        : capacity_(capacity) {
    }

    ThreadSafeQueue(const ThreadSafeQueue&) = delete;
    ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;

    /**
     * @brief Destructor stops the queue and wakes waiters.
     */
    ~ThreadSafeQueue() {
        stop();
    }

    /**
     * @brief Push one item by copy.
     *
     * Bounded strict mode:
     * - if full, the push is rejected
     *
     * @param item Item to push.
     * @return True on success.
     */
    bool push(const T& item) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopped_) {
                return false;
            }
            if (capacity_ > 0U && queue_.size() >= capacity_) {
                return false;
            }
            queue_.push_back(item);
        }
        cv_.notify_one();
        return true;
    }

    /**
     * @brief Push one item by move.
     *
     * Bounded strict mode:
     * - if full, the push is rejected
     *
     * @param item Item to push.
     * @return True on success.
     */
    bool push(T&& item) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopped_) {
                return false;
            }
            if (capacity_ > 0U && queue_.size() >= capacity_) {
                return false;
            }
            queue_.push_back(std::move(item));
        }
        cv_.notify_one();
        return true;
    }

    /**
     * @brief Push one item keeping the latest when bounded.
     *
     * If the queue is bounded and already full, the oldest item is dropped
     * before the new item is inserted.
     *
     * This is the correct policy for frame pipelines where freshness matters
     * more than completeness.
     *
     * @param item Item to push.
     * @return True on success.
     */
    bool push_latest(T item) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopped_) {
                return false;
            }

            if (capacity_ > 0U && queue_.size() >= capacity_) {
                queue_.pop_front();
            }

            queue_.push_back(std::move(item));
        }
        cv_.notify_one();
        return true;
    }

    /**
     * @brief Wait until one item is available or the queue is stopped.
     *
     * @return Item if one was available, or std::nullopt if the queue is
     * stopped and empty.
     */
    std::optional<T> wait_pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] {
            return stopped_ || !queue_.empty();
        });

        if (queue_.empty()) {
            return std::nullopt;
        }

        T item = std::move(queue_.front());
        queue_.pop_front();
        return item;
    }

    /**
     * @brief Try to pop one item without blocking.
     *
     * @return Item if available, otherwise std::nullopt.
     */
    std::optional<T> try_pop() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return std::nullopt;
        }

        T item = std::move(queue_.front());
        queue_.pop_front();
        return item;
    }

    /**
     * @brief Stop the queue and wake all waiting threads.
     */
    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopped_ = true;
        }
        cv_.notify_all();
    }

    /**
     * @brief Reset the stopped state.
     *
     * Does not clear buffered items.
     */
    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        stopped_ = false;
    }

    /**
     * @brief Clear all queued items.
     */
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
    }

    /**
     * @brief Return the current queue size.
     *
     * @return Current queue size.
     */
    std::size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    /**
     * @brief Return whether the queue has been stopped.
     *
     * @return True if stopped.
     */
    bool stopped() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return stopped_;
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<T> queue_;
    std::size_t capacity_{0U};
    bool stopped_{false};
};

} // namespace solar