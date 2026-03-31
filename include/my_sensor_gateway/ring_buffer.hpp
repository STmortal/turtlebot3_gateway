#ifndef MY_SENSOR_GATEWAY_RING_BUFFER_HPP_
#define MY_SENSOR_GATEWAY_RING_BUFFER_HPP_

#include <shared_mutex>
#include <condition_variable>
#include <vector>
#include <atomic>
#include <stdexcept>

/**
 * @brief 线程安全的环形缓冲区模板类
 * @tparam T 缓冲区存储的数据类型
 * 面试核心考点：生产者-消费者模型、线程安全、无锁/轻量级锁设计
 */
template <typename T>
class RingBuffer
{
public:
    /**
     * @brief 构造函数
     * @param capacity 缓冲区最大容量（元素个数）
     */
    explicit RingBuffer(size_t capacity)
        : buffer_(capacity), capacity_(capacity), head_(0), tail_(0), count_(0), is_running_(true)
    {
        if (capacity == 0) {
            throw std::invalid_argument("缓冲区容量不能为0");
        }
    }

    ~RingBuffer() = default;

    /**
    * @brief 左值push（兼容原有逻辑）
    */
    bool push(const T & data, uint32_t timeout_ms = 0)
    {
        std::unique_lock<std::shared_mutex> lock(mutex_);

        // 先检查是否已停止
        if (!is_running_) {
            return false;
        }

        if (timeout_ms > 0) {
            if (!not_full_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                [this]() { return count_ < capacity_ || !is_running_; })) {
                return false;
            }
        } else {
            not_full_cv_.wait(lock, [this]() { return count_ < capacity_ || !is_running_; });
        }

        // 二次检查停止状态
        if (!is_running_) {
            return false;
        }

        // 拷贝赋值
        buffer_[head_] = data;
        head_ = (head_ + 1) % capacity_;
        count_++;

        not_empty_cv_.notify_one();
        return true;
    }

    /**
    * @brief 新增：右值push，零拷贝移动语义，核心优化点
    */
    bool push(T && data, uint32_t timeout_ms = 0)
    {
        std::unique_lock<std::shared_mutex> lock(mutex_);

        // 先检查是否已停止
        if (!is_running_) {
            return false;
        }

        if (timeout_ms > 0) {
            if (!not_full_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                [this]() { return count_ < capacity_ || !is_running_; })) {
                return false;
            }
        } else {
            not_full_cv_.wait(lock, [this]() { return !is_running_ || count_ < capacity_; });
        }

        // 二次检查停止状态
        if (!is_running_) {
            return false;
        }

        // 移动赋值，零内存拷贝，核心优化！
        buffer_[head_] = std::move(data);
        head_ = (head_ + 1) % capacity_;
        count_++;

        not_empty_cv_.notify_one();
        return true;
    }

    /**
    * @brief 读取数据，通过右值引用返回，零拷贝
    */
    bool pop(T & data, uint32_t timeout_ms = 0)
    {
        std::unique_lock<std::shared_mutex> lock(mutex_);

        // 先检查是否已停止
        if (!is_running_) {
            return false;
        }

        if (timeout_ms > 0) {
            if (!not_empty_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                [this]() { return !is_running_ || count_ > 0; })) {
                return false;
            }
        } else {
            not_empty_cv_.wait(lock, [this]() { return count_ > 0 || !is_running_; });
        }

        // 🛑 核心修复：停止后直接返回false，不再读取数据
        if (!is_running_) {
            return false;
        }

        // 移动赋值，零内存拷贝，核心优化！
        data = std::move(buffer_[tail_]);
        tail_ = (tail_ + 1) % capacity_;
        count_--;

        not_full_cv_.notify_one();
        return true;
    }

    /**
     * @brief 获取缓冲区中当前的元素个数
     */
    size_t size() const
    {
        std::lock_guard<std::shared_mutex> lock(mutex_);
        return count_;
    }

    /**
     * @brief 判断缓冲区是否为空
     */
    bool empty() const
    {
        std::lock_guard<std::shared_mutex> lock(mutex_);
        return count_ == 0;
    }

    /**
     * @brief 清空缓冲区
     */
    void clear()
    {
        std::lock_guard<std::shared_mutex> lock(mutex_);
        head_ = 0;
        tail_ = 0;
        count_ = 0;
        not_full_cv_.notify_all();
    }

    /**
     * @brief 停止缓冲区，唤醒所有等待的线程
     */
    void stop()
    {
        std::lock_guard<std::shared_mutex> lock(mutex_);
        is_running_ = false;
        not_full_cv_.notify_all();
        not_empty_cv_.notify_all();
    }

private:
    std::vector<T> buffer_;          // 缓冲区底层存储
    size_t capacity_;                 // 缓冲区最大容量
    size_t head_;                     // 写指针
    size_t tail_;                     // 读指针
    size_t count_;                    // 当前元素个数
    std::atomic<bool> is_running_;   // 运行状态标志

    // 线程同步核心
    mutable std::shared_mutex mutex_;                // 互斥锁，保护共享数据
    std::condition_variable_any not_full_cv_;     // 条件变量：缓冲区非满
    std::condition_variable_any not_empty_cv_;    // 条件变量：缓冲区非空
};

#endif  // MY_SENSOR_GATEWAY_RING_BUFFER_HPP_
