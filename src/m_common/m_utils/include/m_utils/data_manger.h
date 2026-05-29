#ifndef COMMON_COMMON_UTILS_DATA_MANAGER_DATA_MANGER_H_
#define COMMON_COMMON_UTILS_DATA_MANAGER_DATA_MANGER_H_

#define USE_DM_WITH_MUTEX false

#include <any>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <queue>
#include <shared_mutex>
#include <unordered_map>
namespace common_utils
{
template <typename T>
class MessageQueue
{
  public:
    explicit MessageQueue(size_t capacity = 1) : max_capacity_(capacity) {}

    MessageQueue(const MessageQueue& other)
    {
        std::lock_guard<std::mutex> lock(other.mutex_);
        queue_ = other.queue_;
        max_capacity_ = other.max_capacity_;
        adjustQueueSize();
    }

    MessageQueue(MessageQueue&& other) noexcept
    {
        std::lock_guard<std::mutex> lock(other.mutex_);
        queue_ = std::move(other.queue_);
        max_capacity_ = other.max_capacity_;
        other.max_capacity_ = 1; // 保持源对象有效状态
    }

    MessageQueue& operator=(const MessageQueue& other)
    {
        if (this != &other) {
            // 使用分层锁定防止死锁
            std::unique_lock<std::mutex> lock1(mutex_, std::defer_lock);
            std::unique_lock<std::mutex> lock2(other.mutex_, std::defer_lock);
            std::lock(lock1, lock2);

            queue_ = other.queue_;
            max_capacity_ = other.max_capacity_;
            adjustQueueSize();
        }
        return *this;
    }

    MessageQueue& operator=(MessageQueue&& other) noexcept
    {
        if (this != &other) {
            std::unique_lock<std::mutex> lock1(mutex_, std::defer_lock);
            std::unique_lock<std::mutex> lock2(other.mutex_, std::defer_lock);
            std::lock(lock1, lock2);

            queue_ = std::move(other.queue_);
            max_capacity_ = other.max_capacity_;
            other.max_capacity_ = 1; // 保持源对象有效状态
        }
        return *this;
    }

    ~MessageQueue() {}

    void setCapacity(size_t capacity)
    {
        if (capacity == 0)
            return;
        std::lock_guard<std::mutex> lock(mutex_);
        max_capacity_ = capacity;
        adjustQueueSize();
    }

    // 完美转发
    template <typename U>
    bool enqueue(U&& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size() >= max_capacity_) {
            queue_.pop_front();
        }
        queue_.emplace_back(std::forward<U>(msg));
        cond_var_.notify_one(); // 通知对应线程
        return true;
    }

    template<typename... Args>
    bool enqueuev2(Args&&... _args)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size() >= max_capacity_) {
            queue_.pop_front();
        }
        queue_.emplace_back(std::forward<Args>(_args)...);
        cond_var_.notify_one(); // 通知对应线程
        return true;
    }
    
    template <typename U>
    bool enqueue_notify_all(U&& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size() >= max_capacity_) {
            queue_.pop_front();
        }
        queue_.emplace_back(std::forward<U>(msg));
        cond_var_.notify_all(); // 通知所有线程
        return true;
    }

    // 从队首获取消息：非堵塞接口
    bool try_dequeue(T& out_data)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (queue_.empty())
            return false;
        out_data = std::move(queue_.front());
        queue_.pop_front();
        return true;
    }

    // 从队首获取消息：超时/非堵塞接口（不设超时阈值，则默认堵塞）
    bool dequeue(T& out_data, std::chrono::milliseconds timeout = std::chrono::milliseconds(0))
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (timeout.count() > 0) {
            if (!cond_var_.wait_for(lock, timeout, [this] { return !queue_.empty(); })) {
                return false;
            }
        } else {
            cond_var_.wait(lock, [this] { return !queue_.empty(); });
        }
        out_data = std::move(queue_.front());
        queue_.pop_front();
        return true;
    }

    T dequeue(std::chrono::milliseconds timeout = std::chrono::milliseconds(0))
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (timeout.count() > 0) {
            if (!cond_var_.wait_for(lock, timeout, [this] { return !queue_.empty(); })) {
                return false;
            }
        } else {
            cond_var_.wait(lock, [this] { return !queue_.empty(); });
        }
        T msg = std::move(queue_.front());
        queue_.pop_front();
        return msg;
    }

    // 从队尾获取消息：非堵塞接口
    bool try_dequeue_last(T& out_data)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (queue_.empty())
            return false;
        out_data = std::move(queue_.back());
        queue_.pop_back();
        return true;
    }

    // 获取并移除队尾元素：超时/非堵塞接口（不设超时阈值，则默认堵塞）
    bool dequeue_last(T& out_data, std::chrono::milliseconds timeout = std::chrono::milliseconds(0))
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (timeout.count() > 0) {
            if (!cond_var_.wait_for(lock, timeout, [this] { return !queue_.empty(); })) {
                return false;
            }
        } else {
            cond_var_.wait(lock, [this] { return !queue_.empty(); });
        }
        out_data = std::move(queue_.back());
        queue_.pop_back();
        return true;
    }

    // 获取并保留队尾元素：超时/非堵塞接口（不设超时阈值，默认堵塞）
    bool get_last(T& out_data, std::chrono::milliseconds timeout = std::chrono::milliseconds(0))
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (timeout.count() > 0) {
            if (!cond_var_.wait_for(lock, timeout, [this] { return !queue_.empty(); })) {
                return false;
            }
        } else {
            cond_var_.wait(lock, [this] { return !queue_.empty(); });
        }
        out_data = queue_.back();
        return true;
    }

    void clear() noexcept
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
    }
    bool empty() const noexcept
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }
    size_t size() const noexcept
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    // 移除队首元素：非阻塞接口
    bool try_pop_front() noexcept
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty())
            return false;
        queue_.pop_front();
        return true;
    }

    void pop_front() noexcept
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty())
            return;
        queue_.pop_front();
    }
    // 非阻塞尝试移除尾元素
    bool try_pop_back() noexcept
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty())
            return false;
        queue_.pop_back();
        return true;
    }
    void pop_back() noexcept
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty())
            return;
        queue_.pop_back();
    }

    void pop() { this->pop_front(); }

  private:
    inline void adjustQueueSize()
    {
        while (queue_.size() > max_capacity_) {
            queue_.pop_front();
        }
    }

    std::deque<T> queue_; // 消息队列
    mutable std::mutex mutex_; // 保护队列的互斥锁
    std::condition_variable cond_var_; // 条件变量，用于线程同步
    size_t max_capacity_; // 默认最大容量
};

namespace data_manager
{
template <typename K, typename T>
class DataManager
{
#if USE_DM_WITH_MUTEX
    /* 在以下 典型场景 必须保留锁：
    ** 动态键空间的多线程访问
    ** 需要强一致性保证
    ** 无法严格隔离数据分区
    */

  public:
    explicit DataManager(size_t default_capacity = 1)
        : default_capacity_(default_capacity > 0 ? default_capacity : 1)
    {
    }

    void initDataQueue(K id, size_t capacity = 1)
    {
        std::unique_lock<std::shared_mutex> lock(map_mutex_);
        data_queue_map_.try_emplace(id, capacity > 0 ? capacity : 1);
    }

    template <typename U>
    bool updateData(K id, U&& data)
    {
        {
            std::shared_lock<std::shared_mutex> read_lock(map_mutex_);
            auto it = data_queue_map_.find(id);
            if (it != data_queue_map_.end()) {
                return it->second.enqueue(std::forward<U>(data));
            }
        }

        std::unique_lock<std::shared_mutex> write_lock(map_mutex_);
        auto& queue = data_queue_map_.try_emplace(id, default_capacity_).first->second;
        return queue.enqueue(std::forward<U>(data));
    }

    bool getDataById(K id, T& out_data, bool non_blocking = false)
    {
        std::shared_lock<std::shared_mutex> lock(map_mutex_);
        auto it = data_queue_map_.find(id);
        if (it == data_queue_map_.end())
            return false;
        return non_blocking ? it->second.try_dequeue(out_data) : it->second.dequeue(out_data);
    }

    bool getLastDataById(K id, T& out_data, bool non_blocking = false)
    {
        std::shared_lock<std::shared_mutex> lock(map_mutex_);
        auto it = data_queue_map_.find(id);
        if (it == data_queue_map_.end())
            return false;
        return non_blocking ? it->second.try_dequeue_last(out_data)
                            : it->second.dequeue_last(out_data);
    }

    void clear(K id) noexcept
    {
        std::unique_lock<std::shared_mutex> lock(map_mutex_);
        auto it = data_queue_map_.find(id);
        if (it != data_queue_map_.end()) {
            it->second.clear();
        }
    }

    size_t size(K id) const noexcept
    {
        std::shared_lock<std::shared_mutex> lock(map_mutex_);
        auto it = data_queue_map_.find(id);
        return it != data_queue_map_.end() ? it->second.size() : 0;
    }

    bool empty(K id) const noexcept
    {
        std::shared_lock<std::shared_mutex> lock(map_mutex_);
        auto it = data_queue_map_.find(id);
        return it != data_queue_map_.end() ? it->second.empty() : true;
    }

  private:
    mutable std::shared_mutex map_mutex_;
    std::unordered_map<K, MessageQueue<T>> data_queue_map_;

#else
    /*
    ** 使用前提条件（必须由调用方保证）：
    ** 1. 每个K值在初始化阶段通过initDataQueue预先创建
    ** 2. 运行时每个K值仅被单个线程独占访问
    ** 3. 无动态新增/删除K值的需求
    ** 4. 键值K与线程的绑定关系在运行时不可改变
    */

  public:
    // 构造函数：初始化默认容量
    explicit DataManager(size_t default_capacity = 1)
        : default_capacity_(default_capacity > 0 ? default_capacity : 1)
    {
    }

    // 预初始化队列（线程安全初始化阶段调用）
    void initDataQueue(K id, size_t capacity = 1)
    {
        data_queue_map_.try_emplace(id, capacity > 0 ? capacity : 1);
    }

    // 数据更新（要求：当前线程独占此id的操作权）
    template <typename U>
    bool updateData(K id, U&& data)
    {
        auto& queue = data_queue_map_[id]; // 假设id已预初始化
        return queue.enqueue(std::forward<U>(data));
    }

    // 数据获取（要求：当前线程独占此id的操作权）
    bool getDataById(K id, T& out_data, bool non_blocking = false)
    {
        auto it = data_queue_map_.find(id);
        if (it == data_queue_map_.end())
            return false;
        return non_blocking ? it->second.try_dequeue(out_data) : it->second.dequeue(out_data);
    }

    bool getLastDataById(K id, T& out_data, bool non_blocking = false)
    {
        auto it = data_queue_map_.find(id);
        if (it == data_queue_map_.end())
            return false;
        return non_blocking ? it->second.try_dequeue_last(out_data)
                            : it->second.dequeue_last(out_data);
    }

    void clear(K id) noexcept
    {
        auto it = data_queue_map_.find(id);
        if (it != data_queue_map_.end()) {
            it->second.clear();
        }
    }

    size_t size(K id) const noexcept
    {
        auto it = data_queue_map_.find(id);
        return it != data_queue_map_.end() ? it->second.size() : 0;
    }

    bool empty(K id) const noexcept
    {
        auto it = data_queue_map_.find(id);
        return it != data_queue_map_.end() ? it->second.empty() : true;
    }

  private:
    std::unordered_map<K, MessageQueue<T>> data_queue_map_;

#endif

  private:
    const size_t default_capacity_;
};

} // namespace data_manager
} // namespace common_utils

#endif // #define COMMON_COMMON_UTILS_DATA_MANAGER_DATA_MANGER_H_