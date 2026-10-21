#pragma once

// ros.hpp — glue between ROS 2 topics and SmartData transducers.
//
//   Source<Msg>  an observed ROS topic. Each incoming message (with the
//                origin resolved from header.stamp and tf2) is handed to the
//                transducers attached to it — the ROS equivalent of the
//                hardware interrupt that makes a transducer "active".
//   Sink<Msg>    the actuated side: actuator transducers write their field of
//                the message and the whole setpoint is published.

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"

#include "smartdata/wire.hpp"

namespace sd {

inline Time to_us(const builtin_interfaces::msg::Time & t) noexcept
{
    return static_cast<Time>(t.sec) * 1'000'000 + static_cast<Time>(t.nanosec) / 1000;
}

inline builtin_interfaces::msg::Time to_stamp(Time us) noexcept
{
    builtin_interfaces::msg::Time t;
    t.sec     = static_cast<int32_t>(us / 1'000'000);
    t.nanosec = static_cast<uint32_t>((us % 1'000'000) * 1000);
    return t;
}

template<typename Msg>
class Source {
public:
    class Observer {
    public:
        virtual ~Observer() = default;
        virtual void update(const Msg & msg, const Origin & origin) = 0;
    };

    void attach(Observer * o)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        observers_.push_back(o);
    }

    void detach(Observer * o)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        observers_.erase(std::remove(observers_.begin(), observers_.end(), o),
                         observers_.end());
    }

    void publish(const Msg & msg, const Origin & origin)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        for (Observer * o : observers_) o->update(msg, origin);
    }

    std::size_t observers() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return observers_.size();
    }

private:
    mutable std::mutex      mutex_;
    std::vector<Observer *> observers_;
};

// Each field of an actuated message is its own SmartData, so one remote
// write of a whole message arrives as one command per field. With a
// coalescing window, field updates that arrive within it are published as a
// single setpoint instead of a burst of partially updated ones.
template<typename Msg>
class Sink {
public:
    using Publish = std::function<void(const Msg &)>;
    using Locate  = std::function<Origin()>;  // where/when the actuator is now

    Sink(Publish publish, Locate locate,
         std::chrono::microseconds coalesce = std::chrono::microseconds(0))
      : publish_(std::move(publish)), locate_(std::move(locate)), coalesce_(coalesce)
    {
        if (coalesce_.count() > 0) thread_ = std::thread([this] { loop(); });
    }

    ~Sink()
    {
        if (!thread_.joinable()) return;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            stop_ = true;
        }
        cv_.notify_all();
        thread_.join();
    }

    Sink(const Sink &) = delete;
    Sink & operator=(const Sink &) = delete;

    // Applies `set(msg, origin)` to the current setpoint and publishes it
    // (now, or at the end of the coalescing window). Returns the origin of
    // the actuation.
    template<typename F>
    Origin apply(F && set)
    {
        const Origin o = locate_();
        if (coalesce_.count() == 0) {
            Msg copy;
            {
                std::lock_guard<std::mutex> lk(mutex_);
                set(msg_, o);
                copy = msg_;
            }
            publish_(copy);
            return o;
        }
        {
            std::lock_guard<std::mutex> lk(mutex_);
            set(msg_, o);
            if (!pending_) {
                pending_  = true;
                deadline_ = std::chrono::steady_clock::now() + coalesce_;
            }
        }
        cv_.notify_all();
        return o;
    }

    Origin locate() const { return locate_(); }

private:
    void loop()
    {
        std::unique_lock<std::mutex> lk(mutex_);
        while (!stop_) {
            if (!pending_) {
                cv_.wait(lk);
                continue;
            }
            if (cv_.wait_until(lk, deadline_, [this] { return stop_; })) break;
            Msg copy = msg_;
            pending_ = false;
            lk.unlock();
            publish_(copy);
            lk.lock();
        }
    }

    Publish                               publish_;
    Locate                                locate_;
    const std::chrono::microseconds       coalesce_;
    std::mutex                            mutex_;
    std::condition_variable               cv_;
    Msg                                   msg_{};
    bool                                  pending_{false};
    bool                                  stop_{false};
    std::chrono::steady_clock::time_point deadline_{};
    std::thread                           thread_;
};

}  // namespace sd
