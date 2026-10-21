#pragma once

// transducer.hpp — Transducer abstraction (Fröhlich 2018 §2.1).
//
// A transducer is the class a SmartData is bound to. It declares:
//
//   UNIT    the 32-bit SI unit it measures or controls (Figure 1);
//   TYPE    SENSOR, ACTUATOR or both;
//   active  true when the transducer signals new data by itself (the
//           "interrupt" case of the paper) instead of being sampled;
//
// and implements sense() and/or actuate(). One class per physical quantity;
// several instances of the same class on a node are told apart by `dev`.
//
// In the gateway every transducer is active: a ROS message arriving on the
// subscribed topic plays the role of the interrupt. A transducer also knows
// where and when its sample was produced (header.stamp + tf2), because a ROS
// sensor is not located at the node running the gateway.

#include <algorithm>
#include <cstdint>
#include <mutex>
#include <vector>

#include "smartdata/wire.hpp"

namespace sd {

class Transducer_Observer {
public:
    virtual ~Transducer_Observer() = default;
    virtual void transducer_updated() = 0;
};

struct Reading {
    double  value{0.0};
    Origin  origin{};
    uint8_t uncertainty{0};
};

template<uint32_t U>
class Transducer {
public:
    static constexpr uint32_t UNIT = U;

    enum : unsigned { SENSOR = 1u << 0, ACTUATOR = 1u << 1 };

    explicit Transducer(Device_Id dev) : dev_(dev) {}
    virtual ~Transducer() = default;

    Transducer(const Transducer &) = delete;
    Transducer & operator=(const Transducer &) = delete;

    Device_Id device() const { return dev_; }

    // Passive sensors override sense() to sample the hardware; the default
    // returns the last value pushed by an active transducer.
    virtual double sense() { return read().value; }

    // Actuators override actuate() to drive the physical device.
    virtual void actuate(double) {}

    Reading read() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return reading_;
    }

    void attach(Transducer_Observer * o)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        observers_.push_back(o);
    }

    void detach(Transducer_Observer * o)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        observers_.erase(std::remove(observers_.begin(), observers_.end(), o),
                         observers_.end());
    }

    // Used by passive sampling: stores a fresh value without notifying
    // (the SmartData that asked for it already knows).
    void refresh(double v, Time t)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        reading_.value    = v;
        reading_.origin.t = t;
    }

protected:
    // Called by active transducers when new data is available.
    void sample(double v, const Origin & o, uint8_t uncertainty = 0)
    {
        std::vector<Transducer_Observer *> obs;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            reading_ = {v, o, uncertainty};
            obs = observers_;
        }
        for (auto * x : obs) x->transducer_updated();
    }

private:
    const Device_Id                    dev_;
    mutable std::mutex                 mutex_;
    Reading                            reading_{};
    std::vector<Transducer_Observer *> observers_;
};

}  // namespace sd
