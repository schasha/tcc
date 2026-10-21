#pragma once

// smartdata.hpp — SmartData objects (Fröhlich 2018 §2, Figure 3).
//
//   Responsive_SmartData<Transducer>   first constructor: a local transducer
//       (dev, expiry, mode, period). Reading it returns the transducer value;
//       writing it sets a new setpoint for an actuator. When ADVERTISED it
//       binds to interests from the network and answers them — on every
//       update (event-driven interests, period 0) or every period
//       (time-triggered interests). When COMMANDED it also executes commands.
//
//   Interested_SmartData<UNIT>         second constructor: a local proxy to
//       remote transducers (region, expiry, period, mode, fuser). It announces
//       an interest for UNIT inside a space-time region and is updated by the
//       matching responses. Writing it sends a command to the COMMANDED
//       SmartData in the region.
//
// Expiry is relative to the time the data was produced; 0 means the data
// never expires. Specifying an expiry smaller than the period is invalid.

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

#include "smartdata/network.hpp"
#include "smartdata/transducer.hpp"
#include "smartdata/wire.hpp"

namespace sd {

// What a SmartData looks like at a given moment (used for instrumentation).
struct Snapshot {
    uint32_t    unit{0};
    Device_Id   device{0};
    Reading     reading{};
    Microsecond expiry{0};
};

inline void check_timing(Microsecond expiry, Microsecond period)
{
    if (expiry < 0 || period < 0)
        throw std::invalid_argument("SmartData expiry and period must be non-negative");
    if (period > 0 && expiry > 0 && expiry < period)
        throw std::invalid_argument("SmartData expiry smaller than its period is invalid");
}

// ── Local transducers ───────────────────────────────────────────────────────

template<typename T>
class Responsive_SmartData : private Network::Observer, private Transducer_Observer
{
public:
    static constexpr uint32_t UNIT = T::UNIT;

    using Update_Hook = std::function<void(const Snapshot &)>;

    template<typename... Args>
    Responsive_SmartData(Network & net, Device_Id dev, Microsecond expiry,
                         uint8_t mode = PRIVATE, Microsecond period = 0,
                         Args &&... transducer_args)
      : net_(net), mode_(mode), expiry_(expiry), period_(period),
        transducer_(std::make_unique<T>(dev, std::forward<Args>(transducer_args)...))
    {
        check_timing(expiry_, period_);
        if (passive_sensor()) transducer_->refresh(transducer_->sense(), net_.now());
        transducer_->attach(this);
        {
            std::lock_guard<std::mutex> lk(mutex_);  // tick() reads job_
            job_ = net_.scheduler().add(std::chrono::microseconds(period_), [this] { tick(); });
        }
        if (mode_ & ADVERTISED) {
            net_.attach(this, UNIT);
            announce(ADVERTISE);
        }
    }

    ~Responsive_SmartData() override
    {
        if (mode_ & ADVERTISED) {
            net_.detach(this);
            announce(CONCEAL);
        }
        net_.scheduler().remove(job_);
        transducer_->detach(this);
    }

    Responsive_SmartData(const Responsive_SmartData &) = delete;
    Responsive_SmartData & operator=(const Responsive_SmartData &) = delete;

    // Reading a SmartData samples the transducer (passive ones only when the
    // current value has expired; active ones keep it up to date themselves).
    double value()
    {
        if (passive_sensor() && expired()) transducer_->refresh(transducer_->sense(), net_.now());
        return transducer_->read().value;
    }
    operator double() { return value(); }

    // Writing a SmartData defines a new setpoint for the actuator.
    Responsive_SmartData & operator=(double v)
    {
        if (T::TYPE & T::ACTUATOR) transducer_->actuate(v);
        return *this;
    }

    Origin location() const { return transducer_->read().origin; }
    Time   time()     const { return transducer_->read().origin.t; }

    bool expired() const
    {
        return expiry_ > 0 && net_.now() > transducer_->read().origin.t + expiry_;
    }

    // Blocks until the next update of this SmartData (or the timeout).
    bool wait(std::chrono::microseconds timeout)
    {
        std::unique_lock<std::mutex> lk(mutex_);
        const uint64_t seen = updates_;
        return cv_.wait_for(lk, timeout, [&] { return updates_ != seen; });
    }

    void wait()
    {
        std::unique_lock<std::mutex> lk(mutex_);
        const uint64_t seen = updates_;
        cv_.wait(lk, [&] { return updates_ != seen; });
    }

    Device_Id   device() const { return transducer_->device(); }
    uint8_t     mode()   const { return mode_; }
    Microsecond expiry() const { return expiry_; }
    Microsecond period() const { return period_; }
    T &         transducer()   { return *transducer_; }

    // Number of interests currently bound to this SmartData.
    std::size_t interested() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return bindings_.size();
    }

    // Instrumentation: called on every update, whether or not anyone is
    // interested. Set it before the transducer starts producing data.
    void on_update(Update_Hook h)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        hook_ = std::move(h);
    }

    Snapshot snapshot() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return {UNIT, device(), transducer_->read(), response_expiry_locked()};
    }

private:
    using Steady = std::chrono::steady_clock;

    struct Binding {
        Region            region;
        Microsecond       expiry;
        Microsecond       period;
        uint8_t           mode;  // SINGLE or ALL
        Steady::time_point refreshed;
    };

    static constexpr bool passive_sensor()
    {
        return !T::active && (T::TYPE & T::SENSOR);
    }

    // ── transducer side ──

    void transducer_updated() override
    {
        bool        respond_now = false;
        Update_Hook hook;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            ++updates_;
            cv_.notify_all();
            purge_locked();
            for (const auto & b : bindings_) respond_now |= (b.period == 0);
            hook = hook_;
        }
        if (hook) hook(snapshot());
        if (respond_now) respond();
    }

    // Periodic duty: own sampling period and/or time-triggered interests.
    void tick()
    {
        if (passive_sensor()) transducer_->refresh(transducer_->sense(), net_.now());
        bool respond_now = false;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            if (passive_sensor()) {
                ++updates_;
                cv_.notify_all();
            }
            purge_locked();
            for (const auto & b : bindings_) respond_now |= (b.period > 0);
            reschedule_locked();
        }
        if (respond_now) respond();
    }

    // ── network side ──

    void receive(const uint8_t * data, std::size_t len) override
    {
        const auto h = wire::parse<wire::Header>(data, len);
        if (!h || h->unit != UNIT) return;
        if (h->device != ANY_DEVICE && h->device != device()) return;

        switch (wire::type_of(*h)) {
        case INTEREST:
            if (const auto in = wire::parse<wire::Interest>(data, len)) on_interest(*in);
            break;
        case COMMAND:
            if (const auto c = wire::parse<wire::Command>(data, len)) on_command(*c);
            break;
        default:
            break;  // responses are for interested SmartData
        }
    }

    void on_interest(const wire::Interest & in)
    {
        const Region      region = Region::from_wire(in.region);
        const Microsecond expiry = in.expiry;
        const Microsecond period = in.period;
        const uint8_t     imode  = in.header.mode & MODE_MASK;
        const bool        revoke = (in.header.mode & OPERATION_MASK) == REVOKE;

        bool fresh = false;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            auto it = bindings_.begin();
            for (; it != bindings_.end(); ++it) {
                if (it->region == region && it->expiry == expiry &&
                    it->period == period && it->mode == imode) break;
            }

            if (revoke) {
                if (it != bindings_.end()) bindings_.erase(it);
            } else if (it != bindings_.end()) {
                it->refreshed = Steady::now();
            } else if (accepts_locked(region, expiry, period)) {
                bindings_.push_back({region, expiry, period, imode, Steady::now()});
                fresh = true;
            }
            reschedule_locked();
        }
        // a new consumer gets the current value right away
        if (fresh) respond();
    }

    bool accepts_locked(const Region & region, Microsecond expiry, Microsecond period) const
    {
        if (expiry < 0 || period < 0 || (period > 0 && expiry > 0 && expiry < period)) return false;
        if (region.t1 < net_.now()) return false;
        const Reading r = transducer_->read();
        // before the first sample the location is unknown: bind optimistically
        if (r.origin.t != 0 && !region.contains_space(r.origin.x, r.origin.y, r.origin.z)) {
            return false;
        }
        return true;
    }

    void on_command(const wire::Command & c)
    {
        if ((mode_ & MODE_MASK) != COMMANDED || !(T::TYPE & T::ACTUATOR)) return;

        Region region;
        region.x      = from_cm(c.header.origin.space.x);
        region.y      = from_cm(c.header.origin.space.y);
        region.z      = from_cm(c.header.origin.space.z);
        region.radius = c.radius / 100.0;
        region.t0     = c.header.origin.t;
        region.t1     = c.t1;

        if (region.t1 < net_.now()) return;
        const Reading r = transducer_->read();
        if (r.origin.t != 0 && !region.contains_space(r.origin.x, r.origin.y, r.origin.z)) return;

        transducer_->actuate(c.value);  // the transducer reports the new state
    }

    // ── helpers ──

    void respond()
    {
        const Snapshot s = snapshot();
        if (s.reading.origin.t == 0) return;  // never sampled: nothing to tell yet
        wire::Response r{};
        r.header   = make_header(RESPONSE, (mode_ & MODE_MASK) | RESPOND,
                                 s.reading.uncertainty, s.reading.origin.to_wire(),
                                 s.reading.origin.confidence, UNIT, device());
        r.expiry   = s.expiry;
        r.fragment = 0;
        r.value    = s.reading.value;
        net_.send(&r, sizeof(r));
    }

    void announce(uint8_t op)
    {
        const Reading r = transducer_->read();
        wire::Response m{};
        m.header = make_header(RESPONSE, (mode_ & MODE_MASK) | op, r.uncertainty,
                               r.origin.to_wire(), r.origin.confidence, UNIT, device());
        m.expiry = expiry_;
        net_.send(&m, wire::RESPONSE_NO_VALUE);  // advertisements carry no value
    }

    // The shortest of the expiries (ours and the interests') goes in the
    // response, as the paper prescribes.
    Microsecond response_expiry_locked() const
    {
        Microsecond x = expiry_;
        for (const auto & b : bindings_) {
            if (b.expiry > 0 && (x == 0 || b.expiry < x)) x = b.expiry;
        }
        return x;
    }

    void purge_locked()
    {
        const auto lease = std::chrono::duration_cast<Steady::duration>(
            net_.config().interest_lease);
        const auto now_s = Steady::now();
        const Time now   = net_.now();
        auto stale = [&](const Binding & b) {
            return now_s - b.refreshed > lease || b.region.t1 < now;
        };
        const auto before = bindings_.size();
        bindings_.erase(std::remove_if(bindings_.begin(), bindings_.end(), stale),
                        bindings_.end());
        if (bindings_.size() != before) reschedule_locked();
    }

    // Different periods are served by sampling at their greatest common divisor.
    void reschedule_locked()
    {
        Microsecond p = period_;
        for (const auto & b : bindings_) {
            if (b.period > 0) p = p ? std::gcd(p, b.period) : b.period;
        }
        net_.scheduler().set_period(job_, std::chrono::microseconds(p));
    }

    Network &               net_;
    const uint8_t           mode_;
    const Microsecond       expiry_;
    const Microsecond       period_;
    std::unique_ptr<T>      transducer_;
    Scheduler::Job_Id       job_{0};

    mutable std::mutex      mutex_;
    std::condition_variable cv_;
    uint64_t                updates_{0};
    std::vector<Binding>    bindings_;
    Update_Hook             hook_;
};

// ── Remote transducers ──────────────────────────────────────────────────────

template<uint32_t U>
class Interested_SmartData : private Network::Observer
{
public:
    static constexpr uint32_t UNIT = U;

    struct Sample {
        double      value{0.0};
        Origin      origin{};
        Device_Id   device{0};
        Microsecond expiry{0};       // as announced by the responder
        uint8_t     uncertainty{0};
    };

    // Data fusion: receives the current value and the new response and
    // returns the new value of the SmartData.
    using Fuser    = std::function<double(double current, const Sample & incoming)>;
    // Instrumentation / composition: called for every accepted response.
    using Listener = std::function<void(const Sample &)>;

    Interested_SmartData(Network & net, const Region & region, Microsecond expiry,
                         Microsecond period = 0, uint8_t mode = SINGLE,
                         Device_Id device = ANY_DEVICE, Fuser fuser = {},
                         Listener listener = {})
      : net_(net), region_(region), expiry_(expiry), period_(period),
        mode_(mode & MODE_MASK), device_(device),
        fuser_(std::move(fuser)), listener_(std::move(listener))
    {
        check_timing(expiry_, period_);
        net_.attach(this, UNIT);
        announce(ANNOUNCE);
        job_ = net_.scheduler().add(
            std::chrono::duration_cast<std::chrono::microseconds>(net_.config().interest_refresh),
            [this] { announce(ANNOUNCE); });
    }

    ~Interested_SmartData() override
    {
        net_.scheduler().remove(job_);
        net_.detach(this);
        announce(REVOKE);
    }

    Interested_SmartData(const Interested_SmartData &) = delete;
    Interested_SmartData & operator=(const Interested_SmartData &) = delete;

    double value() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return last_.value;
    }
    operator double() const { return value(); }

    // Writing a remote SmartData commands the actuators in the region.
    Interested_SmartData & operator=(double v)
    {
        wire::Command c{};
        c.header = make_header(COMMAND, mode_, 0,
                               wire::Spacetime{region_.to_wire().center, region_.t0}, 0,
                               UNIT, device_);
        c.radius = region_.to_wire().radius;
        c.t1     = region_.t1;
        c.expiry = expiry_;
        c.period = static_cast<int32_t>(period_);
        c.value  = v;
        net_.send(&c, sizeof(c));
        return *this;
    }

    Origin location() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return last_.origin;
    }
    Time time() const { return location().t; }

    // True once at least one response has been received.
    bool valid() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return responses_ > 0;
    }

    // Expiry is client-side: each proxy decides how long data stays usable.
    bool expired() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return responses_ == 0 || (expiry_ > 0 && net_.now() > last_.origin.t + expiry_);
    }

    bool wait(std::chrono::microseconds timeout)
    {
        std::unique_lock<std::mutex> lk(mutex_);
        const uint64_t seen = responses_;
        return cv_.wait_for(lk, timeout, [&] { return responses_ != seen; });
    }

    uint64_t      responses() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return responses_;
    }
    const Region & region() const { return region_; }
    Microsecond    expiry() const { return expiry_; }
    Microsecond    period() const { return period_; }
    Device_Id      device() const { return device_; }

private:
    void announce(uint8_t op)
    {
        wire::Interest in{};
        in.header = make_header(INTEREST, mode_ | op, 0,
                                wire::Spacetime{{0, 0, 0}, net_.now()}, 0, UNIT, device_);
        in.region = region_.to_wire();
        in.expiry = expiry_;
        in.period = static_cast<int32_t>(period_);
        net_.send(&in, sizeof(in));
    }

    void receive(const uint8_t * data, std::size_t len) override
    {
        const auto r = wire::parse<wire::Response>(data, len);  // needs the value
        if (!r) return;
        const auto & h = r->header;
        if (wire::type_of(h) != RESPONSE) return;
        if ((h.mode & OPERATION_MASK) != RESPOND) return;
        if (h.unit != UNIT) return;
        if (device_ != ANY_DEVICE && h.device != device_) return;

        Sample s;
        s.value       = r->value;
        s.origin      = Origin::from_wire(h.origin, h.location_confidence);
        s.device      = h.device;
        s.expiry      = r->expiry;
        s.uncertainty = h.misc;
        if (!region_.contains(s.origin)) return;

        {
            std::lock_guard<std::mutex> lk(mutex_);
            const double v = fuser_ ? fuser_(last_.value, s) : s.value;
            last_       = s;
            last_.value = v;
            ++responses_;
            cv_.notify_all();
        }
        if (listener_) listener_(s);
    }

    Network &         net_;
    const Region      region_;
    const Microsecond expiry_;
    const Microsecond period_;
    const uint8_t     mode_;
    const Device_Id   device_;
    const Fuser       fuser_;
    const Listener    listener_;
    Scheduler::Job_Id job_{0};

    mutable std::mutex      mutex_;
    std::condition_variable cv_;
    Sample                  last_{};
    uint64_t                responses_{0};
};

}  // namespace sd
