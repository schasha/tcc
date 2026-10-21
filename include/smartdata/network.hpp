#pragma once

// network.hpp — run-time support shared by every SmartData of a process.
//
// The paper assumes "the underlying run-time support system" provides
// communication, time and scheduling (Fröhlich 2018 §1).:
//
//   * Network   — UDP multicast segment carrying the messages of wire.hpp.
//                 SmartData objects attach to it for their unit and decide
//                 what else concerns them (device, region, message type).
//   * Scheduler — a single timer thread running the periodic duties
//                 (time-triggered responses, interest refresh).
//   * clock     — injected, so ROS nodes can use the ROS clock (and thus
//                 simulated time under Gazebo).
//
// Differences from the EPOS/SDAV run-time:
//   * the network is an object, not a global, so tests can run isolated
//     segments side by side;
//   * interests are soft state: the interested side re-announces them every
//     `interest_refresh`, and a responder drops a binding not refreshed for
//     `interest_lease`. A best-effort UDP segment has no TSTP reliability,
//     so this is what keeps late joiners and lost packets from breaking the
//     interest/response binding.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "smartdata/wire.hpp"

namespace sd {

class Scheduler {
public:
    using Job_Id = uint64_t;
    using Job    = std::function<void()>;

    Scheduler();
    ~Scheduler();

    Scheduler(const Scheduler &) = delete;
    Scheduler & operator=(const Scheduler &) = delete;

    // Registers `job` to run every `period`. A zero period keeps it paused.
    Job_Id add(std::chrono::microseconds period, Job job);

    // Changes the period (zero pauses). Never blocks on a running job.
    void set_period(Job_Id id, std::chrono::microseconds period);

    // Unregisters the job, waiting for it if it is running right now.
    // Must not be called from inside the job itself.
    void remove(Job_Id id);

private:
    using Clock = std::chrono::steady_clock;

    struct Entry {
        std::chrono::microseconds period;
        Clock::time_point         next;
        Job                       job;
    };

    void loop();

    std::mutex                  mutex_;
    std::condition_variable     cv_;
    std::map<Job_Id, Entry>     jobs_;
    Job_Id                      next_id_{1};
    Job_Id                      running_{0};
    bool                        stop_{false};
    std::thread                 thread_;
};

struct Network_Config {
    std::string group{"224.1.1.1"};
    uint16_t    port{5000};
    std::string ifaddr{};  // local interface address; empty = any
    int         ttl{1};
    std::chrono::milliseconds interest_refresh{1000};
    std::chrono::milliseconds interest_lease{3500};
};

class Network {
public:
    using Config = Network_Config;
    using Clock = std::function<Time()>;

    // Receives every datagram of the segment (including our own, thanks to
    // multicast loopback, so producers and consumers can share a process).
    class Observer {
    public:
        virtual ~Observer() = default;
        virtual void receive(const uint8_t * data, std::size_t len) = 0;
    };

    explicit Network(Config cfg = {}, Clock clock = {});
    ~Network();

    Network(const Network &) = delete;
    Network & operator=(const Network &) = delete;

    // `unit` restricts delivery to datagrams about that unit (every
    // SmartData only cares about its own); 0 = every datagram.
    void attach(Observer * o, uint32_t unit = 0);
    void detach(Observer * o);  // waits for an in-flight delivery to finish

    void send(const void * data, std::size_t len);

    Time now() const { return clock_(); }

    Scheduler &     scheduler() { return scheduler_; }
    const Config &  config() const { return cfg_; }

    uint64_t sent()     const { return sent_.load(); }
    uint64_t received() const { return received_.load(); }

    static Time system_clock_us();

private:
    void open_sockets();
    void recv_loop();

    Config cfg_;
    Clock  clock_;

    int send_fd_{-1};
    int recv_fd_{-1};

    std::mutex                                     observers_mutex_;
    std::vector<Observer *>                        observers_;  // every unit
    std::unordered_map<uint32_t, std::vector<Observer *>> by_unit_;

    std::atomic<uint64_t> sent_{0};
    std::atomic<uint64_t> received_{0};

    std::atomic<bool> stop_{false};
    std::thread       thread_;

    Scheduler scheduler_;
};

}  // namespace sd
