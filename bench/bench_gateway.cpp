// bench_gateway.cpp — isolated performance experiments of the gateway.
//
// One configuration per invocation (fresh process per repetition); the
// result is appended as one CSV row to --out. bench/run_all.sh sweeps the
// factors and bench/analyze.py aggregates the repetitions.
//
//   translate  H1/H3: latency of handing one ROS message to the generated
//              transducers (Source → transducers → Responsive SmartData).
//              --msg scan|imu|temperature, --beams N (scan), --k K: number of
//              beams with an interested proxy (each answered by one 78-byte
//              Response datagram); K = 0 isolates the translation.
//   sendto     H3 baseline: K datagrams of 78 bytes per iteration through a
//              raw UDP multicast socket (--via raw) or sd::Network::send
//              (--via network), without the gateway. With --listen 1 the raw
//              variant also has a socket of the same process joined to the
//              group and drained by a thread, as sd::Network has: multicast
//              loopback then delivers a local copy of every datagram.
//   openloop   H2: messages arrive at a fixed rate (--rate scans/s) for
//              --duration s, independently of how fast they are served, and
//              wait in a keep-last queue of --depth (as SensorDataQoS); the
//              latency counts from the scheduled arrival, so queueing shows
//              up, and overwritten messages are counted as lost.
//
// Common: --iters N, --warmup N, --cpu C (pin the measuring thread),
// --raw FILE (every sample, µs, one per line), --tag TEXT (free column).

#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sched.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include "si_units.hpp"
#include "smartdata/network.hpp"
#include "smartdata/smartdata.hpp"
#include "parsed_transducers.hpp"

namespace gen = semantic_gateway::gen;
using namespace std::chrono_literals;
using Steady = std::chrono::steady_clock;

namespace {

// ── arguments ────────────────────────────────────────────────────────────────

struct Args {
    std::string experiment;
    std::map<std::string, std::string> kv;

    std::string str(const std::string & k, const std::string & def) const
    {
        auto it = kv.find(k);
        return it == kv.end() ? def : it->second;
    }
    long num(const std::string & k, long def) const
    {
        auto it = kv.find(k);
        return it == kv.end() ? def : std::stol(it->second);
    }
    double real(const std::string & k, double def) const
    {
        auto it = kv.find(k);
        return it == kv.end() ? def : std::stod(it->second);
    }
};

Args parse(int argc, char ** argv)
{
    if (argc < 2) throw std::invalid_argument("usage: bench_gateway <translate|sendto|openloop> [--key value]...");
    Args a;
    a.experiment = argv[1];
    for (int i = 2; i < argc; ++i) {
        std::string k = argv[i];
        if (k.rfind("--", 0) != 0 || i + 1 >= argc) throw std::invalid_argument("bad argument: " + k);
        a.kv[k.substr(2)] = argv[++i];
    }
    return a;
}

void pin(long cpu)
{
    if (cpu < 0) return;
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(static_cast<int>(cpu), &set);
    if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0) {
        std::cerr << "warning: could not pin to cpu " << cpu << "\n";
    }
}

// ── statistics ───────────────────────────────────────────────────────────────

struct Summary {
    std::size_t n{0};
    double min{0}, p5{0}, p50{0}, p95{0}, p99{0}, p999{0}, max{0}, mean{0};
};

double quantile(const std::vector<double> & sorted, double q)
{
    const std::size_t i = std::min(sorted.size() - 1, static_cast<std::size_t>(q * sorted.size()));
    return sorted[i];
}

Summary summarize(std::vector<double> us)
{
    Summary s;
    if (us.empty()) return s;
    std::sort(us.begin(), us.end());
    s.n    = us.size();
    s.min  = us.front();
    s.p5   = quantile(us, 0.05);
    s.p50  = quantile(us, 0.50);
    s.p95  = quantile(us, 0.95);
    s.p99  = quantile(us, 0.99);
    s.p999 = quantile(us, 0.999);
    s.max  = us.back();
    s.mean = std::accumulate(us.begin(), us.end(), 0.0) / static_cast<double>(us.size());
    return s;
}

void write_raw(const std::string & path, const std::vector<double> & us)
{
    if (path.empty()) return;
    std::ofstream f(path);
    for (double v : us) f << v << '\n';
}

// One CSV row: the configuration columns, then the summary.
void write_row(const std::string & path, const std::vector<std::pair<std::string, std::string>> & cfg,
               const Summary & s, const std::vector<std::pair<std::string, double>> & extra = {})
{
    const bool fresh = [&] { std::ifstream f(path); return !f.good() || f.peek() == EOF; }();
    std::ofstream f(path, std::ios::app);
    if (fresh) {
        for (const auto & c : cfg) f << c.first << ',';
        f << "n,min,p5,p50,p95,p99,p999,max,mean";
        for (const auto & e : extra) f << ',' << e.first;
        f << '\n';
    }
    for (const auto & c : cfg) f << c.second << ',';
    f << s.n << ',' << s.min << ',' << s.p5 << ',' << s.p50 << ',' << s.p95 << ','
      << s.p99 << ',' << s.p999 << ',' << s.max << ',' << s.mean;
    for (const auto & e : extra) f << ',' << e.second;
    f << '\n';
}

// ── rigs ─────────────────────────────────────────────────────────────────────

sd::Network::Config segment()
{
    std::mt19937 rng{std::random_device{}()};
    sd::Network::Config c;
    c.group = "224.1.1.10";
    c.port  = static_cast<uint16_t>(std::uniform_int_distribution<int>(45000, 49000)(rng));
    c.interest_refresh = 100ms;
    return c;
}

sd::Origin origin_at(sd::Time t)
{
    sd::Origin o;
    o.x = 1.0;
    o.y = 2.0;
    o.t = t;
    o.confidence = 100;
    return o;
}

// A message type, its transducers and (optionally) K interested proxies.
// `deliver(i)` hands message i to the transducers.
class Rig {
public:
    virtual ~Rig() = default;
    virtual void deliver(std::size_t i) = 0;
    sd::Network & net() { return *net_; }

protected:
    std::unique_ptr<sd::Network> net_{std::make_unique<sd::Network>(segment())};
};

template<typename Msg>
class Typed_Rig : public Rig {
public:
    template<typename T>
    void add(sd::Device_Id dev)
    {
        auto p = std::make_unique<sd::Responsive_SmartData<T>>(*net_, dev, 100'000, sd::ADVERTISED, 0, src_);
        bound_.push_back([q = p.get()] { return q->interested() > 0; });
        owned_.push_back(std::shared_ptr<void>(std::move(p)));
    }

    // K device-specific interests on the first K SmartData of unit UNIT.
    template<uint32_t UNIT>
    void interest(sd::Device_Id first, std::size_t k)
    {
        for (std::size_t i = 0; i < k; ++i) {
            owned_.push_back(std::make_shared<sd::Interested_SmartData<UNIT>>(
                *net_, sd::Region::everywhere(), 100'000, 0, sd::ALL,
                static_cast<sd::Device_Id>(first + i)));
        }
        const auto deadline = Steady::now() + 5s;
        while (Steady::now() < deadline &&
               !std::all_of(bound_.begin(), bound_.begin() + static_cast<long>(k),
                            [](auto & f) { return f(); })) {
            std::this_thread::sleep_for(5ms);
        }
        if (!std::all_of(bound_.begin(), bound_.begin() + static_cast<long>(k), [](auto & f) { return f(); })) {
            throw std::runtime_error("interests did not bind");
        }
    }

    void deliver(std::size_t i) override
    {
        src_.publish(msg, origin_at(static_cast<sd::Time>(1'000'000 + i)));
    }

    Msg msg{};

private:
    sd::Source<Msg> src_;
    std::vector<std::function<bool()>> bound_;
    std::vector<std::shared_ptr<void>> owned_;
};

std::unique_ptr<Rig> make_rig(const std::string & kind, std::size_t beams, std::size_t k)
{
    if (kind == "scan") {
        auto r = std::make_unique<Typed_Rig<sensor_msgs::msg::LaserScan>>();
        using T = gen::Scan_Transducer;
        for (std::size_t i = 0; i < beams; ++i) r->add<T>(T::DEV_BASE + static_cast<sd::Device_Id>(i));
        r->interest<T::UNIT>(T::DEV_BASE, std::min(k, beams));
        r->msg.header.frame_id = "laser_frame";
        r->msg.range_min = 0.1f;
        r->msg.range_max = 30.0f;
        r->msg.ranges.assign(beams, 5.0f);
        return r;
    }
    if (kind == "imu") {
        auto r = std::make_unique<Typed_Rig<sensor_msgs::msg::Imu>>();
        using A = gen::ImuDataLinearAcceleration_Transducer;
        using G = gen::ImuDataAngularVelocity_Transducer;
        for (sd::Device_Id i = 0; i < 3; ++i) r->add<A>(A::DEV_BASE + i);
        for (sd::Device_Id i = 0; i < 3; ++i) r->add<G>(G::DEV_BASE + i);
        if (k > 0) r->interest<A::UNIT>(A::DEV_BASE, std::min<std::size_t>(k, 3));
        r->msg.linear_acceleration.z = 9.81;
        return r;
    }
    if (kind == "temperature") {
        auto r = std::make_unique<Typed_Rig<sensor_msgs::msg::Temperature>>();
        using T = gen::Temperature_Transducer;
        r->add<T>(T::DEV_BASE);
        if (k > 0) r->interest<T::UNIT>(T::DEV_BASE, 1);
        r->msg.temperature = 25.0;
        return r;
    }
    throw std::invalid_argument("unknown --msg " + kind);
}

// ── experiments ──────────────────────────────────────────────────────────────

int translate(const Args & a)
{
    const std::string kind = a.str("msg", "scan");
    const auto beams  = static_cast<std::size_t>(a.num("beams", 100));
    const auto k      = static_cast<std::size_t>(a.num("k", 0));
    const auto iters  = static_cast<std::size_t>(a.num("iters", 20000));
    const auto warmup = static_cast<std::size_t>(a.num("warmup", 1000));

    auto rig = make_rig(kind, beams, k);
    pin(a.num("cpu", -1));

    for (std::size_t i = 0; i < warmup; ++i) rig->deliver(i);
    std::vector<double> us;
    us.reserve(iters);
    for (std::size_t i = 0; i < iters; ++i) {
        const auto t0 = Steady::now();
        rig->deliver(warmup + i);
        const auto t1 = Steady::now();
        us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
    }
    write_raw(a.str("raw", ""), us);
    const std::size_t fields = kind == "scan" ? beams : kind == "imu" ? 6 : 1;
    write_row(a.str("out", "translate.csv"),
              {{"tag", a.str("tag", "")}, {"msg", kind}, {"fields", std::to_string(fields)},
               {"k", std::to_string(k)}},
              summarize(us), {{"sent", static_cast<double>(rig->net().sent())}});
    return 0;
}

int sendto_baseline(const Args & a)
{
    const auto k      = static_cast<std::size_t>(a.num("k", 100));
    const auto iters  = static_cast<std::size_t>(a.num("iters", 2000));
    const auto warmup = static_cast<std::size_t>(a.num("warmup", 100));
    const std::string via = a.str("via", "raw");
    const bool listen = a.num("listen", 0) != 0;

    const auto cfg = segment();
    std::unique_ptr<sd::Network> net;
    int fd = -1;
    sockaddr_in dst{};
    if (via == "network") {
        net = std::make_unique<sd::Network>(cfg);
    } else {
        // configured like sd::Network's send socket
        fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        const unsigned char ttl = 1, loop = 1;
        ::setsockopt(fd, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));
        ::setsockopt(fd, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop));
        dst.sin_family = AF_INET;
        dst.sin_port   = htons(cfg.port);
        ::inet_pton(AF_INET, cfg.group.c_str(), &dst.sin_addr);
    }
    int rfd = -1;
    std::atomic<bool> stop{false};
    std::thread drain;
    if (listen && !net) {
        rfd = ::socket(AF_INET, SOCK_DGRAM, 0);
        const int one = 1;
        ::setsockopt(rfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in any{};
        any.sin_family = AF_INET;
        any.sin_port   = htons(cfg.port);
        any.sin_addr.s_addr = htonl(INADDR_ANY);
        if (::bind(rfd, reinterpret_cast<sockaddr *>(&any), sizeof(any)) != 0) throw std::runtime_error("bind");
        ip_mreq mreq{};
        ::inet_pton(AF_INET, cfg.group.c_str(), &mreq.imr_multiaddr);
        mreq.imr_interface.s_addr = htonl(INADDR_ANY);
        ::setsockopt(rfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));
        timeval tv{0, 100000};
        ::setsockopt(rfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        drain = std::thread([&] {
            uint8_t buf[2048];
            while (!stop) ::recv(rfd, buf, sizeof(buf), 0);
        });
    }
    pin(a.num("cpu", -1));

    uint8_t datagram[78]{};
    auto burst = [&] {
        for (std::size_t j = 0; j < k; ++j) {
            if (net) net->send(datagram, sizeof(datagram));
            else ::sendto(fd, datagram, sizeof(datagram), 0, reinterpret_cast<sockaddr *>(&dst), sizeof(dst));
        }
    };
    for (std::size_t i = 0; i < warmup; ++i) burst();
    std::vector<double> us;
    us.reserve(iters);
    for (std::size_t i = 0; i < iters; ++i) {
        const auto t0 = Steady::now();
        burst();
        const auto t1 = Steady::now();
        us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
    }
    stop = true;
    if (drain.joinable()) drain.join();
    if (rfd >= 0) ::close(rfd);
    if (fd >= 0) ::close(fd);
    write_raw(a.str("raw", ""), us);
    const std::string variant = net ? "network" : listen ? "raw+listener" : "raw";
    write_row(a.str("out", "sendto.csv"),
              {{"tag", a.str("tag", "")}, {"via", variant}, {"k", std::to_string(k)}},
              summarize(us));
    return 0;
}

int openloop(const Args & a)
{
    const std::string kind = a.str("msg", "scan");
    const auto beams    = static_cast<std::size_t>(a.num("beams", 100));
    const auto k        = static_cast<std::size_t>(a.num("k", 0));
    const double rate   = a.real("rate", 1000.0);
    const double secs   = a.real("duration", 2.0);
    const auto depth    = static_cast<std::size_t>(a.num("depth", 5));
    const long cpu      = a.num("cpu", -1);

    auto rig = make_rig(kind, beams, k);
    for (std::size_t i = 0; i < 1000; ++i) rig->deliver(i);

    std::mutex m;
    std::deque<Steady::time_point> queue;  // scheduled arrival of each waiting message
    std::size_t offered = 0, lost = 0;
    bool done = false;

    const auto start  = Steady::now() + 10ms;
    const auto period = std::chrono::duration<double>(1.0 / rate);
    const auto total  = static_cast<std::size_t>(rate * secs);

    std::thread producer([&] {
        pin(cpu >= 0 ? cpu + 1 : -1);
        std::size_t next = 0;
        while (next < total) {
            const auto due = start + std::chrono::duration_cast<Steady::duration>(period * static_cast<double>(next));
            // sleep most of the way, spin the rest: the arrivals keep their
            // schedule instead of inheriting the wake-up jitter of the OS
            std::this_thread::sleep_until(due - 200us);
            while (Steady::now() < due) {}
            const auto now = Steady::now();
            std::lock_guard<std::mutex> lk(m);
            // every arrival already due, even if this thread woke up late
            while (next < total &&
                   start + std::chrono::duration_cast<Steady::duration>(period * static_cast<double>(next)) <= now) {
                if (queue.size() == depth) { queue.pop_front(); ++lost; }
                queue.push_back(start + std::chrono::duration_cast<Steady::duration>(period * static_cast<double>(next)));
                ++next;
                ++offered;
            }
        }
        std::lock_guard<std::mutex> lk(m);
        done = true;
    });

    pin(cpu);
    std::vector<double> us;
    us.reserve(total);
    std::size_t i = 1000;
    for (;;) {
        Steady::time_point arrival;
        {
            // poll instead of blocking, so a wake-up is not counted as latency
            std::unique_lock<std::mutex> lk(m);
            while (queue.empty() && !done) {
                lk.unlock();
                std::this_thread::yield();
                lk.lock();
            }
            if (queue.empty()) break;
            arrival = queue.front();
            queue.pop_front();
        }
        rig->deliver(i++);
        us.push_back(std::chrono::duration<double, std::micro>(Steady::now() - arrival).count());
    }
    producer.join();
    const double elapsed = std::chrono::duration<double>(Steady::now() - start).count();

    write_raw(a.str("raw", ""), us);
    const std::size_t fields = kind == "scan" ? beams : kind == "imu" ? 6 : 1;
    write_row(a.str("out", "openloop.csv"),
              {{"tag", a.str("tag", "")}, {"msg", kind}, {"fields", std::to_string(fields)},
               {"k", std::to_string(k)}, {"rate", std::to_string(rate)}, {"depth", std::to_string(depth)}},
              summarize(us),
              {{"offered", static_cast<double>(offered)}, {"served", static_cast<double>(us.size())},
               {"lost", static_cast<double>(lost)},
               {"loss_ratio", offered ? static_cast<double>(lost) / static_cast<double>(offered) : 0.0},
               {"served_rate", static_cast<double>(us.size()) / elapsed}});
    return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
    try {
        const Args a = parse(argc, argv);
        if (a.experiment == "translate") return translate(a);
        if (a.experiment == "sendto")    return sendto_baseline(a);
        if (a.experiment == "openloop")  return openloop(a);
        throw std::invalid_argument("unknown experiment " + a.experiment);
    } catch (const std::exception & e) {
        std::cerr << "bench_gateway: " << e.what() << "\n";
        return 1;
    }
}
