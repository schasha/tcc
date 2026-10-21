// network.cpp — UDP multicast segment and timer thread of the SmartData run-time.

#include "smartdata/network.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace sd {

// ── Scheduler ────────────────────────────────────────────────────────────────

Scheduler::Scheduler() : thread_([this] { loop(); }) {}

Scheduler::~Scheduler()
{
    {
        std::lock_guard<std::mutex> lk(mutex_);
        stop_ = true;
    }
    cv_.notify_all();
    thread_.join();
}

Scheduler::Job_Id Scheduler::add(std::chrono::microseconds period, Job job)
{
    std::lock_guard<std::mutex> lk(mutex_);
    const Job_Id id = next_id_++;
    jobs_.emplace(id, Entry{period, Clock::now() + period, std::move(job)});
    cv_.notify_all();
    return id;
}

void Scheduler::set_period(Job_Id id, std::chrono::microseconds period)
{
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = jobs_.find(id);
    if (it == jobs_.end() || it->second.period == period) return;
    it->second.period = period;
    it->second.next   = Clock::now() + period;
    cv_.notify_all();
}

void Scheduler::remove(Job_Id id)
{
    std::unique_lock<std::mutex> lk(mutex_);
    jobs_.erase(id);
    cv_.wait(lk, [&] { return running_ != id; });
}

void Scheduler::loop()
{
    std::unique_lock<std::mutex> lk(mutex_);
    while (!stop_) {
        auto due = jobs_.end();
        for (auto it = jobs_.begin(); it != jobs_.end(); ++it) {
            if (it->second.period.count() <= 0) continue;
            if (due == jobs_.end() || it->second.next < due->second.next) due = it;
        }

        if (due == jobs_.end()) {
            cv_.wait(lk);
            continue;
        }
        if (Clock::now() < due->second.next) {
            cv_.wait_until(lk, due->second.next);
            continue;  // the job set may have changed while waiting
        }

        const Job_Id id  = due->first;
        Job          job = due->second.job;
        // skip missed activations instead of bursting to catch up
        do {
            due->second.next += due->second.period;
        } while (due->second.next <= Clock::now());

        running_ = id;
        lk.unlock();
        job();
        lk.lock();
        running_ = 0;
        cv_.notify_all();
    }
}

// ── Network ──────────────────────────────────────────────────────────────────

namespace {

[[noreturn]] void throw_errno(const char * what)
{
    throw std::runtime_error(std::string(what) + ": " + std::strerror(errno));
}

}  // namespace

Time Network::system_clock_us()
{
    using namespace std::chrono;
    return duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
}

Network::Network(Config cfg, Clock clock)
  : cfg_(std::move(cfg)),
    clock_(clock ? std::move(clock) : Clock(&Network::system_clock_us))
{
    open_sockets();
    thread_ = std::thread([this] { recv_loop(); });
}

Network::~Network()
{
    stop_.store(true);
    if (recv_fd_ >= 0) ::shutdown(recv_fd_, SHUT_RDWR);
    if (thread_.joinable()) thread_.join();
    if (send_fd_ >= 0) ::close(send_fd_);
    if (recv_fd_ >= 0) ::close(recv_fd_);
}

void Network::open_sockets()
{
    in_addr group{};
    if (::inet_pton(AF_INET, cfg_.group.c_str(), &group) != 1) {
        throw std::runtime_error("invalid multicast group: " + cfg_.group);
    }
    in_addr ifa{};
    ifa.s_addr = htonl(INADDR_ANY);
    if (!cfg_.ifaddr.empty() && ::inet_pton(AF_INET, cfg_.ifaddr.c_str(), &ifa) != 1) {
        throw std::runtime_error("invalid interface address: " + cfg_.ifaddr);
    }

    // sender
    send_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (send_fd_ < 0) throw_errno("socket(send)");
    const unsigned char ttl  = static_cast<unsigned char>(cfg_.ttl);
    const unsigned char loop = 1;  // local consumers must see local producers
    if (::setsockopt(send_fd_, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl)) < 0)
        throw_errno("setsockopt(IP_MULTICAST_TTL)");
    if (::setsockopt(send_fd_, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop)) < 0)
        throw_errno("setsockopt(IP_MULTICAST_LOOP)");
    if (!cfg_.ifaddr.empty() &&
        ::setsockopt(send_fd_, IPPROTO_IP, IP_MULTICAST_IF, &ifa, sizeof(ifa)) < 0)
        throw_errno("setsockopt(IP_MULTICAST_IF)");

    // receiver
    recv_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (recv_fd_ < 0) throw_errno("socket(recv)");
    const int yes = 1;
    if (::setsockopt(recv_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) < 0)
        throw_errno("setsockopt(SO_REUSEADDR)");
    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(cfg_.port);
    if (::bind(recv_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
        throw_errno("bind");
    ip_mreq mreq{};
    mreq.imr_multiaddr = group;
    mreq.imr_interface = ifa;
    if (::setsockopt(recv_fd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0)
        throw_errno("setsockopt(IP_ADD_MEMBERSHIP)");
    timeval tv{0, 100 * 1000};  // lets recv_loop observe stop_
    ::setsockopt(recv_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

void Network::attach(Observer * o, uint32_t unit)
{
    std::lock_guard<std::mutex> lk(observers_mutex_);
    (unit ? by_unit_[unit] : observers_).push_back(o);
}

void Network::detach(Observer * o)
{
    std::lock_guard<std::mutex> lk(observers_mutex_);
    auto drop = [o](std::vector<Observer *> & v) {
        v.erase(std::remove(v.begin(), v.end(), o), v.end());
    };
    drop(observers_);
    for (auto & [unit, v] : by_unit_) drop(v);
}

void Network::send(const void * data, std::size_t len)
{
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port   = htons(cfg_.port);
    ::inet_pton(AF_INET, cfg_.group.c_str(), &dst.sin_addr);

    const ssize_t n = ::sendto(send_fd_, data, len, 0,
                               reinterpret_cast<sockaddr *>(&dst), sizeof(dst));
    // best effort, like the paper's sensor networks: a lost datagram is
    // recovered by the next sample or the next interest refresh
    if (n == static_cast<ssize_t>(len)) ++sent_;
}

void Network::recv_loop()
{
    uint8_t buf[1500];
    while (!stop_.load()) {
        const ssize_t n = ::recv(recv_fd_, buf, sizeof(buf), 0);
        if (n < static_cast<ssize_t>(sizeof(wire::Header))) continue;
        ++received_;
        const auto h = wire::parse<wire::Header>(buf, static_cast<std::size_t>(n));
        std::lock_guard<std::mutex> lk(observers_mutex_);
        for (Observer * o : observers_) o->receive(buf, static_cast<std::size_t>(n));
        const auto it = by_unit_.find(h->unit);
        if (it == by_unit_.end()) continue;
        for (Observer * o : it->second) o->receive(buf, static_cast<std::size_t>(n));
    }
}

}  // namespace sd
