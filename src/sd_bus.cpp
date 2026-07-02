// sd_bus.cpp — POSIX UDP multicast bus implementation.

#include "sd_bus.hpp"
#include "sd_wire.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace semantic_gateway {

namespace {

[[noreturn]] void throw_errno(const char * what)
{
    throw std::runtime_error(std::string(what) + ": " + std::strerror(errno));
}

}  // namespace

SDBus::SDBus(const std::string & group, uint16_t port, const std::string & ifaddr)
  : group_(group), port_(port), ifaddr_(ifaddr) {}

SDBus::~SDBus()
{
    stop();
    if (send_fd_ >= 0) ::close(send_fd_);
    if (recv_fd_ >= 0) ::close(recv_fd_);
}

void SDBus::open_send_socket()
{
    send_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (send_fd_ < 0) throw_errno("socket(send)");

    unsigned char ttl = 1;
    if (::setsockopt(send_fd_, IPPROTO_IP, IP_MULTICAST_TTL,
                     &ttl, sizeof(ttl)) < 0) {
        throw_errno("setsockopt(IP_MULTICAST_TTL)");
    }

    // loopback on so a receiver on the same host can read what we send
    unsigned char loop = 1;
    if (::setsockopt(send_fd_, IPPROTO_IP, IP_MULTICAST_LOOP,
                     &loop, sizeof(loop)) < 0) {
        throw_errno("setsockopt(IP_MULTICAST_LOOP)");
    }

    if (!ifaddr_.empty()) {
        in_addr ifa{};
        if (::inet_pton(AF_INET, ifaddr_.c_str(), &ifa) != 1) {
            throw std::runtime_error("invalid ifaddr: " + ifaddr_);
        }
        if (::setsockopt(send_fd_, IPPROTO_IP, IP_MULTICAST_IF,
                         &ifa, sizeof(ifa)) < 0) {
            throw_errno("setsockopt(IP_MULTICAST_IF)");
        }
    }
}

void SDBus::open_recv_socket()
{
    recv_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (recv_fd_ < 0) throw_errno("socket(recv)");

    int yes = 1;
    if (::setsockopt(recv_fd_, SOL_SOCKET, SO_REUSEADDR,
                     &yes, sizeof(yes)) < 0) {
        throw_errno("setsockopt(SO_REUSEADDR)");
    }

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(port_);
    if (::bind(recv_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        throw_errno("bind");
    }

    ip_mreq mreq{};
    if (::inet_pton(AF_INET, group_.c_str(), &mreq.imr_multiaddr) != 1) {
        throw std::runtime_error("invalid multicast group: " + group_);
    }
    if (ifaddr_.empty()) {
        mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    } else if (::inet_pton(AF_INET, ifaddr_.c_str(), &mreq.imr_interface) != 1) {
        throw std::runtime_error("invalid ifaddr: " + ifaddr_);
    }
    if (::setsockopt(recv_fd_, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                     &mreq, sizeof(mreq)) < 0) {
        throw_errno("setsockopt(IP_ADD_MEMBERSHIP)");
    }
}

void SDBus::broadcast(const msg::SmartData & sd)
{
    if (send_fd_ < 0) open_send_socket();

    const SDWire w = to_wire(sd);

    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port   = htons(port_);
    if (::inet_pton(AF_INET, group_.c_str(), &dst.sin_addr) != 1) {
        throw std::runtime_error("invalid multicast group: " + group_);
    }

    const ssize_t n = ::sendto(send_fd_, &w, sizeof(w), 0,
                                reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
    if (n != static_cast<ssize_t>(sizeof(w))) {
        // best-effort: a transient send error should not kill the gateway
        // (a real deployment would surface a counter through ROS stats)
    }
}

void SDBus::listen(Handler h)
{
    if (thread_.joinable()) {
        throw std::runtime_error("SDBus::listen already running");
    }
    handler_ = std::move(h);
    open_recv_socket();
    stop_.store(false);
    thread_ = std::thread([this] { recv_loop(); });
}

void SDBus::stop()
{
    stop_.store(true);
    if (recv_fd_ >= 0) {
        ::shutdown(recv_fd_, SHUT_RDWR);
    }
    if (thread_.joinable()) thread_.join();
}

void SDBus::recv_loop()
{
    // short recv timeout so we can observe stop_ between datagrams
    timeval tv{0, 100 * 1000};  // 100 ms
    ::setsockopt(recv_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    while (!stop_.load()) {
        SDWire w{};
        const ssize_t n = ::recv(recv_fd_, &w, sizeof(w), 0);
        if (n == static_cast<ssize_t>(sizeof(w))) {
            handler_(from_wire(w));
        }
        // n < 0 with EAGAIN/EWOULDBLOCK is the timeout path — just loop
    }
}

}  // namespace semantic_gateway
