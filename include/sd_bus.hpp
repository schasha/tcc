#pragma once

// sd_bus.hpp — UDP multicast bus carrying SmartData wire envelopes.
//
// Two roles:
//   * SDBus(group, port, ifaddr).broadcast(sd)        — gateway side
//   * SDBus(...).listen([](const msg::SmartData &){}) — sd_to_ros side
//
// One SmartData per datagram (72 bytes). See docs/wire_format.md.
// The bus is intentionally minimal — no batching, no reliability, no
// reordering. SDAV transducers and the SmartData paper assume the same
// best-effort model.

#include <atomic>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>

#include "semantic_gateway/msg/smart_data.hpp"

namespace semantic_gateway {

class SDBus {
public:
    using Handler = std::function<void(const msg::SmartData &)>;

    static constexpr const char * DEFAULT_GROUP  = "224.1.1.1";
    static constexpr uint16_t     DEFAULT_PORT   = 5000;

    // group:  multicast IPv4 group (e.g. "224.1.1.1")
    // port:   UDP port
    // ifaddr: local interface IPv4 for outgoing/joining; "" = any
    SDBus(const std::string & group  = DEFAULT_GROUP,
          uint16_t            port   = DEFAULT_PORT,
          const std::string & ifaddr = "");

    ~SDBus();

    SDBus(const SDBus &) = delete;
    SDBus & operator=(const SDBus &) = delete;

    // sender — opens the send socket lazily on first call
    void broadcast(const msg::SmartData & sd);

    // receiver — starts a background thread that calls `h` for every datagram
    // arriving at the group/port. Throws if already listening.
    void listen(Handler h);

    // stops the listener thread (if any). Idempotent.
    void stop();

    const std::string & group() const noexcept { return group_; }
    uint16_t            port()  const noexcept { return port_;  }

private:
    void open_send_socket();
    void open_recv_socket();
    void recv_loop();

    std::string group_;
    uint16_t    port_;
    std::string ifaddr_;

    int send_fd_{-1};
    int recv_fd_{-1};

    Handler           handler_;
    std::thread       thread_;
    std::atomic<bool> stop_{false};
};

}  // namespace semantic_gateway
