// proxy_node.cpp — the remote side: SmartData network → ROS 2.
//
// Instantiates Interested SmartData (proxies to remote transducers) for every
// transducer class declared in config/sensors.yaml (codegen →
// parsed_proxies.hpp):
//
//   * sensors   — one interest per transducer class covering all its
//                 devices in the configured space-time region. The responses
//                 are regrouped by origin time and republished as the original
//                 ROS message type on <topic>_recovered.
//   * actuators — messages published on <topic>_command are decomposed and
//                 written to the remote actuators as SmartData commands.
//
// Parameters:
//   region_x/y/z, region_radius   region of interest (metres; radius < 0 =
//                                 everywhere)
//   period_ms                     0 = event-driven interests (every update),
//                                 > 0 = time-triggered (one response per
//                                 period)
//   drain_period_ms               flush of incomplete vector samples

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "smartdata/network.hpp"
#include "parsed_proxies.hpp"

namespace sg = semantic_gateway;

class ProxyNode : public rclcpp::Node
{
public:
    ProxyNode() : Node("proxy_node")
    {
        sd::Network::Config net;
        net.group  = declare_parameter<std::string>("bus_group", net.group);
        net.port   = static_cast<uint16_t>(declare_parameter<int>("bus_port", net.port));
        net.ifaddr = declare_parameter<std::string>("bus_ifaddr", "");

        sd::Region region = sd::Region::everywhere();
        const double rx     = declare_parameter<double>("region_x", 0.0);
        const double ry     = declare_parameter<double>("region_y", 0.0);
        const double rz     = declare_parameter<double>("region_z", 0.0);
        const double radius = declare_parameter<double>("region_radius", -1.0);
        if (radius >= 0.0) {
            region.x      = rx;
            region.y      = ry;
            region.z      = rz;
            region.radius = radius;
        }
        const int period_ms = declare_parameter<int>("period_ms", 0);
        const int drain_ms  = declare_parameter<int>("drain_period_ms", 50);

        network_ = std::make_unique<sd::Network>(net, [this] {
            return static_cast<sd::Time>(get_clock()->now().nanoseconds() / 1000);
        });
        proxies_ = std::make_unique<sg::gen::Proxies>(
            this, *network_, region, static_cast<sd::Microsecond>(period_ms) * 1000);

        drain_timer_ = create_wall_timer(std::chrono::milliseconds(drain_ms),
                                         [this] { proxies_->drain(); });

        const std::string timing = period_ms > 0
            ? "time-triggered every " + std::to_string(period_ms) + " ms"
            : std::string("event-driven");
        RCLCPP_INFO(get_logger(),
            "proxy up. bus=%s:%u, region=(%.2f, %.2f, %.2f) r=%.2f m, %s",
            net.group.c_str(), net.port, region.x, region.y, region.z, region.radius,
            timing.c_str());
    }

    ~ProxyNode() override
    {
        proxies_.reset();  // revoke interests before closing the network
        network_.reset();
    }

private:
    std::unique_ptr<sd::Network>      network_;
    std::unique_ptr<sg::gen::Proxies> proxies_;
    rclcpp::TimerBase::SharedPtr      drain_timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProxyNode>());
    rclcpp::shutdown();
    return 0;
}
