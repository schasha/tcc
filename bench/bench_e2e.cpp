// bench_e2e.cpp — H4: end-to-end latency over ROS 2 and the SmartData network.
//
//   bench_e2e pub    publishes sensor_msgs/Imu on /imu/data at `rate` Hz for
//                    `duration` s, each stamped with the ROS clock at publish
//                    time (frame imu_frame).
//   bench_e2e probe  subscribes to /imu/data (one ROS 2 hop, the baseline)
//                    and to /imu/data_recovered (publisher → gateway_node →
//                    SmartData network → proxy_node → probe) and records
//                    now − header.stamp for each message; after `duration` s
//                    it appends one CSV row per path to `out`.
//
// All processes run on the same host and read the same clock, so the
// difference of stamps is a latency. The Imu is composite: the proxy
// republishes it as soon as its six fields arrived, so the recovered path
// carries no reconstruction wait (unlike a LaserScan, which is only complete
// when the next scan begins).

#include <algorithm>
#include <atomic>
#include <chrono>
#include <fstream>
#include <memory>
#include <mutex>
#include <numeric>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;
using Imu = sensor_msgs::msg::Imu;

namespace {

double quantile(const std::vector<double> & sorted, double q)
{
    if (sorted.empty()) return 0.0;
    const std::size_t i = std::min(sorted.size() - 1, static_cast<std::size_t>(q * sorted.size()));
    return sorted[i];
}

class Publisher : public rclcpp::Node
{
public:
    Publisher() : Node("bench_e2e_pub")
    {
        rate_     = declare_parameter<double>("rate", 100.0);
        duration_ = declare_parameter<double>("duration", 10.0);
        pub_ = create_publisher<Imu>("/imu/data", rclcpp::SensorDataQoS());
    }

    void wait_for_subscribers(std::size_t n, std::chrono::seconds timeout)
    {
        const auto end = std::chrono::steady_clock::now() + timeout;
        while (pub_->get_subscription_count() < n && std::chrono::steady_clock::now() < end) {
            std::this_thread::sleep_for(50ms);
        }
        std::this_thread::sleep_for(500ms);  // matched both ways
    }

    void run()
    {
        const auto period = std::chrono::duration<double>(1.0 / rate_);
        const auto total  = static_cast<std::size_t>(rate_ * duration_);
        const auto start  = std::chrono::steady_clock::now();
        Imu m;
        m.header.frame_id = "imu_frame";
        m.linear_acceleration.z = 9.81;
        for (std::size_t i = 0; i < total && rclcpp::ok(); ++i) {
            std::this_thread::sleep_until(
                start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(period * static_cast<double>(i)));
            m.angular_velocity.z = static_cast<double>(i);  // every message differs
            m.header.stamp = now();
            pub_->publish(m);
        }
        RCLCPP_INFO(get_logger(), "published %zu messages at %.0f Hz", total, rate_);
    }

private:
    double rate_{100.0};
    double duration_{10.0};
    rclcpp::Publisher<Imu>::SharedPtr pub_;
};

class Probe : public rclcpp::Node
{
public:
    Probe() : Node("bench_e2e_probe")
    {
        out_      = declare_parameter<std::string>("out", "e2e.csv");
        raw_      = declare_parameter<std::string>("raw_prefix", "");
        tag_      = declare_parameter<std::string>("tag", "");
        rate_     = declare_parameter<double>("rate", 100.0);
        expected_ = declare_parameter<int>("expected", 1000);

        direct_ = create_subscription<Imu>("/imu/data", rclcpp::SensorDataQoS(),
            [this](Imu::ConstSharedPtr m) { record(direct_us_, direct_seen_, *m); });
        recovered_ = create_subscription<Imu>("/imu/data_recovered", rclcpp::SensorDataQoS(),
            [this](Imu::ConstSharedPtr m) { record(gateway_us_, gateway_seen_, *m); });
    }

    void finish()
    {
        std::lock_guard<std::mutex> lk(mutex_);
        write("direct", direct_us_, direct_seen_.size());
        write("gateway", gateway_us_, gateway_seen_.size());
    }

private:
    void record(std::vector<double> & us, std::unordered_set<int64_t> & seen, const Imu & m)
    {
        const double lat = static_cast<double>((now() - rclcpp::Time(m.header.stamp)).nanoseconds()) / 1000.0;
        std::lock_guard<std::mutex> lk(mutex_);
        // the stamp identifies the message (µs resolution on the SmartData side)
        if (!seen.insert(rclcpp::Time(m.header.stamp).nanoseconds() / 1000).second) return;
        us.push_back(lat);
    }

    void write(const std::string & path, std::vector<double> us, std::size_t received)
    {
        if (!raw_.empty()) {
            std::ofstream r(raw_ + "_" + path + ".txt");
            for (double v : us) r << v << '\n';
        }
        std::sort(us.begin(), us.end());
        const double mean = us.empty() ? 0.0 : std::accumulate(us.begin(), us.end(), 0.0) / static_cast<double>(us.size());
        const bool fresh = [&] { std::ifstream f(out_); return !f.good() || f.peek() == EOF; }();
        std::ofstream f(out_, std::ios::app);
        if (fresh) f << "tag,path,rate,expected,received,loss_ratio,n,min,p5,p50,p95,p99,p999,max,mean\n";
        const double loss = expected_ > 0 ? 1.0 - static_cast<double>(received) / static_cast<double>(expected_) : 0.0;
        f << tag_ << ',' << path << ',' << rate_ << ',' << expected_ << ',' << received << ','
          << std::max(0.0, loss) << ',' << us.size() << ','
          << (us.empty() ? 0.0 : us.front()) << ',' << quantile(us, 0.05) << ',' << quantile(us, 0.5) << ','
          << quantile(us, 0.95) << ',' << quantile(us, 0.99) << ',' << quantile(us, 0.999) << ','
          << (us.empty() ? 0.0 : us.back()) << ',' << mean << '\n';
    }

    std::string out_, raw_, tag_;
    double rate_{100.0};
    int expected_{1000};
    std::mutex mutex_;
    std::vector<double> direct_us_, gateway_us_;
    std::unordered_set<int64_t> direct_seen_, gateway_seen_;
    rclcpp::Subscription<Imu>::SharedPtr direct_, recovered_;
};

}  // namespace

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    const std::string mode = argc > 1 ? argv[1] : "";
    if (mode == "pub") {
        auto node = std::make_shared<Publisher>();
        std::thread spin([node] { rclcpp::spin(node); });
        node->wait_for_subscribers(2, 10s);  // the probe and the gateway
        node->run();
        std::this_thread::sleep_for(500ms);
        rclcpp::shutdown();
        spin.join();
        return 0;
    }
    if (mode == "probe") {
        auto node = std::make_shared<Probe>();
        const double secs = node->declare_parameter<double>("listen", 15.0);
        rclcpp::executors::MultiThreadedExecutor exec;
        exec.add_node(node);
        std::thread spin([&exec] { exec.spin(); });
        const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(secs);
        while (rclcpp::ok() && std::chrono::steady_clock::now() < end) std::this_thread::sleep_for(50ms);
        exec.cancel();
        spin.join();
        node->finish();
        rclcpp::shutdown();
        return 0;
    }
    fprintf(stderr, "usage: bench_e2e pub|probe --ros-args -p ...\n");
    rclcpp::shutdown();
    return 1;
}
