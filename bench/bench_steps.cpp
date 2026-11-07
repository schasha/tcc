// bench_steps.cpp — H5: the robot follows setpoints sent through SmartData.
//
// Runs next to gazebo_gateway.launch.py (drive:=false). Once /odom shows
// that the robot exists, every `step_s`
// seconds of simulated time it moves to the next (v, w) pair of `steps_v` /
// `steps_w` and publishes it on /cmd_vel_command at 20 Hz; the proxy turns it
// into SmartData commands and the gateway publishes the setpoint on /cmd_vel.
//
// Two files are written:
//   <out>_odom.csv  t, step, cmd_v, cmd_w, odom_v, odom_w  (one row per /odom)
//   <out>_cmd.csv   step, v, w, delay_us — wall-clock delay between the first
//                   publication of a step on /cmd_vel_command and the first
//                   /cmd_vel carrying it (the command path through the network)
// bench/analyze.py derives the steady-state error and the rise time per step.

#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using Twist = geometry_msgs::msg::Twist;
using Odom  = nav_msgs::msg::Odometry;
using Steady = std::chrono::steady_clock;
using namespace std::chrono_literals;

class Steps : public rclcpp::Node
{
public:
    Steps() : Node("bench_steps")
    {
        steps_v_ = declare_parameter<std::vector<double>>("steps_v", {0.0, 0.10, 0.20, 0.05, 0.15, 0.0});
        steps_w_ = declare_parameter<std::vector<double>>("steps_w", {0.0, 0.30, -0.50, 0.80, 0.0, 0.0});
        step_s_  = declare_parameter<double>("step_s", 8.0);
        const std::string out = declare_parameter<std::string>("out", "steps");
        if (steps_v_.size() != steps_w_.size()) throw std::invalid_argument("steps_v and steps_w differ in size");

        odom_csv_.open(out + "_odom.csv");
        odom_csv_ << "t,step,cmd_v,cmd_w,odom_v,odom_w\n";
        cmd_csv_.open(out + "_cmd.csv");
        cmd_csv_ << "step,v,w,delay_us\n";

        pub_ = create_publisher<Twist>("/cmd_vel_command", 10);
        odom_ = create_subscription<Odom>("/odom", 50, [this](Odom::ConstSharedPtr m) { on_odom(*m); });
        cmd_vel_ = create_subscription<Twist>("/cmd_vel", 50, [this](Twist::ConstSharedPtr m) { on_cmd_vel(*m); });
        timer_ = create_wall_timer(50ms, [this] { tick(); });
    }

    bool finished() const { return finished_; }

private:
    void tick()
    {
        const double t = now().seconds();
        if (!odom_seen_ || t <= 0.0) return;  // robot not spawned / /clock not running yet
        if (t0_ < 0.0) t0_ = t;
        const auto step = static_cast<std::size_t>((t - t0_) / step_s_);
        std::lock_guard<std::mutex> lk(mutex_);
        if (step >= steps_v_.size()) {
            finished_ = true;
            return;
        }
        if (step != step_ || !sent_once_) {
            step_ = step;
            sent_once_ = true;
            first_sent_ = Steady::now();
            awaiting_ = true;
        }
        Twist m;
        m.linear.x  = steps_v_[step_];
        m.angular.z = steps_w_[step_];
        pub_->publish(m);
    }

    void on_cmd_vel(const Twist & m)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        if (!awaiting_) return;
        if (std::abs(m.linear.x - steps_v_[step_]) < 1e-9 && std::abs(m.angular.z - steps_w_[step_]) < 1e-9) {
            const double d = std::chrono::duration<double, std::micro>(Steady::now() - first_sent_).count();
            cmd_csv_ << step_ << ',' << steps_v_[step_] << ',' << steps_w_[step_] << ',' << d << '\n';
            cmd_csv_.flush();
            awaiting_ = false;
        }
    }

    void on_odom(const Odom & m)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        odom_seen_ = true;
        if (!sent_once_ || finished_) return;
        odom_csv_ << rclcpp::Time(m.header.stamp).seconds() - t0_ << ',' << step_ << ','
                  << steps_v_[step_] << ',' << steps_w_[step_] << ','
                  << m.twist.twist.linear.x << ',' << m.twist.twist.angular.z << '\n';
    }

    std::vector<double> steps_v_, steps_w_;
    double step_s_{8.0};
    double t0_{-1.0};
    std::size_t step_{0};
    std::atomic<bool> odom_seen_{false};
    bool sent_once_{false}, awaiting_{false}, finished_{false};
    Steady::time_point first_sent_;
    std::mutex mutex_;
    std::ofstream odom_csv_, cmd_csv_;
    rclcpp::Publisher<Twist>::SharedPtr pub_;
    rclcpp::Subscription<Odom>::SharedPtr odom_;
    rclcpp::Subscription<Twist>::SharedPtr cmd_vel_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Steps>();
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    while (rclcpp::ok() && !node->finished()) exec.spin_once(20ms);
    rclcpp::shutdown();
    return 0;
}
