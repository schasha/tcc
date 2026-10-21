// test_imports.cpp — SmartData produced by other nodes reach ROS through the
// proxy (`imports:` in the YAML; this test is generated from
// test/imports_test.yaml).
//
//   foreign node (own Network, own device ids) ─▶ multicast
//   ─▶ Proxies (interested SmartData of the imports) ─▶ import topics
//
// The foreign producers are plain Responsive SmartData with transducers that
// this package knows nothing about: only the unit, device and region on the
// wire connect them to the proxy, as with any other SmartData node.

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/float64.hpp"

#include "si_units.hpp"
#include "smartdata/network.hpp"
#include "smartdata/smartdata.hpp"
#include "parsed_gateway.hpp"
#include "parsed_proxies.hpp"

namespace gen = semantic_gateway::gen;
using namespace std::chrono_literals;

namespace {

constexpr uint32_t PASCAL_CODE = 0xe48ea924u;  // the raw code in imports_test.yaml
static_assert(PASCAL_CODE == si_units::PASCAL, "test yaml uses the pascal code");

sd::Network::Config segment(uint16_t port)
{
    sd::Network::Config c;
    c.group            = "224.1.1.8";
    c.port             = port;
    c.interest_refresh = 100ms;
    c.interest_lease   = 400ms;
    return c;
}

uint16_t random_port()
{
    std::random_device rd;
    return static_cast<uint16_t>(std::uniform_int_distribution<int>(45000, 49000)(rd));
}

// A transducer of some other node: active, pushed by the test.
template<uint32_t U>
class Foreign : public sd::Transducer<U> {
public:
    static constexpr bool     active = true;
    static constexpr unsigned TYPE   = sd::Transducer<U>::SENSOR;

    explicit Foreign(sd::Device_Id dev) : sd::Transducer<U>(dev) {}

    void push(double v, sd::Time t, double x = 0.0, double y = 0.0)
    {
        sd::Origin o;
        o.x = x;
        o.y = y;
        o.t = t;
        o.confidence = 100;
        this->sample(v, o);
    }
};

template<uint32_t U>
using Producer = sd::Responsive_SmartData<Foreign<U>>;

class Local : public rclcpp::Node {
public:
    explicit Local(uint16_t port) : rclcpp::Node("local_gateway")
    {
        net_ = std::make_unique<sd::Network>(segment(port));
        gw_  = std::make_unique<gen::Gateway>(this, *net_,
            [](const std::string &, const builtin_interfaces::msg::Time &) { return sd::Origin{}; });
        pub_ = create_publisher<sensor_msgs::msg::Temperature>("/temperature", rclcpp::SensorDataQoS());
    }
    ~Local() override { gw_.reset(); net_.reset(); }

    std::unique_ptr<sd::Network>  net_;
    std::unique_ptr<gen::Gateway> gw_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_;
};

class Proxy : public rclcpp::Node {
public:
    explicit Proxy(uint16_t port) : rclcpp::Node("proxy_ut")
    {
        net_   = std::make_unique<sd::Network>(segment(port));
        px_    = std::make_unique<gen::Proxies>(this, *net_, sd::Region::everywhere(), 0);
        drain_ = create_wall_timer(20ms, [this] { px_->drain(); });
    }
    ~Proxy() override { px_.reset(); net_.reset(); }

    std::unique_ptr<sd::Network>  net_;
    std::unique_ptr<gen::Proxies> px_;
    rclcpp::TimerBase::SharedPtr  drain_;
};

template<typename Msg>
struct Inbox {
    std::mutex       mu;
    std::vector<Msg> got;

    std::size_t size()
    {
        std::lock_guard<std::mutex> lk(mu);
        return got.size();
    }
    std::vector<Msg> all()
    {
        std::lock_guard<std::mutex> lk(mu);
        return got;
    }
};

class Listener : public rclcpp::Node {
public:
    Listener() : rclcpp::Node("listener")
    {
        subs_.push_back(keep("/ext/temperature", temperature));
        subs_.push_back(keep("/ext/level", level));
        subs_.push_back(keep("/ext/accel", accel));
        subs_.push_back(keep("/ext/pressure", pressure));
        subs_.push_back(keep("/temperature_recovered", recovered));
    }

    template<typename Msg>
    rclcpp::SubscriptionBase::SharedPtr keep(const std::string & topic, Inbox<Msg> & box)
    {
        return create_subscription<Msg>(topic, rclcpp::SensorDataQoS(),
            [&box](typename Msg::ConstSharedPtr m) {
                std::lock_guard<std::mutex> lk(box.mu);
                box.got.push_back(*m);
            });
    }

    Inbox<sensor_msgs::msg::Temperature>   temperature;
    Inbox<std_msgs::msg::Float64>          level;
    Inbox<geometry_msgs::msg::Vector3>     accel;
    Inbox<sensor_msgs::msg::FluidPressure> pressure;
    Inbox<sensor_msgs::msg::Temperature>   recovered;

private:
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
};

}  // namespace

class ImportTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        if (!rclcpp::ok()) rclcpp::init(0, nullptr);
        const uint16_t port = random_port();
        foreign_  = std::make_unique<sd::Network>(segment(port));
        local_    = std::make_shared<Local>(port);
        proxy_    = std::make_shared<Proxy>(port);
        listener_ = std::make_shared<Listener>();
        exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        exec_->add_node(local_);
        exec_->add_node(proxy_);
        exec_->add_node(listener_);
    }

    void TearDown() override
    {
        exec_.reset();
        producers_.clear();
        listener_.reset();
        proxy_.reset();
        local_.reset();
        foreign_.reset();
    }

    // A foreign SmartData, advertised on the foreign node's network.
    template<uint32_t U>
    Producer<U> & produce(sd::Device_Id dev)
    {
        auto p = std::make_shared<Producer<U>>(*foreign_, dev, 0, sd::ADVERTISED, 0);
        Producer<U> & ref = *p;
        producers_.push_back(p);
        return ref;
    }

    // waits until the proxy's interests are bound to the producers
    void settle() { spin_for(300ms); }

    void spin_for(std::chrono::milliseconds d)
    {
        const auto deadline = std::chrono::steady_clock::now() + d;
        while (std::chrono::steady_clock::now() < deadline) {
            exec_->spin_some(5ms);
            std::this_thread::sleep_for(1ms);
        }
    }

    template<typename Pred>
    bool spin_until(Pred p, std::chrono::milliseconds timeout = 2000ms)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (p()) return true;
            exec_->spin_some(5ms);
            std::this_thread::sleep_for(1ms);
        }
        return p();
    }

    std::unique_ptr<sd::Network>       foreign_;
    std::shared_ptr<Local>             local_;
    std::shared_ptr<Proxy>             proxy_;
    std::shared_ptr<Listener>          listener_;
    std::vector<std::shared_ptr<void>> producers_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
};

TEST_F(ImportTest, AnyDeviceImportKeepsProducersApart)
{
    auto & a = produce<si_units::KELVIN>(42);
    auto & b = produce<si_units::KELVIN>(43);
    settle();

    a.transducer().push(293.15, 1'000'000);  // 20 °C
    b.transducer().push(303.15, 1'000'000);  // 30 °C, same instant: another producer
    ASSERT_TRUE(spin_until([&] { return listener_->temperature.size() >= 2; }));
    spin_for(100ms);

    const auto got = listener_->temperature.all();
    ASSERT_EQ(got.size(), 2u) << "one message per response, none merged";
    for (const auto & m : got) {
        EXPECT_EQ(m.header.stamp.sec, 1) << "stamped with the origin time";
        if (m.header.frame_id == "sd_42") {
            EXPECT_NEAR(m.temperature, 20.0, 1e-9);
        } else {
            EXPECT_EQ(m.header.frame_id, "sd_43");
            EXPECT_NEAR(m.temperature, 30.0, 1e-9);
        }
    }
}

TEST_F(ImportTest, LocalTransducerIsNotImported)
{
    settle();
    sensor_msgs::msg::Temperature m;
    m.header.stamp = sd::to_stamp(2'000'000);
    m.temperature  = 25.0;
    local_->pub_->publish(m);

    ASSERT_TRUE(spin_until([&] { return listener_->recovered.size() > 0; }))
        << "the local sensor still goes through its own proxy";
    spin_for(200ms);
    EXPECT_EQ(listener_->temperature.size(), 0u)
        << "the gateway's own kelvin device is not taken as a foreign one";
}

TEST_F(ImportTest, SpecificDeviceIntoHeaderlessMessage)
{
    auto & wanted = produce<si_units::METER>(7);
    auto & other  = produce<si_units::METER>(8);
    settle();

    other.transducer().push(9.0, 1'000'000);
    wanted.transducer().push(1.25, 1'000'000);
    ASSERT_TRUE(spin_until([&] { return listener_->level.size() > 0; }));
    spin_for(200ms);

    const auto got = listener_->level.all();
    ASSERT_EQ(got.size(), 1u) << "device 8 is not part of the import";
    EXPECT_NEAR(got[0].data, 1.25, 1e-12);
}

TEST_F(ImportTest, CompositeImportRebuildsOneMessage)
{
    auto & x = produce<si_units::METER_PER_SEC_SQ>(10);
    auto & y = produce<si_units::METER_PER_SEC_SQ>(11);
    auto & z = produce<si_units::METER_PER_SEC_SQ>(12);
    auto & stray = produce<si_units::METER_PER_SEC_SQ>(13);
    settle();

    const sd::Time t = 5'000'000;
    x.transducer().push(0.1, t);
    stray.transducer().push(7.0, t);
    y.transducer().push(0.2, t);
    z.transducer().push(9.81, t);
    ASSERT_TRUE(spin_until([&] { return listener_->accel.size() > 0; }));
    spin_for(200ms);

    const auto got = listener_->accel.all();
    ASSERT_EQ(got.size(), 1u) << "the three axes of one instant make one message";
    EXPECT_NEAR(got[0].x, 0.1, 1e-12);
    EXPECT_NEAR(got[0].y, 0.2, 1e-12);
    EXPECT_NEAR(got[0].z, 9.81, 1e-12);
}

TEST_F(ImportTest, RegionFiltersProducersAndStampIsArrivalTime)
{
    auto & inside  = produce<PASCAL_CODE>(1);
    auto & outside = produce<PASCAL_CODE>(2);
    settle();

    inside.transducer().push(101325.0, 3'000'000, 5.2, 4.9);   // 0.22 m from (5, 5)
    outside.transducer().push(90000.0, 3'000'000, 0.0, 0.0);
    ASSERT_TRUE(spin_until([&] { return listener_->pressure.size() > 0; }));
    spin_for(200ms);

    const auto got = listener_->pressure.all();
    ASSERT_EQ(got.size(), 1u) << "the producer outside the import's region is ignored";
    EXPECT_NEAR(got[0].fluid_pressure, 101325.0, 1e-9);
    const double now_s = proxy_->now().seconds();
    EXPECT_NEAR(got[0].header.stamp.sec, now_s, 5.0)
        << "stamp: receive uses the proxy's clock, not the producer's origin time (3 s)";
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    const int rc = RUN_ALL_TESTS();
    if (rclcpp::ok()) rclcpp::shutdown();
    return rc;
}
