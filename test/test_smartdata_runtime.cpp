// test_smartdata_runtime.cpp — the SmartData dynamic of Fröhlich (2018) §2,
// exercised on a real (loopback) multicast segment, without ROS.
//
//   * wire layout matches the TSTP messages of LISHA's implementation;
//   * an ADVERTISED SmartData stays silent until someone is interested;
//   * event-driven interests get every update, time-triggered ones one
//     response per period (passive transducers are sampled for it);
//   * regions, devices, revocation and interest leases select who answers;
//   * the response carries the shortest expiry; expiry < period is invalid;
//   * COMMANDED SmartData execute commands, others ignore them;
//   * a fuser combines the responses of several transducers.

#include <atomic>
#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "si_units.hpp"
#include "smartdata/network.hpp"
#include "smartdata/smartdata.hpp"

using namespace std::chrono_literals;

namespace {

uint16_t random_port()
{
    std::random_device rd;
    return static_cast<uint16_t>(std::uniform_int_distribution<int>(40000, 44999)(rd));
}

sd::Network::Config test_config()
{
    sd::Network::Config c;
    c.group            = "224.1.1.9";
    c.port             = random_port();
    c.interest_refresh = 100ms;
    c.interest_lease   = 350ms;
    return c;
}

template<typename Pred>
bool eventually(Pred p, std::chrono::milliseconds timeout = 1000ms)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (p()) return true;
        std::this_thread::sleep_for(2ms);
    }
    return p();
}

// Active thermometer: the test pushes samples, like a ROS message would.
class Thermometer : public sd::Transducer<si_units::KELVIN> {
public:
    static constexpr bool     active = true;
    static constexpr unsigned TYPE   = SENSOR;

    explicit Thermometer(sd::Device_Id dev, double x = 0.0) : Transducer(dev), x_(x) {}

    void push(double kelvin, sd::Time t)
    {
        sd::Origin o;
        o.x = x_;
        o.t = t;
        o.confidence = 100;
        sample(kelvin, o);
    }

private:
    double x_;
};

// Passive sensor: sampled by the SmartData when needed.
class Counter_Sensor : public sd::Transducer<si_units::SECOND> {
public:
    static constexpr bool     active = false;
    static constexpr unsigned TYPE   = SENSOR;

    explicit Counter_Sensor(sd::Device_Id dev) : Transducer(dev) {}
    double sense() override { return static_cast<double>(++samples); }

    std::atomic<int> samples{0};
};

// Actuator: records the setpoints it receives.
class Heater : public sd::Transducer<si_units::KELVIN> {
public:
    static constexpr bool     active = true;
    static constexpr unsigned TYPE   = SENSOR | ACTUATOR;

    explicit Heater(sd::Device_Id dev) : Transducer(dev) {}

    void actuate(double v) override
    {
        setpoint.store(v);
        ++commands;
        sd::Origin o;
        o.t = 1;
        o.confidence = 100;
        sample(v, o);
    }

    std::atomic<double> setpoint{0.0};
    std::atomic<int>    commands{0};
};

// Counts the datagrams of a given type seen on the segment.
class Sniffer : public sd::Network::Observer {
public:
    explicit Sniffer(sd::Network & n) : net_(n) { net_.attach(this); }
    ~Sniffer() override { net_.detach(this); }

    void receive(const uint8_t * data, std::size_t len) override
    {
        const auto h = sd::wire::parse<sd::wire::Header>(data, len);
        if (!h) return;
        if (sd::wire::type_of(*h) == sd::RESPONSE &&
            (h->mode & sd::OPERATION_MASK) == sd::RESPOND) ++responses;
        if (sd::wire::type_of(*h) == sd::RESPONSE &&
            (h->mode & sd::OPERATION_MASK) == sd::ADVERTISE) ++advertisements;
    }

    std::atomic<int> responses{0};
    std::atomic<int> advertisements{0};

private:
    sd::Network & net_;
};

using Thermo_SD = sd::Responsive_SmartData<Thermometer>;
using Thermo_Proxy = sd::Interested_SmartData<si_units::KELVIN>;

}  // namespace

// ── wire format ──────────────────────────────────────────────────────────────

TEST(Wire, LayoutMatchesTstpMessages)
{
    // offsets of the TSTP _Header / Response / Interest / Command at CM_32
    EXPECT_EQ(offsetof(sd::wire::Header, origin), 4u);
    EXPECT_EQ(offsetof(sd::wire::Header, unit), 24u);
    EXPECT_EQ(offsetof(sd::wire::Header, device), 28u);
    EXPECT_EQ(offsetof(sd::wire::Header, signature), 32u);
    EXPECT_EQ(offsetof(sd::wire::Header, last_hop), 40u);
    EXPECT_EQ(offsetof(sd::wire::Response, expiry), 60u);
    EXPECT_EQ(offsetof(sd::wire::Response, value), 70u);
    EXPECT_EQ(offsetof(sd::wire::Interest, region), 60u);
    EXPECT_EQ(offsetof(sd::wire::Interest, expiry), 92u);
    EXPECT_EQ(offsetof(sd::wire::Interest, period), 100u);
    EXPECT_EQ(offsetof(sd::wire::Command, value), 84u);

    const auto h = sd::make_header(sd::RESPONSE, sd::ADVERTISED | sd::RESPOND, 0,
                                   {{1, 2, 3}, 4}, 100, si_units::KELVIN, 7);
    EXPECT_EQ(sd::wire::type_of(h), sd::RESPONSE);
    EXPECT_EQ(h.config >> 6, sd::SCALE_CM_32);
    EXPECT_EQ(h.config & 0x7, sd::VERSION);
}

TEST(Wire, OriginAndRegionInCentimetres)
{
    sd::Origin o{1.234, -5.0, 0.004, 42, 100};
    const auto w = o.to_wire();
    EXPECT_EQ(w.space.x, 123);
    EXPECT_EQ(w.space.y, -500);
    EXPECT_EQ(w.space.z, 0);
    EXPECT_EQ(w.t, 42);

    sd::Region r;
    r.x = 1.0;
    r.radius = 2.0;
    EXPECT_TRUE(r.contains_space(2.5, 0.0, 0.0));
    EXPECT_FALSE(r.contains_space(3.5, 0.0, 0.0));
    EXPECT_EQ(sd::Region::from_wire(r.to_wire()), r);
    EXPECT_TRUE(sd::Region::everywhere().contains_space(1e6, -1e6, 1e5));
}

// ── interests and responses ──────────────────────────────────────────────────

TEST(Runtime, AdvertisedSmartDataIsSilentWithoutInterest)
{
    sd::Network net(test_config());
    Sniffer sniff(net);
    Thermo_SD sd_obj(net, 0, 1'000'000, sd::ADVERTISED);

    EXPECT_TRUE(eventually([&] { return sniff.advertisements.load() >= 1; }));
    for (int i = 1; i <= 5; ++i) sd_obj.transducer().push(300.0 + i, i * 1000);
    std::this_thread::sleep_for(100ms);
    EXPECT_EQ(sniff.responses.load(), 0) << "no interest, no responses";
    EXPECT_DOUBLE_EQ(sd_obj.value(), 305.0);
}

TEST(Runtime, PrivateSmartDataNeverAnswers)
{
    sd::Network net(test_config());
    Thermo_SD sd_obj(net, 0, 0, sd::PRIVATE);
    Thermo_Proxy proxy(net, sd::Region::everywhere(), 0);

    std::this_thread::sleep_for(150ms);
    sd_obj.transducer().push(300.0, 1000);
    std::this_thread::sleep_for(100ms);
    EXPECT_EQ(sd_obj.interested(), 0u);
    EXPECT_FALSE(proxy.valid());
}

TEST(Runtime, EventDrivenInterestReceivesEveryUpdate)
{
    sd::Network net(test_config());
    Thermo_SD sensor(net, 0, 500'000, sd::ADVERTISED);

    std::mutex m;
    std::vector<double> got;
    Thermo_Proxy proxy(net, sd::Region::everywhere(), 2'000'000, 0, sd::SINGLE, sd::ANY_DEVICE, {},
                       [&](const Thermo_Proxy::Sample & s) {
                           std::lock_guard<std::mutex> lk(m);
                           got.push_back(s.value);
                       });

    ASSERT_TRUE(eventually([&] { return sensor.interested() == 1; }));
    for (int i = 1; i <= 10; ++i) {
        sensor.transducer().push(290.0 + i, i * 1000);
        std::this_thread::sleep_for(2ms);
    }
    ASSERT_TRUE(eventually([&] { std::lock_guard<std::mutex> lk(m); return got.size() >= 10; }));
    std::lock_guard<std::mutex> lk(m);
    EXPECT_DOUBLE_EQ(got.back(), 300.0);
    EXPECT_DOUBLE_EQ(proxy.value(), 300.0);
    EXPECT_EQ(proxy.time(), 10'000);
}

TEST(Runtime, ResponseCarriesShortestExpiry)
{
    sd::Network net(test_config());
    Thermo_SD sensor(net, 0, 500'000, sd::ADVERTISED);
    Thermo_Proxy proxy(net, sd::Region::everywhere(), 50'000);

    ASSERT_TRUE(eventually([&] { return sensor.interested() == 1; }));
    std::atomic<sd::Microsecond> seen{-1};
    Thermo_Proxy probe(net, sd::Region::everywhere(), 0, 0, sd::SINGLE, sd::ANY_DEVICE, {},
                       [&](const Thermo_Proxy::Sample & s) { seen = s.expiry; });
    ASSERT_TRUE(eventually([&] { return sensor.interested() == 2; }));
    sensor.transducer().push(300.0, 1000);
    ASSERT_TRUE(eventually([&] { return seen.load() >= 0; }));
    EXPECT_EQ(seen.load(), 50'000) << "the shortest of the expiries goes in the packet";
}

TEST(Runtime, TimeTriggeredInterestSamplesPassiveSensorEveryPeriod)
{
    sd::Network net(test_config());
    sd::Responsive_SmartData<Counter_Sensor> clock_sd(net, 3, 0, sd::ADVERTISED);
    const int before = clock_sd.transducer().samples.load();

    sd::Interested_SmartData<si_units::SECOND> proxy(
        net, sd::Region::everywhere(), 40'000, 20'000 /* 20 ms period */);

    std::this_thread::sleep_for(400ms);
    const auto n = proxy.responses();
    EXPECT_GE(n, 10u) << "≈ 20 periods elapsed";
    EXPECT_LE(n, 30u);
    EXPECT_GT(clock_sd.transducer().samples.load() - before, 10)
        << "the transducer is sampled to answer each period";
}

TEST(Runtime, NothingIsSentBeforeTheFirstSample)
{
    sd::Network net(test_config());
    Sniffer sniff(net);
    Thermo_SD sensor(net, 0, 0, sd::ADVERTISED);
    Thermo_Proxy proxy(net, sd::Region::everywhere(), 40'000, 20'000);

    ASSERT_TRUE(eventually([&] { return sensor.interested() == 1; }));
    std::this_thread::sleep_for(100ms);
    EXPECT_EQ(sniff.responses.load(), 0) << "time-triggered ticks with no data stay silent";

    sensor.transducer().push(300.0, 1000);
    EXPECT_TRUE(proxy.wait(500ms));
    EXPECT_DOUBLE_EQ(proxy.value(), 300.0);
}

TEST(Runtime, RegionSelectsWhichTransducersAnswer)
{
    sd::Network net(test_config());
    Thermo_SD near(net, 1, 0, sd::ADVERTISED, 0, 1.0);     // at x = 1 m
    Thermo_SD far(net, 2, 0, sd::ADVERTISED, 0, 100.0);    // at x = 100 m
    near.transducer().push(301.0, 1000);
    far.transducer().push(399.0, 1000);

    sd::Region region;
    region.radius = 5.0;  // 5 m around the origin
    std::mutex m;
    std::vector<sd::Device_Id> devices;
    Thermo_Proxy proxy(net, region, 0, 0, sd::ALL, sd::ANY_DEVICE, {},
                       [&](const Thermo_Proxy::Sample & s) {
                           std::lock_guard<std::mutex> lk(m);
                           devices.push_back(s.device);
                       });

    ASSERT_TRUE(eventually([&] { return near.interested() == 1; }));
    std::this_thread::sleep_for(150ms);
    EXPECT_EQ(far.interested(), 0u) << "outside the region: no binding";

    near.transducer().push(302.0, 2000);
    far.transducer().push(398.0, 2000);
    ASSERT_TRUE(eventually([&] { return proxy.value() == 302.0; }));
    std::lock_guard<std::mutex> lk(m);
    for (auto d : devices) EXPECT_EQ(d, 1u);
}

TEST(Runtime, DeviceSpecificInterest)
{
    sd::Network net(test_config());
    Thermo_SD a(net, 10, 0, sd::ADVERTISED);
    Thermo_SD b(net, 11, 0, sd::ADVERTISED);
    Thermo_Proxy proxy(net, sd::Region::everywhere(), 0, 0, sd::SINGLE, 11);

    ASSERT_TRUE(eventually([&] { return b.interested() == 1; }));
    EXPECT_EQ(a.interested(), 0u);
    a.transducer().push(1.0, 1000);
    b.transducer().push(2.0, 1000);
    ASSERT_TRUE(proxy.wait(500ms));
    EXPECT_DOUBLE_EQ(proxy.value(), 2.0);
}

TEST(Runtime, RevokedAndLapsedInterestsUnbind)
{
    sd::Network net(test_config());
    Thermo_SD sensor(net, 0, 0, sd::ADVERTISED);
    {
        Thermo_Proxy proxy(net, sd::Region::everywhere(), 0);
        ASSERT_TRUE(eventually([&] { return sensor.interested() == 1; }));
    }  // destructor revokes
    EXPECT_TRUE(eventually([&] { return sensor.interested() == 0; }, 300ms));

    // an interest announced once and never refreshed lapses after the lease
    sd::wire::Interest in{};
    in.header = sd::make_header(sd::INTEREST, sd::SINGLE | sd::ANNOUNCE, 0, {{0, 0, 0}, 0}, 0,
                                si_units::KELVIN, sd::ANY_DEVICE);
    in.region = sd::Region::everywhere().to_wire();
    net.send(&in, sizeof(in));
    ASSERT_TRUE(eventually([&] { return sensor.interested() == 1; }));
    sensor.transducer().push(300.0, 1000);  // updates purge stale bindings
    std::this_thread::sleep_for(450ms);
    sensor.transducer().push(301.0, 2000);
    EXPECT_EQ(sensor.interested(), 0u);
}

// ── timing rules ─────────────────────────────────────────────────────────────

TEST(Runtime, ExpirySmallerThanPeriodIsInvalid)
{
    sd::Network net(test_config());
    EXPECT_THROW(Thermo_SD(net, 0, 10'000, sd::ADVERTISED, 20'000), std::invalid_argument);
    EXPECT_THROW(Thermo_Proxy(net, sd::Region::everywhere(), 10'000, 20'000), std::invalid_argument);

    // an invalid interest coming from the network is ignored
    Thermo_SD sensor(net, 0, 0, sd::ADVERTISED);
    sd::wire::Interest in{};
    in.header = sd::make_header(sd::INTEREST, sd::SINGLE | sd::ANNOUNCE, 0, {{0, 0, 0}, 0}, 0,
                                si_units::KELVIN, sd::ANY_DEVICE);
    in.region = sd::Region::everywhere().to_wire();
    in.expiry = 10'000;
    in.period = 20'000;
    net.send(&in, sizeof(in));
    std::this_thread::sleep_for(100ms);
    EXPECT_EQ(sensor.interested(), 0u);
}

TEST(Runtime, PassiveSensorIsSampledWhenExpiredAndWaitBlocksForUpdates)
{
    sd::Network net(test_config());
    sd::Responsive_SmartData<Counter_Sensor> s(net, 0, 20'000, sd::PRIVATE);
    const double first = s.value();
    EXPECT_DOUBLE_EQ(s.value(), first) << "still valid: no new sample";
    std::this_thread::sleep_for(30ms);
    EXPECT_TRUE(s.expired());
    EXPECT_GT(s.value(), first) << "reading an expired SmartData triggers an update";

    Thermo_SD t(net, 0, 0, sd::PRIVATE);
    EXPECT_FALSE(t.wait(20ms));
    std::thread producer([&] {
        std::this_thread::sleep_for(20ms);
        t.transducer().push(1.0, 1);
    });
    EXPECT_TRUE(t.wait(500ms));
    producer.join();
}

// ── actuation ────────────────────────────────────────────────────────────────

TEST(Runtime, CommandsReachOnlyCommandedSmartData)
{
    sd::Network net(test_config());
    sd::Responsive_SmartData<Heater> commanded(net, 5, 0, sd::COMMANDED);
    sd::Responsive_SmartData<Heater> advertised(net, 6, 0, sd::ADVERTISED);

    Thermo_Proxy to5(net, sd::Region::everywhere(), 0, 0, sd::SINGLE, 5);
    Thermo_Proxy to6(net, sd::Region::everywhere(), 0, 0, sd::SINGLE, 6);
    ASSERT_TRUE(eventually([&] { return commanded.interested() == 1; }));

    to5 = 295.0;
    to6 = 295.0;
    ASSERT_TRUE(eventually([&] { return commanded.transducer().commands.load() == 1; }));
    EXPECT_DOUBLE_EQ(commanded.transducer().setpoint.load(), 295.0);
    // the actuator reports its new state to the interested proxy
    EXPECT_TRUE(eventually([&] { return to5.value() == 295.0; }));
    std::this_thread::sleep_for(50ms);
    EXPECT_EQ(advertised.transducer().commands.load(), 0) << "ADVERTISED is not COMMANDED";

    commanded = 280.0;  // local write = local setpoint
    EXPECT_DOUBLE_EQ(commanded.transducer().setpoint.load(), 280.0);
}

// ── fusion ───────────────────────────────────────────────────────────────────

TEST(Runtime, FuserCombinesResponsesOfSeveralTransducers)
{
    sd::Network net(test_config());
    Thermo_SD a(net, 1, 0, sd::ADVERTISED);
    Thermo_SD b(net, 2, 0, sd::ADVERTISED);

    // exponential smoothing over every response in the region
    Thermo_Proxy avg(net, sd::Region::everywhere(), 0, 0, sd::ALL, sd::ANY_DEVICE,
                     [](double current, const Thermo_Proxy::Sample & s) {
                         return current == 0.0 ? s.value : 0.5 * current + 0.5 * s.value;
                     });
    ASSERT_TRUE(eventually([&] { return a.interested() == 1 && b.interested() == 1; }));
    a.transducer().push(300.0, 1000);
    ASSERT_TRUE(eventually([&] { return avg.responses() >= 1; }));
    b.transducer().push(310.0, 1000);
    ASSERT_TRUE(eventually([&] { return avg.responses() >= 2; }));
    EXPECT_DOUBLE_EQ(avg.value(), 305.0);
}
