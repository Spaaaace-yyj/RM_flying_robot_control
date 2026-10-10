#include "px4_hover/core.hpp"
#include <functional>
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace px4_hover;

namespace
{
    constexpr double pi = 3.14159265358979323846;
    const std::array<double, 4> identity{0., 0., 0., 1.};
#define CHECK(expression) do { if (!(expression)) { throw std::runtime_error(std::string(#expression)+" at line "+std::to_string(__LINE__)); } } while (false)
    void near(double a, double b, double tolerance = 1e-10) { CHECK(std::abs(a-b) < tolerance); }
    void near(const Vec3& a, const Vec3& b, double tolerance = 1e-10) { CHECK((a-b).norm() < tolerance); }

    template <class Function>
    void throws(Function fn)
    {
        bool thrown = false;
        try { fn(); }
        catch (const std::exception&) { thrown = true; }
        CHECK(thrown);
    }

    Observation good()
    {
        Observation o;
        o.healthy = o.status_fresh = o.armed = true;
        o.reason = "valid";
        o.nav_state = 2;
        o.position = Vec3(2., 3., -.2);
        o.heading = .4;
        return o;
    }

    struct Context
    {
        Flight flight;
        Observation observation = good();
        explicit Context(FlightConfig config = {}) : flight(config) { flight.tick(0., 100., observation); }
        void begin(bool takeoff = true) { CHECK(flight.begin(0., 100., observation, takeoff).first); }

        void flying(bool takeoff = true)
        {
            begin(takeoff);
            flight.tick(1.6, 101.6, observation);
            observation.nav_state = kOffboard;
            flight.tick(1.7, 101.7, observation);
        }
    };

    VioGate gate()
    {
        VioConfig config;
        config.stable_seconds = .1;
        return VioGate(config);
    }

    bool sample(VioGate& gate, double t, const Vec3& p = Vec3::Zero(),
                const Vec3& v = Vec3::Zero(), const std::array<double, 4>& q = identity,
                const std::string& frame = "world")
    {
        return gate.accept(100. + t, p, q, v, frame, t, 100. + t);
    }
} // namespace

int main()
{
    using Test = std::pair<const char*, std::function<void()>>;
    const std::vector<Test> tests{
        {
            "quaternion normalization and 180 degree rotations", []
            {
                for (auto q : {
                         identity, std::array<double, 4>{1., 0., 0., 0.}, std::array<double, 4>{-.5, .5, -.5, .5}
                     })
                {
                    const auto a = checkedQuaternion(q);
                    const Quat b(a.toRotationMatrix());
                    near(std::abs(a.dot(b)), 1.);
                }
                near(checkedQuaternion({0., 0., 0., 1.1}).norm(), 1.);
            }
        },
        {
            "invalid quaternion rejected", []
            {
                throws([] { checkedQuaternion({0., 0., 0., 0.}); });
                throws([] { checkedQuaternion({NAN, 0., 0., 1.}); });
            }
        },
        {
            "frozen yaw origin and world velocity", []
            {
                Alignment a;
                a.initialize(Vec3(10., 20., 3.), Quat::Identity(),
                             Quat(Eigen::AngleAxisd(pi / 2., Vec3::UnitZ())), Vec3(2., 4., -.5));
                const auto result = a.convert(Vec3(11., 20., 4.), Quat::Identity(), Vec3(1., 0., 0.));
                near(result.position, Vec3(2., 5., -1.5));
                near(result.velocity, Vec3(0., 1., 0.));
                near(result.orientation.angularDistance(Quat(Eigen::AngleAxisd(pi / 2., Vec3::UnitZ()))), 0.);
            }
        },
        {
            "body velocity distinct from world velocity", []
            {
                Alignment a;
                a.initialize(Vec3::Zero(), Quat::Identity(), Quat::Identity(), Vec3::Zero());
                const Quat q(Eigen::AngleAxisd(pi / 2., Vec3::UnitZ()));
                const auto body = a.convert(Vec3::Zero(), q, Vec3::UnitX(), true);
                const auto world = a.convert(Vec3::Zero(), q, Vec3::UnitX(), false);
                CHECK((body.velocity-world.velocity).norm() > 1.);
            }
        },
        {
            "alignment cannot be updated in flight", []
            {
                Alignment a;
                throws([&] { a.convert(Vec3::Zero(), Quat::Identity(), Vec3::Zero()); });
                a.initialize(Vec3::Zero(), Quat::Identity(), Quat::Identity(), Vec3::Zero());
                throws([&] { a.initialize(Vec3::Ones(), Quat::Identity(), Quat::Identity(), Vec3::Ones()); });
            }
        },
        {
            "mounting rotation and roll pitch preserved", []
            {
                const Quat mount(Eigen::AngleAxisd(.3, Vec3::UnitX()));
                Alignment a({mount.x(), mount.y(), mount.z(), mount.w()});
                const Quat imu(Eigen::AngleAxisd(.2, Vec3::UnitY()));
                Eigen::Matrix3d enu;
                enu << 0, 1, 0, 1, 0, 0, 0, 0, -1;
                const Eigen::Matrix3d frd = Vec3(1., -1., -1.).asDiagonal();
                const Quat px4(rotationZ(.4) * enu * imu.toRotationMatrix() * mount.toRotationMatrix() * frd);
                a.initialize(Vec3::Zero(), imu, px4, Vec3::Zero());
                near(a.convert(Vec3::Zero(), imu, Vec3::Zero()).orientation.angularDistance(px4), 0.);
            }
        },
        {
            "warmup and visual backend watchdog", []
            {
                auto g = gate();
                g.backend(100., 0., 100.);
                CHECK(sample(g, 0.));
                CHECK(!g.health(0.,100.).first);
                CHECK(sample(g,.2));
                CHECK(g.health(.2,100.2).first);
                for (double t : {.4, .6, .8, 1.}) { CHECK(sample(g,t)); }
                CHECK(g.health(1.,101.).second.find("visual optimization") != std::string::npos);
            }
        },
        {
            "duplicates cannot refresh watchdog", []
            {
                auto g = gate();
                sample(g, 0.);
                CHECK(!g.accept(100.,Vec3::Zero(),identity,Vec3::Zero(),"world",.2,100.2));
                near(g.received(), 0.);
            }
        },
        {
            "stale future and zero samples rejected", []
            {
                auto g = gate();
                for (double stamp : {99., 101., 0.})
                {
                    CHECK(!g.accept(stamp,Vec3::Zero(),identity,Vec3::Zero(),"world",0.,100.));
                }
            }
        },
        {
            "frame invalid data and speed rejected", []
            {
                auto g = gate();
                CHECK(!sample(g,0.,Vec3(NAN,0.,0.)));
                CHECK(!sample(g,0.,Vec3::Zero(),Vec3(9.,0.,0.)));
                CHECK(!sample(g,0.,Vec3::Zero(),Vec3::Zero(),identity,"map"));
                CHECK(!sample(g,0.,Vec3::Zero(),Vec3::Zero(),{0.,0.,0.,0.}));
            }
        },
        {
            "position jump stays latched until reset", []
            {
                auto g = gate();
                sample(g, 0.);
                CHECK(!sample(g,.01,Vec3(2.,0.,0.)));
                CHECK(!sample(g,.02));
                const auto counter = g.resetCounter();
                g.clear();
                CHECK(g.resetCounter() == counter+1);
                CHECK(sample(g,.03));
            }
        },
        {
            "yaw jump latched", []
            {
                auto g = gate();
                sample(g, 0.);
                const auto q = Quat(Eigen::AngleAxisd(1., Vec3::UnitZ()));
                CHECK(!sample(g,.01,Vec3::Zero(),Vec3::Zero(),{q.x(),q.y(),q.z(),q.w()}));
                CHECK(!g.latched().empty());
            }
        },
        {
            "backend and propagated clock rollback latched", []
            {
                auto g = gate();
                g.backend(100.2, .2, 100.2);
                CHECK(!g.backend(100.1,.3,100.3));
                CHECK(!g.latched().empty());
                g.clear();
                sample(g, .2);
                CHECK(!sample(g,.1));
                CHECK(!g.latched().empty());
            }
        },
        {
            "stream gap needs fresh warmup", []
            {
                auto g = gate();
                g.backend(100., 0., 100.);
                sample(g, 0.);
                sample(g, .2);
                g.backend(100.6, .6, 100.6);
                CHECK(sample(g,.6));
                CHECK(!g.health(.6,100.6).first);
            }
        },
        {
            "warmup publication requires both streams", []
            {
                auto g = gate();
                sample(g, 0.);
                CHECK(!g.publishable(100.,0.,100.));
                g.backend(100., 0., 100.);
                CHECK(g.publishable(100.,0.,100.));
                CHECK(!g.health(0.,100.).first);
                g.invalidate("tracker restart");
                CHECK(!g.publishable(100.,0.,100.));
            }
        },
        {
            "no startup arming or heartbeat", []
            {
                Context c;
                const auto a = c.flight.tick(0., 100., c.observation);
                CHECK(c.flight.state == State::Ready);
                CHECK(!a.heartbeat);
                CHECK(!a.command);
            }
        },
        {
            "default disarmed takeoff rejected", []
            {
                Context c;
                c.observation.armed = false;
                CHECK(!c.flight.begin(0.,100.,c.observation,true).first);
            }
        },
        {
            "unsafe takeovers rejected", []
            {
                Context c;
                c.observation.healthy = false;
                CHECK(!c.flight.begin(0.,100.,c.observation,true).first);
                c.observation = good();
                c.observation.velocity.x() = 1.;
                CHECK(!c.flight.begin(0.,100.,c.observation,true).first);
                c.observation = good();
                c.observation.position.x() = NAN;
                CHECK(!c.flight.begin(0.,100.,c.observation,true).first);
            }
        },
        {
            "priming and actual mode confirmation", []
            {
                Context c;
                c.begin();
                CHECK(!c.flight.tick(1.,101.,c.observation).command);
                const auto a = c.flight.tick(1.6, 101.6, c.observation);
                CHECK(a.command && a.command->id==176);
                c.flight.ack(176, 0);
                c.flight.tick(1.7, 101.7, c.observation);
                CHECK(c.flight.state==State::RequestOffboard);
                c.observation.nav_state = 14;
                c.flight.tick(1.8, 101.8, c.observation);
                CHECK(c.flight.state==State::Takeoff);
            }
        },
        {
            "explicit software arm only after offboard confirmation", []
            {
                FlightConfig cfg;
                cfg.allow_arm_requests = true;
                Context c(cfg);
                c.observation.armed = false;
                c.begin();
                CHECK(c.flight.tick(1.6,101.6,c.observation).command->id==176);
                c.observation.nav_state = 14;
                CHECK(c.flight.tick(1.7,101.7,c.observation).command->id==400);
                CHECK(c.flight.state==State::RequestArm);
                c.observation.armed = true;
                c.flight.tick(1.8, 101.8, c.observation);
                CHECK(c.flight.state==State::Takeoff);
            }
        },
        {
            "hold target frozen", []
            {
                Context c;
                c.flying(false);
                const Vec3 target = *c.flight.target;
                c.observation.position += Vec3(.1, .1, 0.);
                c.flight.tick(1.8, 101.8, c.observation);
                near(*c.flight.target, target);
                CHECK(c.flight.state==State::Hover);
            }
        },
        {
            "climb capped and paused ROS clock does not climb", []
            {
                Context c;
                c.flying();
                const double z = c.flight.target->z();
                c.flight.tick(1.8, 101.7, c.observation);
                near(c.flight.target->z(), z);
                c.flight.tick(1.9, 101.8, c.observation);
                near(c.flight.target->z(), z - .025);
                const double before = c.flight.target->z();
                c.flight.tick(2., 102.8, c.observation);
                near(c.flight.target->z(), before - .025);
            }
        },
        {
            "RC takeover never reenters or arms", []
            {
                Context c;
                c.flying();
                c.observation.nav_state = 2;
                for (double t : {1.8, 4., 10.})
                {
                    const auto a = c.flight.tick(t, 100. + t, c.observation);
                    CHECK(!a.heartbeat && !a.command);
                }
                CHECK(c.flight.state==State::Manual);
            }
        },
        {
            "unexpected disarm never rearms", []
            {
                Context c;
                c.flying();
                c.observation.armed = false;
                const auto a = c.flight.tick(1.8, 101.8, c.observation);
                CHECK(c.flight.state==State::Manual);
                CHECK(!a.command && !a.heartbeat);
            }
        },
        {
            "VINS loss stops heartbeat and requests land", []
            {
                Context c;
                c.flying();
                c.observation.healthy = false;
                c.observation.reason = "VINS reset";
                const auto a = c.flight.tick(1.8, 101.8, c.observation);
                CHECK(!a.heartbeat && a.command && a.command->id==21);
                CHECK(c.flight.state==State::Fault);
                c.observation.nav_state = 2;
                CHECK(!c.flight.tick(3.,103.,c.observation).command);
            }
        },
        {
            "ROS clock rollback stops control", []
            {
                Context c;
                c.flying();
                CHECK(!c.flight.tick(1.8,99.,c.observation).heartbeat);
                CHECK(c.flight.state==State::Fault);
            }
        },
        {
            "PX4 failsafe has priority over VINS failure", []
            {
                Context c;
                c.flying();
                c.observation.failsafe = true;
                c.observation.healthy = false;
                const auto a = c.flight.tick(1.8, 101.8, c.observation);
                CHECK(c.flight.state==State::Manual);
                CHECK(!a.command && !a.heartbeat);
            }
        },
        {
            "land cannot override manual mode", []
            {
                Context c;
                CHECK(!c.flight.land(0.,c.observation).first);
                CHECK(c.flight.state==State::Ready);
            }
        },
        {
            "permanent command rejection faults", []
            {
                Context c;
                c.begin();
                c.flight.tick(1.6, 101.6, c.observation);
                c.flight.ack(176, 2);
                CHECK(!c.flight.tick(1.7,101.7,c.observation).heartbeat);
                CHECK(c.flight.state==State::Fault);
            }
        },
        {
            "temporary rejection has bounded retries", []
            {
                Context c;
                c.begin();
                c.flight.tick(1.6, 101.6, c.observation);
                c.flight.ack(176, 1);
                CHECK(c.flight.state==State::RequestOffboard);
                for (double t : {2.7, 3.8, 4.9, 6., 7.1}) { c.flight.tick(t, 100. + t, c.observation); }
                CHECK(c.flight.state==State::Fault);
            }
        },
        {
            "missing actual state confirmation times out", []
            {
                Context c;
                c.begin();
                c.flight.tick(1.6, 101.6, c.observation);
                c.flight.ack(176, 0);
                c.flight.tick(10., 110., c.observation);
                CHECK(c.flight.state==State::Fault);
            }
        },
        {
            "tracking error aborts", []
            {
                Context c;
                c.flying(false);
                c.observation.position.x() = 10.;
                CHECK(!c.flight.tick(2.,102.,c.observation).heartbeat);
                CHECK(c.flight.state==State::Fault);
            }
        },
        {
            "no in air reset or forced disarm", []
            {
                Context c;
                c.flying();
                CHECK(!c.flight.reset(2.,c.observation).first);
                CHECK(c.flight.land(2.,c.observation).first);
                const auto a = c.flight.tick(2., 102., c.observation);
                CHECK(a.command->id==21 && !a.heartbeat);
                c.observation.armed = false;
                c.flight.tick(3., 103., c.observation);
                CHECK(c.flight.state==State::Manual);
                CHECK(c.flight.reset(4.,c.observation).first);
                CHECK(c.flight.state==State::WaitReady);
            }
        },
        {
            "land retries bounded and stale status sends nothing", []
            {
                Context c;
                c.flying();
                c.observation.healthy = false;
                int commands = 0;
                for (double t : {2., 3.1, 4.2, 5.3, 6.4})
                {
                    if (c.flight.tick(t, 100. + t, c.observation).command) { ++commands; }
                }
                CHECK(commands==3);
                c.observation.status_fresh = false;
                CHECK(!c.flight.tick(8.,108.,c.observation).command);
            }
        },
        {
            "invalid configuration rejected", []
            {
                throws([]
                {
                    FlightConfig c;
                    c.climb_speed = -1.;
                    Flight f(c);
                });
                throws([]
                {
                    FlightConfig c;
                    c.prime_seconds = .5;
                    Flight f(c);
                });
                throws([]
                {
                    VioConfig c;
                    c.max_age = NAN;
                    VioGate g(c);
                });
            }
        },
        {
            "JSON escapes reasons and never emits nan", []
            {
                CHECK(jsonString("a\"b\\c\n") == "\"a\\\"b\\\\c\\n\"");
                CHECK(jsonNumber(NAN)=="null");
                CHECK(jsonNumber(INFINITY)=="null");
                CHECK(jsonVector(Vec3(1.,NAN,-2.))=="[1,null,-2]");
                CHECK(jsonString(std::string(1,'\x01'))=="\"\\u0001\"");
            }
        }
    };
    for (const auto& test : tests)
    {
        try
        {
            test.second();
            std::cout << "PASS " << test.first << '\n';
        }
        catch (const std::exception& error)
        {
            std::cerr << "FAIL " << test.first << ": " << error.what() << '\n';
            return 1;
        }
    }
    std::cout << tests.size() << " C++ core cases passed\n";
    return 0;
}
