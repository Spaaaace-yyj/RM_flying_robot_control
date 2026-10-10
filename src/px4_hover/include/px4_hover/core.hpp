#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace px4_hover
{
    using Vec3 = Eigen::Vector3d;
    using Quat = Eigen::Quaterniond;
    using Check = std::pair<bool, std::string>;
    constexpr double kNever = -std::numeric_limits<double>::infinity();
    constexpr int kOffboard = 14; // PX4 v1.14 VehicleStatus

    Quat checkedQuaternion(const std::array<double, 4>& xyzw);
    double yaw(const Eigen::Matrix3d& rotation);
    double wrap(double angle);
    Eigen::Matrix3d rotationZ(double angle);

    struct ConvertedState
    {
        Vec3 position = Vec3::Zero();
        Quat orientation = Quat::Identity();
        Vec3 velocity = Vec3::Zero();
    };

    class Alignment
    {
    public:
        explicit Alignment(const std::array<double, 4>& imu_from_body_xyzw = {0., 0., 0., 1.});
        void initialize(const Vec3& position, const Quat& orientation,
                        const Quat& px4_body_to_ned, const Vec3& anchor);
        ConvertedState convert(const Vec3& position, const Quat& orientation,
                               const Vec3& velocity, bool body_velocity = false) const;
        bool initialized() const { return initialized_; }

    private:
        Eigen::Matrix3d imu_from_body_;
        Eigen::Matrix3d world_to_local_ = Eigen::Matrix3d::Identity();
        Vec3 translation_ = Vec3::Zero();
        bool initialized_ = false;
    };

    struct VioConfig
    {
        double max_age = .3, backend_age = .8, max_speed = 8.;
        double jump = .4, yaw_jump = .6, stable_seconds = 2.;
        std::string expected_frame = "world";
        void validate() const;
    };

    class VioGate
    {
    public:
        explicit VioGate(VioConfig config = {});
        void clear();
        void invalidate(const std::string& reason);
        bool backend(double stamp, double steady_now, double ros_now);
        bool accept(double stamp, const Vec3& position, const std::array<double, 4>& xyzw,
                    const Vec3& velocity, const std::string& frame, double steady_now, double ros_now);
        Check health(double steady_now, double ros_now) const;
        bool publishable(double sample, double steady_now, double ros_now) const;
        uint64_t resetCounter() const { return reset_counter_; }
        const std::string& latched() const { return latched_; }
        double received() const { return received_; }

    private:
        struct Sample
        {
            double stamp;
            Vec3 position;
            double heading;
        };

        VioConfig config_;
        std::optional<Sample> last_;
        std::optional<double> backend_stamp_, stable_since_;
        double backend_received_ = kNever, received_ = kNever;
        std::string latched_, rejection_;
        uint64_t reset_counter_ = 0;
    };

    enum class State
    {
        WaitReady, Ready, Priming, RequestOffboard, RequestArm,
        Takeoff, Hover, Manual, Fault, Landing
    };

    const char* stateName(State state);
    bool active(State state);

    struct FlightConfig
    {
        bool allow_arm_requests = false;
        double takeoff_height = 1., climb_speed = .25, prime_seconds = 1.5;
        double command_interval = 1., command_timeout = 8.;
        int max_command_attempts = 5;
        double max_start_speed = .35, max_hover_error = 1., hover_tolerance = .15;
        void validate() const;
    };

    struct Observation
    {
        bool healthy = false, status_fresh = false, armed = false, failsafe = false;
        std::string reason = "waiting for localization and PX4";
        int nav_state = -1;
        Vec3 position = Vec3::Zero(), velocity = Vec3::Zero();
        double heading = 0.;
    };

    struct Command
    {
        uint32_t id;
        float param1 = 0., param2 = 0.;
    };

    struct Action
    {
        bool heartbeat = false;
        Vec3 position = Vec3::Zero();
        double heading = 0.;
        std::optional<Command> command;
    };

    class Flight
    {
    public:
        explicit Flight(FlightConfig config = {});
        Check begin(double steady_now, double ros_now, const Observation& observation, bool takeoff);
        void ack(uint32_t command, uint8_t result);
        void fault(double steady_now, const std::string& reason);
        Check reset(double steady_now, const Observation& observation);
        Check land(double steady_now, const Observation& observation);
        Action tick(double steady_now, double ros_now, const Observation& observation);

        State state = State::WaitReady;
        std::string reason = "waiting for localization and PX4";
        std::optional<Vec3> target;

    private:
        void transition(State next, double steady_now, const std::string& reason);
        FlightConfig config_;
        double heading_ = 0., goal_z_ = 0., entered_ = 0., last_command_ = kNever;
        int attempts_ = 0, land_attempts_ = 0;
        bool ever_armed_ = false, command_denied_ = false;
        std::optional<double> last_ros_;
        std::optional<uint32_t> pending_ack_;
    };

    // Dependency-free JSON diagnostics: invalid floating values become null.
    std::string jsonString(const std::string& value);
    std::string jsonNumber(double value);
    std::string jsonVector(const Vec3& value);
} // namespace px4_hover
