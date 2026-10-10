#include "px4_hover/core.hpp"

#include <algorithm>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace px4_hover
{
    namespace
    {
        Eigen::Matrix3d enuToNed()
        {
            Eigen::Matrix3d rotation;
            rotation << 0., 1., 0., 1., 0., 0., 0., 0., -1.;
            return rotation;
        }

        Eigen::Matrix3d fluToFrd()
        {
            return Vec3(1., -1., -1.).asDiagonal();
        }

        bool fresh(double stamp, double now, double age)
        {
            return std::isfinite(stamp) && stamp > 0. && now - stamp >= -.05 && now - stamp <= age;
        }

        void positive(double value, const char* name)
        {
            if (!std::isfinite(value) || value <= 0.)
            {
                throw std::invalid_argument(std::string(name) + " must be positive and finite");
            }
        }
    } // namespace

    Quat checkedQuaternion(const std::array<double, 4>& xyzw)
    {
        for (double value : xyzw)
        {
            if (!std::isfinite(value)) { throw std::invalid_argument("invalid quaternion"); }
        }
        Quat result(xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
        if (result.norm() < .8 || result.norm() > 1.2)
        {
            throw std::invalid_argument("quaternion norm outside [0.8, 1.2]");
        }
        return result.normalized();
    }

    double yaw(const Eigen::Matrix3d& rotation) { return std::atan2(rotation(1, 0), rotation(0, 0)); }
    double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

    Eigen::Matrix3d rotationZ(double angle)
    {
        return Eigen::AngleAxisd(angle, Vec3::UnitZ()).toRotationMatrix();
    }

    Alignment::Alignment(const std::array<double, 4>& mounting)
        : imu_from_body_(checkedQuaternion(mounting).toRotationMatrix())
    {
    }

    void Alignment::initialize(const Vec3& position, const Quat& orientation,
                               const Quat& px4_body_to_ned, const Vec3& anchor)
    {
        if (initialized_) { throw std::logic_error("alignment is already frozen; disarmed reset required"); }
        if (!position.allFinite() || !anchor.allFinite()) { throw std::invalid_argument("invalid anchor"); }
        const auto nominal = (enuToNed() * orientation.toRotationMatrix() * imu_from_body_ * fluToFrd()).eval();
        world_to_local_ = rotationZ(wrap(yaw(px4_body_to_ned.toRotationMatrix()) - yaw(nominal))) * enuToNed();
        translation_ = anchor - world_to_local_ * position;
        initialized_ = true;
    }

    ConvertedState Alignment::convert(const Vec3& position, const Quat& orientation,
                                      const Vec3& velocity, bool body_velocity) const
    {
        if (!initialized_) { throw std::logic_error("alignment is not initialized"); }
        ConvertedState state;
        state.position = world_to_local_ * position + translation_;
        state.orientation = Quat(world_to_local_ * orientation.toRotationMatrix() * imu_from_body_ * fluToFrd()).
            normalized();
        if (state.orientation.w() < 0.) { state.orientation.coeffs() *= -1.; }
        state.velocity = world_to_local_ * (body_velocity ? orientation * velocity : velocity);
        return state;
    }

    void VioConfig::validate() const
    {
        positive(max_age, "max_vio_age");
        positive(backend_age, "max_backend_age");
        positive(max_speed, "max_vio_speed");
        positive(jump, "max_pose_jump");
        positive(yaw_jump, "max_yaw_jump");
        positive(stable_seconds, "stable_seconds");
        if (expected_frame.empty()) { throw std::invalid_argument("vins_world_frame is empty"); }
    }

    VioGate::VioGate(VioConfig config) : config_(std::move(config))
    {
        config_.validate();
        clear();
    }

    void VioGate::clear()
    {
        last_.reset();
        backend_stamp_.reset();
        stable_since_.reset();
        received_ = backend_received_ = kNever;
        latched_.clear();
        rejection_ = "waiting for VINS";
        ++reset_counter_;
    }

    void VioGate::invalidate(const std::string& reason)
    {
        latched_ = reason;
        stable_since_.reset();
    }

    bool VioGate::backend(double stamp, double now, double ros_now)
    {
        if (!fresh(stamp, ros_now, config_.backend_age)) { return false; }
        if (backend_stamp_ && stamp < *backend_stamp_)
        {
            invalidate("VINS backend time moved backwards; disarmed reset required");
            return false;
        }
        if (backend_stamp_ && stamp == *backend_stamp_) { return false; }
        backend_stamp_ = stamp;
        backend_received_ = now;
        return true;
    }

    bool VioGate::accept(double stamp, const Vec3& position, const std::array<double, 4>& xyzw,
                         const Vec3& velocity, const std::string& frame, double now, double ros_now)
    {
        if (!latched_.empty()) { return false; }
        try
        {
            if (frame != config_.expected_frame)
            {
                throw std::invalid_argument("VINS frame mismatch: expected " + config_.expected_frame);
            }
            if (!position.allFinite() || !velocity.allFinite())
            {
                throw std::invalid_argument("non-finite VINS position/velocity");
            }
            const double heading = yaw(checkedQuaternion(xyzw).toRotationMatrix());
            if (velocity.norm() > config_.max_speed)
            {
                throw std::invalid_argument("VINS velocity exceeds max_vio_speed");
            }
            if (!fresh(stamp, ros_now, config_.max_age))
            {
                throw std::invalid_argument("VINS sample timestamp stale/future; check camera/IMU clock");
            }
            if (last_)
            {
                const double dt = stamp - last_->stamp;
                if (dt < 0.)
                {
                    invalidate("VINS time moved backwards; disarmed reset required");
                    return false;
                }
                if (dt == 0.) { return false; }
                if (now - received_ > config_.max_age || dt > config_.max_age) { stable_since_.reset(); }
                if ((position - last_->position).norm() > config_.jump + config_.max_speed * dt)
                {
                    invalidate("VINS position jump; disarmed reset required");
                    return false;
                }
                if (std::abs(wrap(heading - last_->heading)) > config_.yaw_jump + 3. * dt)
                {
                    invalidate("VINS yaw jump; disarmed reset required");
                    return false;
                }
            }
            last_ = Sample{stamp, position, heading};
            received_ = now;
            if (!stable_since_) { stable_since_ = now; }
            rejection_.clear();
            return true;
        }
        catch (const std::invalid_argument& error)
        {
            rejection_ = error.what();
            stable_since_.reset();
            return false;
        }
    }

    Check VioGate::health(double now, double ros_now) const
    {
        if (!latched_.empty()) { return {false, latched_}; }
        if (!last_ || now - received_ > config_.max_age)
        {
            return {false, rejection_.empty() ? "VINS propagated state timed out" : rejection_};
        }
        if (!fresh(last_->stamp, ros_now, config_.max_age)) { return {false, "VINS sample time is stale/future"}; }
        if (!backend_stamp_ || now - backend_received_ > config_.backend_age ||
            !fresh(*backend_stamp_, ros_now, config_.backend_age))
        {
            return {false, "VINS visual optimization timed out (IMU propagation alone is insufficient)"};
        }
        if (!stable_since_ || now - *stable_since_ < config_.stable_seconds)
        {
            return {false, "waiting for stable VINS measurements"};
        }
        return {true, "VINS measurements valid"};
    }

    bool VioGate::publishable(double sample, double now, double ros_now) const
    {
        return latched_.empty() && rejection_.empty() && backend_stamp_ &&
            now - backend_received_ <= config_.backend_age && now - received_ <= config_.max_age &&
            fresh(sample, ros_now, config_.max_age) && fresh(*backend_stamp_, ros_now, config_.backend_age);
    }

    const char* stateName(State state)
    {
        switch (state)
        {
        case State::WaitReady: return "WAIT_READY";
        case State::Ready: return "READY";
        case State::Priming: return "PRIMING";
        case State::RequestOffboard: return "REQUEST_OFFBOARD";
        case State::RequestArm: return "REQUEST_ARM";
        case State::Takeoff: return "TAKEOFF";
        case State::Hover: return "HOVER";
        case State::Manual: return "MANUAL";
        case State::Fault: return "FAULT";
        case State::Landing: return "LANDING";
        }
        return "UNKNOWN";
    }

    bool active(State state)
    {
        return state == State::Priming || state == State::RequestOffboard || state == State::RequestArm ||
            state == State::Takeoff || state == State::Hover;
    }

    void FlightConfig::validate() const
    {
        positive(takeoff_height, "takeoff_height");
        positive(climb_speed, "climb_speed");
        positive(prime_seconds, "prime_seconds");
        positive(command_interval, "command_interval");
        positive(command_timeout, "command_timeout");
        positive(max_start_speed, "max_start_speed");
        positive(max_hover_error, "max_hover_error");
        positive(hover_tolerance, "hover_tolerance");
        if (prime_seconds < 1.1 || max_command_attempts < 1)
        {
            throw std::invalid_argument("prime_seconds >= 1.1 and max_command_attempts >= 1 required");
        }
    }

    Flight::Flight(FlightConfig config) : config_(config) { config_.validate(); }

    void Flight::transition(State next, double now, const std::string& detail)
    {
        state = next;
        reason = detail;
        entered_ = now;
        attempts_ = 0;
        last_command_ = kNever;
        pending_ack_.reset();
        command_denied_ = false;
    }

    Check Flight::begin(double now, double ros_now, const Observation& o, bool takeoff)
    {
        if ((state != State::Ready && state != State::Manual) || !o.healthy || !o.status_fresh)
        {
            return {false, "not ready: " + o.reason};
        }
        if (o.failsafe) { return {false, "PX4 is in failsafe"}; }
        if (!o.position.allFinite() || !o.velocity.allFinite() || !std::isfinite(o.heading))
        {
            return {false, "non-finite PX4 local state"};
        }
        if (!takeoff && !o.armed) { return {false, "hold requires an already armed vehicle"}; }
        if (!o.armed && !config_.allow_arm_requests)
        {
            return {false, "arm with RC first, or explicitly enable allow_arm_requests"};
        }
        if (o.velocity.norm() > config_.max_start_speed) { return {false, "initial speed exceeds max_start_speed"}; }
        target = o.position;
        heading_ = o.heading;
        goal_z_ = o.position.z() - (takeoff ? config_.takeoff_height : 0.);
        ever_armed_ = o.armed;
        last_ros_ = ros_now;
        transition(State::Priming, now, takeoff ? "explicit takeoff request" : "explicit hold request");
        return {true, "accepted; monitor status for actual OFFBOARD and arming"};
    }

    void Flight::ack(uint32_t command, uint8_t result)
    {
        if (pending_ack_ && command == *pending_ack_ && (result == 2 || result == 3 || result == 4 || result == 6))
        {
            command_denied_ = true;
            reason = "command " + std::to_string(command) + " rejected, result=" + std::to_string(result);
        }
    }

    void Flight::fault(double now, const std::string& detail)
    {
        transition(State::Fault, now, detail);
        land_attempts_ = 0;
    }

    Check Flight::reset(double now, const Observation& o)
    {
        if (o.armed || !o.status_fresh) { return {false, "reset requires fresh disarmed PX4 status"}; }
        transition(State::WaitReady, now, "operator reset; waiting for fresh localization");
        target.reset();
        last_ros_.reset();
        return {true, "reset accepted; no automatic flight restart"};
    }

    Check Flight::land(double now, const Observation& o)
    {
        if (!o.status_fresh || !o.armed) { return {false, "land requires fresh armed PX4 status"}; }
        if (o.nav_state != kOffboard || o.failsafe)
        {
            return {false, "manager LAND requires OFFBOARD without PX4 failsafe; use RC/PX4 otherwise"};
        }
        transition(State::Landing, now, "operator requested PX4 LAND");
        land_attempts_ = 0;
        return {true, "LAND requested; never sends in-air disarm"};
    }

    Action Flight::tick(double now, double ros_now, const Observation& o)
    {
        Action out;
        if (state == State::WaitReady || state == State::Ready || state == State::Manual)
        {
            if (state != State::Manual)
            {
                state = o.healthy ? State::Ready : State::WaitReady;
                reason = o.healthy ? "ready; waiting for explicit service" : o.reason;
            }
            return out;
        }
        if (active(state))
        {
            if (o.status_fresh && o.failsafe) { transition(State::Manual, now, "PX4 failsafe owns control"); }
            else if (last_ros_ && ros_now < *last_ros_) { fault(now, "ROS clock moved backwards"); }
            else if (!o.healthy) { fault(now, o.reason); }
            else if ((state == State::Takeoff || state == State::Hover || state == State::RequestArm) && o.nav_state !=
                kOffboard)
            {
                transition(State::Manual, now, "PX4/RC left OFFBOARD; no automatic re-entry");
            }
            else if (ever_armed_ && !o.armed) { transition(State::Manual, now, "PX4 disarmed; no automatic re-arm"); }
            else if (command_denied_) { fault(now, reason); }
        }
        if (state == State::Fault || state == State::Landing)
        {
            if (o.status_fresh && o.armed && !o.failsafe && o.nav_state == kOffboard &&
                now - last_command_ >= config_.command_interval && land_attempts_ < 3)
            {
                out.command = Command{21, 0., 0.};
                last_command_ = now;
                ++land_attempts_;
            }
            if (state == State::Landing && o.status_fresh && !o.armed)
            {
                transition(State::Manual, now, "landing completed/disarmed; explicit restart required");
            }
            return out;
        }
        if (!active(state)) { return out; }
        const auto previous_ros = last_ros_;
        last_ros_ = ros_now;
        ever_armed_ = ever_armed_ || o.armed;
        if (state == State::Priming && now - entered_ >= config_.prime_seconds)
        {
            transition(State::RequestOffboard, now, "requesting OFFBOARD");
        }
        if (state == State::RequestOffboard && o.nav_state == kOffboard)
        {
            transition(o.armed ? State::Takeoff : State::RequestArm, now, "OFFBOARD confirmed by PX4");
        }
        if (state == State::RequestArm && o.armed) { transition(State::Takeoff, now, "arming confirmed by PX4"); }
        if (state == State::RequestOffboard || state == State::RequestArm)
        {
            if (now - entered_ > config_.command_timeout)
            {
                fault(now, "command/state confirmation timed out");
                return out;
            }
            if (now - last_command_ >= config_.command_interval)
            {
                if (attempts_ >= config_.max_command_attempts)
                {
                    fault(now, "command retry limit reached");
                    return out;
                }
                out.command = state == State::RequestOffboard ? Command{176, 1., 6.} : Command{400, 1., 0.};
                pending_ack_ = out.command->id;
                last_command_ = now;
                ++attempts_;
            }
        }
        if (state == State::Takeoff)
        {
            const double dt = std::clamp(ros_now - previous_ros.value_or(ros_now), 0., .1);
            target->z() = std::max(goal_z_, target->z() - config_.climb_speed * dt);
            if (target->z() == goal_z_ && std::abs(o.position.z() - goal_z_) < config_.hover_tolerance)
            {
                transition(State::Hover, now, "target reached");
            }
        }
        if ((state == State::Takeoff || state == State::Hover) && (o.position - *target).norm() > config_.
            max_hover_error)
        {
            fault(now, "tracking error exceeds max_hover_error");
            return out;
        }
        out.heartbeat = true;
        out.position = *target;
        out.heading = heading_;
        return out;
    }

    std::string jsonString(const std::string& value)
    {
        std::ostringstream out;
        out.imbue(std::locale::classic());
        out << '"';
        for (unsigned char character : value)
        {
            switch (character)
            {
            case '"': out << "\\\"";
                break;
            case '\\': out << "\\\\";
                break;
            case '\n': out << "\\n";
                break;
            case '\r': out << "\\r";
                break;
            case '\t': out << "\\t";
                break;
            default:
                if (character < 0x20)
                {
                    out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(character) <<
                        std::dec;
                }
                else { out << static_cast<char>(character); }
            }
        }
        out << '"';
        return out.str();
    }

    std::string jsonNumber(double value)
    {
        if (!std::isfinite(value)) { return "null"; }
        std::ostringstream out;
        out.imbue(std::locale::classic());
        out << std::setprecision(15) << value;
        return out.str();
    }

    std::string jsonVector(const Vec3& value)
    {
        return "[" + jsonNumber(value.x()) + "," + jsonNumber(value.y()) + "," + jsonNumber(value.z()) + "]";
    }
} // namespace px4_hover
