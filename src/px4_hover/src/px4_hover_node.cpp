#include "px4_hover/core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/estimator_status_flags.hpp>

#include <algorithm>
#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace px4_hover
{
    using namespace px4_msgs::msg;
    using VinsOdom = nav_msgs::msg::Odometry;
    using Trigger = std_srvs::srv::Trigger;

    namespace
    {
        double steadySeconds()
        {
            return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        }

        double stampSeconds(const builtin_interfaces::msg::Time& stamp)
        {
            return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
        }

        std::array<float, 3> floats(const Vec3& v)
        {
            return {static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z())};
        }
    } // namespace

    class HoverNode : public rclcpp::Node
    {
    public:
        HoverNode() : Node("px4_hover")
        {
            //VINS用IMU前向预测的里程计话题
            state_topic_ = parameter<std::string>("vins_state_topic", "/vins_estimator/imu_propagate");
            //VINS最后输出的里程计话题
            backend_topic_ = parameter<std::string>("vins_backend_topic", "/vins_estimator/odometry");
            //VINS视觉失效重置后发布标志位
            restart_topic_ = parameter<std::string>("vins_restart_topic", "/feature_tracker/restart");
            vio_config_.expected_frame = parameter<std::string>("vins_world_frame", "world");
            const auto velocity_frame = parameter<std::string>("vins_velocity_frame", "world");
            if (velocity_frame != "world" && velocity_frame != "body")
            {
                throw std::invalid_argument("vins_velocity_frame must be world or body");
            }
            body_velocity_ = velocity_frame == "body";

            //无人机机体坐标到VINS使用的IMU坐标系之间的安装旋转
            const auto mounting = parameter<std::vector<double>>("imu_from_body_xyzw", {0., 0., 0., 1.});
            if (mounting.size() != 4)
            {
                throw std::invalid_argument("imu_from_body_xyzw must contain four values");
            }
            std::copy(mounting.begin(), mounting.end(), mounting_.begin());

            //px4消息名字空间
            std::string ns = parameter<std::string>("px4_namespace", "/fmu");
            while (!ns.empty() && ns.back() == '/')
            {
                ns.pop_back();
            }

            target_system_ = idParameter("target_system", 1);
            target_component_ = idParameter("target_component", 1);
            source_system_ = idParameter("source_system", 245);
            source_component_ = idParameter("source_component", 191);
            rate_ = positiveParameter("publish_rate_hz", 50.);
            if (rate_ < 10. || rate_ > 200.) { throw std::invalid_argument("publish_rate_hz must be in [10, 200]"); }
            vio_config_.max_age = positiveParameter("max_vio_age", .3);
            vio_config_.backend_age = positiveParameter("max_backend_age", .8);
            vio_config_.max_speed = positiveParameter("max_vio_speed", 8.);
            vio_config_.jump = positiveParameter("max_pose_jump", .4);
            vio_config_.yaw_jump = positiveParameter("max_yaw_jump", .6);
            vio_config_.stable_seconds = positiveParameter("stable_seconds", 2.);
            max_px4_age_ = positiveParameter("max_px4_age", .5);
            max_sync_age_ = positiveParameter("max_timesync_age", 5.);
            max_round_trip_ms_ = positiveParameter("max_round_trip_ms", 20.);
            require_timesync_ = parameter<bool>("require_timesync", true);
            require_ev_status_ = parameter<bool>("require_ev_status", true);
            require_ev_yaw_ = parameter<bool>("require_ev_yaw", true);
            position_stddev_ = positiveParameter("position_stddev", .1);
            velocity_stddev_ = positiveParameter("velocity_stddev", .15);
            orientation_stddev_ = positiveParameter("orientation_stddev", .1);

            FlightConfig fc;
            fc.allow_arm_requests = parameter<bool>("allow_arm_requests", false);
            fc.takeoff_height = positiveParameter("takeoff_height", 1.);
            fc.climb_speed = positiveParameter("climb_speed", .25);
            fc.prime_seconds = positiveParameter("prime_seconds", 1.5);
            fc.command_interval = positiveParameter("command_interval", 1.);
            fc.command_timeout = positiveParameter("command_timeout", 8.);
            const int64_t attempts = parameter<int64_t>("max_command_attempts", 5);
            if (attempts < 1 || attempts > std::numeric_limits<int>::max())
            {
                throw std::invalid_argument("invalid max_command_attempts");
            }
            fc.max_command_attempts = static_cast<int>(attempts);
            fc.max_start_speed = positiveParameter("max_start_speed", .35);
            fc.max_hover_error = positiveParameter("max_hover_error", 1.);
            fc.hover_tolerance = positiveParameter("hover_tolerance", .15);
            flight_ = Flight(fc);
            gate_ = VioGate(vio_config_);
            alignment_ = Alignment(mounting_);

            mode_topic_ = ns + "/in/offboard_control_mode";
            visual_pub_ = create_publisher<VehicleOdometry>(ns + "/in/vehicle_visual_odometry", 10);
            mode_pub_ = create_publisher<OffboardControlMode>(mode_topic_, 10); //心跳数据
            setpoint_pub_ = create_publisher<TrajectorySetpoint>(ns + "/in/trajectory_setpoint", 10);
            command_pub_ = create_publisher<VehicleCommand>(ns + "/in/vehicle_command", 10);
            diagnostic_pub_ = create_publisher<std_msgs::msg::String>("~/status", 10);
            const auto qos = rclcpp::SensorDataQoS();
            subscribe<VinsOdom>(state_topic_, qos, [this](VinsOdom::ConstSharedPtr m) { onVio(*m); });
            subscribe<VinsOdom>(backend_topic_, qos, [this](VinsOdom::ConstSharedPtr m)
            {
                gate_.backend(stampSeconds(m->header.stamp), steadySeconds(), rosSeconds());
            });
            subscribe<std_msgs::msg::Bool>(restart_topic_, rclcpp::QoS(10),
                                           [this](std_msgs::msg::Bool::ConstSharedPtr m)
                                           {
                                               if (m->data)
                                               {
                                                   gate_.invalidate(
                                                       "feature_tracker restarted; disarmed reset required");
                                               }
                                           });
            subscribe<VehicleStatus>(ns + "/out/vehicle_status", qos, [this](VehicleStatus::ConstSharedPtr m)
            {
                if (last_status_stamp_ && m->timestamp < *last_status_stamp_)
                {
                    clock_guard_ = "PX4 status time moved backwards; reset after disarming";
                    gate_.invalidate(clock_guard_);
                }
                if (last_status_stamp_ && m->timestamp == *last_status_stamp_) { return; }
                last_status_stamp_ = m->timestamp;
                status_ = m;
                status_rx_ = steadySeconds();
            });
            subscribe<VehicleLocalPosition>(ns + "/out/vehicle_local_position", qos,
                                            [this](VehicleLocalPosition::ConstSharedPtr m) { onLocal(m); });
            subscribe<VehicleAttitude>(ns + "/out/vehicle_attitude", qos, [this](VehicleAttitude::ConstSharedPtr m)
            {
                try
                {
                    const auto q = checkedQuaternion({m->q[1], m->q[2], m->q[3], m->q[0]});
                    attitudes_.push_back({static_cast<double>(m->timestamp_sample) * 1e-6, steadySeconds(), q});
                    if (attitudes_.size() > 100) { attitudes_.pop_front(); }
                }
                catch (const std::invalid_argument&)
                {
                    /* Invalid attitude cannot establish alignment. */
                }
            });
            subscribe<TimesyncStatus>(ns + "/out/timesync_status", qos, [this](TimesyncStatus::ConstSharedPtr m)
            {
                sync_ = m;
                sync_rx_ = steadySeconds();
            });
            subscribe<EstimatorStatusFlags>(ns + "/out/estimator_status_flags", qos,
                                            [this](EstimatorStatusFlags::ConstSharedPtr m)
                                            {
                                                ev_ = m;
                                                ev_rx_ = steadySeconds();
                                            });
            subscribe<VehicleCommandAck>(ns + "/out/vehicle_command_ack", qos,
                                         [this](VehicleCommandAck::ConstSharedPtr m)
                                         {
                                             if (m->target_system == source_system_ && m->target_component ==
                                                 source_component_)
                                             {
                                                 flight_.ack(m->command, m->result);
                                                 RCLCPP_INFO(get_logger(), "PX4 ACK command=%u result=%u",
                                                             static_cast<unsigned>(m->command),
                                                             static_cast<unsigned>(m->result));
                                             }
                                         });
            service("~/takeoff", [this] { return begin(true); });
            service("~/hold", [this] { return begin(false); });
            service("~/land", [this]
            {
                const double t = steadySeconds();
                return flight_.land(t, observation(t, rosSeconds()));
            });
            service("~/reset", [this] { return reset(); });
            // Wall clock keeps watchdog evaluation running while /clock is paused.
            timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                           std::chrono::duration<double>(1. / rate_)), [this] { tick(); });
            RCLCPP_INFO(get_logger(), "C++ PX4 hover: no startup arming; wait for READY then call takeoff/hold");
        }

    private:
        template <class T>
        T parameter(const std::string& name, const T& initial)
        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.read_only = true; // Launch overrides accepted; reconfiguration requires restart.
            return declare_parameter<T>(name, initial, descriptor);
        }

        double positiveParameter(const std::string& name, double initial)
        {
            const double value = parameter<double>(name, initial);
            if (!std::isfinite(value) || value <= 0.)
            {
                throw std::invalid_argument(name + " must be positive and finite");
            }
            return value;
        }

        uint8_t idParameter(const std::string& name, int64_t initial)
        {
            const auto value = parameter<int64_t>(name, initial);
            if (value < 1 || value > 255) { throw std::invalid_argument(name + " must be in [1, 255]"); }
            return static_cast<uint8_t>(value);
        }

        template <class Message, class Callback>
        void subscribe(const std::string& topic, const rclcpp::QoS& qos, Callback callback)
        {
            subscriptions_.push_back(create_subscription<Message>(topic, qos, std::move(callback)));
        }

        void service(const char* name, std::function<Check()> callback)
        {
            services_.push_back(create_service<Trigger>(name,
                                                        [callback](const std::shared_ptr<Trigger::Request>,
                                                                   std::shared_ptr<Trigger::Response> response)
                                                        {
                                                            const auto result = callback();
                                                            response->success = result.first;
                                                            response->message = result.second;
                                                        }));
        }

        double rosSeconds() const { return static_cast<double>(this->now().nanoseconds()) * 1e-9; }

        template <class Message>
        bool px4Fresh(const std::shared_ptr<const Message>& message, double received, double now, double ros_now) const
        {
            if (!message || now - received > max_px4_age_) { return false; }
            const double age = ros_now - static_cast<double>(message->timestamp) * 1e-6;
            return age >= -.05 && age <= max_px4_age_;
        }

        bool timeReady(double now, double ros_now) const
        {
            if (!clock_guard_.empty()) { return false; }
            if (!require_timesync_) { return true; }
            return sync_ && now - sync_rx_ <= max_sync_age_ && sync_->source_protocol ==
                TimesyncStatus::SOURCE_PROTOCOL_DDS &&
                sync_->round_trip_time <= max_round_trip_ms_ * 1000. && px4Fresh(status_, status_rx_, now, ros_now);
        }

        void onLocal(const VehicleLocalPosition::ConstSharedPtr& m)
        {
            const std::array<uint8_t, 5> counters{
                m->xy_reset_counter, m->z_reset_counter, m->heading_reset_counter,
                m->vxy_reset_counter, m->vz_reset_counter
            };
            if (local_counters_ && counters != *local_counters_ && active(flight_.state))
            {
                gate_.invalidate("PX4 local estimator reset during flight");
            }
            if (local_ && m->timestamp <= local_->timestamp) { return; }
            local_ = m;
            local_rx_ = steadySeconds();
            local_counters_ = counters;
        }

        void onVio(const VinsOdom& m)
        {
            if (!clock_guard_.empty()) { return; }
            const double now = steadySeconds(), ros_now = rosSeconds(), sample = stampSeconds(m.header.stamp);
            const auto& p = m.pose.pose.position;
            const auto& q = m.pose.pose.orientation;
            const auto& v = m.twist.twist.linear;
            const Vec3 position(p.x, p.y, p.z), velocity(v.x, v.y, v.z);
            const std::array<double, 4> orientation{q.x, q.y, q.z, q.w};
            if (!gate_.accept(sample, position, orientation, velocity, m.header.frame_id, now, ros_now)) { return; }
            const auto imu_orientation = checkedQuaternion(orientation);
            if (!alignment_.initialized())
            {
                if (attitudes_.empty() || !timeReady(now, ros_now)) { return; }
                const auto nearest = std::min_element(attitudes_.begin(), attitudes_.end(),
                                                      [sample](const Attitude& a, const Attitude& b)
                                                      {
                                                          return std::abs(a.sample - sample) < std::abs(
                                                              b.sample - sample);
                                                      });
                if (std::abs(nearest->sample - sample) > .1 || now - nearest->received > max_px4_age_) { return; }
                Vec3 anchor = Vec3::Zero();
                if (px4Fresh(local_, local_rx_, now, ros_now) && local_->xy_valid && local_->z_valid)
                {
                    anchor = Vec3(local_->x, local_->y, local_->z);
                }
                if (!anchor.allFinite()) { return; }
                alignment_.initialize(position, imu_orientation, nearest->orientation, anchor);
                RCLCPP_INFO(get_logger(), "Frozen VINS -> PX4 local yaw/origin alignment");
            }
            last_vio_ = VioState{sample, alignment_.convert(position, imu_orientation, velocity, body_velocity_)};
        }

        Observation observation(double now, double ros_now) const
        {
            Observation out;
            auto check = gate_.health(now, ros_now);
            auto reject = [&check](const std::string& reason) { check = {false, reason}; };
            out.status_fresh = px4Fresh(status_, status_rx_, now, ros_now);
            if (!timeReady(now, ros_now))
            {
                reject(clock_guard_.empty()
                           ? "waiting for DDS timesync; timestamps must share ROS clock"
                           : clock_guard_);
            }
            else if (!alignment_.initialized()) { reject("waiting for timestamp-matched PX4 attitude to align VINS"); }
            else if (!out.status_fresh || !px4Fresh(local_, local_rx_, now, ros_now))
            {
                reject("PX4 status/local position stale or timestamp domain mismatch");
            }
            else if (!(local_->xy_valid && local_->z_valid && local_->v_xy_valid && local_->v_z_valid &&
                local_->heading_good_for_control && !local_->dead_reckoning))
            {
                reject("PX4 local position/velocity/heading is invalid or dead reckoning");
            }
            else if (require_ev_status_)
            {
                if (!px4Fresh(ev_, ev_rx_, now, ros_now))
                {
                    reject("missing estimator_status_flags; add PX4 DDS publication");
                }
                else if (!(ev_->cs_ev_pos && ev_->cs_ev_hgt && ev_->cs_tilt_align && ev_->cs_yaw_align &&
                    (!require_ev_yaw_ || ev_->cs_ev_yaw)))
                {
                    reject("EKF2 external vision position/height/yaw fusion not active");
                }
                else if (ev_->reject_hor_pos || ev_->reject_ver_pos || ev_->reject_yaw)
                {
                    reject("EKF2 rejects external vision measurement");
                }
            }
            if (local_)
            {
                out.position = Vec3(local_->x, local_->y, local_->z);
                out.velocity = Vec3(local_->vx, local_->vy, local_->vz);
                out.heading = local_->heading;
            }
            if (!out.position.allFinite() || !out.velocity.allFinite() || !std::isfinite(out.heading))
            {
                reject("non-finite PX4 local state");
            }
            if (count_publishers(mode_topic_) > 1)
            {
                reject("another Offboard heartbeat publisher exists; stop competing controllers");
            }
            out.healthy = check.first;
            out.reason = check.second;
            if (status_)
            {
                out.armed = status_->arming_state == VehicleStatus::ARMING_STATE_ARMED;
                out.nav_state = status_->nav_state;
                out.failsafe = status_->failsafe || status_->failure_detector_status != 0;
            }
            return out;
        }

        Check begin(bool takeoff)
        {
            const double now = steadySeconds(), ros_now = rosSeconds();
            return flight_.begin(now, ros_now, observation(now, ros_now), takeoff);
        }

        Check reset()
        {
            const double now = steadySeconds();
            const auto result = flight_.reset(now, observation(now, rosSeconds()));
            if (result.first)
            {
                gate_.clear();
                alignment_ = Alignment(mounting_);
                last_vio_.reset();
                last_ev_sample_.reset();
                local_counters_.reset();
                last_status_stamp_.reset();
                local_.reset();
                ev_.reset();
                sync_.reset();
                attitudes_.clear();
                clock_guard_.clear();
                last_ros_.reset();
            }
            return result;
        }

        void sendCommand(const Command& c, uint64_t timestamp)
        {
            VehicleCommand m;
            m.timestamp = timestamp;
            m.command = c.id;
            m.param1 = c.param1;
            m.param2 = c.param2;
            m.target_system = target_system_;
            m.target_component = target_component_;
            m.source_system = source_system_;
            m.source_component = source_component_;
            m.from_external = true;
            command_pub_->publish(m);
        }

        void tick()
        {
            const double now = steadySeconds(), ros_now = rosSeconds();
            if (last_ros_ && ros_now < *last_ros_)
            {
                clock_guard_ = "ROS clock moved backwards; disarmed reset required";
                gate_.invalidate(clock_guard_);
            }
            last_ros_ = ros_now;
            const auto timestamp = static_cast<uint64_t>(std::max<int64_t>(0, get_clock()->now().nanoseconds() / 1000));
            // PX4 v1.14 DDS already translates both timestamps. Send ROS time, without subtracting the offset again.
            if (last_vio_ && gate_.publishable(last_vio_->sample, now, ros_now) && timeReady(now, ros_now) &&
                (!last_ev_sample_ || *last_ev_sample_ != last_vio_->sample))
            {
                VehicleOdometry m;
                m.timestamp = timestamp;
                m.timestamp_sample = static_cast<uint64_t>(std::llround(last_vio_->sample * 1e6));
                m.pose_frame = VehicleOdometry::POSE_FRAME_NED;
                m.velocity_frame = VehicleOdometry::VELOCITY_FRAME_NED;
                m.position = floats(last_vio_->state.position);
                m.velocity = floats(last_vio_->state.velocity);
                const auto& q = last_vio_->state.orientation;
                m.q = {
                    static_cast<float>(q.w()), static_cast<float>(q.x()), static_cast<float>(q.y()),
                    static_cast<float>(q.z())
                };
                m.angular_velocity.fill(std::numeric_limits<float>::quiet_NaN());
                m.position_variance.fill(static_cast<float>(position_stddev_ * position_stddev_));
                m.velocity_variance.fill(static_cast<float>(velocity_stddev_ * velocity_stddev_));
                m.orientation_variance.fill(static_cast<float>(orientation_stddev_ * orientation_stddev_));
                m.reset_counter = static_cast<uint8_t>(gate_.resetCounter() % 256);
                m.quality = -1;
                visual_pub_->publish(m);
                last_ev_sample_ = last_vio_->sample;
            }
            const auto o = observation(now, ros_now);
            const auto action = flight_.tick(now, ros_now, o);
            if (action.heartbeat)
            {
                OffboardControlMode mode;
                mode.timestamp = timestamp;
                mode.position = true;
                TrajectorySetpoint sp;
                sp.timestamp = timestamp;
                sp.position = floats(action.position);
                sp.yaw = static_cast<float>(action.heading);
                const auto nan = std::numeric_limits<float>::quiet_NaN();
                sp.velocity.fill(nan);
                sp.acceleration.fill(nan);
                sp.jerk.fill(nan);
                sp.yawspeed = nan;
                setpoint_pub_->publish(sp);
                mode_pub_->publish(mode);
            }
            if (action.command) { sendCommand(*action.command, timestamp); }
            const std::string current = std::string(stateName(flight_.state)) + ": " + flight_.reason;
            if (current != last_print_)
            {
                RCLCPP_INFO(get_logger(), "%s", current.c_str());
                last_print_ = current;
            }
            if (now - last_diagnostic_ >= .2)
            {
                std::ostringstream data;
                data << std::boolalpha << "{\"state\":" << jsonString(stateName(flight_.state))
                    << ",\"reason\":" << jsonString(flight_.reason) << ",\"ready\":" << o.healthy
                    << ",\"health_reason\":" << jsonString(o.reason) << ",\"armed\":" << o.armed
                    << ",\"nav_state\":" << o.nav_state << ",\"position_ned\":" << jsonVector(o.position)
                    << ",\"target_ned\":" << (flight_.target ? jsonVector(*flight_.target) : "null")
                    << ",\"vio_valid\":" << gate_.health(now, ros_now).first << ",\"reset_counter\":" << gate_.
                    resetCounter()
                    << ",\"ev_status_required\":" << require_ev_status_ << "}";
                std_msgs::msg::String m;
                m.data = data.str();
                diagnostic_pub_->publish(m);
                last_diagnostic_ = now;
            }
        }

        struct Attitude
        {
            double sample, received;
            Quat orientation;
        };

        struct VioState
        {
            double sample;
            ConvertedState state;
        };

        VioConfig vio_config_;
        VioGate gate_;
        Alignment alignment_;
        Flight flight_;
        std::array<double, 4> mounting_{0., 0., 0., 1.};
        std::string state_topic_, backend_topic_, restart_topic_, mode_topic_, clock_guard_, last_print_;
        bool body_velocity_ = false, require_timesync_ = true, require_ev_status_ = true, require_ev_yaw_ = true;
        double rate_ = 50., max_px4_age_ = .5, max_sync_age_ = 5., max_round_trip_ms_ = 20.;
        double position_stddev_ = .1, velocity_stddev_ = .15, orientation_stddev_ = .1;
        uint8_t target_system_ = 1, target_component_ = 1, source_system_ = 245, source_component_ = 191;
        VehicleStatus::ConstSharedPtr status_;
        VehicleLocalPosition::ConstSharedPtr local_;
        EstimatorStatusFlags::ConstSharedPtr ev_;
        TimesyncStatus::ConstSharedPtr sync_;
        double status_rx_ = kNever, local_rx_ = kNever, ev_rx_ = kNever, sync_rx_ = kNever, last_diagnostic_ = kNever;
        std::deque<Attitude> attitudes_;
        std::optional<VioState> last_vio_;
        std::optional<double> last_ros_, last_ev_sample_;
        std::optional<uint64_t> last_status_stamp_;
        std::optional<std::array<uint8_t, 5>> local_counters_;
        std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
        std::vector<rclcpp::Service<Trigger>::SharedPtr> services_;
        rclcpp::Publisher<VehicleOdometry>::SharedPtr visual_pub_;
        rclcpp::Publisher<OffboardControlMode>::SharedPtr mode_pub_;
        rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_pub_;
        rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostic_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
    };
} // namespace px4_hover

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    int result = 0;
    try
    {
        // SingleThreadedExecutor: subscription, service and timer state never races.
        rclcpp::spin(std::make_shared<px4_hover::HoverNode>());
    }
    catch (const std::exception& error)
    {
        RCLCPP_FATAL(rclcpp::get_logger("px4_hover"), "%s", error.what());
        result = 1;
    }
    // No shutdown arming/disarming command. Stopped heartbeats invoke configured PX4 failsafe.
    rclcpp::shutdown();
    return result;
}
