#include "ego_depth_inputs/core.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

namespace ego_depth_inputs
{
    using Image = sensor_msgs::msg::Image;
    using Info = sensor_msgs::msg::CameraInfo;
    using Odom = nav_msgs::msg::Odometry;
    using Policy = message_filters::sync_policies::ExactTime<Image, Info>;
    using Steady = std::chrono::steady_clock;

    static std::string quoted(const std::string& s)
    {
        std::string out = "\"";
        for (unsigned char c : s)
        {
            if (c == '\\' || c == '"') { out += '\\'; }
            out += c < 0x20 ? ' ' : static_cast<char>(c);
        }
        return out + '"';
    }

    class DepthInputs : public rclcpp::Node
    {
    public:
        DepthInputs() : Node("ego_depth_inputs")
        {
            rotated_ = setting<bool>("images_rotated_180", true);
            need_bridge_ = setting<bool>("publish_camera_bridge", true);
            local_ = frame("local_frame", "odom");
            body_ = frame("body_frame", "body");
            root_ = frame("camera_root", "camera_link");
            infra_ = frame("cam0_physical", "camera_infra2_optical_frame");
            cam0_ = frame("cam0_calibrated", "camera");
            if (local_ == body_ || root_ == cam0_)
            {
                throw std::invalid_argument("local/body and calibrated/root frames must differ");
            }
            mode_ = setting<std::string>("pose_source", "imu_propagate");
            const bool fixed = setting<bool>("fixed_extrinsics", true);
            if (mode_ != "imu_propagate" && mode_ != "tf")
            {
                throw std::invalid_argument("pose_source must be imu_propagate or tf");
            }
            if (mode_ == "imu_propagate" && !fixed)
            {
                throw std::invalid_argument("imu_propagate mode requires fixed_extrinsics and estimate_extrinsic=0");
            }
            const auto input_queue = count("input_queue_size", 10, 1, 200);
            const auto sync_queue = count("sync_queue_size", 30, 1, 500);
            max_pending_ = count("max_pending_frames", 24, 1, 500);
            max_wait_ = seconds("max_pending_wait_s", .25);
            max_age_ns_ = nanoseconds(seconds("max_input_age_s", .5));
            stall_warning_ = seconds("stall_warning_s", .5);
            const double camera_offset = setting<double>("camera_imu_time_offset_s", 0.);
            if (!std::isfinite(camera_offset) || std::abs(camera_offset) > 1.)
            {
                throw std::invalid_argument("camera_imu_time_offset_s must be finite and within +/-1 s");
            }
            image_pose_offset_ns_ = nanoseconds(camera_offset);
            const auto tolerance = count("timestamp_tolerance_ns", 1000, 0, 1000);
            history_ = std::make_unique<PoseHistory>(
                count("pose_buffer_size", 200, 2, 10000),
                nanoseconds(seconds("pose_buffer_seconds", 2.)),
                nanoseconds(seconds("max_pose_gap_s", .1)), static_cast<int64_t>(tolerance));
            const double rate = seconds("processing_rate_hz", 100.);
            if (rate < 1. || rate > 500.) { throw std::invalid_argument("processing_rate_hz outside [1,500]"); }

            buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
            // Receive TF in this executor. All lookups are nonblocking so TF callbacks
            // and image callbacks can continue while a matching pose is unavailable.
            listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_, this, false);
            broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
            const auto output_qos = rclcpp::QoS(5).reliable().durability_volatile();
            depth_pub_ = create_publisher<Image>("ego_input/depth", output_qos);
            info_pub_ = create_publisher<Info>("ego_input/camera_info", output_qos);
            pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("ego_input/camera_pose", output_qos);
            status_pub_ = create_publisher<std_msgs::msg::String>("ego_input/status", 5);
            const auto depth_qos = inputQos("depth_reliability", "reliable", input_queue);
            const auto info_qos = inputQos("camera_info_reliability", "best_effort", input_queue);
            depth_sub_.subscribe(this, frame("depth_topic", "/camera/camera/depth/image_rect_raw"),
                                 depth_qos.get_rmw_qos_profile());
            info_sub_.subscribe(this, frame("camera_info_topic", "/camera/camera/depth/camera_info"),
                                info_qos.get_rmw_qos_profile());
            sync_ = std::make_unique<message_filters::Synchronizer<Policy>>(
                Policy(static_cast<uint32_t>(sync_queue)), depth_sub_, info_sub_);
            sync_->registerCallback(std::bind(&DepthInputs::receive, this,
                                              std::placeholders::_1, std::placeholders::_2));
            if (mode_ == "imu_propagate")
            {
                odom_sub_ = create_subscription<Odom>(frame("odometry_topic", "/imu_propagate"),
                                                      rclcpp::SensorDataQoS().keep_last(100),
                                                      std::bind(&DepthInputs::odometry, this, std::placeholders::_1));
            }
            else
            {
                // Still declare this parameter so a shared YAML works in both modes.
                frame("odometry_topic", "/imu_propagate");
            }
            timer_ = create_wall_timer(std::chrono::nanoseconds(nanoseconds(1. / rate)),
                                       std::bind(&DepthInputs::flush, this));
            last_diagnostic_ = Steady::now();
            diagnostic_timer_ = create_wall_timer(std::chrono::seconds(1),
                                                  std::bind(&DepthInputs::diagnostics, this));
            RCLCPP_INFO(get_logger(), "C++ depth adapter: pose_source=%s, depth QoS=%s; fixed optical extrinsics",
                        mode_.c_str(), get_parameter("depth_reliability").as_string().c_str());
        }

    private:
        template <class T>
        T setting(const std::string& name, const T& value)
        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.read_only = true;
            return declare_parameter<T>(name, value, descriptor);
        }

        std::string frame(const std::string& name, const std::string& value)
        {
            const auto out = setting<std::string>(name, value);
            if (out.empty()) { throw std::invalid_argument(name + " must not be empty"); }
            return out;
        }

        size_t count(const std::string& name, int64_t value, int64_t low, int64_t high)
        {
            const auto out = setting<int64_t>(name, value);
            if (out < low || out > high) { throw std::invalid_argument(name + " outside supported limits"); }
            return static_cast<size_t>(out);
        }

        double seconds(const std::string& name, double value)
        {
            const auto out = setting<double>(name, value);
            if (!std::isfinite(out) || out <= 0. || out > 1000.)
            {
                throw std::invalid_argument(name + " must be finite and in (0,1000]");
            }
            return out;
        }

        static int64_t nanoseconds(double seconds) { return static_cast<int64_t>(std::llround(seconds * 1e9)); }

        static double elapsed(Steady::time_point start)
        {
            return std::chrono::duration<double>(Steady::now() - start).count();
        }

        static int64_t stamp(const builtin_interfaces::msg::Time& t)
        {
            return static_cast<int64_t>(t.sec) * 1000000000LL + static_cast<int64_t>(t.nanosec);
        }

        rclcpp::QoS inputQos(const std::string& name, const std::string& value, size_t size)
        {
            const auto reliability = setting<std::string>(name, value);
            auto qos = rclcpp::QoS(size).durability_volatile();
            if (reliability == "reliable") { qos.reliable(); }
            else if (reliability == "best_effort") { qos.best_effort(); }
            else { throw std::invalid_argument(name + " must be reliable or best_effort"); }
            return qos;
        }

        static tf2::Transform transform(const geometry_msgs::msg::Transform& t)
        {
            Pose p;
            p.stamp = 1;
            p.position = {t.translation.x, t.translation.y, t.translation.z};
            p.orientation = {t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w};
            if (!validPose(p)) { throw std::invalid_argument("non-finite or invalid TF extrinsics"); }
            return tf2::Transform(
                tf2::Quaternion(p.orientation[0], p.orientation[1], p.orientation[2], p.orientation[3]),
                tf2::Vector3(p.position[0], p.position[1], p.position[2]));
        }

        void odometry(const Odom::ConstSharedPtr message)
        {
            if (message->header.frame_id != local_ || message->child_frame_id != body_ ||
                message->header.stamp.nanosec >= 1000000000U)
            {
                ++invalid_pose_;
                wait_reason_ = "odometry frame must be " + local_ + " -> " + body_;
                return;
            }
            const auto& p = message->pose.pose.position;
            const auto& q = message->pose.pose.orientation;
            const auto result = history_->add(Pose{
                stamp(message->header.stamp), {p.x, p.y, p.z}, {q.x, q.y, q.z, q.w}
            });
            if (result == Insert::Invalid) { ++invalid_pose_; }
            if (result == Insert::Reset)
            {
                dropped_ += pending_.size();
                pending_.clear();
                last_output_stamp_ = 0;
                last_publish_.reset();
                first_pair_.reset();
                ++clock_resets_;
                wait_reason_ = "odometry clock moved backwards; cleared pose/image history";
            }
        }

        struct Pending
        {
            Image::ConstSharedPtr image;
            Info::ConstSharedPtr info;
            Steady::time_point received;
        };

        void receive(const Image::ConstSharedPtr& image, const Info::ConstSharedPtr& info)
        {
            const int64_t t = stamp(image->header.stamp);
            const bool same_stamp = image->header.stamp.sec == info->header.stamp.sec &&
                image->header.stamp.nanosec == info->header.stamp.nanosec;
            if (!same_stamp || t <= 0 || image->header.stamp.nanosec >= 1000000000U ||
                image->header.frame_id.empty() || image->header.frame_id != info->header.frame_id ||
                image->encoding != sensor_msgs::image_encodings::TYPE_16UC1 ||
                image->width != info->width || image->height != info->height ||
                !validDepthLayout(image->width, image->height, image->step, image->data.size()) ||
                !(info->p[0] > 0.) || !(info->p[5] > 0.) || !std::isfinite(info->p[0]) ||
                !std::isfinite(info->p[5]) || !std::isfinite(info->p[2]) || !std::isfinite(info->p[6]))
            {
                ++invalid_image_;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "Rejected mismatched depth/CameraInfo geometry or stamp");
                return;
            }
            ++paired_;
            if (!first_pair_) { first_pair_ = Steady::now(); }
            if (t <= last_output_stamp_)
            {
                ++dropped_;
                return;
            }
            if (!pending_.empty() && t <= stamp(pending_.back().image->header.stamp))
            {
                ++dropped_;
                return;
            }
            if (pending_.size() >= max_pending_)
            {
                pending_.pop_front();
                ++dropped_;
            }
            pending_.push_back({image, info, Steady::now()});
        }

        bool bridge()
        {
            if (!need_bridge_) { return true; }
            try
            {
                auto ext = transform(buffer_->lookupTransform(infra_, root_, tf2::TimePointZero).transform);
                if (rotated_)
                {
                    // Premultiply by Rz(pi), rotating both the translation and quaternion.
                    ext = tf2::Transform(tf2::Quaternion(0., 0., 1., 0.), tf2::Vector3(0., 0., 0.)) * ext;
                }
                geometry_msgs::msg::TransformStamped message;
                message.header.stamp = now();
                message.header.frame_id = cam0_;
                message.child_frame_id = root_;
                message.transform = tf2::toMsg(ext);
                broadcaster_->sendTransform(message);
                need_bridge_ = false;
                return true;
            }
            catch (const std::exception& e)
            {
                wait_reason_ = std::string("waiting for D435 physical extrinsics: ") + e.what();
                return false;
            }
        }

        bool cameraPose(const Image& image, geometry_msgs::msg::Pose& out, bool& drop, bool& rounded)
        {
            try
            {
                tf2::Transform local_depth;
                if (mode_ == "tf")
                {
                    // Dynamic world pose is always queried at the image stamp, never Time(0).
                    local_depth = transform(buffer_->lookupTransform(local_, image.header.frame_id,
                                                                     rclcpp::Time(
                                                                         image.header.stamp,
                                                                         get_clock()->get_clock_type())).transform);
                }
                else
                {
                    // VINS defines t_imu = t_image + td. Keep output headers at the original
                    // image stamp so depth/info/pose still form one exact-time observation.
                    const auto sample = history_->sample(stamp(image.header.stamp) + image_pose_offset_ns_);
                    if (sample.state != Match::Ready)
                    {
                        drop = sample.state == Match::TooOld || sample.state == Match::Gap;
                        if (sample.state == Match::Empty) { wait_reason_ = "waiting for imu_propagate pose samples"; }
                        else if (sample.state == Match::Future)
                        {
                            wait_reason_ = "waiting for a bracketing IMU pose at image time";
                        }
                        else { wait_reason_ = "image outside pose history or pose sample gap exceeds limit"; }
                        return false;
                    }
                    rounded = sample.rounded;
                    const auto& p = sample.pose.position;
                    const auto& q = sample.pose.orientation;
                    const tf2::Transform local_body(tf2::Quaternion(q[0], q[1], q[2], q[3]),
                                                    tf2::Vector3(p[0], p[1], p[2]));
                    local_depth = local_body * *body_depth_;
                }
                out.position.x = local_depth.getOrigin().x();
                out.position.y = local_depth.getOrigin().y();
                out.position.z = local_depth.getOrigin().z();
                out.orientation = tf2::toMsg(local_depth.getRotation());
                return true;
            }
            catch (const std::exception& e)
            {
                wait_reason_ = std::string("waiting for TF: ") + e.what();
                return false;
            }
        }

        bool prepareExtrinsics(const std::string& optical_frame)
        {
            if (mode_ == "tf") { return true; }
            if (history_->size() == 0)
            {
                wait_reason_ = "waiting for imu_propagate pose samples";
                return false;
            }
            if (body_depth_ && extrinsic_frame_ == optical_frame) { return true; }
            try
            {
                // Only the rigid, explicitly fixed optical mounting is read at latest.
                // Freeze it once. During startup, retry once per timer, not once per image.
                body_depth_ = transform(buffer_->lookupTransform(body_, optical_frame, tf2::TimePointZero).transform);
                extrinsic_frame_ = optical_frame;
                RCLCPP_INFO(get_logger(), "Frozen calibrated extrinsics: %s -> %s", body_.c_str(),
                            extrinsic_frame_.c_str());
                return true;
            }
            catch (const std::exception& e)
            {
                wait_reason_ = std::string("waiting for fixed body/optical extrinsics: ") + e.what();
                return false;
            }
        }

        void flush()
        {
            const int64_t ros_now = now().nanoseconds();
            if (ros_now <= 0)
            {
                wait_reason_ = "waiting for ROS clock";
                return;
            }
            if (last_ros_now_ && ros_now < *last_ros_now_)
            {
                dropped_ += pending_.size();
                pending_.clear();
                history_->clear();
                last_output_stamp_ = 0;
                last_publish_.reset();
                first_pair_.reset();
                ++clock_resets_;
            }
            last_ros_now_ = ros_now;
            if (pending_.empty() || !bridge() || !prepareExtrinsics(pending_.back().image->header.frame_id)) { return; }
            // Select the newest usable image. Older ready frames are explicitly dropped
            // instead of publishing a burst of stale depth to a real-time planner.
            for (size_t i = pending_.size(); i-- > 0;)
            {
                const auto candidate = pending_[i];
                if (mode_ == "imu_propagate" && candidate.image->header.frame_id != extrinsic_frame_)
                {
                    pending_.erase(pending_.begin() + static_cast<std::ptrdiff_t>(i));
                    ++dropped_;
                    continue;
                }
                const int64_t t = stamp(candidate.image->header.stamp);
                if (ros_now - t > max_age_ns_ || t - ros_now > 100000000LL || elapsed(candidate.received) > max_wait_)
                {
                    pending_.erase(pending_.begin() + static_cast<std::ptrdiff_t>(i));
                    ++dropped_;
                    ++expired_;
                    wait_reason_ = "discarded stale/future image or expired pose wait";
                    continue;
                }
                geometry_msgs::msg::PoseStamped pose;
                bool drop = false, rounded = false;
                if (!cameraPose(*candidate.image, pose.pose, drop, rounded))
                {
                    if (drop)
                    {
                        pending_.erase(pending_.begin() + static_cast<std::ptrdiff_t>(i));
                        ++dropped_;
                    }
                    continue;
                }
                pose.header.stamp = candidate.image->header.stamp;
                pose.header.frame_id = local_;
                if (rotated_)
                {
                    Image image;
                    image.header = candidate.image->header;
                    image.height = candidate.image->height;
                    image.width = candidate.image->width;
                    image.encoding = candidate.image->encoding;
                    image.is_bigendian = candidate.image->is_bigendian;
                    image.step = image.width * 2;
                    image.data = rotateDepth180(candidate.image->data, image.width, image.height,
                                                candidate.image->step);
                    info_pub_->publish(*candidate.info);
                    pose_pub_->publish(pose);
                    depth_pub_->publish(std::move(image));
                }
                else
                {
                    info_pub_->publish(*candidate.info);
                    pose_pub_->publish(pose);
                    depth_pub_->publish(*candidate.image);
                }
                dropped_ += i;
                pending_.erase(pending_.begin(), pending_.begin() + static_cast<std::ptrdiff_t>(i + 1));
                last_output_stamp_ = t;
                last_publish_ = Steady::now();
                ++published_;
                if (rounded) { ++rounding_clamps_; }
                wait_reason_ = "ready";
                return;
            }
        }

        void diagnostics()
        {
            const double dt = elapsed(last_diagnostic_);
            const auto current = Steady::now();
            const double quiet = last_publish_ ? elapsed(*last_publish_) : (first_pair_ ? elapsed(*first_pair_) : 0.);
            const bool stalled = first_pair_ && quiet >= stall_warning_;
            if (stalled && wait_reason_ == "ready") { wait_reason_ = "waiting for a new depth/CameraInfo pair"; }
            if (stalled)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "Depth output stalled %.3f s, pending=%zu: %s", quiet, pending_.size(),
                                     wait_reason_.c_str());
            }
            const int64_t ros_now = now().nanoseconds();
            auto age = [ros_now](int64_t t)
            {
                return t > 0 ? std::to_string(static_cast<double>(ros_now - t) * 1e-9) : std::string("null");
            };
            std::ostringstream out;
            out << "{\"pose_source\":" << quoted(mode_) << ",\"state\":" << quoted(
                    stalled ? "stalled" : (last_publish_ ? "running" : "waiting"))
                << ",\"reason\":" << quoted(wait_reason_) << ",\"paired\":" << paired_
                << ",\"published\":" << published_ << ",\"dropped\":" << dropped_ << ",\"expired\":" << expired_
                << ",\"invalid_image\":" << invalid_image_ << ",\"invalid_pose\":" << invalid_pose_
                << ",\"rounding_clamps\":" << rounding_clamps_ << ",\"clock_resets\":" << clock_resets_
                << ",\"pending\":" << pending_.size() << ",\"pose_samples\":" << history_->size()
                << ",\"output_hz\":" << (dt > 0. ? static_cast<double>(published_ - last_published_count_) / dt : 0.)
                << ",\"last_output_age_s\":" << age(last_output_stamp_)
                << ",\"last_pose_age_s\":" << age(history_->latest())
                << ",\"seconds_without_output\":" << quiet << ",\"extrinsics_ready\":" << (
                    body_depth_ ? "true" : "false") << '}';
            std_msgs::msg::String message;
            message.data = out.str();
            status_pub_->publish(message);
            last_diagnostic_ = current;
            last_published_count_ = published_;
        }

        bool rotated_, need_bridge_;
        std::string local_, body_, root_, infra_, cam0_, mode_, extrinsic_frame_;
        std::string wait_reason_ = "waiting for paired depth and CameraInfo";
        size_t max_pending_;
        double max_wait_, stall_warning_;
        int64_t max_age_ns_, image_pose_offset_ns_ = 0, last_output_stamp_ = 0;
        std::optional<int64_t> last_ros_now_;
        std::optional<Steady::time_point> first_pair_, last_publish_;
        Steady::time_point last_diagnostic_;
        uint64_t paired_ = 0, published_ = 0, dropped_ = 0, expired_ = 0, invalid_image_ = 0;
        uint64_t invalid_pose_ = 0, rounding_clamps_ = 0, clock_resets_ = 0, last_published_count_ = 0;
        std::deque<Pending> pending_;
        std::unique_ptr<PoseHistory> history_;
        std::optional<tf2::Transform> body_depth_;
        std::unique_ptr<tf2_ros::Buffer> buffer_;
        std::unique_ptr<tf2_ros::TransformListener> listener_;
        std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
        message_filters::Subscriber<Image> depth_sub_;
        message_filters::Subscriber<Info> info_sub_;
        std::unique_ptr<message_filters::Synchronizer<Policy>> sync_;
        rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
        rclcpp::Publisher<Image>::SharedPtr depth_pub_;
        rclcpp::Publisher<Info>::SharedPtr info_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
        rclcpp::TimerBase::SharedPtr timer_, diagnostic_timer_;
    };
} // namespace ego_depth_inputs

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    int result = 0;
    try { rclcpp::spin(std::make_shared<ego_depth_inputs::DepthInputs>()); }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("ego_depth_inputs"), "%s", e.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
