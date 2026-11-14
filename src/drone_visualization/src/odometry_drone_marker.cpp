#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using Odom = nav_msgs::msg::Odometry;
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using Color = std_msgs::msg::ColorRGBA;
using SteadyClock = std::chrono::steady_clock;

int64_t stampNs(const builtin_interfaces::msg::Time &stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

geometry_msgs::msg::Point point(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

class DroneVisualization : public rclcpp::Node
{
public:
  DroneVisualization() : Node("drone_visualization")
  {
    const auto input = parameter<std::string>("odometry_topic", "/imu_propagate");
    const auto output = parameter<std::string>("marker_topic", "/drone/markers");
    if (input.empty() || output.empty()) { throw std::invalid_argument("topic names must not be empty"); }
    const double rate = positive("publish_rate_hz", 20.);
    timeout_ = positive("stale_timeout", .5);
    if (rate < 1. || rate > 60. || timeout_ > 60.)
    {
      throw std::invalid_argument("publish_rate_hz must be in [1, 60] and stale_timeout <= 60");
    }
    body_ = parameter<std::vector<double>>("body_size", {.12, .08, .04});
    if (body_.size() != 3) { throw std::invalid_argument("body_size must contain three dimensions"); }
    for (double v : body_)
    {
      if (!std::isfinite(v) || v <= 0.) { throw std::invalid_argument("body_size must be positive and finite"); }
    }
    span_ = positive("motor_span", .22);
    diameter_ = positive("propeller_diameter", .10);
    thickness_ = positive("propeller_thickness", .004);
    arm_width_ = positive("arm_width", .012);
    motor_height_ = positive("motor_height", .024);
    body_color_ = color("body_color", {.22, .28, .38, 1.});
    front_color_ = color("front_color", {1., .35, .10, 1.});
    rear_color_ = color("rear_color", {.10, .75, 1., 1.});

    publisher_ = create_publisher<MarkerArray>(output, rclcpp::QoS(1));
    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(1);
    subscription_ = create_subscription<Odom>(input, qos, [this](Odom::ConstSharedPtr msg) {
      const auto &p = msg->pose.pose.position;
      const auto &q = msg->pose.pose.orientation;
      const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
      if (msg->header.frame_id.empty() || stampNs(msg->header.stamp) <= 0 ||
          !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
          !std::isfinite(norm) || norm < 1e-6)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring invalid odometry pose/frame/timestamp");
        return;
      }
      // Duplicate cached results must not keep an old model visible indefinitely.
      if (latest_ && msg->header.frame_id == latest_->header.frame_id &&
          stampNs(msg->header.stamp) <= stampNs(latest_->header.stamp))
      {
        return;
      }
      latest_ = msg;
      received_ = SteadyClock::now();
    });
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1. / rate)),
      [this] { tick(); });
    RCLCPP_INFO(get_logger(), "Drone model: %s -> %s at up to %.1f Hz; orange +X nose, cyan rear",
                input.c_str(), output.c_str(), rate);
  }

private:
  template<class T>
  T parameter(const std::string &name, const T &value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }

  double positive(const std::string &name, double initial)
  {
    const double value = parameter<double>(name, initial);
    if (!std::isfinite(value) || value <= 0.) { throw std::invalid_argument(name + " must be positive and finite"); }
    return value;
  }

  Color color(const std::string &name, const std::vector<double> &initial)
  {
    const auto values = parameter<std::vector<double>>(name, initial);
    if (values.size() != 4) { throw std::invalid_argument(name + " must contain RGBA"); }
    for (double v : values)
    {
      if (!std::isfinite(v) || v < 0. || v > 1.) { throw std::invalid_argument(name + " values must be in [0, 1]"); }
    }
    Color c;
    c.r = static_cast<float>(values[0]);
    c.g = static_cast<float>(values[1]);
    c.b = static_cast<float>(values[2]);
    c.a = static_cast<float>(values[3]);
    return c;
  }

  Marker marker(const Odom &odom, int id, int type, const Color &c) const
  {
    Marker m;
    m.header = odom.header;
    m.ns = "drone";
    m.id = id;
    m.type = type;
    m.action = Marker::ADD;
    m.pose = odom.pose.pose;
    auto &q = m.pose.orientation;
    const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
    q.w /= norm;
    m.color = c;
    const int64_t lifetime = static_cast<int64_t>(timeout_ * 1e9);
    m.lifetime.sec = static_cast<int32_t>(lifetime / 1000000000LL);
    m.lifetime.nanosec = static_cast<uint32_t>(lifetime % 1000000000LL);
    // All geometry uses the odometry pose directly; no world->body TF is required.
    m.frame_locked = false;
    return m;
  }

  void tick()
  {
    if (!latest_) { return; }
    const double age = now().seconds() - static_cast<double>(stampNs(latest_->header.stamp)) * 1e-9;
    const double receive_age = std::chrono::duration<double>(SteadyClock::now() - received_).count();
    if (age > timeout_ || age < -.1 || receive_age > timeout_)
    {
      if (visible_)
      {
        MarkerArray removal;
        for (int id = 0; id < 5; ++id)
        {
          Marker m;
          m.header = latest_->header;
          m.header.stamp = now();
          m.ns = "drone";
          m.id = id;
          m.action = Marker::DELETE;
          removal.markers.push_back(m);
        }
        publisher_->publish(removal);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Odometry stale or clock mismatch; hiding drone model");
      }
      visible_ = false;
      drawn_stamp_ = 0;
      latest_.reset();
      return;
    }
    const int64_t stamp = stampNs(latest_->header.stamp);
    if (visible_ && stamp == drawn_stamp_ && latest_->header.frame_id == drawn_frame_) { return; }

    auto body = marker(*latest_, 0, Marker::CUBE, body_color_);
    body.scale.x = body_[0];
    body.scale.y = body_[1];
    body.scale.z = body_[2];
    auto arms = marker(*latest_, 1, Marker::LINE_LIST, body_color_);
    arms.scale.x = arm_width_;
    auto motors = marker(*latest_, 2, Marker::CUBE_LIST, body_color_);
    motors.scale.x = motors.scale.y = arm_width_ * 2.;
    motors.scale.z = motor_height_;
    auto rotors = marker(*latest_, 3, Marker::SPHERE_LIST, front_color_);
    rotors.scale.x = rotors.scale.y = diameter_;
    rotors.scale.z = thickness_;
    for (double x : {-span_ * .5, span_ * .5})
    {
      for (double y : {-span_ * .5, span_ * .5})
      {
        arms.points.push_back(point(0., 0., 0.));
        arms.points.push_back(point(x, y, 0.));
        motors.points.push_back(point(x, y, 0.));
        rotors.points.push_back(point(x, y, motor_height_ * .5 + thickness_ * .5));
        rotors.colors.push_back(x > 0. ? front_color_ : rear_color_);
      }
    }
    auto nose = marker(*latest_, 4, Marker::ARROW, front_color_);
    nose.points = {
      point(body_[0] * .4, 0., body_[2] * .5 + arm_width_),
      point(span_ * .8, 0., body_[2] * .5 + arm_width_)};
    nose.scale.x = arm_width_ * .6;
    nose.scale.y = nose.scale.z = arm_width_ * 1.5;

    MarkerArray model;
    model.markers = {body, arms, motors, rotors, nose};
    publisher_->publish(model);
    visible_ = true;
    drawn_stamp_ = stamp;
    drawn_frame_ = latest_->header.frame_id;
  }

  double timeout_ = .5, span_ = .22, diameter_ = .10, thickness_ = .004, arm_width_ = .012, motor_height_ = .024;
  std::vector<double> body_;
  Color body_color_, front_color_, rear_color_;
  Odom::ConstSharedPtr latest_;
  SteadyClock::time_point received_;
  bool visible_ = false;
  int64_t drawn_stamp_ = 0;
  std::string drawn_frame_;
  rclcpp::Publisher<MarkerArray>::SharedPtr publisher_;
  rclcpp::Subscription<Odom>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try { rclcpp::spin(std::make_shared<DroneVisualization>()); }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("drone_visualization"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
