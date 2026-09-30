#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace driver_interface
{

class DriverInterfaceNode : public rclcpp::Node
{
public:
  DriverInterfaceNode()
  : Node("driver_interface")
  {
    camera_input_topic_ = declare_parameter<std::string>(
      "camera_input_topic", "/camera/image_raw");
    gray_output_topic_ = declare_parameter<std::string>(
      "gray_output_topic", "/camera/image_gray");
    px4_imu_input_topic_ = declare_parameter<std::string>(
      "px4_imu_input_topic", "/fmu/out/sensor_combined");
    imu_output_topic_ = declare_parameter<std::string>(
      "imu_output_topic", "/px4/imu");

    image_frame_id_ = declare_parameter<std::string>("image_frame_id", "");
    imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "imu_link_flu");
    convert_frd_to_flu_ = declare_parameter<bool>("convert_frd_to_flu", true);

    timestamp_mode_ = declare_parameter<std::string>("timestamp_mode", "auto");
    timestamp_offset_sec_ = declare_parameter<double>("timestamp_offset_sec", 0.0);
    auto_direct_threshold_sec_ = declare_parameter<double>(
      "auto_direct_threshold_sec", 10.0);
    drop_non_monotonic_imu_ = declare_parameter<bool>("drop_non_monotonic_imu", true);

    angular_velocity_variance_ = declare_parameter<double>(
      "angular_velocity_variance", 0.0);
    linear_acceleration_variance_ = declare_parameter<double>(
      "linear_acceleration_variance", 0.0);

    if (
      timestamp_mode_ != "auto" && timestamp_mode_ != "first_arrival" &&
      timestamp_mode_ != "px4" && timestamp_mode_ != "ros_now")
    {
      throw std::invalid_argument(
              "timestamp_mode must be one of: auto, first_arrival, px4, ros_now");
    }
    if (auto_direct_threshold_sec_ < 0.0) {
      throw std::invalid_argument("auto_direct_threshold_sec must be non-negative");
    }
    if (angular_velocity_variance_ < 0.0 || linear_acceleration_variance_ < 0.0) {
      throw std::invalid_argument("IMU covariance parameters must be non-negative");
    }

    timestamp_offset_ns_ = static_cast<int64_t>(
      std::llround(timestamp_offset_sec_ * 1.0e9));

    const rclcpp::SensorDataQoS sensor_qos;

    gray_publisher_ = create_publisher<sensor_msgs::msg::Image>(
      gray_output_topic_, sensor_qos);
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      imu_output_topic_, sensor_qos);

    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      camera_input_topic_, sensor_qos,
      std::bind(&DriverInterfaceNode::image_callback, this, std::placeholders::_1));

    px4_imu_subscription_ = create_subscription<px4_msgs::msg::SensorCombined>(
      px4_imu_input_topic_, sensor_qos,
      std::bind(&DriverInterfaceNode::imu_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "RGB image: %s -> mono8: %s",
      camera_input_topic_.c_str(), gray_output_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "PX4 SensorCombined: %s -> sensor_msgs/Imu: %s",
      px4_imu_input_topic_.c_str(), imu_output_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "IMU axes: %s; timestamp mode: %s; manual offset: %.9f s",
      convert_frd_to_flu_ ? "PX4 FRD -> ROS FLU" : "unchanged",
      timestamp_mode_.c_str(), timestamp_offset_sec_);

    if (timestamp_mode_ == "ros_now") {
      RCLCPP_WARN(
        get_logger(),
        "timestamp_mode=ros_now is for connectivity tests only; reception-time jitter degrades VINS");
    }
  }

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg)
  {
    try {
      cv::Mat gray;

      if (
        image_msg->encoding == sensor_msgs::image_encodings::MONO8 ||
        image_msg->encoding == sensor_msgs::image_encodings::TYPE_8UC1)
      {
        const auto cv_ptr = cv_bridge::toCvShare(image_msg);
        gray = cv_ptr->image;
      } else if (image_msg->encoding == sensor_msgs::image_encodings::RGB8) {
        const auto cv_ptr = cv_bridge::toCvShare(image_msg);
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_RGB2GRAY);
      } else if (image_msg->encoding == sensor_msgs::image_encodings::BGR8) {
        const auto cv_ptr = cv_bridge::toCvShare(image_msg);
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
      } else if (image_msg->encoding == sensor_msgs::image_encodings::RGBA8) {
        const auto cv_ptr = cv_bridge::toCvShare(image_msg);
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_RGBA2GRAY);
      } else if (image_msg->encoding == sensor_msgs::image_encodings::BGRA8) {
        const auto cv_ptr = cv_bridge::toCvShare(image_msg);
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGRA2GRAY);
      } else {
        // Let cv_bridge handle other convertible encodings (for example 8UC3).
        const auto cv_ptr = cv_bridge::toCvShare(
          image_msg, sensor_msgs::image_encodings::BGR8);
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
      }

      auto output_header = image_msg->header;
      if (!image_frame_id_.empty()) {
        output_header.frame_id = image_frame_id_;
      }

      auto gray_msg = cv_bridge::CvImage(
        output_header, sensor_msgs::image_encodings::MONO8, gray).toImageMsg();
      gray_publisher_->publish(*gray_msg);
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to convert image encoding '%s' to mono8: %s",
        image_msg->encoding.c_str(), error.what());
    } catch (const cv::Exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "OpenCV grayscale conversion failed: %s", error.what());
    }
  }

  builtin_interfaces::msg::Time make_imu_stamp(uint64_t timestamp_us)
  {
    const int64_t raw_px4_ns = static_cast<int64_t>(timestamp_us) * 1000LL;
    int64_t stamp_ns = 0;

    if (timestamp_mode_ == "ros_now") {
      stamp_ns = now().nanoseconds() + timestamp_offset_ns_;
    } else if (timestamp_mode_ == "px4") {
      stamp_ns = raw_px4_ns + timestamp_offset_ns_;
    } else {
      if (!first_arrival_offset_initialized_) {
        const int64_t now_ns = now().nanoseconds();
        const int64_t direct_difference_ns = std::llabs(now_ns - raw_px4_ns);
        const int64_t direct_threshold_ns = static_cast<int64_t>(
          std::llround(auto_direct_threshold_sec_ * 1.0e9));

        const bool use_direct_px4_time =
          timestamp_mode_ == "auto" && direct_difference_ns <= direct_threshold_ns;
        first_arrival_offset_ns_ = use_direct_px4_time ? 0 : now_ns - raw_px4_ns;
        first_arrival_offset_initialized_ = true;

        if (use_direct_px4_time) {
          RCLCPP_INFO(
            get_logger(),
            "timestamp_mode=auto selected direct PX4 time (PX4/ROS difference %.6f s)",
            static_cast<double>(direct_difference_ns) * 1.0e-9);
        } else {
          RCLCPP_INFO(
            get_logger(),
            "Captured first-arrival PX4-to-ROS clock offset: %.9f s",
            static_cast<double>(first_arrival_offset_ns_) * 1.0e-9);
        }
      }
      stamp_ns = raw_px4_ns + first_arrival_offset_ns_ + timestamp_offset_ns_;
    }

    builtin_interfaces::msg::Time stamp_msg;
    stamp_msg.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
    stamp_msg.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
    return stamp_msg;
  }

  static void set_diagonal_covariance(
    std::array<double, 9> & covariance, double variance)
  {
    covariance.fill(0.0);
    covariance[0] = variance;
    covariance[4] = variance;
    covariance[8] = variance;
  }

  void imu_callback(const px4_msgs::msg::SensorCombined::ConstSharedPtr px4_msg)
  {
    // PX4 1.14 SensorCombined.timestamp is the gyro sample timestamp. The
    // accelerometer timestamp is expressed relative to it in the message.
    const uint64_t timestamp_us = px4_msg->timestamp;

    if (timestamp_us == 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping SensorCombined message with zero timestamp");
      return;
    }

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = make_imu_stamp(timestamp_us);
    imu_msg.header.frame_id = imu_frame_id_;

    const int64_t stamp_ns =
      static_cast<int64_t>(imu_msg.header.stamp.sec) * 1000000000LL +
      static_cast<int64_t>(imu_msg.header.stamp.nanosec);

    if (last_imu_stamp_ns_ >= 0 && stamp_ns <= last_imu_stamp_ns_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Non-monotonic IMU stamp: current=%ld ns, previous=%ld ns",
        static_cast<long>(stamp_ns), static_cast<long>(last_imu_stamp_ns_));
      if (drop_non_monotonic_imu_) {
        return;
      }
    }
    last_imu_stamp_ns_ = std::max(last_imu_stamp_ns_, stamp_ns);

    if (convert_frd_to_flu_) {
      // PX4 body frame: Forward-Right-Down (FRD)
      // ROS body frame: Forward-Left-Up (FLU)
      imu_msg.angular_velocity.x = px4_msg->gyro_rad[0];
      imu_msg.angular_velocity.y = -px4_msg->gyro_rad[1];
      imu_msg.angular_velocity.z = -px4_msg->gyro_rad[2];

      imu_msg.linear_acceleration.x = px4_msg->accelerometer_m_s2[0];
      imu_msg.linear_acceleration.y = -px4_msg->accelerometer_m_s2[1];
      imu_msg.linear_acceleration.z = -px4_msg->accelerometer_m_s2[2];
    } else {
      imu_msg.angular_velocity.x = px4_msg->gyro_rad[0];
      imu_msg.angular_velocity.y = px4_msg->gyro_rad[1];
      imu_msg.angular_velocity.z = px4_msg->gyro_rad[2];

      imu_msg.linear_acceleration.x = px4_msg->accelerometer_m_s2[0];
      imu_msg.linear_acceleration.y = px4_msg->accelerometer_m_s2[1];
      imu_msg.linear_acceleration.z = px4_msg->accelerometer_m_s2[2];
    }

    // SensorCombined has no attitude estimate. -1 means orientation unavailable.
    imu_msg.orientation_covariance.fill(0.0);
    imu_msg.orientation_covariance[0] = -1.0;
    set_diagonal_covariance(
      imu_msg.angular_velocity_covariance, angular_velocity_variance_);
    set_diagonal_covariance(
      imu_msg.linear_acceleration_covariance, linear_acceleration_variance_);

    imu_publisher_->publish(imu_msg);
  }

  std::string camera_input_topic_;
  std::string gray_output_topic_;
  std::string px4_imu_input_topic_;
  std::string imu_output_topic_;
  std::string image_frame_id_;
  std::string imu_frame_id_;
  std::string timestamp_mode_;

  bool convert_frd_to_flu_{true};
  bool drop_non_monotonic_imu_{true};
  double timestamp_offset_sec_{0.0};
  double auto_direct_threshold_sec_{10.0};
  double angular_velocity_variance_{0.0};
  double linear_acceleration_variance_{0.0};

  int64_t timestamp_offset_ns_{0};
  int64_t first_arrival_offset_ns_{0};
  int64_t last_imu_stamp_ns_{-1};
  bool first_arrival_offset_initialized_{false};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr px4_imu_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr gray_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
};

}  // namespace driver_interface

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<driver_interface::DriverInterfaceNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("driver_interface"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
