#include <cstdio>
#include <vector>
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

static double stampToSec(const builtin_interfaces::msg::Time &t)
{
    return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

static builtin_interfaces::msg::Time stampFromDouble(double t)
{
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(t);
    stamp.nanosec = static_cast<uint32_t>((t - stamp.sec) * 1e9);
    return stamp;
}

struct Data
{
    Data(FILE *f)
    {
        if (fscanf(f, " %lf,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &t,
               &px, &py, &pz,
               &qw, &qx, &qy, &qz,
               &vx, &vy, &vz,
               &wx, &wy, &wz,
               &ax, &ay, &az) != EOF)
        {
            t /= 1e9;
        }
    }
    double t;
    float px, py, pz;
    float qw, qx, qy, qz;
    float vx, vy, vz;
    float wx, wy, wz;
    float ax, ay, az;
};

class BenchmarkPublisherNode : public rclcpp::Node
{
  public:
    BenchmarkPublisherNode() : Node("benchmark_publisher"), idx_(1), init_(0)
    {
        data_name_ = declare_parameter<std::string>("data_name", "");
        if (data_name_.empty())
        {
            RCLCPP_ERROR(get_logger(), "Failed to load data_name");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(get_logger(), "Loaded data_name: %s", data_name_.c_str());

        std::cout << "load ground truth " << data_name_ << std::endl;
        FILE *f = fopen(data_name_.c_str(), "r");
        if (f == NULL)
        {
            RCLCPP_WARN(get_logger(), "can't load ground truth; wrong path");
            rclcpp::shutdown();
            return;
        }
        char tmp[10000];
        if (fgets(tmp, 10000, f) == NULL)
            RCLCPP_WARN(get_logger(), "can't load ground truth; no data available");
        while (!feof(f))
            benchmark_.emplace_back(f);
        fclose(f);
        benchmark_.pop_back();
        RCLCPP_INFO(get_logger(), "Data loaded: %d", (int)benchmark_.size());

        pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("odometry", 1000);
        pub_path_ = create_publisher<nav_msgs::msg::Path>("path", 1000);
        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>("estimated_odometry", 1000,
            std::bind(&BenchmarkPublisherNode::odom_callback, this, std::placeholders::_1));

        path_.header.frame_id = "world";
    }

  private:
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
    {
        if (stampToSec(odom_msg->header.stamp) > benchmark_.back().t)
            return;

        for (; idx_ < static_cast<int>(benchmark_.size()) && benchmark_[idx_].t <= stampToSec(odom_msg->header.stamp); idx_++)
            ;

        if (init_++ < SKIP)
        {
            baseRgt_ = Quaterniond(odom_msg->pose.pose.orientation.w,
                                   odom_msg->pose.pose.orientation.x,
                                   odom_msg->pose.pose.orientation.y,
                                   odom_msg->pose.pose.orientation.z) *
                       Quaterniond(benchmark_[idx_ - 1].qw,
                                   benchmark_[idx_ - 1].qx,
                                   benchmark_[idx_ - 1].qy,
                                   benchmark_[idx_ - 1].qz).inverse();
            baseTgt_ = Vector3d{odom_msg->pose.pose.position.x,
                                odom_msg->pose.pose.position.y,
                                odom_msg->pose.pose.position.z} -
                       baseRgt_ * Vector3d{benchmark_[idx_ - 1].px, benchmark_[idx_ - 1].py, benchmark_[idx_ - 1].pz};
            return;
        }

        nav_msgs::msg::Odometry odometry;
        odometry.header.stamp = stampFromDouble(benchmark_[idx_ - 1].t);
        odometry.header.frame_id = "world";
        odometry.child_frame_id = "world";

        Vector3d tmp_T = baseTgt_ + baseRgt_ * Vector3d{benchmark_[idx_ - 1].px, benchmark_[idx_ - 1].py, benchmark_[idx_ - 1].pz};
        odometry.pose.pose.position.x = tmp_T.x();
        odometry.pose.pose.position.y = tmp_T.y();
        odometry.pose.pose.position.z = tmp_T.z();

        Quaterniond tmp_R = baseRgt_ * Quaterniond{benchmark_[idx_ - 1].qw,
                                                   benchmark_[idx_ - 1].qx,
                                                   benchmark_[idx_ - 1].qy,
                                                   benchmark_[idx_ - 1].qz};
        odometry.pose.pose.orientation.w = tmp_R.w();
        odometry.pose.pose.orientation.x = tmp_R.x();
        odometry.pose.pose.orientation.y = tmp_R.y();
        odometry.pose.pose.orientation.z = tmp_R.z();

        Vector3d tmp_V = baseRgt_ * Vector3d{benchmark_[idx_ - 1].vx,
                                             benchmark_[idx_ - 1].vy,
                                             benchmark_[idx_ - 1].vz};
        odometry.twist.twist.linear.x = tmp_V.x();
        odometry.twist.twist.linear.y = tmp_V.y();
        odometry.twist.twist.linear.z = tmp_V.z();
        pub_odom_->publish(odometry);

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = odometry.header;
        pose_stamped.pose = odometry.pose.pose;
        path_.header = odometry.header;
        path_.poses.push_back(pose_stamped);
        pub_path_->publish(path_);
    }

    static constexpr int SKIP = 50;

    std::string data_name_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    nav_msgs::msg::Path path_;
    std::vector<Data> benchmark_;
    int idx_;
    int init_;
    Quaterniond baseRgt_;
    Vector3d baseTgt_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BenchmarkPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
