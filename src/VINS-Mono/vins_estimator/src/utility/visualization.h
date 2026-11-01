#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "CameraPoseVisualization.h"
#include <eigen3/Eigen/Dense>
#include "../estimator.h"
#include "../parameters.h"
#include <fstream>

class Visualization
{
  public:
    explicit Visualization(rclcpp::Node *node);

    void registerPub(rclcpp::Node *node);

    void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q,
                           const Eigen::Vector3d &V, const std_msgs::msg::Header &header);

    void printStatistics(const Estimator &estimator, double t);

    void pubOdometry(const Estimator &estimator, const std_msgs::msg::Header &header);

    void pubKeyPoses(const Estimator &estimator, const std_msgs::msg::Header &header);

    void pubCameraPose(const Estimator &estimator, const std_msgs::msg::Header &header);

    void pubPointCloud(const Estimator &estimator, const std_msgs::msg::Header &header);

    void pubTF(const Estimator &estimator, const std_msgs::msg::Header &header);

    void pubKeyframe(const Estimator &estimator);

    void pubRelocalization(const Estimator &estimator);

  private:
    rclcpp::Node *node_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_, pub_latest_odometry_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_, pub_relo_path_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_point_cloud_, pub_margin_cloud_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_key_poses_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_relo_relative_pose_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_camera_pose_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_camera_pose_visual_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_keyframe_pose_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_keyframe_point_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_extrinsic_;

    nav_msgs::msg::Path path_, relo_path_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> br_;

    CameraPoseVisualization cameraposevisual_;
    CameraPoseVisualization keyframebasevisual_;

    double sum_of_path_;
    Eigen::Vector3d last_path_;
};
