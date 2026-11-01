#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Dense>
#include "data_generator.h"

using namespace std;
using namespace Eigen;

#define ROW 480
#define COL 752

static builtin_interfaces::msg::Time stampFromDouble(double t)
{
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(t);
    stamp.nanosec = static_cast<uint32_t>((t - stamp.sec) * 1e9);
    return stamp;
}

static void initCloud(sensor_msgs::msg::PointCloud2 &cloud, const std::string &frame_id,
                      size_t num_points, size_t num_fields)
{
    cloud.header.frame_id = frame_id;
    cloud.height = 1;
    cloud.width = num_points;
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_fields * sizeof(float);
    cloud.row_step = cloud.point_step * num_points;
    cloud.fields.resize(num_fields);
    cloud.data.resize(cloud.row_step);
}

static void addField(sensor_msgs::msg::PointCloud2 &cloud, size_t idx, const std::string &name)
{
    cloud.fields[idx].name = name;
    cloud.fields[idx].offset = idx * sizeof(float);
    cloud.fields[idx].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[idx].count = 1;
}

class DataGeneratorNode : public rclcpp::Node
{
  public:
    DataGeneratorNode() : Node("data_generator")
    {
        pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("~/imu", rclcpp::SensorDataQoS());
        pub_feature_ = create_publisher<sensor_msgs::msg::PointCloud2>("/feature_tracker/feature", 1000);
        pub_wifi_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/wifi", 1000);
        pub_flow_ = create_publisher<nav_msgs::msg::Odometry>("~/flow", 1000);
        pub_path_ = create_publisher<nav_msgs::msg::Path>("~/path", 1000);
        pub_odometry_ = create_publisher<nav_msgs::msg::Odometry>("~/odometry", 1000);
        pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 1000);
        pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/cloud", 1000);
        pub_ap_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/ap", 1000);
        pub_line_ = create_publisher<visualization_msgs::msg::Marker>("~/sar", 1000);
        pub_image_ = create_publisher<sensor_msgs::msg::Image>("~/image", rclcpp::SensorDataQoS());
    }

    void run()
    {
        DataGenerator generator;

        sensor_msgs::msg::PointCloud2 point_cloud;
        initCloud(point_cloud, "world", generator.getCloud().size(), 3);
        point_cloud.header.stamp = this->now();
        addField(point_cloud, 0, "x");
        addField(point_cloud, 1, "y");
        addField(point_cloud, 2, "z");
        int ci = 0;
        for (auto &it : generator.getCloud())
        {
            float *p = reinterpret_cast<float *>(point_cloud.data.data() + ci * point_cloud.point_step);
            p[0] = it(0);
            p[1] = it(1);
            p[2] = it(2);
            ci++;
        }
        pub_cloud_->publish(point_cloud);

        point_cloud.data.clear();
        point_cloud.fields.clear();
        point_cloud.width = 0;

        visualization_msgs::msg::Marker line_ap[DataGenerator::NUMBER_OF_AP];
        initCloud(point_cloud, "world", DataGenerator::NUMBER_OF_AP, 3);
        addField(point_cloud, 0, "x");
        addField(point_cloud, 1, "y");
        addField(point_cloud, 2, "z");
        for (int i = 0; i < DataGenerator::NUMBER_OF_AP; i++)
        {
            Vector3d p_ap = generator.getAP(i);
            float *p = reinterpret_cast<float *>(point_cloud.data.data() + i * point_cloud.point_step);
            p[0] = p_ap(0);
            p[1] = p_ap(1);
            p[2] = p_ap(2);

            line_ap[i].id = i;
            line_ap[i].header.frame_id = "world";
            line_ap[i].ns = "line";
            line_ap[i].action = visualization_msgs::msg::Marker::ADD;
            line_ap[i].pose.orientation.w = 1.0;
            line_ap[i].type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_ap[i].scale.x = 0.1;
            line_ap[i].color.r = 1.0;
            line_ap[i].color.a = 1.0;

            geometry_msgs::msg::Point p2;
            p2.x = p_ap(0);
            p2.y = p_ap(1);
            p2.z = p_ap(2);
            line_ap[i].points.push_back(p2);
            line_ap[i].points.push_back(p2);
        }
        pub_ap_->publish(point_cloud);

        int publish_count = 0;

        nav_msgs::msg::Path path;
        path.header.frame_id = "world";

        rclcpp::Rate loop_rate(DataGenerator::FREQ);
        while (rclcpp::ok())
        {
            double current_time = generator.getTime();
            RCLCPP_INFO(get_logger(), "time: %lf", current_time);

            Vector3d position = generator.getPosition();
            Vector3d velocity = generator.getVelocity();
            Matrix3d rotation = generator.getRotation();
            Quaterniond q(rotation);

            Vector3d linear_acceleration = generator.getLinearAcceleration();
            Vector3d angular_velocity = generator.getAngularVelocity();

            nav_msgs::msg::Odometry odometry;
            odometry.header.frame_id = "world";
            odometry.header.stamp = stampFromDouble(current_time);
            odometry.pose.pose.position.x = position(0);
            odometry.pose.pose.position.y = position(1);
            odometry.pose.pose.position.z = position(2);
            odometry.pose.pose.orientation.x = q.x();
            odometry.pose.pose.orientation.y = q.y();
            odometry.pose.pose.orientation.z = q.z();
            odometry.pose.pose.orientation.w = q.w();
            odometry.twist.twist.linear.x = velocity(0);
            odometry.twist.twist.linear.y = velocity(1);
            odometry.twist.twist.linear.z = velocity(2);
            pub_odometry_->publish(odometry);

            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = "world";
            pose_stamped.header.stamp = stampFromDouble(current_time);
            pose_stamped.pose = odometry.pose.pose;
            path.poses.push_back(pose_stamped);
            pub_path_->publish(path);
            pub_pose_->publish(pose_stamped);

            for (int i = 0; i < DataGenerator::NUMBER_OF_AP; i++)
            {
                line_ap[i].header.stamp = stampFromDouble(current_time);
                line_ap[i].points.back().x = position(0);
                line_ap[i].points.back().y = position(1);
                line_ap[i].points.back().z = position(2);
                pub_line_->publish(line_ap[i]);
            }

            sensor_msgs::msg::Imu imu;
            imu.header.frame_id = "world";
            imu.header.stamp = stampFromDouble(current_time);
            imu.linear_acceleration.x = linear_acceleration(0);
            imu.linear_acceleration.y = linear_acceleration(1);
            imu.linear_acceleration.z = linear_acceleration(2);
            imu.angular_velocity.x = angular_velocity(0);
            imu.angular_velocity.y = angular_velocity(1);
            imu.angular_velocity.z = angular_velocity(2);
            imu.orientation.x = q.x();
            imu.orientation.y = q.y();
            imu.orientation.z = q.z();
            imu.orientation.w = q.w();
            pub_imu_->publish(imu);

            if (publish_count % DataGenerator::IMU_PER_WIFI == 0)
            {
                sensor_msgs::msg::PointCloud2 wifi;
                initCloud(wifi, "world", DataGenerator::NUMBER_OF_AP, 4);
                wifi.header.stamp = stampFromDouble(current_time);
                addField(wifi, 0, "x");
                addField(wifi, 1, "y");
                addField(wifi, 2, "z");
                addField(wifi, 3, "id");

                for (int i = 0; i < DataGenerator::NUMBER_OF_AP; i++)
                {
                    Vector3d sar;
                    sar(0) = line_ap[i].points[0].x - line_ap[i].points[1].x;
                    sar(1) = line_ap[i].points[0].y - line_ap[i].points[1].y;
                    sar(2) = line_ap[i].points[0].z - line_ap[i].points[1].z;
                    sar.normalize();
                    float *p = reinterpret_cast<float *>(wifi.data.data() + i * wifi.point_step);
                    p[0] = sar.dot(rotation.col(0));
                    p[1] = 0.0;
                    p[2] = 0.0;
                    p[3] = i;
                }
                pub_wifi_->publish(wifi);
            }

            if (publish_count % DataGenerator::IMU_PER_IMG == 0)
            {
                vector<pair<int, Vector3d>> image = generator.getImage();
                RCLCPP_INFO(get_logger(), "feature count: %lu", image.size());

                sensor_msgs::msg::PointCloud2 feature;
                initCloud(feature, "world", image.size(), 8);
                feature.header.stamp = stampFromDouble(current_time);
                addField(feature, 0, "x");
                addField(feature, 1, "y");
                addField(feature, 2, "z");
                addField(feature, 3, "id");
                addField(feature, 4, "u");
                addField(feature, 5, "v");
                addField(feature, 6, "velocity_x");
                addField(feature, 7, "velocity_y");

                cv::Mat simu_img[DataGenerator::NUMBER_OF_CAMERA];
                for (int i = 0; i < DataGenerator::NUMBER_OF_CAMERA; i++)
                    simu_img[i] = cv::Mat(600, 600, CV_8UC3, cv::Scalar(0, 0, 0));

                int tmp_idx = 0;
                for (auto &id_pts : image)
                {
                    int id = id_pts.first;
                    float *p = reinterpret_cast<float *>(feature.data.data() + tmp_idx * feature.point_step);
                    p[0] = id_pts.second(0);
                    p[1] = id_pts.second(1);
                    p[2] = id_pts.second(2);
                    p[3] = id;
                    p[4] = static_cast<float>((p[0] + 1.0) * 300.0);
                    p[5] = static_cast<float>((p[1] + 1.0) * 300.0);
                    p[6] = 0.0f;
                    p[7] = 0.0f;
                    tmp_idx++;

                    char label[10];
                    sprintf(label, "%d", id / DataGenerator::NUMBER_OF_CAMERA);
                    cv::putText(simu_img[id % DataGenerator::NUMBER_OF_CAMERA], label,
                                cv::Point2d(p[0] + 1, p[1] + 1) * 0.5 * 600,
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
                }

                for (int i = 0; i < 6; i++)
                {
                    if (generator.output_Axis[i].empty())
                        continue;
                    cv::Point2d origin((generator.output_Axis[i][0].x() + 1) * 300, (generator.output_Axis[i][0].y() + 1) * 300);
                    cv::Point2d axis_x((generator.output_Axis[i][1].x() + 1) * 300, (generator.output_Axis[i][1].y() + 1) * 300);
                    cv::Point2d axis_y((generator.output_Axis[i][2].x() + 1) * 300, (generator.output_Axis[i][2].y() + 1) * 300);
                    cv::Point2d axis_z((generator.output_Axis[i][3].x() + 1) * 300, (generator.output_Axis[i][3].y() + 1) * 300);
                }

                pub_feature_->publish(feature);
                RCLCPP_INFO(get_logger(), "publish image data with stamp %lf", current_time);

                for (int k = 0; k < DataGenerator::NUMBER_OF_CAMERA; k++)
                {
                    char name[] = "camera 1";
                    name[7] += k;
                    cv::imshow(name, simu_img[k]);
                    cv::Mat gray_image;
                    cv::cvtColor(simu_img[k], gray_image, CV_BGR2GRAY);
                    sensor_msgs::msg::Image::SharedPtr img_msg =
                        cv_bridge::CvImage(feature.header, "mono8", gray_image).toImageMsg();
                    pub_image_->publish(*img_msg);
                }
                cv::waitKey(1);
                if (generator.getTime() > 3 * DataGenerator::MAX_TIME)
                    break;
            }

            generator.update();
            publish_count++;
            loop_rate.sleep();
        }
    }

  private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_feature_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_wifi_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_flow_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ap_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_line_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataGeneratorNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
