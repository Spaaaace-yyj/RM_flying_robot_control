#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <chrono>
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

using namespace std;
using namespace Eigen;
using namespace camodocal;

static bool stampLess(const builtin_interfaces::msg::Time &a, const builtin_interfaces::msg::Time &b)
{
    if (a.sec != b.sec)
        return a.sec < b.sec;
    return a.nanosec < b.nanosec;
}

class ARDemoNode : public rclcpp::Node
{
  public:
    ARDemoNode() : Node("ar_demo"), pose_init_(false), img_cnt_(0), look_ground_(false)
    {
        USE_UNDISTORED_IMG_ = declare_parameter<bool>("use_undistored_img", false);
        std::string calib_file = declare_parameter<std::string>("calib_file", "");

        object_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("AR_object", 10);
        pub_ARimage_ = create_publisher<sensor_msgs::msg::Image>("AR_image", 1000);

        if (USE_UNDISTORED_IMG_)
        {
            ROW_ = 600;
            COL_ = 480;
            FOCAL_LENGTH_ = 320.0;
            sub_img_ = create_subscription<sensor_msgs::msg::Image>("image_undistored", rclcpp::SensorDataQoS(),
                std::bind(&ARDemoNode::img_callback, this, std::placeholders::_1));
        }
        else
        {
            ROW_ = 752;
            COL_ = 480;
            FOCAL_LENGTH_ = 460.0;
            sub_img_ = create_subscription<sensor_msgs::msg::Image>("image_raw", rclcpp::SensorDataQoS(),
                std::bind(&ARDemoNode::img_callback, this, std::placeholders::_1));
        }

        Axis_[0] = Vector3d(0, 1.5, -1.2);
        Axis_[1] = Vector3d(-10, 5, 0);
        Axis_[2] = Vector3d(3, 3, 3);
        Axis_[3] = Vector3d(-2, 2, 0);
        Axis_[4] = Vector3d(5, 10, -5);
        Axis_[5] = Vector3d(0, 10, -1);

        Cube_center_[0] = Vector3d(0, 1.5, -1.2 + box_length / 2.0);
        Cube_center_[1] = Vector3d(4, -2, -1.2 + box_length / 2.0);
        Cube_center_[2] = Vector3d(0, -2, -1.2 + box_length / 2.0);

        sub_pose_ = create_subscription<nav_msgs::msg::Odometry>("camera_pose", 100,
            std::bind(&ARDemoNode::pose_callback, this, std::placeholders::_1));
        sub_point_ = create_subscription<sensor_msgs::msg::PointCloud2>("pointcloud", 2000,
            std::bind(&ARDemoNode::point_callback, this, std::placeholders::_1));

        line_color_r_.r = 1.0;
        line_color_r_.a = 1.0;
        line_color_g_.g = 1.0;
        line_color_g_.a = 1.0;
        line_color_b_.b = 1.0;
        line_color_b_.a = 1.0;

        RCLCPP_INFO(get_logger(), "reading paramerter of camera %s", calib_file.c_str());
        m_camera_ = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);

        rclcpp::sleep_for(std::chrono::seconds(1));
        add_object();
        add_object();
    }

    void axis_generate(visualization_msgs::msg::Marker &line_list, const Vector3d &origin, int id)
    {
        line_list.id = id;
        line_list.header.frame_id = "world";
        line_list.header.stamp = this->now();
        line_list.action = visualization_msgs::msg::Marker::ADD;
        line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
        line_list.scale.x = 0.1;
        line_list.color.a = 1.0;
        line_list.lifetime = builtin_interfaces::msg::Duration();

        line_list.pose.orientation.w = 1.0;
        line_list.color.b = 1.0;
        geometry_msgs::msg::Point p;
        p.x = origin.x();
        p.y = origin.y();
        p.z = origin.z();
        line_list.points.push_back(p);
        line_list.colors.push_back(line_color_r_);
        p.x += 1.0;
        line_list.points.push_back(p);
        line_list.colors.push_back(line_color_r_);
        p.x -= 1.0;
        line_list.points.push_back(p);
        line_list.colors.push_back(line_color_g_);
        p.y += 1.0;
        line_list.points.push_back(p);
        line_list.colors.push_back(line_color_g_);
        p.y -= 1.0;
        line_list.points.push_back(p);
        line_list.colors.push_back(line_color_b_);
        p.z += 1.0;
        line_list.points.push_back(p);
        line_list.colors.push_back(line_color_b_);
    }

    void cube_generate(visualization_msgs::msg::Marker &marker, const Vector3d &origin, int id)
    {
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "basic_shapes";
        marker.id = 0;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.scale.x = box_length;
        marker.scale.y = box_length;
        marker.scale.z = box_length;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = builtin_interfaces::msg::Duration();
        geometry_msgs::msg::Point p;
        p.x = origin.x();
        p.y = origin.y();
        p.z = origin.z();
        marker.points.push_back(p);
        marker.colors.push_back(line_color_r_);
        Cube_corner_[id].clear();
        Cube_corner_[id].push_back(Vector3d(origin.x() - box_length / 2, origin.y() - box_length / 2, origin.z() - box_length / 2));
        Cube_corner_[id].push_back(Vector3d(origin.x() + box_length / 2, origin.y() - box_length / 2, origin.z() - box_length / 2));
        Cube_corner_[id].push_back(Vector3d(origin.x() - box_length / 2, origin.y() + box_length / 2, origin.z() - box_length / 2));
        Cube_corner_[id].push_back(Vector3d(origin.x() + box_length / 2, origin.y() + box_length / 2, origin.z() - box_length / 2));
        Cube_corner_[id].push_back(Vector3d(origin.x() - box_length / 2, origin.y() - box_length / 2, origin.z() + box_length / 2));
        Cube_corner_[id].push_back(Vector3d(origin.x() + box_length / 2, origin.y() - box_length / 2, origin.z() + box_length / 2));
        Cube_corner_[id].push_back(Vector3d(origin.x() - box_length / 2, origin.y() + box_length / 2, origin.z() + box_length / 2));
        Cube_corner_[id].push_back(Vector3d(origin.x() + box_length / 2, origin.y() + box_length / 2, origin.z() + box_length / 2));
    }

    void add_object()
    {
        visualization_msgs::msg::MarkerArray markerArray_msg;

        visualization_msgs::msg::Marker line_list;
        visualization_msgs::msg::Marker cube_list;

        for (int i = 0; i < axis_num; i++)
        {
            axis_generate(line_list, Axis_[i], i);
            markerArray_msg.markers.push_back(line_list);
        }

        for (int i = 0; i < cube_num; i++)
        {
            cube_generate(cube_list, Cube_center_[i], i);
        }
        markerArray_msg.markers.push_back(cube_list);

        object_pub_->publish(markerArray_msg);
    }

    void project_object(const Vector3d &camera_p, const Quaterniond &camera_q)
    {
        for (int i = 0; i < axis_num; i++)
        {
            output_Axis_[i].clear();
            Vector3d local_point;
            Vector2d local_uv;
            local_point = camera_q.inverse() * (Axis_[i] - camera_p);
            m_camera_->spaceToPlane(local_point, local_uv);

            if (local_point.z() > 0)
            {
                output_Axis_[i].push_back(Vector3d(local_uv.x(), local_uv.y(), 1));

                local_point = camera_q.inverse() * (Axis_[i] + Vector3d(1, 0, 0) - camera_p);
                m_camera_->spaceToPlane(local_point, local_uv);
                output_Axis_[i].push_back(Vector3d(local_uv.x(), local_uv.y(), 1));

                local_point = camera_q.inverse() * (Axis_[i] + Vector3d(0, 1, 0) - camera_p);
                m_camera_->spaceToPlane(local_point, local_uv);
                output_Axis_[i].push_back(Vector3d(local_uv.x(), local_uv.y(), 1));

                local_point = camera_q.inverse() * (Axis_[i] + Vector3d(0, 0, 1) - camera_p);
                m_camera_->spaceToPlane(local_point, local_uv);
                output_Axis_[i].push_back(Vector3d(local_uv.x(), local_uv.y(), 1));
            }
        }

        for (int i = 0; i < cube_num; i++)
        {
            output_Cube_[i].clear();
            output_corner_dis_[i].clear();
            Vector3d local_point;
            Vector2d local_uv;
            local_point = camera_q.inverse() * (Cube_center_[i] - camera_p);
            if (USE_UNDISTORED_IMG_)
            {
                local_uv.x() = local_point(0) / local_point(2) * FOCAL_LENGTH_ + COL_ / 2;
                local_uv.y() = local_point(1) / local_point(2) * FOCAL_LENGTH_ + ROW_ / 2;
            }
            else
                m_camera_->spaceToPlane(local_point, local_uv);
            if (local_point.z() > box_length / 2)
            {
                Cube_center_depth_[i] = local_point.z();
                for (int j = 0; j < 8; j++)
                {
                    local_point = camera_q.inverse() * (Cube_corner_[i][j] - camera_p);
                    output_corner_dis_[i].push_back(local_point.norm());
                    if (USE_UNDISTORED_IMG_)
                    {
                        local_uv.x() = local_point(0) / local_point(2) * FOCAL_LENGTH_ + COL_ / 2;
                        local_uv.y() = local_point(1) / local_point(2) * FOCAL_LENGTH_ + ROW_ / 2;
                    }
                    else
                    {
                        m_camera_->spaceToPlane(local_point, local_uv);
                        local_uv.x() = std::min(std::max(-5000.0, local_uv.x()), 5000.0);
                        local_uv.y() = std::min(std::max(-5000.0, local_uv.y()), 5000.0);
                    }
                    output_Cube_[i].push_back(Vector3d(local_uv.x(), local_uv.y(), 1));
                }
            }
            else
            {
                Cube_center_depth_[i] = -1;
            }
        }
    }

    void draw_object(cv::Mat &AR_image)
    {
        for (int i = 0; i < axis_num; i++)
        {
            if (output_Axis_[i].empty())
                continue;
            cv::Point2d origin(output_Axis_[i][0].x(), output_Axis_[i][0].y());
            cv::Point2d axis_x(output_Axis_[i][1].x(), output_Axis_[i][1].y());
            cv::Point2d axis_y(output_Axis_[i][2].x(), output_Axis_[i][2].y());
            cv::Point2d axis_z(output_Axis_[i][3].x(), output_Axis_[i][3].y());
            cv::line(AR_image, origin, axis_x, cv::Scalar(0, 0, 255), 2, 8, 0);
            cv::line(AR_image, origin, axis_y, cv::Scalar(0, 255, 0), 2, 8, 0);
            cv::line(AR_image, origin, axis_z, cv::Scalar(255, 0, 0), 2, 8, 0);
        }

        int index[cube_num];
        for (int i = 0; i < cube_num; i++)
        {
            index[i] = i;
        }
        for (int i = 0; i < cube_num; i++)
            for (int j = 0; j < cube_num - i - 1; j++)
            {
                if (Cube_center_depth_[j] < Cube_center_depth_[j + 1])
                {
                    double tmp = Cube_center_depth_[j];
                    Cube_center_depth_[j] = Cube_center_depth_[j + 1];
                    Cube_center_depth_[j + 1] = tmp;
                    int tmp_index = index[j];
                    index[j] = index[j + 1];
                    index[j + 1] = tmp_index;
                }
            }

        for (int k = 0; k < cube_num; k++)
        {
            int i = index[k];
            if (output_Cube_[i].empty())
                continue;
            cv::Point *p = new cv::Point[8];
            p[0] = cv::Point(output_Cube_[i][0].x(), output_Cube_[i][0].y());
            p[1] = cv::Point(output_Cube_[i][1].x(), output_Cube_[i][1].y());
            p[2] = cv::Point(output_Cube_[i][2].x(), output_Cube_[i][2].y());
            p[3] = cv::Point(output_Cube_[i][3].x(), output_Cube_[i][3].y());
            p[4] = cv::Point(output_Cube_[i][4].x(), output_Cube_[i][4].y());
            p[5] = cv::Point(output_Cube_[i][5].x(), output_Cube_[i][5].y());
            p[6] = cv::Point(output_Cube_[i][6].x(), output_Cube_[i][6].y());
            p[7] = cv::Point(output_Cube_[i][7].x(), output_Cube_[i][7].y());

            int npts[1] = {4};
            float min_depth = 100000;
            int min_index = 5;
            for (int j = 0; j < (int)output_corner_dis_[i].size(); j++)
            {
                if (output_corner_dis_[i][j] < min_depth)
                {
                    min_depth = output_corner_dis_[i][j];
                    min_index = j;
                }
            }

            cv::Point plain[1][4];
            const cv::Point *ppt[1] = {plain[0]};
            int point_group[8][12] = {{0,1,5,4, 0,4,6,2, 0,1,3,2},
                {0,1,5,4, 1,5,7,3, 0,1,3,2},
                {2,3,7,6, 0,4,6,2, 0,1,3,2},
                {2,3,7,6, 1,5,7,3, 0,1,3,2},
                {0,1,5,4, 0,4,6,2, 4,5,7,6},
                {0,1,5,4, 1,5,7,3, 4,5,7,6},
                {2,3,7,6, 0,4,6,2, 4,5,7,6},
                {2,3,7,6, 1,5,7,3, 4,5,7,6}};

            plain[0][0] = p[point_group[min_index][4]];
            plain[0][1] = p[point_group[min_index][5]];
            plain[0][2] = p[point_group[min_index][6]];
            plain[0][3] = p[point_group[min_index][7]];
            cv::fillPoly(AR_image, ppt, npts, 1, cv::Scalar(0, 200, 0));

            plain[0][0] = p[point_group[min_index][0]];
            plain[0][1] = p[point_group[min_index][1]];
            plain[0][2] = p[point_group[min_index][2]];
            plain[0][3] = p[point_group[min_index][3]];
            cv::fillPoly(AR_image, ppt, npts, 1, cv::Scalar(200, 0, 0));

            if (output_corner_dis_[i][point_group[min_index][2]] + output_corner_dis_[i][point_group[min_index][3]] >
                output_corner_dis_[i][point_group[min_index][5]] + output_corner_dis_[i][point_group[min_index][6]])
            {
                plain[0][0] = p[point_group[min_index][4]];
                plain[0][1] = p[point_group[min_index][5]];
                plain[0][2] = p[point_group[min_index][6]];
                plain[0][3] = p[point_group[min_index][7]];
                cv::fillPoly(AR_image, ppt, npts, 1, cv::Scalar(0, 200, 0));
            }
            plain[0][0] = p[point_group[min_index][8]];
            plain[0][1] = p[point_group[min_index][9]];
            plain[0][2] = p[point_group[min_index][10]];
            plain[0][3] = p[point_group[min_index][11]];
            cv::fillPoly(AR_image, ppt, npts, 1, cv::Scalar(0, 0, 200));
            delete[] p;
        }
    }

    void callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg, const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
    {
        if (img_cnt_ < 50)
        {
            img_cnt_++;
            return;
        }
        Vector3d camera_p(pose_msg->pose.pose.position.x,
                          pose_msg->pose.pose.position.y,
                          pose_msg->pose.pose.position.z);
        Quaterniond camera_q(pose_msg->pose.pose.orientation.w,
                             pose_msg->pose.pose.orientation.x,
                             pose_msg->pose.pose.orientation.y,
                             pose_msg->pose.pose.orientation.z);

        Vector3d cam_z(0, 0, -1);
        Vector3d w_cam_z = camera_q * cam_z;
        if (acos(w_cam_z.dot(Vector3d(0, 0, 1))) * 180.0 / M_PI < 90)
        {
            look_ground_ = true;
        }
        else
            look_ground_ = false;

        project_object(camera_p, camera_q);

        cv_bridge::CvImageConstPtr ptr;
        if (img_msg->encoding == "8UC1")
        {
            sensor_msgs::msg::Image img;
            img.header = img_msg->header;
            img.height = img_msg->height;
            img.width = img_msg->width;
            img.is_bigendian = img_msg->is_bigendian;
            img.step = img_msg->step;
            img.data = img_msg->data;
            img.encoding = "mono8";
            ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        }
        else
            ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

        cv::Mat AR_image;
        AR_image = ptr->image.clone();
        cv::cvtColor(AR_image, AR_image, cv::COLOR_GRAY2RGB);
        draw_object(AR_image);

        sensor_msgs::msg::Image::SharedPtr AR_msg = cv_bridge::CvImage(img_msg->header, "bgr8", AR_image).toImageMsg();
        pub_ARimage_->publish(*AR_msg);
    }

    void point_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr point_msg)
    {
        if (!look_ground_)
            return;
        int height_range[30];
        double height_sum[30];
        for (int i = 0; i < 30; i++)
        {
            height_range[i] = 0;
            height_sum[i] = 0;
        }

        int x_off = -1, y_off = -1, z_off = -1;
        for (const auto &f : point_msg->fields)
        {
            if (f.name == "x")
                x_off = f.offset;
            else if (f.name == "y")
                y_off = f.offset;
            else if (f.name == "z")
                z_off = f.offset;
        }
        if (x_off < 0 || y_off < 0 || z_off < 0)
            return;

        const unsigned char *base = point_msg->data.data();
        for (unsigned int i = 0; i < point_msg->width; i++)
        {
            const unsigned char *ptr = base + i * point_msg->point_step;
            float z = *reinterpret_cast<const float *>(ptr + z_off);
            int index = (z + 2.0) / 0.1;
            if (0 <= index && index < 30)
            {
                height_range[index]++;
                height_sum[index] += z;
            }
        }
        int max_num = 0;
        int max_index = -1;
        for (int i = 1; i < 29; i++)
        {
            if (max_num < height_range[i])
            {
                max_num = height_range[i];
                max_index = i;
            }
        }
        if (max_index == -1)
            return;
        int tmp_num = height_range[max_index - 1] + height_range[max_index] + height_range[max_index + 1];
        double new_height = (height_sum[max_index - 1] + height_sum[max_index] + height_sum[max_index + 1]) / tmp_num;
        if (tmp_num < (int)point_msg->width / 2)
        {
            return;
        }
        for (int i = 0; i < cube_num; i++)
        {
            Cube_center_[i].z() = new_height + box_length / 2.0;
        }
        add_object();
    }

    void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
    {
        if (pose_init_)
        {
            img_buf_.push(img_msg);
        }
    }

    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
    {
        if (!pose_init_)
        {
            pose_init_ = true;
            return;
        }

        if (img_buf_.empty())
        {
            return;
        }

        while (!img_buf_.empty() && stampLess(img_buf_.front()->header.stamp, pose_msg->header.stamp))
        {
            img_buf_.pop();
        }

        if (!img_buf_.empty())
        {
            callback(img_buf_.front(), pose_msg);
            img_buf_.pop();
        }
    }

  private:
    static constexpr int axis_num = 0;
    static constexpr int cube_num = 1;
    static constexpr double box_length = 0.8;

    int ROW_;
    int COL_;
    double FOCAL_LENGTH_;
    bool USE_UNDISTORED_IMG_;
    bool pose_init_;
    int img_cnt_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr object_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_ARimage_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_pose_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_;

    Vector3d Axis_[6];
    Vector3d Cube_center_[3];
    vector<Vector3d> Cube_corner_[3];
    vector<Vector3d> output_Axis_[6];
    vector<Vector3d> output_Cube_[3];
    vector<double> output_corner_dis_[3];
    double Cube_center_depth_[3];
    std::queue<sensor_msgs::msg::Image::ConstSharedPtr> img_buf_;
    camodocal::CameraPtr m_camera_;
    bool look_ground_;
    std_msgs::msg::ColorRGBA line_color_r_;
    std_msgs::msg::ColorRGBA line_color_g_;
    std_msgs::msg::ColorRGBA line_color_b_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ARDemoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
