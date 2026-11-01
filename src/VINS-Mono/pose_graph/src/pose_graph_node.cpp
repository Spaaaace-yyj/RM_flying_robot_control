#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <fstream>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <atomic>
#include <sys/select.h>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "pose_graph.h"
#include "utility/CameraPoseVisualization.h"
#include "parameters.h"

#define SKIP_FIRST_CNT 10
using namespace std;

// Globals declared extern in parameters.h (shared with algorithm files)
camodocal::CameraPtr m_camera;
Eigen::Vector3d tic = Eigen::Vector3d::Zero();
Eigen::Matrix3d qic = Eigen::Matrix3d::Identity();
int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
int ROW;
int COL;
std::string VINS_RESULT_PATH;
int DEBUG_IMAGE;
int FAST_RELOCALIZATION = 0;

static inline double stampToSec(const builtin_interfaces::msg::Time &t)
{
    return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

static inline builtin_interfaces::msg::Time stampFromDouble(double t)
{
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(t);
    stamp.nanosec = static_cast<uint32_t>((t - static_cast<double>(stamp.sec)) * 1e9);
    return stamp;
}

class PoseGraphNode : public rclcpp::Node
{
public:
    PoseGraphNode()
        : Node("pose_graph"),
          cameraposevisual_(1, 0, 0, 1),
          last_t_(-100, -100, -100),
          last_image_time_(-1),
          frame_index_(0),
          sequence_(1),
          skip_first_cnt_(0),
          skip_cnt_(0),
          load_flag_(0),
          start_flag_(0),
          SKIP_CNT_(0),
          LOOP_CLOSURE_(0),
          VISUALIZE_IMU_FORWARD_(0),
          SKIP_DIS_(0.0)
    {
        posegraph_.registerPub(this);

        VISUALIZATION_SHIFT_X = this->declare_parameter("visualization_shift_x", 0);
        VISUALIZATION_SHIFT_Y = this->declare_parameter("visualization_shift_y", 0);
        SKIP_CNT_ = this->declare_parameter("skip_cnt", 0);
        SKIP_DIS_ = this->declare_parameter("skip_dis", 0.0);
        std::string config_file = this->declare_parameter("config_file", std::string(""));

        cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Wrong path to settings");
        }

        double camera_visual_size = fsSettings["visualize_camera_size"];
        cameraposevisual_.setScale(camera_visual_size);
        cameraposevisual_.setLineWidth(camera_visual_size / 10.0);

        LOOP_CLOSURE_ = fsSettings["loop_closure"];
        std::string IMAGE_TOPIC;
        int LOAD_PREVIOUS_POSE_GRAPH;
        if (LOOP_CLOSURE_)
        {
            ROW = fsSettings["image_height"];
            COL = fsSettings["image_width"];
            std::string pkg_path = ament_index_cpp::get_package_share_directory("pose_graph");
            string vocabulary_file = pkg_path + "/support_files/brief_k10L6.bin";
            cout << "vocabulary_file " << vocabulary_file << endl;
            posegraph_.loadVocabulary(vocabulary_file);

            BRIEF_PATTERN_FILE = pkg_path + "/support_files/brief_pattern.yml";
            cout << "BRIEF_PATTERN_FILE " << BRIEF_PATTERN_FILE << endl;
            m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file.c_str());

            fsSettings["image_topic"] >> IMAGE_TOPIC;
            fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
            fsSettings["output_path"] >> VINS_RESULT_PATH;
            fsSettings["save_image"] >> DEBUG_IMAGE;

            FileSystemHelper::createDirectoryIfNotExists(POSE_GRAPH_SAVE_PATH.c_str());
            FileSystemHelper::createDirectoryIfNotExists(VINS_RESULT_PATH.c_str());

            VISUALIZE_IMU_FORWARD_ = fsSettings["visualize_imu_forward"];
            LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"];
            FAST_RELOCALIZATION = fsSettings["fast_relocalization"];
            VINS_RESULT_PATH = VINS_RESULT_PATH + "/vins_result_loop.csv";
            std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
            fout.close();
            fsSettings.release();

            if (LOAD_PREVIOUS_POSE_GRAPH)
            {
                printf("load pose graph\n");
                m_process_.lock();
                posegraph_.loadPoseGraph();
                m_process_.unlock();
                printf("load pose graph finish\n");
                load_flag_ = 1;
            }
            else
            {
                printf("no previous pose graph\n");
                load_flag_ = 1;
            }
        }

        fsSettings.release();

        sub_imu_forward_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/vins_estimator/imu_propagate", 2000,
            std::bind(&PoseGraphNode::imu_forward_callback, this, std::placeholders::_1));
        sub_vio_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/vins_estimator/odometry", 2000,
            std::bind(&PoseGraphNode::vio_callback, this, std::placeholders::_1));
        sub_extrinsic_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/vins_estimator/extrinsic", 2000,
            std::bind(&PoseGraphNode::extrinsic_callback, this, std::placeholders::_1));

        // The following streams are only meaningful when loop closure is enabled.
        // In particular, IMAGE_TOPIC is intentionally not read when it is disabled.
        if (LOOP_CLOSURE_)
        {
            sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
                IMAGE_TOPIC, rclcpp::SensorDataQoS(),
                std::bind(&PoseGraphNode::image_callback, this, std::placeholders::_1));
            sub_pose_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/vins_estimator/keyframe_pose", 2000,
                std::bind(&PoseGraphNode::pose_callback, this, std::placeholders::_1));
            sub_point_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/vins_estimator/keyframe_point", 2000,
                std::bind(&PoseGraphNode::point_callback, this, std::placeholders::_1));
            sub_relo_relative_pose_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/vins_estimator/relo_relative_pose", 2000,
                std::bind(&PoseGraphNode::relo_relative_pose_callback, this, std::placeholders::_1));
        }

        pub_camera_pose_visual_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/camera_pose_visual", 1000);
        pub_key_odometrys_ = this->create_publisher<visualization_msgs::msg::Marker>("~/key_odometrys", 1000);
        pub_vio_path_ = this->create_publisher<nav_msgs::msg::Path>("~/no_loop_path", 1000);
    }

    void startThreads()
    {
        measurement_process_ = std::thread(&PoseGraphNode::process, this);
        keyboard_command_process_ = std::thread(&PoseGraphNode::command, this);
    }

    ~PoseGraphNode()
    {
        running_ = false;
        if (measurement_process_.joinable())
            measurement_process_.join();
        if (keyboard_command_process_.joinable())
            keyboard_command_process_.join();
    }

private:
    void new_sequence()
    {
        printf("new sequence\n");
        sequence_++;
        printf("sequence cnt %d \n", sequence_);
        if (sequence_ > 5)
        {
            RCLCPP_WARN(this->get_logger(), "only support 5 sequences since it's boring to copy code for more sequences.");
            assert(false);
        }
        posegraph_.posegraph_visualization->reset();
        posegraph_.publish();
        m_buf_.lock();
        while (!image_buf_.empty())
            image_buf_.pop();
        while (!point_buf_.empty())
            point_buf_.pop();
        while (!pose_buf_.empty())
            pose_buf_.pop();
        while (!odometry_buf_.empty())
            odometry_buf_.pop();
        m_buf_.unlock();
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg)
    {
        if (!LOOP_CLOSURE_)
            return;
        m_buf_.lock();
        image_buf_.push(image_msg);
        m_buf_.unlock();

        if (last_image_time_ == -1)
            last_image_time_ = stampToSec(image_msg->header.stamp);
        else if (stampToSec(image_msg->header.stamp) - last_image_time_ > 1.0 ||
                 stampToSec(image_msg->header.stamp) < last_image_time_)
        {
            RCLCPP_WARN(this->get_logger(), "image discontinue! detect a new sequence!");
            new_sequence();
        }
        last_image_time_ = stampToSec(image_msg->header.stamp);
    }

    void point_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr point_msg)
    {
        if (!LOOP_CLOSURE_)
            return;
        m_buf_.lock();
        point_buf_.push(point_msg);
        m_buf_.unlock();
    }

    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
    {
        if (!LOOP_CLOSURE_)
            return;
        m_buf_.lock();
        pose_buf_.push(pose_msg);
        m_buf_.unlock();
    }

    void imu_forward_callback(const nav_msgs::msg::Odometry::ConstSharedPtr forward_msg)
    {
        if (VISUALIZE_IMU_FORWARD_)
        {
            Vector3d vio_t(forward_msg->pose.pose.position.x, forward_msg->pose.pose.position.y, forward_msg->pose.pose.position.z);
            Quaterniond vio_q;
            vio_q.w() = forward_msg->pose.pose.orientation.w;
            vio_q.x() = forward_msg->pose.pose.orientation.x;
            vio_q.y() = forward_msg->pose.pose.orientation.y;
            vio_q.z() = forward_msg->pose.pose.orientation.z;

            vio_t = posegraph_.w_r_vio * vio_t + posegraph_.w_t_vio;
            vio_q = posegraph_.w_r_vio * vio_q;

            vio_t = posegraph_.r_drift * vio_t + posegraph_.t_drift;
            vio_q = posegraph_.r_drift * vio_q;

            Vector3d vio_t_cam;
            Quaterniond vio_q_cam;
            vio_t_cam = vio_t + vio_q * tic;
            vio_q_cam = vio_q * qic;

            cameraposevisual_.reset();
            cameraposevisual_.add_pose(vio_t_cam, vio_q_cam);
            cameraposevisual_.publish_by(pub_camera_pose_visual_, forward_msg->header);
        }
    }

    void relo_relative_pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
    {
        Vector3d relative_t = Vector3d(pose_msg->pose.pose.position.x,
                                       pose_msg->pose.pose.position.y,
                                       pose_msg->pose.pose.position.z);
        Quaterniond relative_q;
        relative_q.w() = pose_msg->pose.pose.orientation.w;
        relative_q.x() = pose_msg->pose.pose.orientation.x;
        relative_q.y() = pose_msg->pose.pose.orientation.y;
        relative_q.z() = pose_msg->pose.pose.orientation.z;
        double relative_yaw = pose_msg->twist.twist.linear.x;
        int index = static_cast<int>(pose_msg->twist.twist.linear.y);
        Eigen::Matrix<double, 8, 1> loop_info;
        loop_info << relative_t.x(), relative_t.y(), relative_t.z(),
                     relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                     relative_yaw;
        posegraph_.updateKeyFrameLoop(index, loop_info);
    }

    void vio_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
    {
        Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
        Quaterniond vio_q;
        vio_q.w() = pose_msg->pose.pose.orientation.w;
        vio_q.x() = pose_msg->pose.pose.orientation.x;
        vio_q.y() = pose_msg->pose.pose.orientation.y;
        vio_q.z() = pose_msg->pose.pose.orientation.z;

        vio_t = posegraph_.w_r_vio * vio_t + posegraph_.w_t_vio;
        vio_q = posegraph_.w_r_vio * vio_q;

        vio_t = posegraph_.r_drift * vio_t + posegraph_.t_drift;
        vio_q = posegraph_.r_drift * vio_q;

        Vector3d vio_t_cam;
        Quaterniond vio_q_cam;
        vio_t_cam = vio_t + vio_q * tic;
        vio_q_cam = vio_q * qic;

        if (!VISUALIZE_IMU_FORWARD_)
        {
            cameraposevisual_.reset();
            cameraposevisual_.add_pose(vio_t_cam, vio_q_cam);
            cameraposevisual_.publish_by(pub_camera_pose_visual_, pose_msg->header);
        }

        odometry_buf_.push(vio_t_cam);
        if (odometry_buf_.size() > 10)
        {
            odometry_buf_.pop();
        }

        visualization_msgs::msg::Marker key_odometrys;
        key_odometrys.header = pose_msg->header;
        key_odometrys.header.frame_id = "world";
        key_odometrys.ns = "key_odometrys";
        key_odometrys.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        key_odometrys.action = visualization_msgs::msg::Marker::ADD;
        key_odometrys.pose.orientation.w = 1.0;
        key_odometrys.lifetime = builtin_interfaces::msg::Duration();

        key_odometrys.id = 0;
        key_odometrys.scale.x = 0.1;
        key_odometrys.scale.y = 0.1;
        key_odometrys.scale.z = 0.1;
        key_odometrys.color.r = 1.0;
        key_odometrys.color.a = 1.0;

        for (unsigned int i = 0; i < odometry_buf_.size(); i++)
        {
            geometry_msgs::msg::Point pose_marker;
            Vector3d vio_t_local;
            vio_t_local = odometry_buf_.front();
            odometry_buf_.pop();
            pose_marker.x = vio_t_local.x();
            pose_marker.y = vio_t_local.y();
            pose_marker.z = vio_t_local.z();
            key_odometrys.points.push_back(pose_marker);
            odometry_buf_.push(vio_t_local);
        }
        pub_key_odometrys_->publish(key_odometrys);

        if (!LOOP_CLOSURE_)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = pose_msg->header;
            pose_stamped.header.frame_id = "world";
            pose_stamped.pose.position.x = vio_t.x();
            pose_stamped.pose.position.y = vio_t.y();
            pose_stamped.pose.position.z = vio_t.z();
            no_loop_path_.header = pose_msg->header;
            no_loop_path_.header.frame_id = "world";
            no_loop_path_.poses.push_back(pose_stamped);
            pub_vio_path_->publish(no_loop_path_);
        }
    }

    void extrinsic_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
    {
        m_process_.lock();
        tic = Vector3d(pose_msg->pose.pose.position.x,
                       pose_msg->pose.pose.position.y,
                       pose_msg->pose.pose.position.z);
        qic = Quaterniond(pose_msg->pose.pose.orientation.w,
                          pose_msg->pose.pose.orientation.x,
                          pose_msg->pose.pose.orientation.y,
                          pose_msg->pose.pose.orientation.z).toRotationMatrix();
        m_process_.unlock();
    }

    static inline float pointField(const sensor_msgs::msg::PointCloud2 &cloud, size_t point_idx, size_t field_idx)
    {
        if (point_idx >= cloud.width || field_idx >= cloud.fields.size())
            return 0.0f;
        const float *data_ptr = reinterpret_cast<const float *>(&cloud.data[point_idx * cloud.point_step]);
        return data_ptr[field_idx];
    }

    void process()
    {
        if (!LOOP_CLOSURE_)
            return;
        while (running_)
        {
            sensor_msgs::msg::Image::ConstSharedPtr image_msg = nullptr;
            sensor_msgs::msg::PointCloud2::ConstSharedPtr point_msg = nullptr;
            nav_msgs::msg::Odometry::ConstSharedPtr pose_msg = nullptr;

            m_buf_.lock();
            if (!image_buf_.empty() && !point_buf_.empty() && !pose_buf_.empty())
            {
                if (stampToSec(image_buf_.front()->header.stamp) > stampToSec(pose_buf_.front()->header.stamp))
                {
                    pose_buf_.pop();
                    printf("throw pose at beginning\n");
                }
                else if (stampToSec(image_buf_.front()->header.stamp) > stampToSec(point_buf_.front()->header.stamp))
                {
                    point_buf_.pop();
                    printf("throw point at beginning\n");
                }
                else if (stampToSec(image_buf_.back()->header.stamp) >= stampToSec(pose_buf_.front()->header.stamp) &&
                         stampToSec(point_buf_.back()->header.stamp) >= stampToSec(pose_buf_.front()->header.stamp))
                {
                    pose_msg = pose_buf_.front();
                    pose_buf_.pop();
                    while (!pose_buf_.empty())
                        pose_buf_.pop();
                    while (stampToSec(image_buf_.front()->header.stamp) < stampToSec(pose_msg->header.stamp))
                        image_buf_.pop();
                    image_msg = image_buf_.front();
                    image_buf_.pop();

                    while (stampToSec(point_buf_.front()->header.stamp) < stampToSec(pose_msg->header.stamp))
                        point_buf_.pop();
                    point_msg = point_buf_.front();
                    point_buf_.pop();
                }
            }
            m_buf_.unlock();

            if (pose_msg != nullptr)
            {
                if (skip_first_cnt_ < SKIP_FIRST_CNT)
                {
                    skip_first_cnt_++;
                    continue;
                }

                if (skip_cnt_ < SKIP_CNT_)
                {
                    skip_cnt_++;
                    continue;
                }
                else
                {
                    skip_cnt_ = 0;
                }

                cv_bridge::CvImageConstPtr ptr;
                if (image_msg->encoding == "8UC1")
                {
                    sensor_msgs::msg::Image img;
                    img.header = image_msg->header;
                    img.height = image_msg->height;
                    img.width = image_msg->width;
                    img.is_bigendian = image_msg->is_bigendian;
                    img.step = image_msg->step;
                    img.data = image_msg->data;
                    img.encoding = "mono8";
                    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
                }
                else
                    ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);

                cv::Mat image = ptr->image;

                Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                                      pose_msg->pose.pose.position.y,
                                      pose_msg->pose.pose.position.z);
                Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                                         pose_msg->pose.pose.orientation.x,
                                         pose_msg->pose.pose.orientation.y,
                                         pose_msg->pose.pose.orientation.z).toRotationMatrix();
                if ((T - last_t_).norm() > SKIP_DIS_)
                {
                    vector<cv::Point3f> point_3d;
                    vector<cv::Point2f> point_2d_uv;
                    vector<cv::Point2f> point_2d_normal;
                    vector<double> point_id;

                    for (unsigned int i = 0; i < point_msg->width; i++)
                    {
                        cv::Point3f p_3d;
                        p_3d.x = pointField(*point_msg, i, 0);
                        p_3d.y = pointField(*point_msg, i, 1);
                        p_3d.z = pointField(*point_msg, i, 2);
                        point_3d.push_back(p_3d);

                        cv::Point2f p_2d_uv, p_2d_normal;
                        double p_id;
                        p_2d_normal.x = pointField(*point_msg, i, 3);
                        p_2d_normal.y = pointField(*point_msg, i, 4);
                        p_2d_uv.x = pointField(*point_msg, i, 5);
                        p_2d_uv.y = pointField(*point_msg, i, 6);
                        p_id = pointField(*point_msg, i, 7);
                        point_2d_normal.push_back(p_2d_normal);
                        point_2d_uv.push_back(p_2d_uv);
                        point_id.push_back(p_id);
                    }

                    KeyFrame *keyframe = new KeyFrame(stampToSec(pose_msg->header.stamp), frame_index_, T, R, image,
                                                      point_3d, point_2d_uv, point_2d_normal, point_id, sequence_);
                    m_process_.lock();
                    start_flag_ = 1;
                    posegraph_.addKeyFrame(keyframe, 1);
                    m_process_.unlock();
                    frame_index_++;
                    last_t_ = T;
                }
            }

            std::chrono::milliseconds dura(5);
            std::this_thread::sleep_for(dura);
        }
    }

    void command()
    {
        if (!LOOP_CLOSURE_)
            return;
        while (running_)
        {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(STDIN_FILENO, &fds);
            timeval tv{0, 50000};  // 50 ms timeout
            int ret = select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv);
            if (ret > 0 && FD_ISSET(STDIN_FILENO, &fds))
            {
                char c = getchar();
                if (c == 's')
                {
                    m_process_.lock();
                    posegraph_.savePoseGraph();
                    m_process_.unlock();
                    printf("save pose graph finish\nyou can set 'load_previous_pose_graph' to 1 in the config file to reuse it next time\n");
                }
                if (c == 'n')
                    new_sequence();
            }

            std::chrono::milliseconds dura(5);
            std::this_thread::sleep_for(dura);
        }
    }

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_imu_forward_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_vio_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_pose_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_extrinsic_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_relo_relative_pose_;

    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_camera_pose_visual_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_key_odometrys_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_vio_path_;

    // Buffers
    std::queue<sensor_msgs::msg::Image::ConstSharedPtr> image_buf_;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> point_buf_;
    std::queue<nav_msgs::msg::Odometry::ConstSharedPtr> pose_buf_;
    std::queue<Eigen::Vector3d> odometry_buf_;
    std::mutex m_buf_;
    std::mutex m_process_;

    // State
    PoseGraph posegraph_;
    CameraPoseVisualization cameraposevisual_;
    nav_msgs::msg::Path no_loop_path_;
    Eigen::Vector3d last_t_;
    double last_image_time_;

    int frame_index_;
    int sequence_;
    int skip_first_cnt_;
    int skip_cnt_;
    int load_flag_;
    bool start_flag_;

    int SKIP_CNT_;
    int LOOP_CLOSURE_;
    int VISUALIZE_IMU_FORWARD_;
    double SKIP_DIS_;

    std::atomic<bool> running_{true};
    std::thread measurement_process_;
    std::thread keyboard_command_process_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseGraphNode>();
    node->startThreads();
    rclcpp::spin(node);
    // Destroy the node (and join its threads) BEFORE shutting down the context:
    // otherwise FastDDS crashes while destroying the participant after shutdown.
    node.reset();
    rclcpp::shutdown();
    return 0;
}
