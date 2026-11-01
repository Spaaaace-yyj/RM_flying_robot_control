#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

inline double stampToSec(const builtin_interfaces::msg::Time& t)
{
    return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

class FeatureTrackerNode : public rclcpp::Node
{
public:
    FeatureTrackerNode()
        : Node("feature_tracker"),
          first_image_time_(0.0),
          pub_count_(1),
          first_image_flag_(true),
          last_image_time_(0.0),
          init_pub_(false)
    {
        // ROS1 private parameters "config_file" and "vins_folder"
        this->declare_parameter<std::string>("config_file", "");
        this->declare_parameter<std::string>("vins_folder", "");
        readParameters(this);

        for (int i = 0; i < NUM_OF_CAM; i++)
            trackerData_[i].readIntrinsicParameter(CAM_NAMES[i]);

        if (FISHEYE)
        {
            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                trackerData_[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
                if (!trackerData_[i].fisheye_mask.data)
                {
                    RCLCPP_INFO(this->get_logger(), "load mask fail");
                    rclcpp::shutdown();
                    return;
                }
                else
                    RCLCPP_INFO(this->get_logger(), "load mask success");
            }
        }

        sub_img_ = this->create_subscription<sensor_msgs::msg::Image>(
            IMAGE_TOPIC, rclcpp::SensorDataQoS(),
            std::bind(&FeatureTrackerNode::img_callback, this, std::placeholders::_1));

        // ROS 1 used a private NodeHandle ("~"). Use ROS 2 private names
        // explicitly so these resolve below /feature_tracker.
        pub_img_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/feature", 1000);
        pub_match_ = this->create_publisher<sensor_msgs::msg::Image>("~/feature_img", 1000);
        pub_restart_ = this->create_publisher<std_msgs::msg::Bool>("~/restart", 1000);
    }

private:
    void img_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {
        if (first_image_flag_)
        {
            first_image_flag_ = false;
            first_image_time_ = stampToSec(img_msg->header.stamp);
            last_image_time_ = stampToSec(img_msg->header.stamp);
            return;
        }
        // detect unstable camera stream
        if (stampToSec(img_msg->header.stamp) - last_image_time_ > 1.0 ||
            stampToSec(img_msg->header.stamp) < last_image_time_)
        {
            RCLCPP_WARN(this->get_logger(), "image discontinue! reset the feature tracker!");
            first_image_flag_ = true;
            last_image_time_ = 0;
            pub_count_ = 1;
            std_msgs::msg::Bool restart_flag;
            restart_flag.data = true;
            pub_restart_->publish(restart_flag);
            return;
        }
        last_image_time_ = stampToSec(img_msg->header.stamp);
        // frequency control
        if (round(1.0 * pub_count_ / (stampToSec(img_msg->header.stamp) - first_image_time_)) <= FREQ)
        {
            PUB_THIS_FRAME = true;
            // reset the frequency control
            if (abs(1.0 * pub_count_ / (stampToSec(img_msg->header.stamp) - first_image_time_) - FREQ) <
                0.01 * FREQ)
            {
                first_image_time_ = stampToSec(img_msg->header.stamp);
                pub_count_ = 0;
            }
        }
        else
            PUB_THIS_FRAME = false;

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

        cv::Mat show_img = ptr->image;
        TicToc t_r;
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            RCLCPP_DEBUG(this->get_logger(), "processing camera %d", i);
            if (i != 1 || !STEREO_TRACK)
                trackerData_[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)),
                                          stampToSec(img_msg->header.stamp));
            else
            {
                if (EQUALIZE)
                {
                    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                    clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData_[i].cur_img);
                }
                else
                    trackerData_[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
            }

#if SHOW_UNDISTORTION
            trackerData_[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
        }

        for (unsigned int i = 0;; i++)
        {
            bool completed = false;
            for (int j = 0; j < NUM_OF_CAM; j++)
                if (j != 1 || !STEREO_TRACK)
                    completed |= trackerData_[j].updateID(i);
            if (!completed)
                break;
        }

        if (PUB_THIS_FRAME)
        {
            pub_count_++;

            std::vector<float> xs, ys, zs, ids, us, vs, vel_xs, vel_ys;

            std::vector<std::set<int>> hash_ids(NUM_OF_CAM);
            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                auto& un_pts = trackerData_[i].cur_un_pts;
                auto& cur_pts = trackerData_[i].cur_pts;
                auto& pt_ids = trackerData_[i].ids;
                auto& pts_velocity = trackerData_[i].pts_velocity;
                for (unsigned int j = 0; j < pt_ids.size(); j++)
                {
                    if (trackerData_[i].track_cnt[j] > 1)
                    {
                        int p_id = pt_ids[j];
                        hash_ids[i].insert(p_id);
                        xs.push_back(un_pts[j].x);
                        ys.push_back(un_pts[j].y);
                        zs.push_back(1.0f);
                        ids.push_back(static_cast<float>(p_id * NUM_OF_CAM + i));
                        us.push_back(cur_pts[j].x);
                        vs.push_back(cur_pts[j].y);
                        vel_xs.push_back(pts_velocity[j].x);
                        vel_ys.push_back(pts_velocity[j].y);
                    }
                }
            }

            sensor_msgs::msg::PointCloud2 feature_points;
            feature_points.header = img_msg->header;
            feature_points.header.frame_id = "world";
            feature_points.height = 1;
            feature_points.width = xs.size();
            feature_points.is_bigendian = false;
            feature_points.is_dense = true;
            feature_points.point_step = 8 * sizeof(float);
            feature_points.row_step = feature_points.point_step * feature_points.width;
            feature_points.fields.resize(8);
            int off = 0;
            for (int k = 0; k < 8; k++)
            {
                feature_points.fields[k].datatype = sensor_msgs::msg::PointField::FLOAT32;
                feature_points.fields[k].count = 1;
                feature_points.fields[k].offset = off;
                off += sizeof(float);
            }
            feature_points.fields[0].name = "x";
            feature_points.fields[1].name = "y";
            feature_points.fields[2].name = "z";
            feature_points.fields[3].name = "id";
            feature_points.fields[4].name = "u";
            feature_points.fields[5].name = "v";
            feature_points.fields[6].name = "velocity_x";
            feature_points.fields[7].name = "velocity_y";

            feature_points.data.resize(xs.size() * feature_points.point_step);
            uint8_t* data_ptr = feature_points.data.data();
            for (size_t j = 0; j < xs.size(); j++)
            {
                float* p = reinterpret_cast<float*>(data_ptr + j * feature_points.point_step);
                p[0] = xs[j];
                p[1] = ys[j];
                p[2] = zs[j];
                p[3] = ids[j];
                p[4] = us[j];
                p[5] = vs[j];
                p[6] = vel_xs[j];
                p[7] = vel_ys[j];
            }

            RCLCPP_DEBUG(this->get_logger(), "publish %f, at %f",
                         stampToSec(feature_points.header.stamp),
                         this->now().seconds());
            // skip the first image; since no optical speed on first image
            if (!init_pub_)
                init_pub_ = true;
            else
                pub_img_->publish(feature_points);

            if (SHOW_TRACK)
            {
                ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
                cv::Mat stereo_img = ptr->image;

                for (int i = 0; i < NUM_OF_CAM; i++)
                {
                    cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                    cv::cvtColor(show_img, tmp_img, cv::COLOR_GRAY2RGB);

                    for (unsigned int j = 0; j < trackerData_[i].cur_pts.size(); j++)
                    {
                        double len = std::min(1.0, 1.0 * trackerData_[i].track_cnt[j] / WINDOW_SIZE);
                        cv::circle(tmp_img, trackerData_[i].cur_pts[j], 2,
                                   cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                    }
                }
                pub_match_->publish(*ptr->toImageMsg());
            }
        }
        RCLCPP_INFO(this->get_logger(), "whole feature tracker processing costs: %f", t_r.toc());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_img_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_restart_;

    FeatureTracker trackerData_[NUM_OF_CAM];
    double first_image_time_;
    int pub_count_;
    bool first_image_flag_;
    double last_image_time_;
    bool init_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FeatureTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
