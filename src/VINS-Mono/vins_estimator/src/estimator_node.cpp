#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"

static inline double stampToSec(const builtin_interfaces::msg::Time &t)
{
    return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

static inline float pointField(const sensor_msgs::msg::PointCloud2 &cloud, size_t point_idx, size_t field_idx)
{
    const uint8_t *p = cloud.data.data() + point_idx * cloud.point_step;
    return *reinterpret_cast<const float *>(p + cloud.fields[field_idx].offset);
}

class EstimatorNode : public rclcpp::Node
{
  public:
    EstimatorNode()
        : Node("vins_estimator"),
          visualization_(this),
          current_time_(-1.0),
          sum_of_wait_(0),
          latest_time_(0.0),
          tmp_P_(Eigen::Vector3d::Zero()),
          tmp_Q_(Eigen::Quaterniond::Identity()),
          tmp_V_(Eigen::Vector3d::Zero()),
          tmp_Ba_(Eigen::Vector3d::Zero()),
          tmp_Bg_(Eigen::Vector3d::Zero()),
          acc_0_(Eigen::Vector3d::Zero()),
          gyr_0_(Eigen::Vector3d::Zero()),
          init_feature_(false),
          init_imu_(true),
          last_imu_t_(0.0),
          running_(true)
    {
        this->declare_parameter<std::string>("config_file", "");
        readParameters(this);
        estimator_.setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
        RCLCPP_DEBUG(this->get_logger(), "EIGEN_DONT_PARALLELIZE");
#endif
        RCLCPP_WARN(this->get_logger(), "waiting for image and imu...");

        visualization_.registerPub(this);

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            IMU_TOPIC, rclcpp::SensorDataQoS(),
            std::bind(&EstimatorNode::imu_callback, this, std::placeholders::_1));
        sub_image_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/feature_tracker/feature", 2000,
            std::bind(&EstimatorNode::feature_callback, this, std::placeholders::_1));
        sub_restart_ = this->create_subscription<std_msgs::msg::Bool>(
            "/feature_tracker/restart", 2000,
            std::bind(&EstimatorNode::restart_callback, this, std::placeholders::_1));
        sub_relo_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pose_graph/match_points", 2000,
            std::bind(&EstimatorNode::relocalization_callback, this, std::placeholders::_1));

        process_thread_ = std::thread(&EstimatorNode::process, this);
    }

    ~EstimatorNode()
    {
        running_ = false;
        con_.notify_all();
        if (process_thread_.joinable())
            process_thread_.join();
    }

  private:
    void predict(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
    {
        double t = stampToSec(imu_msg->header.stamp);
        if (init_imu_)
        {
            latest_time_ = t;
            acc_0_ = Eigen::Vector3d(
                imu_msg->linear_acceleration.x,
                imu_msg->linear_acceleration.y,
                imu_msg->linear_acceleration.z);
            gyr_0_ = Eigen::Vector3d(
                imu_msg->angular_velocity.x,
                imu_msg->angular_velocity.y,
                imu_msg->angular_velocity.z);
            init_imu_ = false;
            return;
        }
        double dt = t - latest_time_;
        latest_time_ = t;

        double dx = imu_msg->linear_acceleration.x;
        double dy = imu_msg->linear_acceleration.y;
        double dz = imu_msg->linear_acceleration.z;
        Eigen::Vector3d linear_acceleration{dx, dy, dz};

        double rx = imu_msg->angular_velocity.x;
        double ry = imu_msg->angular_velocity.y;
        double rz = imu_msg->angular_velocity.z;
        Eigen::Vector3d angular_velocity{rx, ry, rz};

        Eigen::Vector3d un_acc_0 = tmp_Q_ * (acc_0_ - tmp_Ba_) - estimator_.g;

        Eigen::Vector3d un_gyr = 0.5 * (gyr_0_ + angular_velocity) - tmp_Bg_;
        tmp_Q_ = tmp_Q_ * Utility::deltaQ(un_gyr * dt);

        Eigen::Vector3d un_acc_1 = tmp_Q_ * (linear_acceleration - tmp_Ba_) - estimator_.g;

        Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

        tmp_P_ = tmp_P_ + dt * tmp_V_ + 0.5 * dt * dt * un_acc;
        tmp_V_ = tmp_V_ + dt * un_acc;

        acc_0_ = linear_acceleration;
        gyr_0_ = angular_velocity;
    }

    void update()
    {
        TicToc t_predict;
        latest_time_ = current_time_;
        tmp_P_ = estimator_.Ps[WINDOW_SIZE];
        tmp_Q_ = estimator_.Rs[WINDOW_SIZE];
        tmp_V_ = estimator_.Vs[WINDOW_SIZE];
        tmp_Ba_ = estimator_.Bas[WINDOW_SIZE];
        tmp_Bg_ = estimator_.Bgs[WINDOW_SIZE];
        acc_0_ = estimator_.acc_0;
        gyr_0_ = estimator_.gyr_0;

        std::queue<sensor_msgs::msg::Imu::ConstSharedPtr> tmp_imu_buf = imu_buf_;
        for (sensor_msgs::msg::Imu::ConstSharedPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
            predict(tmp_imu_buf.front());
    }

    std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>, sensor_msgs::msg::PointCloud2::ConstSharedPtr>>
    getMeasurements()
    {
        std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>, sensor_msgs::msg::PointCloud2::ConstSharedPtr>> measurements;

        while (true)
        {
            if (imu_buf_.empty() || feature_buf_.empty())
                return measurements;

            if (!(stampToSec(imu_buf_.back()->header.stamp) > stampToSec(feature_buf_.front()->header.stamp) + estimator_.td))
            {
                sum_of_wait_++;
                return measurements;
            }

            if (!(stampToSec(imu_buf_.front()->header.stamp) < stampToSec(feature_buf_.front()->header.stamp) + estimator_.td))
            {
                RCLCPP_WARN(this->get_logger(), "throw img, only should happen at the beginning");
                feature_buf_.pop();
                continue;
            }
            sensor_msgs::msg::PointCloud2::ConstSharedPtr img_msg = feature_buf_.front();
            feature_buf_.pop();

            std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> IMUs;
            while (stampToSec(imu_buf_.front()->header.stamp) < stampToSec(img_msg->header.stamp) + estimator_.td)
            {
                IMUs.emplace_back(imu_buf_.front());
                imu_buf_.pop();
            }
            IMUs.emplace_back(imu_buf_.front());
            if (IMUs.empty())
                RCLCPP_WARN(this->get_logger(), "no imu between two image");
            measurements.emplace_back(IMUs, img_msg);
        }
        return measurements;
    }

    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
    {
        if (stampToSec(imu_msg->header.stamp) <= last_imu_t_)
        {
            RCLCPP_WARN(this->get_logger(), "imu message in disorder!");
            return;
        }
        last_imu_t_ = stampToSec(imu_msg->header.stamp);

        m_buf_.lock();
        imu_buf_.push(imu_msg);
        m_buf_.unlock();
        con_.notify_one();

        last_imu_t_ = stampToSec(imu_msg->header.stamp);

        {
            std::lock_guard<std::mutex> lg(m_state_);
            predict(imu_msg);
            std_msgs::msg::Header header = imu_msg->header;
            header.frame_id = "world";
            if (estimator_.solver_flag == Estimator::SolverFlag::NON_LINEAR)
                visualization_.pubLatestOdometry(tmp_P_, tmp_Q_, tmp_V_, header);
        }
    }

    void feature_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr feature_msg)
    {
        if (!init_feature_)
        {
            // skip the first detected feature, which doesn't contain optical flow speed
            init_feature_ = true;
            return;
        }
        m_buf_.lock();
        feature_buf_.push(feature_msg);
        m_buf_.unlock();
        con_.notify_one();
    }

    void restart_callback(const std_msgs::msg::Bool::ConstSharedPtr restart_msg)
    {
        if (restart_msg->data == true)
        {
            RCLCPP_WARN(this->get_logger(), "restart the estimator!");
            m_buf_.lock();
            while (!feature_buf_.empty())
                feature_buf_.pop();
            while (!imu_buf_.empty())
                imu_buf_.pop();
            m_buf_.unlock();
            m_estimator_.lock();
            estimator_.clearState();
            estimator_.setParameter();
            m_estimator_.unlock();
            {
                std::lock_guard<std::mutex> state_lock(m_state_);
                current_time_ = -1.0;
                latest_time_ = 0.0;
                last_imu_t_ = 0.0;
                init_feature_ = false;
                init_imu_ = true;
                tmp_P_.setZero();
                tmp_Q_.setIdentity();
                tmp_V_.setZero();
                tmp_Ba_.setZero();
                tmp_Bg_.setZero();
                acc_0_.setZero();
                gyr_0_.setZero();
            }
        }
        return;
    }

    void relocalization_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr points_msg)
    {
        m_buf_.lock();
        relo_buf_.push(points_msg);
        m_buf_.unlock();
    }

    // thread: visual-inertial odometry
    void process()
    {
        while (running_)
        {
            std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>, sensor_msgs::msg::PointCloud2::ConstSharedPtr>> measurements;
            std::unique_lock<std::mutex> lk(m_buf_);
            con_.wait(lk, [&] {
                if (!running_)
                    return true;
                return (measurements = getMeasurements()).size() != 0;
            });
            if (!running_)
                break;
            lk.unlock();
            m_estimator_.lock();
            for (auto &measurement : measurements)
            {
                auto img_msg = measurement.second;
                double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
                for (auto &imu_msg : measurement.first)
                {
                    double t = stampToSec(imu_msg->header.stamp);
                    double img_t = stampToSec(img_msg->header.stamp) + estimator_.td;
                    if (t <= img_t)
                    {
                        if (current_time_ < 0)
                            current_time_ = t;
                        double dt = t - current_time_;
                        assert(dt >= 0);
                        current_time_ = t;
                        dx = imu_msg->linear_acceleration.x;
                        dy = imu_msg->linear_acceleration.y;
                        dz = imu_msg->linear_acceleration.z;
                        rx = imu_msg->angular_velocity.x;
                        ry = imu_msg->angular_velocity.y;
                        rz = imu_msg->angular_velocity.z;
                        estimator_.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    }
                    else
                    {
                        double dt_1 = img_t - current_time_;
                        double dt_2 = t - img_t;
                        current_time_ = img_t;
                        assert(dt_1 >= 0);
                        assert(dt_2 >= 0);
                        assert(dt_1 + dt_2 > 0);
                        double w1 = dt_2 / (dt_1 + dt_2);
                        double w2 = dt_1 / (dt_1 + dt_2);
                        dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                        dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                        dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                        rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                        ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                        rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                        estimator_.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    }
                }
                // set relocalization frame
                sensor_msgs::msg::PointCloud2::ConstSharedPtr relo_msg = nullptr;
                while (!relo_buf_.empty())
                {
                    relo_msg = relo_buf_.front();
                    relo_buf_.pop();
                }
                if (relo_msg != nullptr)
                {
                    std::vector<Vector3d> match_points;
                    double frame_stamp = stampToSec(relo_msg->header.stamp);
                    for (unsigned int i = 0; i < relo_msg->width; i++)
                    {
                        Vector3d u_v_id;
                        u_v_id.x() = pointField(*relo_msg, i, 0);
                        u_v_id.y() = pointField(*relo_msg, i, 1);
                        u_v_id.z() = pointField(*relo_msg, i, 2);
                        match_points.push_back(u_v_id);
                    }
                    Vector3d relo_t(pointField(*relo_msg, 0, 3), pointField(*relo_msg, 0, 4), pointField(*relo_msg, 0, 5));
                    Quaterniond relo_q(pointField(*relo_msg, 0, 6), pointField(*relo_msg, 0, 7), pointField(*relo_msg, 0, 8), pointField(*relo_msg, 0, 9));
                    Matrix3d relo_r = relo_q.toRotationMatrix();
                    int frame_index;
                    frame_index = static_cast<int>(pointField(*relo_msg, 0, 10));
                    estimator_.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
                }

                RCLCPP_DEBUG(this->get_logger(), "processing vision data with stamp %f \n", stampToSec(img_msg->header.stamp));

                TicToc t_s;
                std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> image;
                for (unsigned int i = 0; i < img_msg->width; i++)
                {
                    int v = static_cast<int>(pointField(*img_msg, i, 3) + 0.5);
                    int feature_id = v / NUM_OF_CAM;
                    int camera_id = v % NUM_OF_CAM;
                    double x = pointField(*img_msg, i, 0);
                    double y = pointField(*img_msg, i, 1);
                    double z = pointField(*img_msg, i, 2);
                    double p_u = pointField(*img_msg, i, 4);
                    double p_v = pointField(*img_msg, i, 5);
                    double velocity_x = pointField(*img_msg, i, 6);
                    double velocity_y = pointField(*img_msg, i, 7);
                    assert(z == 1);
                    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                    xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                    image[feature_id].emplace_back(camera_id, xyz_uv_velocity);
                }
                estimator_.processImage(image, img_msg->header);

                double whole_t = t_s.toc();
                visualization_.printStatistics(estimator_, whole_t);
                std_msgs::msg::Header header = img_msg->header;
                header.frame_id = "world";

                visualization_.pubOdometry(estimator_, header);
                visualization_.pubKeyPoses(estimator_, header);
                visualization_.pubCameraPose(estimator_, header);
                visualization_.pubPointCloud(estimator_, header);
                visualization_.pubTF(estimator_, header);
                visualization_.pubKeyframe(estimator_);
                if (relo_msg != nullptr)
                    visualization_.pubRelocalization(estimator_);
            }
            m_estimator_.unlock();
            m_buf_.lock();
            m_state_.lock();
            if (estimator_.solver_flag == Estimator::SolverFlag::NON_LINEAR)
                update();
            m_state_.unlock();
            m_buf_.unlock();
        }
    }

    Estimator estimator_;
    Visualization visualization_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_image_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_restart_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_relo_points_;

    std::condition_variable con_;
    double current_time_;
    std::queue<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buf_;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> feature_buf_;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> relo_buf_;
    int sum_of_wait_;

    std::mutex m_buf_;
    std::mutex m_state_;
    std::mutex m_estimator_;

    double latest_time_;
    Eigen::Vector3d tmp_P_;
    Eigen::Quaterniond tmp_Q_;
    Eigen::Vector3d tmp_V_;
    Eigen::Vector3d tmp_Ba_;
    Eigen::Vector3d tmp_Bg_;
    Eigen::Vector3d acc_0_;
    Eigen::Vector3d gyr_0_;
    bool init_feature_;
    bool init_imu_;
    double last_imu_t_;

    std::atomic<bool> running_;
    std::thread process_thread_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EstimatorNode>());
    rclcpp::shutdown();
    return 0;
}
