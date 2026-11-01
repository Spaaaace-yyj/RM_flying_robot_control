#include "visualization.h"

#include <array>

using namespace Eigen;

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

static void addField(sensor_msgs::msg::PointCloud2 &cloud, const std::string &name, int &offset)
{
    sensor_msgs::msg::PointField f;
    f.name = name;
    f.offset = offset;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    cloud.fields.push_back(f);
    offset += sizeof(float);
}

Visualization::Visualization(rclcpp::Node *node)
    : node_(node),
      cameraposevisual_(0, 1, 0, 1),
      keyframebasevisual_(0.0, 0.0, 1.0, 1.0),
      sum_of_path_(0.0),
      last_path_(0.0, 0.0, 0.0)
{
}

void Visualization::registerPub(rclcpp::Node *node)
{
    // Preserve the ROS 1 private-node topic layout under /vins_estimator/*.
    pub_latest_odometry_ = node->create_publisher<nav_msgs::msg::Odometry>("~/imu_propagate", 1000);
    pub_path_ = node->create_publisher<nav_msgs::msg::Path>("~/path", 1000);
    pub_relo_path_ = node->create_publisher<nav_msgs::msg::Path>("~/relocalization_path", 1000);
    pub_odometry_ = node->create_publisher<nav_msgs::msg::Odometry>("~/odometry", 1000);
    pub_point_cloud_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("~/point_cloud", 1000);
    pub_margin_cloud_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("~/history_cloud", 1000);
    pub_key_poses_ = node->create_publisher<visualization_msgs::msg::Marker>("~/key_poses", 1000);
    pub_camera_pose_ = node->create_publisher<nav_msgs::msg::Odometry>("~/camera_pose", 1000);
    pub_camera_pose_visual_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("~/camera_pose_visual", 1000);
    pub_keyframe_pose_ = node->create_publisher<nav_msgs::msg::Odometry>("~/keyframe_pose", 1000);
    pub_keyframe_point_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("~/keyframe_point", 1000);
    pub_extrinsic_ = node->create_publisher<nav_msgs::msg::Odometry>("~/extrinsic", 1000);
    pub_relo_relative_pose_ = node->create_publisher<nav_msgs::msg::Odometry>("~/relo_relative_pose", 1000);

    br_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);

    cameraposevisual_.setScale(1);
    cameraposevisual_.setLineWidth(0.05);
    keyframebasevisual_.setScale(0.1);
    keyframebasevisual_.setLineWidth(0.01);
}

void Visualization::pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, const std_msgs::msg::Header &header)
{
    Eigen::Quaterniond quadrotor_Q = Q;

    nav_msgs::msg::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = quadrotor_Q.x();
    odometry.pose.pose.orientation.y = quadrotor_Q.y();
    odometry.pose.pose.orientation.z = quadrotor_Q.z();
    odometry.pose.pose.orientation.w = quadrotor_Q.w();
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    pub_latest_odometry_->publish(odometry);
}

void Visualization::printStatistics(const Estimator &estimator, double t)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    printf("position: %f, %f, %f\r", estimator.Ps[WINDOW_SIZE].x(), estimator.Ps[WINDOW_SIZE].y(), estimator.Ps[WINDOW_SIZE].z());
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "position: " << estimator.Ps[WINDOW_SIZE].transpose());
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "orientation: " << estimator.Vs[WINDOW_SIZE].transpose());
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        RCLCPP_DEBUG_STREAM(node_->get_logger(), "extirnsic tic: " << estimator.tic[i].transpose());
        RCLCPP_DEBUG_STREAM(node_->get_logger(), "extrinsic ric: " << Utility::R2ypr(estimator.ric[i]).transpose());
        if (ESTIMATE_EXTRINSIC)
        {
            cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
            Eigen::Matrix3d eigen_R;
            Eigen::Vector3d eigen_T;
            eigen_R = estimator.ric[i];
            eigen_T = estimator.tic[i];
            cv::Mat cv_R, cv_T;
            cv::eigen2cv(eigen_R, cv_R);
            cv::eigen2cv(eigen_T, cv_T);
            fs << "extrinsicRotation" << cv_R << "extrinsicTranslation" << cv_T;
            fs.release();
        }
    }

    static double sum_of_time = 0;
    static int sum_of_calculation = 0;
    sum_of_time += t;
    sum_of_calculation++;
    RCLCPP_DEBUG(node_->get_logger(), "vo solver costs: %f ms", t);
    RCLCPP_DEBUG(node_->get_logger(), "average of time %f ms", sum_of_time / sum_of_calculation);

    sum_of_path_ += (estimator.Ps[WINDOW_SIZE] - last_path_).norm();
    last_path_ = estimator.Ps[WINDOW_SIZE];
    RCLCPP_DEBUG(node_->get_logger(), "sum of path %f", sum_of_path_);
    if (ESTIMATE_TD)
        RCLCPP_INFO(node_->get_logger(), "td %f", estimator.td);
}

void Visualization::pubOdometry(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        nav_msgs::msg::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.child_frame_id = "body";
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
        odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
        odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
        odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
        odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
        odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
        pub_odometry_->publish(odometry);

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose = odometry.pose.pose;
        path_.header = header;
        path_.header.frame_id = "world";
        path_.poses.push_back(pose_stamped);
        pub_path_->publish(path_);

        Vector3d correct_t;
        Vector3d correct_v;
        Quaterniond correct_q;
        correct_t = estimator.drift_correct_r * estimator.Ps[WINDOW_SIZE] + estimator.drift_correct_t;
        correct_q = estimator.drift_correct_r * estimator.Rs[WINDOW_SIZE];
        odometry.pose.pose.position.x = correct_t.x();
        odometry.pose.pose.position.y = correct_t.y();
        odometry.pose.pose.position.z = correct_t.z();
        odometry.pose.pose.orientation.x = correct_q.x();
        odometry.pose.pose.orientation.y = correct_q.y();
        odometry.pose.pose.orientation.z = correct_q.z();
        odometry.pose.pose.orientation.w = correct_q.w();

        pose_stamped.pose = odometry.pose.pose;
        relo_path_.header = header;
        relo_path_.header.frame_id = "world";
        relo_path_.poses.push_back(pose_stamped);
        pub_relo_path_->publish(relo_path_);

        // write result to file
        ofstream foutC(VINS_RESULT_PATH, ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(0);
        foutC << stampToSec(header.stamp) * 1e9 << ",";
        foutC.precision(5);
        foutC << estimator.Ps[WINDOW_SIZE].x() << ","
              << estimator.Ps[WINDOW_SIZE].y() << ","
              << estimator.Ps[WINDOW_SIZE].z() << ","
              << tmp_Q.w() << ","
              << tmp_Q.x() << ","
              << tmp_Q.y() << ","
              << tmp_Q.z() << ","
              << estimator.Vs[WINDOW_SIZE].x() << ","
              << estimator.Vs[WINDOW_SIZE].y() << ","
              << estimator.Vs[WINDOW_SIZE].z() << "," << endl;
        foutC.close();
    }
}

void Visualization::pubKeyPoses(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    if (estimator.key_poses.size() == 0)
        return;
    visualization_msgs::msg::Marker key_poses;
    key_poses.header = header;
    key_poses.header.frame_id = "world";
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    key_poses.action = visualization_msgs::msg::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = builtin_interfaces::msg::Duration();

    key_poses.id = 0;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        geometry_msgs::msg::Point pose_marker;
        Vector3d correct_pose;
        correct_pose = estimator.key_poses[i];
        pose_marker.x = correct_pose.x();
        pose_marker.y = correct_pose.y();
        pose_marker.z = correct_pose.z();
        key_poses.points.push_back(pose_marker);
    }
    pub_key_poses_->publish(key_poses);
}

void Visualization::pubCameraPose(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    int idx2 = WINDOW_SIZE - 1;

    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        int i = idx2;
        Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[0]);

        nav_msgs::msg::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        pub_camera_pose_->publish(odometry);

        cameraposevisual_.reset();
        cameraposevisual_.add_pose(P, R);
        cameraposevisual_.publish_by(pub_camera_pose_visual_, odometry.header);
    }
}


void Visualization::pubPointCloud(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    std::vector<Vector3d> point_cloud_pts, margin_cloud_pts;

    for (auto &it_per_id : estimator.f_manager.feature)
    {
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
            continue;
        int imu_i = it_per_id.start_frame;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];
        point_cloud_pts.push_back(w_pts_i);
    }

    // pub margined potin
    for (auto &it_per_id : estimator.f_manager.feature)
    {
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2
            && it_per_id.solve_flag == 1)
        {
            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];
            margin_cloud_pts.push_back(w_pts_i);
        }
    }

    auto buildCloud = [&header](const std::vector<Vector3d> &pts) {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = header;
        cloud.height = 1;
        cloud.width = pts.size();
        cloud.is_bigendian = false;
        cloud.is_dense = true;
        cloud.point_step = 3 * sizeof(float);
        cloud.row_step = cloud.point_step * cloud.width;
        int off = 0;
        addField(cloud, "x", off);
        addField(cloud, "y", off);
        addField(cloud, "z", off);
        cloud.data.resize(pts.size() * cloud.point_step);
        uint8_t *p = cloud.data.data();
        for (size_t i = 0; i < pts.size(); i++)
        {
            float *f = reinterpret_cast<float *>(p + i * cloud.point_step);
            f[0] = static_cast<float>(pts[i].x());
            f[1] = static_cast<float>(pts[i].y());
            f[2] = static_cast<float>(pts[i].z());
        }
        return cloud;
    };

    pub_point_cloud_->publish(buildCloud(point_cloud_pts));
    pub_margin_cloud_->publish(buildCloud(margin_cloud_pts));
}


void Visualization::pubTF(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;

    // body frame
    Vector3d correct_t;
    Quaterniond correct_q;
    correct_t = estimator.Ps[WINDOW_SIZE];
    correct_q = estimator.Rs[WINDOW_SIZE];

    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = header.stamp;
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "body";
        transformStamped.transform.translation.x = correct_t(0);
        transformStamped.transform.translation.y = correct_t(1);
        transformStamped.transform.translation.z = correct_t(2);
        tf2::Quaternion q;
        q.setW(correct_q.w());
        q.setX(correct_q.x());
        q.setY(correct_q.y());
        q.setZ(correct_q.z());
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        br_->sendTransform(transformStamped);
    }

    // camera frame
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = header.stamp;
        transformStamped.header.frame_id = "body";
        transformStamped.child_frame_id = "camera";
        transformStamped.transform.translation.x = estimator.tic[0].x();
        transformStamped.transform.translation.y = estimator.tic[0].y();
        transformStamped.transform.translation.z = estimator.tic[0].z();
        Quaterniond cam_q{estimator.ric[0]};
        transformStamped.transform.rotation.x = cam_q.x();
        transformStamped.transform.rotation.y = cam_q.y();
        transformStamped.transform.rotation.z = cam_q.z();
        transformStamped.transform.rotation.w = cam_q.w();
        br_->sendTransform(transformStamped);
    }

    nav_msgs::msg::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = estimator.tic[0].x();
    odometry.pose.pose.position.y = estimator.tic[0].y();
    odometry.pose.pose.position.z = estimator.tic[0].z();
    Quaterniond tmp_q{estimator.ric[0]};
    odometry.pose.pose.orientation.x = tmp_q.x();
    odometry.pose.pose.orientation.y = tmp_q.y();
    odometry.pose.pose.orientation.z = tmp_q.z();
    odometry.pose.pose.orientation.w = tmp_q.w();
    pub_extrinsic_->publish(odometry);
}

void Visualization::pubKeyframe(const Estimator &estimator)
{
    // pub camera pose, 2D-3D points of keyframe
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR && estimator.marginalization_flag == 0)
    {
        int i = WINDOW_SIZE - 2;
        Vector3d P = estimator.Ps[i];
        Quaterniond R = Quaterniond(estimator.Rs[i]);

        nav_msgs::msg::Odometry odometry;
        odometry.header = estimator.Headers[WINDOW_SIZE - 2];
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        pub_keyframe_pose_->publish(odometry);

        std::vector<Vector3d> pts;
        std::vector<std::array<float, 5>> channels;
        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int frame_size = it_per_id.feature_per_frame.size();
            if (it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.start_frame + frame_size - 1 >= WINDOW_SIZE - 2 && it_per_id.solve_flag == 1)
            {
                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0])
                                      + estimator.Ps[imu_i];
                pts.push_back(w_pts_i);

                int imu_j = WINDOW_SIZE - 2 - it_per_id.start_frame;
                std::array<float, 5> p_2d;
                p_2d[0] = static_cast<float>(it_per_id.feature_per_frame[imu_j].point.x());
                p_2d[1] = static_cast<float>(it_per_id.feature_per_frame[imu_j].point.y());
                p_2d[2] = static_cast<float>(it_per_id.feature_per_frame[imu_j].uv.x());
                p_2d[3] = static_cast<float>(it_per_id.feature_per_frame[imu_j].uv.y());
                p_2d[4] = static_cast<float>(it_per_id.feature_id);
                channels.push_back(p_2d);
            }
        }

        sensor_msgs::msg::PointCloud2 point_cloud;
        point_cloud.header = estimator.Headers[WINDOW_SIZE - 2];
        point_cloud.height = 1;
        point_cloud.width = pts.size();
        point_cloud.is_bigendian = false;
        point_cloud.is_dense = true;
        point_cloud.point_step = 8 * sizeof(float);
        point_cloud.row_step = point_cloud.point_step * point_cloud.width;
        int off = 0;
        addField(point_cloud, "x", off);
        addField(point_cloud, "y", off);
        addField(point_cloud, "z", off);
        addField(point_cloud, "p_2d_normal_x", off);
        addField(point_cloud, "p_2d_normal_y", off);
        addField(point_cloud, "p_2d_uv_x", off);
        addField(point_cloud, "p_2d_uv_y", off);
        addField(point_cloud, "p_id", off);
        point_cloud.data.resize(pts.size() * point_cloud.point_step);
        uint8_t *data_ptr = point_cloud.data.data();
        for (size_t j = 0; j < pts.size(); j++)
        {
            float *f = reinterpret_cast<float *>(data_ptr + j * point_cloud.point_step);
            f[0] = static_cast<float>(pts[j].x());
            f[1] = static_cast<float>(pts[j].y());
            f[2] = static_cast<float>(pts[j].z());
            f[3] = channels[j][0];
            f[4] = channels[j][1];
            f[5] = channels[j][2];
            f[6] = channels[j][3];
            f[7] = channels[j][4];
        }
        pub_keyframe_point_->publish(point_cloud);
    }
}

void Visualization::pubRelocalization(const Estimator &estimator)
{
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stampFromDouble(estimator.relo_frame_stamp);
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = estimator.relo_relative_t.x();
    odometry.pose.pose.position.y = estimator.relo_relative_t.y();
    odometry.pose.pose.position.z = estimator.relo_relative_t.z();
    odometry.pose.pose.orientation.x = estimator.relo_relative_q.x();
    odometry.pose.pose.orientation.y = estimator.relo_relative_q.y();
    odometry.pose.pose.orientation.z = estimator.relo_relative_q.z();
    odometry.pose.pose.orientation.w = estimator.relo_relative_q.w();
    odometry.twist.twist.linear.x = estimator.relo_relative_yaw;
    odometry.twist.twist.linear.y = estimator.relo_frame_index;

    pub_relo_relative_pose_->publish(odometry);
}
