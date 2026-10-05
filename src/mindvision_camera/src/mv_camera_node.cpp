// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

// MindVision Camera SDK
#include <CameraApi.h>
#include "frame_rate_limiter.hpp"

// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// C++ system
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace mindvision_camera
{
    class MVCameraNode : public rclcpp::Node
    {
    public:
        explicit MVCameraNode(const rclcpp::NodeOptions& options) : Node("mv_camera", options)
        {
            RCLCPP_INFO(this->get_logger(), "Starting MVCameraNode!");

            // Output format/rate/QoS are startup-only: changing them requires
            // reconfiguring the SDK and publisher together, so restart the node.
            rcl_interfaces::msg::ParameterDescriptor startup_desc;
            startup_desc.read_only = true;
            full_speed_ = declare_parameter("full_speed", true, startup_desc);
            target_fps_ = declare_parameter("target_fps", 30, startup_desc);
            output_encoding_ = declare_parameter<std::string>(
                "output_encoding", "rgb8", startup_desc);
            image_topic_ = declare_parameter<std::string>(
                "image_topic", "/image_raw", startup_desc);
            frame_id_ = declare_parameter<std::string>(
                "frame_id", "camera_optical_frame", startup_desc);
            const bool use_sensor_data_qos = declare_parameter(
                "use_sensor_data_qos", false, startup_desc);
            const int qos_depth = declare_parameter("qos_depth", 1, startup_desc);
            if (target_fps_ < 1 || target_fps_ > 1000000000 || qos_depth < 1 ||
                image_topic_.empty() ||
                (output_encoding_ != "rgb8" && output_encoding_ != "mono8"))
            {
                throw std::invalid_argument(
                    "target_fps must be in [1, 1000000000], qos_depth >= 1, "
                    "image_topic nonempty, output_encoding rgb8 or mono8");
            }
            channels_ = output_encoding_ == "mono8" ? 1 : 3;
            rate_limiter_.configure(full_speed_, target_fps_);

            CameraSdkInit(1);

            // 枚举设备，并建立设备列表
            int i_camera_counts = 1;
            int i_status = -1;
            tSdkCameraDevInfo t_camera_enum_list;
            i_status = CameraEnumerateDevice(&t_camera_enum_list, &i_camera_counts);
            RCLCPP_INFO(this->get_logger(), "Enumerate state = %d", i_status);
            RCLCPP_INFO(this->get_logger(), "Found camera count = %d", i_camera_counts);

            // 没有连接设备
            if (i_camera_counts == 0)
            {
                RCLCPP_ERROR(this->get_logger(), "No camera found!");
                return;
            }

            // 相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
            i_status = CameraInit(&t_camera_enum_list, -1, -1, &h_camera_);

            // 初始化失败
            RCLCPP_INFO(this->get_logger(), "Init state = %d", i_status);
            if (i_status != CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(this->get_logger(), "Init failed!");
                return;
            }
            camera_initialized_ = true;

            // 获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
            CameraGetCapability(h_camera_, &t_capability_);

            // 直接使用vector的内存作为相机输出buffer
            image_msg_.data.reserve(
                static_cast<size_t>(t_capability_.sResolutionRange.iHeightMax) *
                static_cast<size_t>(t_capability_.sResolutionRange.iWidthMax) * channels_);

            // 设置手动曝光
            CameraSetAeState(h_camera_, false);

            // 设置相机高速模式
            // iFrameSpeedDesc is a count, not a valid index.
            if (t_capability_.iFrameSpeedDesc > 0)
            {
                i_status = CameraSetFrameSpeed(h_camera_, t_capability_.iFrameSpeedDesc - 1);
                if (i_status != CAMERA_STATUS_SUCCESS)
                    RCLCPP_WARN(get_logger(), "CameraSetFrameSpeed failed: %d", i_status);
            }

            // Only some cameras support exact hardware rate control. Always
            // enforce the limit before ISP processing as a software fallback.
            i_status = CameraSetFrameRate(h_camera_, full_speed_ ? 0 : target_fps_);
            if (i_status != CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_WARN(get_logger(),
                    "CameraSetFrameRate unsupported/failed: %d; hardware rate unchanged, "
                    "software limit %s", i_status, full_speed_ ? "disabled" : "enabled");
            }

            // Declare camera parameters
            declareParameters();

            // 让SDK进入工作模式，开始接收来自相机发送的图像
            // 数据。如果当前相机是触发模式，则需要接收到
            // 触发帧以后才会更新图像。
            i_status = CameraSetIspOutFormat(h_camera_,
                channels_ == 1 ? CAMERA_MEDIA_TYPE_MONO8 : CAMERA_MEDIA_TYPE_RGB8);
            if (i_status == CAMERA_STATUS_SUCCESS)
                i_status = CameraPlay(h_camera_);
            if (i_status != CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(get_logger(), "Camera format/start failed: %d", i_status);
                CameraUnInit(h_camera_);
                camera_initialized_ = false;
                return;
            }

            // Create camera publisher
            // rqt_image_view can't subscribe image msg with sensor_data QoS
            // https://github.com/ros-visualization/rqt/issues/187
            auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
            qos.depth = static_cast<size_t>(qos_depth);
            camera_pub_ = image_transport::create_camera_publisher(this, image_topic_, qos);
            RCLCPP_INFO(get_logger(), "Image %s: %s, %s, target=%d Hz, QoS=%s/depth=%d",
                image_topic_.c_str(), output_encoding_.c_str(),
                full_speed_ ? "full speed" : "rate limited", target_fps_,
                use_sensor_data_qos ? "best effort" : "reliable", qos_depth);

            // Load camera info
            camera_name_ = this->declare_parameter("camera_name", "mv_camera");
            camera_info_manager_ =
                std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
            auto camera_info_url = this->declare_parameter(
                "camera_info_url", "package://mindvision_camera/config/camera_info.yaml");
            if (camera_info_manager_->validateURL(camera_info_url))
            {
                camera_info_manager_->loadCameraInfo(camera_info_url);
                camera_info_msg_ = camera_info_manager_->getCameraInfo();
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
            }

            // Add callback to the set parameter event
            params_callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&MVCameraNode::parametersCallback, this, std::placeholders::_1));

            capture_thread_ = std::thread{
                [this]() -> void
                {
                    RCLCPP_INFO(this->get_logger(), "Publishing image!");

                    camera_info_msg_.header.frame_id = image_msg_.header.frame_id = frame_id_;
                    image_msg_.encoding = output_encoding_;
                    image_msg_.is_bigendian = false;
                    size_t captured = 0, published = 0;
                    auto report_start = std::chrono::steady_clock::now();

                    while (running_.load() && rclcpp::ok())
                    {
                        int status = CameraGetImageBuffer(h_camera_, &s_frame_info_, &pby_buffer_, 1000);

                        if (status == CAMERA_STATUS_SUCCESS)
                        {
                            // Release on every exit path, including allocation failures.
                            const auto release_buffer = [this](uint8_t* buffer) {
                                CameraReleaseImageBuffer(h_camera_, buffer);
                            };
                            std::unique_ptr<uint8_t, decltype(release_buffer)> raw_buffer(
                                pby_buffer_, release_buffer);
                            ++captured;
                            RCLCPP_DEBUG_THROTTLE(
                                this->get_logger(),
                                *this->get_clock(),
                                1000,
                                "camera timestamp=%u, exposure=%u us",
                                s_frame_info_.uiTimeStamp,
                                s_frame_info_.uiExpTime);
                            const auto receive_time = this->get_clock()->now();

                            const auto image_stamp = make_camera_stamp(
                                s_frame_info_.uiTimeStamp,
                                receive_time);

                            const bool process_frame =
                                rate_limiter_.accept(image_stamp.nanoseconds());
                            if (process_frame && s_frame_info_.iWidth > 0 && s_frame_info_.iHeight > 0)
                            {
                                image_msg_.height = s_frame_info_.iHeight;
                                image_msg_.width = s_frame_info_.iWidth;
                                image_msg_.step = image_msg_.width * channels_;
                                // reserve() alone does not make the vector writable.
                                image_msg_.data.resize(
                                    static_cast<size_t>(image_msg_.step) * image_msg_.height);
                                const int process_status = CameraImageProcess(
                                    h_camera_, raw_buffer.get(), image_msg_.data.data(), &s_frame_info_);
                                // Do not hold an SDK buffer while serializing/publishing ROS data.
                                raw_buffer.reset();
                                if (process_status != CAMERA_STATUS_SUCCESS)
                                {
                                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                        "CameraImageProcess failed: %d", process_status);
                                    continue;
                                }
                                if (flip_image_)
                                {
                                    CameraFlipFrameBuffer(image_msg_.data.data(), &s_frame_info_, 3);
                                }
                                camera_info_msg_.header.stamp = image_msg_.header.stamp = image_stamp;
                                camera_pub_.publish(image_msg_, camera_info_msg_);
                                ++published;
                            }
                            raw_buffer.reset();

                            const auto report_now = std::chrono::steady_clock::now();
                            const double elapsed = std::chrono::duration<double>(
                                report_now - report_start).count();
                            if (elapsed >= 1.0)
                            {
                                RCLCPP_INFO(get_logger(), "Camera FPS: capture=%.1f, publish=%.1f",
                                    captured / elapsed, published / elapsed);
                                captured = published = 0;
                                report_start = report_now;
                            }
                        }
                        else
                        {
                            RCLCPP_WARN_THROTTLE(
                                this->get_logger(),
                                *this->get_clock(),
                                1000,
                                "CameraGetImageBuffer failed, status = %d",
                                status
                            );
                            continue;
                        }
                    }
                }
            };
        }

        ~MVCameraNode() override
        {
            running_.store(false);
            if (capture_thread_.joinable())
            {
                capture_thread_.join();
            }

            if (camera_initialized_)
                CameraUnInit(h_camera_);

            RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
        }

    private:
        bool restartCameraSoft()
        {
            RCLCPP_WARN(this->get_logger(), "Trying soft camera restart...");

            int status = CameraPause(h_camera_);
            RCLCPP_WARN(this->get_logger(), "CameraPause status = %d", status);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            status = CameraPlay(h_camera_);
            RCLCPP_WARN(this->get_logger(), "CameraPlay status = %d", status);

            if (status != CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(this->get_logger(), "Soft camera restart failed.");
                return false;
            }

            RCLCPP_WARN(this->get_logger(), "Soft camera restart success.");
            return true;
        }

        bool reconnectCamera()
        {
            RCLCPP_ERROR(this->get_logger(), "Trying CameraReConnect...");

            int status = CameraReConnect(h_camera_);
            RCLCPP_ERROR(this->get_logger(), "CameraReConnect status = %d", status);

            std::this_thread::sleep_for(std::chrono::milliseconds(300));

            status = CameraPlay(h_camera_);
            RCLCPP_ERROR(this->get_logger(), "CameraPlay after reconnect status = %d", status);

            if (status != CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(this->get_logger(), "CameraReConnect failed.");
                return false;
            }

            RCLCPP_ERROR(this->get_logger(), "CameraReConnect success.");
            return true;
        }

        void declareParameters()
        {
            rcl_interfaces::msg::ParameterDescriptor param_desc;
            param_desc.integer_range.resize(1);
            param_desc.integer_range[0].step = 1;

            // Exposure time
            param_desc.description = "Exposure time in microseconds";
            // 对于CMOS传感器，其曝光的单位是按照行来计算的
            double exposure_line_time;
            CameraGetExposureLineTime(h_camera_, &exposure_line_time);
            param_desc.integer_range[0].from_value =
                t_capability_.sExposeDesc.uiExposeTimeMin * exposure_line_time;
            param_desc.integer_range[0].to_value =
                t_capability_.sExposeDesc.uiExposeTimeMax * exposure_line_time;
            double exposure_time = this->declare_parameter("exposure_time", 5000, param_desc);
            CameraSetExposureTime(h_camera_, exposure_time);
            RCLCPP_INFO(this->get_logger(), "Exposure time = %f", exposure_time);

            // Analog gain
            param_desc.description = "Analog gain";
            param_desc.integer_range[0].from_value = t_capability_.sExposeDesc.uiAnalogGainMin;
            param_desc.integer_range[0].to_value = t_capability_.sExposeDesc.uiAnalogGainMax;
            int analog_gain;
            CameraGetAnalogGain(h_camera_, &analog_gain);
            analog_gain = this->declare_parameter("analog_gain", analog_gain, param_desc);
            CameraSetAnalogGain(h_camera_, analog_gain);
            RCLCPP_INFO(this->get_logger(), "Analog gain = %d", analog_gain);

            // RGB Gain
            // Get default value
            CameraGetGain(h_camera_, &r_gain_, &g_gain_, &b_gain_);
            // R Gain
            param_desc.integer_range[0].from_value = t_capability_.sRgbGainRange.iRGainMin;
            param_desc.integer_range[0].to_value = t_capability_.sRgbGainRange.iRGainMax;
            r_gain_ = this->declare_parameter("rgb_gain.r", r_gain_, param_desc);
            // G Gain
            param_desc.integer_range[0].from_value = t_capability_.sRgbGainRange.iGGainMin;
            param_desc.integer_range[0].to_value = t_capability_.sRgbGainRange.iGGainMax;
            g_gain_ = this->declare_parameter("rgb_gain.g", g_gain_, param_desc);
            // B Gain
            param_desc.integer_range[0].from_value = t_capability_.sRgbGainRange.iBGainMin;
            param_desc.integer_range[0].to_value = t_capability_.sRgbGainRange.iBGainMax;
            b_gain_ = this->declare_parameter("rgb_gain.b", b_gain_, param_desc);
            // Set gain
            CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
            RCLCPP_INFO(this->get_logger(), "RGB Gain: R = %d", r_gain_);
            RCLCPP_INFO(this->get_logger(), "RGB Gain: G = %d", g_gain_);
            RCLCPP_INFO(this->get_logger(), "RGB Gain: B = %d", b_gain_);

            // Saturation
            param_desc.description = "Saturation";
            param_desc.integer_range[0].from_value = t_capability_.sSaturationRange.iMin;
            param_desc.integer_range[0].to_value = t_capability_.sSaturationRange.iMax;
            int saturation;
            CameraGetSaturation(h_camera_, &saturation);
            saturation = this->declare_parameter("saturation", saturation, param_desc);
            CameraSetSaturation(h_camera_, saturation);
            RCLCPP_INFO(this->get_logger(), "Saturation = %d", saturation);

            // Gamma
            param_desc.integer_range[0].from_value = t_capability_.sGammaRange.iMin;
            param_desc.integer_range[0].to_value = t_capability_.sGammaRange.iMax;
            int gamma;
            CameraGetGamma(h_camera_, &gamma);
            gamma = this->declare_parameter("gamma", gamma, param_desc);
            CameraSetGamma(h_camera_, gamma);
            RCLCPP_INFO(this->get_logger(), "Gamma = %d", gamma);

            // Flip
            flip_image_ = this->declare_parameter("flip_image", false);
        }

        rcl_interfaces::msg::SetParametersResult parametersCallback(
            const std::vector<rclcpp::Parameter>& parameters)
        {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto& param : parameters)
            {
                if (param.get_name() == "exposure_time")
                {
                    int status = CameraSetExposureTime(h_camera_, param.as_int());
                    if (status != CAMERA_STATUS_SUCCESS)
                    {
                        result.successful = false;
                        result.reason = "Failed to set exposure time, status = " + std::to_string(status);
                    }
                }
                else if (param.get_name() == "analog_gain")
                {
                    int status = CameraSetAnalogGain(h_camera_, param.as_int());
                    if (status != CAMERA_STATUS_SUCCESS)
                    {
                        result.successful = false;
                        result.reason = "Failed to set analog gain, status = " + std::to_string(status);
                    }
                }
                else if (param.get_name() == "rgb_gain.r")
                {
                    r_gain_ = param.as_int();
                    int status = CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
                    if (status != CAMERA_STATUS_SUCCESS)
                    {
                        result.successful = false;
                        result.reason = "Failed to set RGB gain, status = " + std::to_string(status);
                    }
                }
                else if (param.get_name() == "rgb_gain.g")
                {
                    g_gain_ = param.as_int();
                    int status = CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
                    if (status != CAMERA_STATUS_SUCCESS)
                    {
                        result.successful = false;
                        result.reason = "Failed to set RGB gain, status = " + std::to_string(status);
                    }
                }
                else if (param.get_name() == "rgb_gain.b")
                {
                    b_gain_ = param.as_int();
                    int status = CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
                    if (status != CAMERA_STATUS_SUCCESS)
                    {
                        result.successful = false;
                        result.reason = "Failed to set RGB gain, status = " + std::to_string(status);
                    }
                }
                else if (param.get_name() == "saturation")
                {
                    int status = CameraSetSaturation(h_camera_, param.as_int());
                    if (status != CAMERA_STATUS_SUCCESS)
                    {
                        result.successful = false;
                        result.reason = "Failed to set saturation, status = " + std::to_string(status);
                    }
                }
                else if (param.get_name() == "gamma")
                {
                    int gamma = param.as_int();
                    int status = CameraSetGamma(h_camera_, gamma);
                    if (status != CAMERA_STATUS_SUCCESS)
                    {
                        result.successful = false;
                        result.reason = "Failed to set Gamma, status = " + std::to_string(status);
                    }
                }
                else if (param.get_name() == "flip_image")
                {
                    flip_image_ = param.as_bool();
                }
                else
                {
                    result.successful = false;
                    result.reason = "Unknown parameter: " + param.get_name();
                }
            }
            return result;
        }

        rclcpp::Time make_camera_stamp(
            uint32_t camera_tick,
            const rclcpp::Time& receive_time)
        {
            constexpr uint64_t kTickNs = 100000ULL; // 0.1 ms
            constexpr uint64_t kUint32Range = 1ULL << 32;

            if (!camera_stamp_initialized_)
            {
                camera_stamp_initialized_ = true;
                last_camera_tick_ = camera_tick;
                first_camera_tick_ = camera_tick;
                first_ros_stamp_ns_ = receive_time.nanoseconds();

                return rclcpp::Time(first_ros_stamp_ns_, receive_time.get_clock_type());
            }

            // 处理uint32时间戳回绕
            if (camera_tick < last_camera_tick_ &&
                static_cast<uint32_t>(last_camera_tick_ - camera_tick) >
                0x80000000U)
            {
                camera_wrap_ticks_ += kUint32Range;
            }

            last_camera_tick_ = camera_tick;

            const uint64_t extended_tick =
                camera_wrap_ticks_ + camera_tick;

            const uint64_t delta_tick =
                extended_tick - first_camera_tick_;

            const int64_t stamp_ns =
                first_ros_stamp_ns_ +
                static_cast<int64_t>(delta_tick * kTickNs);

            return rclcpp::Time(stamp_ns, receive_time.get_clock_type());
        }

        int h_camera_{-1};
        bool camera_initialized_{false};
        std::atomic<bool> running_{true};
        uint8_t* pby_buffer_{nullptr};
        tSdkCameraCapbility t_capability_; // 设备描述信息
        tSdkFrameHead s_frame_info_; // 图像帧头信息

        sensor_msgs::msg::Image image_msg_;

        image_transport::CameraPublisher camera_pub_;

        // RGB Gain
        int r_gain_, g_gain_, b_gain_;

        std::atomic<bool> flip_image_{false};
        bool full_speed_{true};
        int target_fps_{30};
        size_t channels_{3};
        std::string output_encoding_;
        std::string image_topic_;
        std::string frame_id_;
        FrameRateLimiter rate_limiter_;

        std::string camera_name_;
        std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
        sensor_msgs::msg::CameraInfo camera_info_msg_;

        std::thread capture_thread_;

        OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

        //时间同步
        bool camera_stamp_initialized_{false};

        uint32_t last_camera_tick_{0};
        uint64_t camera_wrap_ticks_{0};
        uint64_t first_camera_tick_{0};

        int64_t first_ros_stamp_ns_{0};
    };
} // namespace mindvision_camera

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(mindvision_camera::MVCameraNode)
