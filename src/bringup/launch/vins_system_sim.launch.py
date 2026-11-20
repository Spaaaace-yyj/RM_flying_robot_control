from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vins_config_file = LaunchConfiguration("vins_config_file")
    use_pose_graph = LaunchConfiguration("use_pose_graph")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_vins_config_file_cmd = DeclareLaunchArgument(
        "vins_config_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("vins_estimator"),
                "config",
                "euroc",
                "euroc_config_sim.yaml",
            ]
        ),
        description="VINS 配置文件绝对路径",
    )
    declare_use_pose_graph_cmd = DeclareLaunchArgument(
        "use_pose_graph",
        default_value="true",
        description="是否启动 VINS 回环检测",
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="是否启动 RViz2",
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="实时传感器设为 false，播放 rosbag 设为 true",
    )
    declare_driver_config_file_cmd = DeclareLaunchArgument(
        "driver_config_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("bringup"),
                "config",
                "driver_interface.yaml",
            ]
        ),
        description="driver_interface 参数文件（bringup 包内副本）",
    )
    declare_camera_config_file_cmd = DeclareLaunchArgument(
        "camera_config_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("mindvision_camera"),
                "config",
                "camera_params.yaml",
            ]
        ),
        description="相机参数文件；也可与 driver_config_file 指向同一多节点 YAML",
    )
    declare_camera_full_speed_cmd = DeclareLaunchArgument("camera_full_speed", default_value="")
    declare_camera_target_fps_cmd = DeclareLaunchArgument("camera_target_fps", default_value="")
    declare_camera_output_encoding_cmd = DeclareLaunchArgument(
        "camera_output_encoding", default_value=""
    )
    declare_camera_image_topic_cmd = DeclareLaunchArgument("camera_image_topic", default_value="")
    declare_camera_use_sensor_data_qos_cmd = DeclareLaunchArgument(
        "camera_use_sensor_data_qos", default_value=""
    )
    declare_camera_qos_depth_cmd = DeclareLaunchArgument("camera_qos_depth", default_value="")
    declare_enable_image_bridge_cmd = DeclareLaunchArgument("enable_image_bridge", default_value="")

    vins_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("vins_estimator"),
                    "launch",
                    "vins.launch.py",
                ]
            )
        ),
        launch_arguments={
            "config_file": vins_config_file,
            "use_pose_graph": use_pose_graph,
            "use_rviz": use_rviz,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        [
            declare_vins_config_file_cmd,
            declare_use_pose_graph_cmd,
            declare_use_rviz_cmd,
            declare_use_sim_time_cmd,
            declare_driver_config_file_cmd,
            declare_camera_config_file_cmd,
            declare_camera_full_speed_cmd,
            declare_camera_target_fps_cmd,
            declare_camera_output_encoding_cmd,
            declare_camera_image_topic_cmd,
            declare_camera_use_sensor_data_qos_cmd,
            declare_camera_qos_depth_cmd,
            declare_enable_image_bridge_cmd,
            vins_launch,
        ]
    )
