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

    driver_config_file = LaunchConfiguration("driver_config_file")

    mindvision_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("mindvision_camera"),
                "launch",
                "mv_launch.py",
            ])
        ),
    )

    px4_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("bringup"),
                "launch",
                "px4_xrce_agent.launch.py",
            ])
        ),
    )

    driver_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("driver_interface"),
                "launch",
                "driver_interface.launch.py",
            ])
        ),
        launch_arguments={
            "config_file": driver_config_file,
        }.items(),
    )

    vins_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("vins_estimator"),
                "launch",
                "vins.launch.py",
            ])
        ),
        launch_arguments={
            "config_file": vins_config_file,
            "use_pose_graph": use_pose_graph,
            "use_rviz": use_rviz,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "vins_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("vins_estimator"),
                "config",
                "euroc",
                "euroc_config_px4.yaml",
            ]),
            description="VINS 配置文件绝对路径",
        ),

        DeclareLaunchArgument(
            "use_pose_graph",
            default_value="true                                                                      ",
            description="是否启动 VINS 回环检测",
        ),

        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="是否启动 RViz2",
        ),

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="实时传感器设为 false，播放 rosbag 设为 true",
        ),

        DeclareLaunchArgument(
            "driver_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("bringup"),
                "config",
                "driver_interface.yaml",
            ]),
            description="driver_interface 参数文件（bringup 包内副本）",
        ),

        mindvision_camera_launch,
        px4_system_launch,
        driver_interface_launch,
        vins_launch,
    ])