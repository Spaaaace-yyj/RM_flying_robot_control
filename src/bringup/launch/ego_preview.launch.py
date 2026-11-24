"""Plan and display in odom using D435 and VINS, including from a rosbag."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    config = PathJoinSubstitution([FindPackageShare("bringup"), "config", "ego_preview.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("publish_camera_bridge", default_value="true"),
        DeclareLaunchArgument("images_rotated_180", default_value="true"),
        Node(package="bringup", executable="ego_depth_inputs.py", output="screen", parameters=[
            {"use_sim_time": sim_time, "images_rotated_180": ParameterValue(
                LaunchConfiguration("images_rotated_180"), value_type=bool),
             "publish_camera_bridge": ParameterValue(
                LaunchConfiguration("publish_camera_bridge"), value_type=bool)}]),
        Node(package="ego_planner", executable="ego_planner_node", name="planner",
             namespace="ego_preview", output="screen", parameters=[config, {"use_sim_time": sim_time}],
             remappings=[
                 ("odom_world", "/imu_propagate"), ("grid_map/odom", "/imu_propagate"),
                 ("grid_map/depth", "/ego_input/depth"), ("grid_map/pose", "/ego_input/camera_pose"),
                 ("grid_map/camera_info", "/ego_input/camera_info"),
                 ("/move_base_simple/goal", "/ego_preview/goal"),
             ]),
    ])
