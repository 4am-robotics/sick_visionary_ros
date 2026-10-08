from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("remote_device_ip", default_value="192.168.1.10"),
        DeclareLaunchArgument("frame_id", default_value="camera"),
        DeclareLaunchArgument("enable_depth", default_value="true"),
        DeclareLaunchArgument("enable_intensity", default_value="true"),
        DeclareLaunchArgument("enable_statemap", default_value="true"),
        DeclareLaunchArgument("enable_points", default_value="true"),
        DeclareLaunchArgument("desired_frequency", default_value="15.0"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("node_name", default_value="sick_visionary_t_mini"),
    ]

    node = Node(
        package="sick_visionary_ros",
        executable="sick_visionary_t_mini_node",
        name=LaunchConfiguration("node_name"),
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            {
                "remote_device_ip": LaunchConfiguration("remote_device_ip"),
                "frame_id": LaunchConfiguration("frame_id"),
                "enable_depth": LaunchConfiguration("enable_depth"),
                "enable_intensity": LaunchConfiguration("enable_intensity"),
                "enable_statemap": LaunchConfiguration("enable_statemap"),
                "enable_points": LaunchConfiguration("enable_points"),
                "desired_frequency": LaunchConfiguration("desired_frequency"),
            }
        ],
    )

    return LaunchDescription(args + [node])
