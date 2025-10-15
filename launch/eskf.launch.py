from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments (topics + params)
    args = [
        DeclareLaunchArgument("node_name", default_value="eskf"),
        DeclareLaunchArgument("namespace", default_value=""),

        DeclareLaunchArgument("imu_topic", default_value="/livox/imu"),
        DeclareLaunchArgument("mag_topic", default_value="/mavros/imu/mag"),
        DeclareLaunchArgument("vision_topic", default_value="/svo/pose"),
        DeclareLaunchArgument("gps_topic", default_value="/mavros/global_position/local"),
        DeclareLaunchArgument("rangefinder_topic", default_value="/mavros/distance_sensor/hrlv_ez4_pub"),

        DeclareLaunchArgument("fusion_mask", default_value="7"),
        DeclareLaunchArgument("publish_rate", default_value="10"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]

    node = Node(
        package="eskf",
        # If your executable target is named differently, change this:
        executable="eskf_node",
        name=LaunchConfiguration("node_name"),
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[{
            "fusion_mask": LaunchConfiguration("fusion_mask"),
            "publish_rate": LaunchConfiguration("publish_rate"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
        remappings=[
            # Use relative names in your code (e.g., "imu") so these work cleanly
            ("imu", LaunchConfiguration("imu_topic")),
            ("vision", LaunchConfiguration("vision_topic")),
            ("gps", LaunchConfiguration("gps_topic")),
            ("mag", LaunchConfiguration("mag_topic")),
            ("rangefinder", LaunchConfiguration("rangefinder_topic")),
        ],
    )

    return LaunchDescription(args + [node])
