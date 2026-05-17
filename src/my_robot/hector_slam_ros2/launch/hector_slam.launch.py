"""
hector_slam.launch.py
─────────────────────
ROS2 Humble launch file for hector_slam_ros2.

Usage:
  ros2 launch hector_slam_ros2 hector_slam.launch.py
  ros2 launch hector_slam_ros2 hector_slam.launch.py scan_topic:=/my_lidar/scan
  ros2 launch hector_slam_ros2 hector_slam.launch.py use_rviz:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("hector_slam_ros2")

    # ── Launch arguments ────────────────────────────────────────────────────
    scan_topic_arg = DeclareLaunchArgument(
        "scan_topic", default_value="/scan",
        description="LaserScan topic to subscribe to")

    map_frame_arg = DeclareLaunchArgument(
        "map_frame", default_value="map",
        description="Fixed (world) frame id")

    base_frame_arg = DeclareLaunchArgument(
        "base_frame", default_value="base_link",
        description="Robot base frame id")

    map_resolution_arg = DeclareLaunchArgument(
        "map_resolution", default_value="0.05",
        description="Occupancy grid resolution [m/pixel]")

    map_size_arg = DeclareLaunchArgument(
        "map_size", default_value="4096",
        description="Map size in pixels (square)")

    # ĐÃ SỬA: Thay đổi default_value từ "false" thành "true" để tự động mở RViz
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true",
        description="Launch RViz2 for visualisation")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation time (Gazebo)")

    laser_x_arg = DeclareLaunchArgument(
        "laser_x", default_value="0.0",
        description="Laser x offset relative to base frame")
    laser_y_arg = DeclareLaunchArgument(
        "laser_y", default_value="0.0",
        description="Laser y offset relative to base frame")
    laser_z_arg = DeclareLaunchArgument(
        "laser_z", default_value="0.18",
        description="Laser z offset relative to base frame")
    laser_roll_arg = DeclareLaunchArgument(
        "laser_roll", default_value="0.0",
        description="Laser roll relative to base frame")
    laser_pitch_arg = DeclareLaunchArgument(
        "laser_pitch", default_value="0.0",
        description="Laser pitch relative to base frame")
    laser_yaw_arg = DeclareLaunchArgument(
        "laser_yaw", default_value="0.0",
        description="Laser yaw relative to base frame")

    pub_map_odom_tf_arg = DeclareLaunchArgument(
        "pub_map_odom_tf", default_value="false",
        description="Publish map→odom TF (needs odom source)")

    # ── hector_mapping_node ─────────────────────────────────────────────────
    hector_node = Node(
        package="hector_slam_ros2",
        executable="hector_mapping_node",
        name="hector_mapping",
        output="screen",
        parameters=[{
            # Lifecycle / ROS time
            "use_sim_time":            LaunchConfiguration("use_sim_time"),

            # Frames
            "map_frame":               LaunchConfiguration("map_frame"),
            "base_frame":              LaunchConfiguration("base_frame"),
            "odom_frame":              "odom",

            # Topics
            "scan_topic":              LaunchConfiguration("scan_topic"),

            # Map parameters
            "map_resolution":          LaunchConfiguration("map_resolution"),
            "map_size":                LaunchConfiguration("map_size"),
            "map_start_x":             0.5,
            "map_start_y":             0.5,
            "map_multi_res_levels":    3,
            "map_pub_period":          1.0,

            # Update thresholds
            "update_factor_free":      0.4,
            "update_factor_occupied":  0.9,
            "map_update_dist_thresh":  0.4,
            "map_update_angle_thresh": 0.9,

            # Laser filter
            "laser_min_dist":          0.12,
            "laser_max_dist":          8.0,
            "laser_z_min":             -1.0,
            "laser_z_max":              1.0,

            # TF outputs
            "pub_map_scanmatch_tf":    True,
            "pub_map_odom_tf":         LaunchConfiguration("pub_map_odom_tf"),
        }],
        remappings=[
            # Remap if your robot uses a different topic name:
            # ("/scan", "/my_lidar/scan"),
        ]
    )

    # ── Static TF: base_link → laser (adjust for your robot) ───────────────
    # This transform tells hector_slam where the laser is relative to
    # the robot base.  It is safer to publish this transform from your robot
    # URDF via robot_state_publisher, but the parameterized static transform
    # is provided as a simple fallback.
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_laser_tf",
        arguments=[
            "--x",   LaunchConfiguration("laser_x"),
            "--y",   LaunchConfiguration("laser_y"),
            "--z",   LaunchConfiguration("laser_z"),
            "--roll",  LaunchConfiguration("laser_roll"),
            "--pitch", LaunchConfiguration("laser_pitch"),
            "--yaw",   LaunchConfiguration("laser_yaw"),
            "--frame-id",       LaunchConfiguration("base_frame"),
            "--child-frame-id", "laser",
        ]
    )

    # ── RViz2 (optional) ────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_share, "rviz", "hector_slam.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz"))
    )

    return LaunchDescription([
        # Declare args first
        scan_topic_arg,
        map_frame_arg,
        base_frame_arg,
        map_resolution_arg,
        map_size_arg,
        use_rviz_arg,
        use_sim_time_arg,
        laser_x_arg,
        laser_y_arg,
        laser_z_arg,
        laser_roll_arg,
        laser_pitch_arg,
        laser_yaw_arg,
        pub_map_odom_tf_arg,

        # Log useful startup info
        LogInfo(msg=["Launching Hector SLAM | scan=",
                     LaunchConfiguration("scan_topic"),
                     " | map_frame=", LaunchConfiguration("map_frame")]),

        # Nodes
        static_tf_node,
        hector_node,
        rviz_node,
    ])