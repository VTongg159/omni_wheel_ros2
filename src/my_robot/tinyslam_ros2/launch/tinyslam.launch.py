"""
tinyslam.launch.py

TF tree sau khi launch:
  map  ──(tinyslam_node)──▶  odom
  odom ──(static_tf)──────▶  base_link
  base_link ──(static_tf)──▶ laser
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("tinyslam_ros2")

    # ----------------------------------------------------------------
    # Launch arguments
    # ----------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([pkg_share, "config", "tinyslam_params.yaml"]),
        description="Path to the ROS 2 parameters YAML file",
    )
    scan_topic_arg = DeclareLaunchArgument(
        "scan_topic",
        default_value="scan",
        description="Input laser scan topic",
    )
    map_frame_arg = DeclareLaunchArgument(
        "map_frame",
        default_value="map",
        description="Global map frame",
    )
    odom_frame_arg = DeclareLaunchArgument(       # ← THÊM
        "odom_frame",
        default_value="odom",
        description="Odometry frame (intermediate between map and base_link)",
    )
    base_frame_arg = DeclareLaunchArgument(
        "base_frame",
        default_value="base_link",
        description="Robot base frame",
    )
    laser_frame_arg = DeclareLaunchArgument(
        "laser_frame",
        default_value="laser",
        description="Laser sensor frame",
    )

    # ----------------------------------------------------------------
    # TinySLAM node — publishes TF: map → odom
    # ----------------------------------------------------------------
    tinyslam_node = Node(
        package="tinyslam_ros2",
        executable="tinyslam_node",
        name="tinyslam_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "use_sim_time":  LaunchConfiguration("use_sim_time"),
                "map_frame":     LaunchConfiguration("map_frame"),
                "odom_frame":    LaunchConfiguration("odom_frame"),   # ← THÊM
                "base_frame":    LaunchConfiguration("base_frame"),
                "laser_frame":   LaunchConfiguration("laser_frame"),
            },
        ],
        remappings=[
            ("scan", LaunchConfiguration("scan_topic")),
        ],
    )

    # ----------------------------------------------------------------
    # FIX: static TF odom → base_link
    # Nếu bạn có odometry node thật (wheel encoder, IMU…),
    # XÓA node này và để odometry node publish TF odom→base_link.
    # ----------------------------------------------------------------
    odom_to_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_to_base_static_tf",
        output="screen",
        arguments=[
            "0.0", "0.0", "0.0",   # x y z
            "0.0", "0.0", "0.0",   # roll pitch yaw
            LaunchConfiguration("odom_frame"),   # parent
            LaunchConfiguration("base_frame"),   # child
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # ----------------------------------------------------------------
    # static TF base_link → laser  (giữ nguyên từ file gốc)
    # ----------------------------------------------------------------
    base_to_laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_laser_static_tf",
        output="screen",
        arguments=[
            "0.0", "0.0", "0.0",
            "0.0", "0.0", "0.0",
            LaunchConfiguration("base_frame"),
            LaunchConfiguration("laser_frame"),
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # ----------------------------------------------------------------
    # RViz2
    # ----------------------------------------------------------------
    rviz_config = PathJoinSubstitution([pkg_share, "config", "tinyslam.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        scan_topic_arg,
        map_frame_arg,
        odom_frame_arg,      # ← THÊM
        base_frame_arg,
        laser_frame_arg,
        tinyslam_node,
        odom_to_base_tf,     # ← THÊM
        base_to_laser_tf,
        rviz_node,
    ])