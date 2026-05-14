from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. Khai báo đường dẫn đến gói và file params
    pkg_share = get_package_share_directory('slam_gmapping')
    default_params_path = os.path.join(pkg_share, 'params', 'slam_gmapping.yaml')

    # 2. Khai báo các đối số (Launch Arguments)
    # Cho phép thay đổi file params hoặc dùng sim_time từ terminal
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_path,
        description='Full path to the ROS2 parameters file to use')

    # 3. Định nghĩa Node slam_gmapping
    start_gmapping_node = Node(
        package='slam_gmapping',
        executable='slam_gmapping',
        name='slam_gmapping',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        # Remap topic nếu cần (ví dụ topic laser của bạn tên là /lidar thay vì /scan)
        # remappings=[('/scan', '/lidar')] 
    )

    # 4. Trả về đối tượng LaunchDescription
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(start_gmapping_node)

    return ld