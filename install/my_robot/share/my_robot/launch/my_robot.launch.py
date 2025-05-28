from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ros_gz_bridge.actions import RosGzBridge
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    sdf_file = os.path.join(
        get_package_share_directory('my_robot'),
        'urdf',
        'run.sdf',
    )

    # 1. Сначала объявляем все аргументы
    declare_bridge_name = DeclareLaunchArgument(
        'bridge_name',
        default_value='ROS2_BRIDGE',
        description='Name of ros_gz_bridge node'
    )

    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value='/home/alexey/ros2_ws/src/my_robot/config/bridge_config.yaml',
        description='YAML config file'
    )

    # 2. Затем создаем действия, которые используют эти аргументы
    bridge = RosGzBridge(
        bridge_name=LaunchConfiguration('bridge_name'),
        config_file=LaunchConfiguration('config_file'),
    )

    gazebo_process = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', sdf_file],
        output='screen'
    )

    # 3. Собираем все в правильном порядке
    return LaunchDescription([
        # Сначала объявляем аргументы
        declare_bridge_name,
        declare_config_file,
        
        # Затем запускаем процессы
        gazebo_process,
        
        # И только потом мост
        bridge,
        
        # Можно добавить задержку если нужно
        # TimerAction(...)
    ])