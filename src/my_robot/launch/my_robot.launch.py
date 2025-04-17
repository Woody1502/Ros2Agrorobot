import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Путь к URDF-файлу
    urdf_file = 'robot.urdf'
    package_name = 'my_robot'
    
    # Полный путь к URDF
    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        urdf_file
    )
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    # Проверка существования файла
    if not os.path.exists(urdf_path):
        raise RuntimeError(f"URDF file not found: {urdf_path}")
    
    # Аргумент для выбора RViz config (опционально)
    rviz_config_file = LaunchConfiguration('rviz_config')
    
    return LaunchDescription([
        # Аргумент для RViz config
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                get_package_share_directory(package_name),
                'rviz',
                'view_robot.rviz'
            ),
            description='Path to RViz config file'
        ),
        
        # Запуск узла robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf_path],
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # Запуск RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
        ),
        
        # Запуск joint_state_publisher (для ручного управления суставами)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        )
    ])