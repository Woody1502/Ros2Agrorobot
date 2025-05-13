from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, Command
import os
from ament_index_python.packages import get_package_share_path
from launch.substitutions import PathJoinSubstitution, FindExecutable

def generate_launch_description():
    # Аргументы запуска
    xacro_file = DeclareLaunchArgument(
        'xacro_file',
        default_value=os.path.join(get_package_share_path('my_robot'),'urdf','robot.xacro'),
        description='Path to XACRO file to spawn'
    )
    
    world_file = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
        FindPackageShare('my_robot'),
        'urdf',
        'run.sdf'
    ]),
        description='Path to Gazebo world file'
    )
    
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Запуск Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
            )
        ),
        launch_arguments={
            'gz_args': PathJoinSubstitution([
                FindPackageShare('my_robot'),
                'urdf',
                'run.sdf'
            ])
        }.items()
    )
    
    # Конвертация XACRO в SDF
    robot_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': Command([
                FindExecutable(name='xacro'),' ',
                PathJoinSubstitution([
                    FindPackageShare('my_robot'),
                    'urdf',
                    'robot.xacro'
                ]),
                ' use_sim_time:=', LaunchConfiguration('use_sim_time')
            ])
        }]
    )
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},  # Критически важно!
        ],
        output='screen'
    )
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen')
    # Спавн модели в Gazebo
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'my_robot',
            '-allow_renaming', 'true'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        controller_manager,
        xacro_file,
        world_file,
        use_sim_time,
        gazebo,
        robot_description,
        spawn,joint_state_publisher
    ])