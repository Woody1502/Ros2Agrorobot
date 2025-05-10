import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.xacro')
   
    gazebo_world = os.path.join(pkg_path, 'urdf', 'run.sdf')
    controllers_file = os.path.join(pkg_path, 'config', 'controllers.yaml')

    # Генерация URDF из XACRO
    robot_description = Command(
        ['xacro ', xacro_file, ' use_gazebo:=true']
    )

    # Запуск Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': gazebo_world}.items()
    )

    # Запуск робота в Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    # Узлы для ROS 2 Control
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controllers_file],
        output='screen'
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen'
    )

    steering_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['steering_controller'],
        output='screen'
    )



    return LaunchDescription([
        gazebo,
        spawn_entity,
        controller_manager,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[diff_drive_controller, steering_controller],
            )
        )
        
    ])