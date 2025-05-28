from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Пути к файлам
    pkg_my_robot = get_package_share_directory('my_robot')
    xacro_file = PathJoinSubstitution([pkg_my_robot, 'urdf', 'fito.xacro'])
    world_file = PathJoinSubstitution([pkg_my_robot, 'urdf', 'run.sdf'])  # Измените на ваш мир

    # Аргументы запуска
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_ros2_control = LaunchConfiguration('use_ros2_control', default='true')
    
    # Описание робота из xacro
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' sim_mode:=', use_sim_time, ' use_ros2_control:=', use_ros2_control
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    # Узел robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }]
    )

    # Запуск Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
            )
        ),
        launch_arguments={
            'gz_args': ['-r ' , world_file, ' -v' ' 4 '],
            'on_exit_shutdown': 'true',
        }.items()
    )

    # Spawn робота в Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    # Контроллеры
    ros2_control_params=PathJoinSubstitution([pkg_my_robot, 'config', 'controllers.yaml']) 
    rear_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--param-file",ros2_control_params],
    )
    
    # front_stearing_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["front_steering_controller", "--param-file",ros2_control_params],
    # )
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--param-file",ros2_control_params],
    )

    steer_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--param-file",ros2_control_params],
    )
    # vel_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["forward_velocity_controller", "--param-file",ros2_control_params],
    # )
    # pos_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["forward_position_controller", "--param-file",ros2_control_params],
    # )
    # joint_rear_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["front_wheels_velocity_controller"],
    # )

    bridge_params=os.path.join(get_package_share_directory('my_robot'),'config','bridge_config.yaml')
    ros_gz_bridge=Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}'
        ]
    )
    # Загрузка контроллеров (после спавна робота)
    #load_joint_state_broadcaster = ExecuteProcess(
     #   cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
     #   output='screen'
    #)
    #load_diff_drive_controller = ExecuteProcess(
    #    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_controller'],
    #    output='screen'
    #
    return LaunchDescription([
     
        # Запуск компонентов
        robot_state_publisher,
        gazebo,
        steer_spawner,
        rear_drive_spawner,
        spawn_entity,
        ros_gz_bridge,
        joint_broad_spawner,
        #rear_drive_spawner,
        
        #front_stearing_spawner,
      
        # Загрузка контроллеров после спавна робота
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=spawn_entity,
        #         on_exit=[load_joint_state_broadcaster] # type: ignore
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_broadcaster,
        #         on_exit=[load_diff_drive_controller] # type: ignore
        #     )
        # )
    ])