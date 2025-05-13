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
    xacro_file = PathJoinSubstitution([pkg_my_robot, 'urdf', 'robot.xacro'])
    world_file = PathJoinSubstitution([pkg_my_robot, 'urdf', 'run.sdf'])  # Измените на ваш мир

    # Аргументы запуска
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    use_ros2_control = LaunchConfiguration('use_ros2_control', default=True)
    
    # Описание робота из xacro
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' use_sim_time:=', use_sim_time, ' use_ros2_control:=', use_ros2_control
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
            'gz_args': ['-r ' , world_file],
            'on_exit_shutdiwn': 'true',
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
    
    acker_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_controller"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

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
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Запуск компонентов
        gazebo,
        robot_state_publisher,
        spawn_entity,
        acker_drive_spawner,
        ros_gz_bridge,
        joint_broad_spawner

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