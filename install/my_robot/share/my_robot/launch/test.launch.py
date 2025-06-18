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
    urdf_file = os.path.join(pkg_my_robot, 'urdf', 'fito.urdf') 
    world_file = PathJoinSubstitution([pkg_my_robot, 'urdf', 'run.sdf'])  # Измените на ваш мир

    # Аргументы запуска
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_ros2_control = LaunchConfiguration('use_ros2_control', default='true')
    
    # Описание робота из xacro
    with open(urdf_file, 'r') as f:
        robot_description = f.read()


    # Узел robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
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
            '-z', '1.5',
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    # Контроллеры
    ros2_control_params=os.path.join(get_package_share_directory('my_robot'),'config','new_controllers.yaml') 
    # acker_spawner=Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=["ackermann_steering_controller", "--param-file",ros2_control_params],
    #         output="screen",
    #     )
    
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
    rviz_config_file = PathJoinSubstitution([pkg_my_robot, 'rviz', 'view_robot.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',  # Укажите правильное устройство джойстика
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
            'use_sim_time': use_sim_time
        }]
    )

    # Ваш узел для управления роботом через джойстик
    joy_control_node = Node(
        package='my_robot',  # Замените на имя вашего пакета
        executable='joy_control',  # Исполняемый файл вашего узла
        name='joy_control',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_linear_speed': 2.0,  # Максимальная линейная скорость
            'max_angular_speed': 1.0   # Максимальная угловая скорость
        }]
    )
    odom_node = Node(
        package='my_robot',  # Замените на имя вашего пакета
        executable='acker_odom',  # Исполняемый файл вашего узла
        name='acker_odom',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_linear_speed': 2.0,  # Максимальная линейная скорость
            'max_angular_speed': 1.0   # Максимальная угловая скорость
        }]
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

#     robot_localization_node = Node(
#     package='robot_localization',
#     executable='ekf_node',
#     name='ekf_node',
#     output='screen',
#     parameters=[os.path.join(get_package_share_directory('my_robot'), 'config/ekf.yaml'), {'use_sim_time': True}]
# )
    qos = LaunchConfiguration('qos')
    parameters={
        'frame_id': 'base_link',
        'odom_frame_id':'odom',
        'use_sim_time': use_sim_time,
        'subscribe_depth': True,
        'subscribe_scan': False,
        'subscribe_rgbd': False,
        'qos_image': qos,
        'qos_imu': qos,
        
        # Оптимизация для 2D
        'Reg/Force3DoF': 'true',
        'Optimizer/GravitySigma': '0',
        
        # Настройки петлевого замыкания
        'RGBD/LoopClosureReextractFeatures': 'false',
        'RGBD/LinearUpdate': '0.1',
        'RGBD/AngularUpdate': '0.2',
        'Mem/RehearsalSimilarity': '0.65',
        
        # Фильтрация данных
        'RGBD/ProximityBySpace': 'true',
        'RGBD/AngularUpdate': '0.2',
        'Grid/RangeMax': '5.0',
        'Kp/MaxDepth': '10.0',
        
        # Производительность
        'Mem/STMSize': '30',
        'Mem/UseOdomFeatures': 'false',
}

    remappings = [
    ('rgb/image', '/camera/depth/pure_image'),
    ('rgb/camera_info', '/camera/depth/camera_info'),
    ('depth/image', '/camera/depth/image_depth'),]
    rtabmap_slam = Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d'])
    rtab_viz=  Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[parameters],
            remappings=remappings)
        # Node(
   
    return LaunchDescription([
     DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),
        # Запуск компонентов
        robot_state_publisher,
        gazebo,
        
       #robot_localization_node,
         steer_spawner,
         rear_drive_spawner,
        spawn_entity,
        ros_gz_bridge,
         joint_broad_spawner,
        rviz_node,
        joy_node,
        joy_control_node,
        #odom_node,
        rtabmap_slam,
        rtab_viz
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