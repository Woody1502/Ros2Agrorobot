from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

pkg_vs_nav = get_package_share_directory('visual_multi_crop_row_navigation')

def generate_launch_description():
    # Пути к файлам
    pkg_my_robot = get_package_share_directory('my_robot')
    urdf_file = os.path.join(pkg_my_robot, 'urdf', 'fito.urdf') 
    world_file = os.path.join(pkg_my_robot, 'urdf', 'garden_rows_world.sdf')

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

    # Запуск Gazebo (GZ_CONFIG_PATH нужен для gz sim)
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file, '-v', '4'],
        output='screen',
        additional_env={'GZ_CONFIG_PATH': '/usr/share/gz'},
    )

    # Spawn робота в Gazebo через gz service (без ros_gz_sim)
    sdf_file = os.path.join(pkg_my_robot, 'urdf', 'fito.sdf')
    spawn_entity = Node(
        package='my_robot',
        executable='spawn_robot',
        name='spawn_robot',
        output='screen',
        parameters=[{
            'sdf_path': sdf_file,
            'robot_name': 'my_robot',
            'x': -12.0,
            'y': 0.5,
            'z': 1.5,
            'world_name': 'garden_rows_world',
        }],
    )

    # Контроллеры
    ros2_control_params=os.path.join(get_package_share_directory('my_robot'),'config','new_controllers.yaml') 
  
    
    rear_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--param-file",ros2_control_params],
    )
    

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
    rviz_config_file = os.path.join(pkg_my_robot, 'rviz', 'autopilot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        additional_env={'QT_QPA_PLATFORM': 'xcb'},
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
    yolo_node = Node(
        package='my_robot',
        executable='rfdetr',
        name='rfdetr',
        output='screen',
    )

    # Visual servoing autopilot node
    vs_params = os.path.join(pkg_vs_nav, 'configs', 'params.yaml')
    vs_navigation_node = Node(
        package='visual_multi_crop_row_navigation',
        executable='vs_navigation',
        name='vs_navigation',
        output='screen',
        parameters=[vs_params, {'use_sim_time': use_sim_time}],
    )

    # Forward drive node: drives wheels at fixed speed when autopilot is active
    row_driver_node = Node(
        package='my_robot',
        executable='row_driver',
        name='row_driver',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'forward_speed': 1.5,
            'publish_rate': 20.0,
        }],
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

    robot_localization_node = Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[os.path.join(get_package_share_directory('my_robot'), 'config/ekf.yaml'), {"use_sim_time": True}],
                remappings=[("odometry/filtered", "odometry/local")],
            )
    
    map_node=Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[os.path.join(get_package_share_directory('my_robot'), 'config/ekf.yaml'), {"use_sim_time": True}],
                remappings=[("odometry/filtered", "odometry/global")],
            )
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

    navsat=Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[os.path.join(get_package_share_directory('my_robot'), 'config/ekf.yaml'), {"use_sim_time": True}],
                remappings=[
                    ("imu/data", "imu/data"),
                    ("gps/fix", "gps/fix"),
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                    ("odometry/filtered", "odometry/global"),
                ],
            )
       
    return LaunchDescription([
     DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),
        # Запуск компонентов
        robot_state_publisher,
        gazebo,
        navsat,
        map_node,
       robot_localization_node,
         steer_spawner,
         rear_drive_spawner,
        spawn_entity,
        ros_gz_bridge,
         joint_broad_spawner,
        rviz_node,
        joy_node,
        joy_control_node,
        odom_node,
        vs_navigation_node,
        row_driver_node,
        #yolo_node
        #rtabmap_slam,
        #rtab_viz
        

    ])