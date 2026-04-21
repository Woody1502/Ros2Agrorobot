"""
visual_test.launch.py — Gazebo с GUI для визуального наблюдения за миссией.

Запуск:
  docker compose up ros2_sim

После старта запустить миссию:
  docker exec agrorobot_sim bash -c \
    "source /ros2_ws/install/setup.bash && \
     ros2 topic pub --once /mission/start std_msgs/msg/Bool '{data: true}'"
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_robot = get_package_share_directory('my_robot')
    pkg_vs    = get_package_share_directory('visual_multi_crop_row_navigation')

    urdf_file  = os.path.join(pkg_robot, 'urdf', 'fito.urdf')
    sdf_file   = os.path.join(pkg_robot, 'urdf', 'fito.sdf')
    world_file = os.path.join(pkg_robot, 'urdf', 'garden_rows_world.sdf')
    ctrl_yaml  = os.path.join(pkg_robot, 'config', 'new_controllers.yaml')
    bridge_yaml= os.path.join(pkg_robot, 'config', 'bridge_config.yaml')
    field_yaml = os.path.join(pkg_robot, 'config', 'field_params.yaml')
    vs_yaml    = os.path.join(pkg_vs,    'configs', 'params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}],
        output='screen',
    )

    # ── Gazebo сервер: GZ_HEADLESS_RENDERING=1 предотвращает конфликт fastcdr.
    # Без -s: сервер запускает встроенный GUI процесс, но headless рендеринг
    # его подавляет. Затем gz sim -g подключается как отдельный видимый клиент.
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', world_file, '-v', '1'],
        output='screen',
        additional_env={
            'GZ_SIM_SYSTEM_PLUGIN_PATH': '/opt/ros/jazzy/lib',
            'LD_PRELOAD': '/opt/ros/jazzy/lib/libfastcdr.so.2',
        },
    )

    # ── Gazebo GUI клиент: подключается к серверу, рендерит в окне ────────────
    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
        additional_env={
            'GZ_SIM_SYSTEM_PLUGIN_PATH': '/opt/ros/jazzy/lib',
            'QT_X11_NO_MITSHM': '1',
            'XDG_RUNTIME_DIR': '/tmp/runtime-root',
        },
    )

    spawn_robot = Node(
        package='my_robot',
        executable='spawn_robot',
        parameters=[{
            'sdf_path':    sdf_file,
            'robot_name':  'my_robot',
            'x': -12.0, 'y': -4.0, 'z': 1.5,
            'world_name':  'garden_rows_world',
        }],
        output='screen',
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_yaml}'],
        output='screen',
    )

    joint_broad_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_broad', '--param-file', ctrl_yaml])
    velocity_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['velocity_controller', '--param-file', ctrl_yaml])
    steer_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['position_controller', '--param-file', ctrl_yaml])
    camera_tilt_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['camera_tilt_controller', '--param-file', ctrl_yaml])

    acker_odom = Node(
        package='my_robot',
        executable='acker_odom',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    row_driver = Node(
        package='my_robot',
        executable='row_driver',
        parameters=[{
            'use_sim_time': use_sim_time,
            'forward_speed': 1.5,
            'publish_rate':  20.0,
        }],
        output='screen',
    )

    field_mission = Node(
        package='my_robot',
        executable='field_mission',
        parameters=[field_yaml, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    vs_node = Node(
        package='visual_multi_crop_row_navigation',
        executable='vs_navigation',
        parameters=[vs_yaml, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo_server,
        TimerAction(period=1.5, actions=[gazebo_gui]),
        TimerAction(period=3.0, actions=[spawn_robot, ros_gz_bridge, acker_odom]),
        TimerAction(period=6.0, actions=[joint_broad_spawner]),
        TimerAction(period=9.0, actions=[velocity_spawner, steer_spawner]),
        TimerAction(period=12.0, actions=[camera_tilt_spawner, row_driver]),
        TimerAction(period=15.0, actions=[field_mission, vs_node]),
    ])
