"""
Headless launch file — Gazebo без GUI, без rviz, без джойстика.
Используется для тестирования навигации по координатам и полного обхода поля.

Запуск в Docker:
  docker compose run --rm ros2_headless

Тест движения к точке (из другого терминала внутри контейнера):
  ros2 topic pub --once /mission/goto geometry_msgs/Point "{x: -5.0, y: 0.0, z: 0.0}"

Запуск полной миссии:
  ros2 topic pub --once /mission/start std_msgs/Bool "{data: true}"

Остановка:
  ros2 topic pub --once /mission/stop std_msgs/Bool "{data: true}"

Мониторинг:
  ros2 topic echo /mission/status
  ros2 topic echo /odom
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
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

    # ── описание робота ────────────────────────────────────────────────────────
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}],
        output='screen',
    )

    # ── Gazebo headless (server only, без GUI) ─────────────────────────────────
    # -s  = server only (нет рендеринга, нет GUI)
    # -r  = запустить сразу (не ждать кнопку Play)
    # -v 1 = минимальный вывод
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim','-r', world_file, '-v', '1'],
        output='screen',
        additional_env={
            'GZ_SIM_SYSTEM_PLUGIN_PATH': '/opt/ros/jazzy/lib',
            'GZ_HEADLESS_RENDERING': '1',
        },
    )

    # ── спавн робота ───────────────────────────────────────────────────────────
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

    # ── мост ROS2 ↔ Gazebo ────────────────────────────────────────────────────
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_yaml}'],
        output='screen',
    )

    # ── контроллеры ───────────────────────────────────────────────────────────
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

    # ── одометрия ─────────────────────────────────────────────────────────────
    acker_odom = Node(
        package='my_robot',
        executable='acker_odom',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ── row_driver (управляется mission node через /autopilot/enable) ─────────
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

    # ── mission node — запускаем с задержкой после спавна ─────────────────────
    field_mission = Node(
        package='my_robot',
        executable='field_mission',
        parameters=[field_yaml, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ── visual servoing (опционально, нужен для FULL_MISSION) ─────────────────
    vs_node = Node(
        package='visual_multi_crop_row_navigation',
        executable='vs_navigation',
        parameters=[vs_yaml, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        # Сразу: симуляция и описание робота
        robot_state_publisher,
        gazebo,

        # +2 сек: спавн робота и мост (Gazebo должна загрузиться)
        TimerAction(period=2.0, actions=[spawn_robot, ros_gz_bridge, acker_odom]),

        # +5 сек: joint_broad первым (нужен для остальных контроллеров)
        TimerAction(period=5.0, actions=[joint_broad_spawner]),

        # +8 сек: velocity и position контроллеры (после joint_broad)
        TimerAction(period=8.0, actions=[velocity_spawner, steer_spawner]),

        # +11 сек: camera tilt и row_driver
        TimerAction(period=11.0, actions=[camera_tilt_spawner, row_driver]),

        # +14 сек: mission node и VS (всё готово)
        TimerAction(period=14.0, actions=[field_mission, vs_node]),
    ])
