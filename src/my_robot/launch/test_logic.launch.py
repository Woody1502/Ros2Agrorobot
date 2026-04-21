"""
test_logic.launch.py — тест навигационной логики БЕЗ Gazebo.

fake_robot_node эмулирует joint_states → acker_odom считает odom →
field_mission_node управляет поведением.

Запуск локально (без Docker):
  source /opt/ros/jazzy/setup.bash
  colcon build --symlink-install
  source install/setup.bash
  ros2 launch my_robot test_logic.launch.py

Запуск в Docker (headless контейнер):
  docker compose run --rm ros2_headless bash -c \
    "source install/setup.bash && ros2 launch my_robot test_logic.launch.py"

Тест GOTO:
  ros2 topic pub --once /mission/goto geometry_msgs/msg/Point \
    '{x: -5.0, y: 0.0, z: 0.0}'

Запуск полной миссии:
  ros2 topic pub --once /mission/start std_msgs/msg/Bool '{data: true}'

Мониторинг:
  ros2 topic echo /mission/status
  ros2 topic echo /odom --field pose.pose.position
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_robot = get_package_share_directory('my_robot')
    field_yaml = os.path.join(pkg_robot, 'config', 'field_params.yaml')

    # ── fake robot (эмулятор joint_states, нет Gazebo) ────────────────────────
    fake_robot = Node(
        package='my_robot',
        executable='fake_robot',
        parameters=[{
            'speed_multiplier': 3.0,   # 3x быстрее реального для быстрого теста
            'publish_rate': 50.0,
            'use_sim_time': False,
        }],
        output='screen',
    )

    # ── одометрия (стандартная нода) ──────────────────────────────────────────
    acker_odom = Node(
        package='my_robot',
        executable='acker_odom',
        parameters=[{
            'use_sim_time': False,
            'wheel_radius': 0.37,
            'wheel_base': 2.11,
        }],
        output='screen',
    )

    # ── row_driver (управляется mission node) ─────────────────────────────────
    row_driver = Node(
        package='my_robot',
        executable='row_driver',
        parameters=[{
            'use_sim_time': False,
            'forward_speed': 1.5,
            'publish_rate': 20.0,
        }],
        output='screen',
    )

    # ── mission node ──────────────────────────────────────────────────────────
    field_mission = Node(
        package='my_robot',
        executable='field_mission',
        parameters=[field_yaml, {'use_sim_time': False}],
        output='screen',
    )

    return LaunchDescription([
        fake_robot,
        acker_odom,
        row_driver,
        field_mission,
    ])
