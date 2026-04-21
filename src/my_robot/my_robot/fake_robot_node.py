#!/usr/bin/env python3
"""
Fake Robot Node — эмулятор физики робота без Gazebo.

Имитирует joint_states на основе команд velocity_controller и position_controller.
acker_odom подхватывает joint_states и считает одометрию.

Позволяет тестировать логику mission node в реальном времени без симулятора.

Параметры:
  speed_multiplier  — во сколько раз ускорить робота (по умолчанию 3.0)
  publish_rate      — Гц публикации joint_states (по умолчанию 50)
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class FakeRobotNode(Node):

    JOINT_NAMES = [
        'back_left_base_to_back_left_wheel',
        'back_right_base_to_back_right_wheel',
        'base_link_to_wheeling_mech',
        'front_left_base_to_front_left_wheel',
        'front_right_base_to_front_right_wheel',
        'front_wheels_base_to_depth_camera',
    ]

    def __init__(self):
        super().__init__('fake_robot')

        self.declare_parameter('speed_multiplier', 3.0)
        self.declare_parameter('publish_rate', 50.0)

        self.speed_mult  = self.get_parameter('speed_multiplier').value
        self.pub_rate    = self.get_parameter('publish_rate').value

        # Текущее состояние
        self.wheel_vel     = 0.0   # рад/с (одинаково для всех колёс)
        self.steer_pos     = 0.0   # рад (позиция рулевого шарнира)
        self.steer_vel     = 0.0   # рад/с (движение руля, всегда 0 в fake)
        self.camera_pos    = 0.5   # рад (угол камеры, фиксирован)

        # Накопленная позиция рулевого шарнира
        self._steer_accumulated = 0.0

        # Подписки
        self.create_subscription(
            Float64MultiArray, '/velocity_controller/commands',
            self._vel_cb, 10)
        self.create_subscription(
            Float64MultiArray, '/position_controller/commands',
            self._steer_cb, 10)

        # Публикация joint_states
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(1.0 / self.pub_rate, self._publish)

        self.get_logger().info(
            f'[FAKE_ROBOT] Ready. speed_multiplier={self.speed_mult}x, '
            f'rate={self.pub_rate}Hz')
        self.get_logger().info(
            '[FAKE_ROBOT] Emulating joint_states — acker_odom will compute odometry.')

    def _vel_cb(self, msg: Float64MultiArray):
        if msg.data:
            # Берём среднее всех 4 колёс (обычно все одинаковы)
            self.wheel_vel = sum(msg.data) / len(msg.data) * self.speed_mult

    def _steer_cb(self, msg: Float64MultiArray):
        if msg.data:
            # position_controller командует позицию
            # В acker_odom: steering_angle = position[i] / 2
            # Поэтому чтобы получить steering_angle = cmd[0], нужно position = cmd[0] * 2
            self._steer_accumulated = msg.data[0] * 2.0
            self.steer_pos = self._steer_accumulated

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.JOINT_NAMES

        # Порядок: back_left, back_right, wheeling_mech, front_left, front_right, camera
        msg.position = [
            0.0,                    # back_left (не используется для odom)
            0.0,                    # back_right
            self.steer_pos,         # wheeling_mech (рулевой шарнир)
            0.0,                    # front_left
            0.0,                    # front_right
            self.camera_pos,        # camera tilt
        ]
        msg.velocity = [
            self.wheel_vel,         # back_left — используется в acker_odom
            self.wheel_vel,         # back_right — используется в acker_odom
            self.steer_vel,         # wheeling_mech velocity
            self.wheel_vel,         # front_left
            self.wheel_vel,         # front_right
            0.0,                    # camera
        ]
        msg.effort = [0.0] * 6

        self.js_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
