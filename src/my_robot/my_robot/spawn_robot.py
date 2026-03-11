#!/usr/bin/env python3
"""
Spawns the robot into a running Gazebo simulation
using `gz service` (no ros_gz_sim needed).
"""
import subprocess
import time
import sys
import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class SpawnRobotNode(Node):
    def __init__(self):
        super().__init__('spawn_robot')
        self.declare_parameter('sdf_path', '')
        self.declare_parameter('robot_name', 'my_robot')
        self.declare_parameter('x', -12.0)
        self.declare_parameter('y', 0.5)
        self.declare_parameter('z', 1.5)
        self.declare_parameter('world_name', 'garden_rows_world')
        self.declare_parameter('retry_delay', 2.0)
        self.declare_parameter('max_retries', 30)

        sdf_path = self.get_parameter('sdf_path').value
        if not sdf_path:
            pkg = get_package_share_directory('my_robot')
            sdf_path = os.path.join(pkg, 'urdf', 'fito.sdf')

        self.sdf_path = sdf_path
        self.robot_name = self.get_parameter('robot_name').value
        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.z = self.get_parameter('z').value
        self.world_name = self.get_parameter('world_name').value
        self.retry_delay = self.get_parameter('retry_delay').value
        self.max_retries = self.get_parameter('max_retries').value

        self.timer = self.create_timer(0.5, self.try_spawn)
        self.attempts = 0
        self.get_logger().info(f'Waiting for Gazebo to start...')

    def try_spawn(self):
        self.timer.cancel()
        service = f'/world/{self.world_name}/create'
        req = (
            f'sdf_filename: "{self.sdf_path}" '
            f'name: "{self.robot_name}" '
            f'pose: {{position: {{x: {self.x}, y: {self.y}, z: {self.z}}}}}'
        )

        for attempt in range(self.max_retries):
            self.get_logger().info(
                f'Spawning robot (attempt {attempt + 1}/{self.max_retries})...')
            result = subprocess.run(
                ['gz', 'service', '-s', service,
                 '--reqtype', 'gz.msgs.EntityFactory',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '3000',
                 '--req', req],
                capture_output=True, text=True)

            if result.returncode == 0 and 'data: true' in result.stdout:
                self.get_logger().info('Robot spawned successfully!')
                return
            else:
                err = result.stderr.strip() or result.stdout.strip()
                self.get_logger().warn(f'Attempt {attempt + 1} failed: {err}')
                time.sleep(self.retry_delay)

        self.get_logger().error(
            f'Failed to spawn robot after {self.max_retries} attempts')


def main(args=None):
    rclpy.init(args=args)
    node = SpawnRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
