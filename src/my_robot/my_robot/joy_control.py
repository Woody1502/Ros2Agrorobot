#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

class JoyControlNode(Node):
    def __init__(self):
        super().__init__('joy_control_node')
        
        # Параметры
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.0)
        
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
        # Подписка на джойстик
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        
        # Публикаторы для контроллеров
        self.velocity_pub = self.create_publisher(
            Float64MultiArray,
            '/velocity_controller/commands',
            10)
            
        self.position_pub = self.create_publisher(
            Float64MultiArray,
            '/position_controller/commands',
            10)
            
        self.get_logger().info("Joy control node has been started")

    def joy_callback(self, msg):
        try:
            # Чтение значений с джойстика
            left_trigger = msg.axes[5]  # Левый триггер (задний ход)
            right_trigger = msg.axes[2]  # Правый триггер (передний ход)
            left_stick_x = msg.axes[0]   # Левый стик (повороты)
            
            # Вычисление линейной скорости (передний/задний ход)
            forward_speed = right_trigger * self.max_linear_speed
            backward_speed = left_trigger * self.max_linear_speed
            linear_speed = forward_speed - backward_speed
            
            # Вычисление угловой скорости (повороты)
            angular_speed = left_stick_x * self.max_angular_speed
            
            # Управление колесами (velocity controller)
            wheel_commands = Float64MultiArray()
            
            # Для дифференциального привода:
            # left_speed = linear_speed - angular_speed
            # right_speed = linear_speed + angular_speed
            # wheel_commands.data = [right_speed, left_speed, right_speed, left_speed]
            
            # Для простоты будем считать, что все колеса управляются одинаково
            # и повороты осуществляются механизмом wheeling_mech
            wheel_commands.data = [linear_speed, linear_speed, linear_speed, linear_speed]
            self.velocity_pub.publish(wheel_commands)
            
            # Управление механизмом поворота (position controller)
            position_command = Float64MultiArray()
            # Здесь нужно преобразовать angular_speed в угол поворота механизма
            # Это зависит от вашей конкретной реализации
            position_command.data = [angular_speed]
            #self.position_pub.publish(position_command)
            
        except Exception as e:
            self.get_logger().error(f"Error in joy_callback: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = JoyControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()