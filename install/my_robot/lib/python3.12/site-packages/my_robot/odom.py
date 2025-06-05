#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
from rclpy.time import Time

class JoyControlNode(Node):
    def __init__(self):
        super().__init__('joy_control_node')
        
        # Параметры
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_steering_angle', 0.4)  # Жестко установлено 0.4 радиана как в URDF
        self.declare_parameter('wheel_base', 1.863)  # Должно соответствовать URDF (расстояние между осями)
        
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_steering_angle = min(0.4, self.get_parameter('max_steering_angle').value)  # Не больше 0.4
        self.wheel_base = self.get_parameter('wheel_base').value
        
        # Переменные для одометрии
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()
        self.last_steering_angle = 0.0
        self.last_linear_speed = 0.0
        
        # Публикаторы
        self.velocity_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.steering_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Подписка на джойстик
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.get_logger().info(f"Joy control node started with max steering angle: {self.max_steering_angle} rad")

    def joy_callback(self, msg):
        try:
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            
            # Чтение значений с джойстика
            left_trigger = (1.0 - msg.axes[2]) / 2.0  # Преобразуем [-1,1] в [0,1]
            right_trigger = (1.0 - msg.axes[5]) / 2.0
            left_stick_x = msg.axes[0]
            
            # Вычисление линейной скорости с ограничением
            forward_speed = right_trigger * self.max_linear_speed
            backward_speed = left_trigger * self.max_linear_speed
            linear_speed = forward_speed - backward_speed
            
            # Вычисление угла поворота с жестким ограничением ±0.4 рад
            steering_angle = max(-0.4, min(0.4, left_stick_x * self.max_steering_angle))
            
            self.last_linear_speed = linear_speed
            self.last_steering_angle = steering_angle
            
            # Публикация команд управления
            # Для 4 колес (2 передних управляемых, 2 задних ведущих)
            wheel_commands = Float64MultiArray()
            wheel_commands.data = [linear_speed] * 4  # Все колеса получают одинаковую скорость
            self.velocity_pub.publish(wheel_commands)
            
            # Публикация угла поворота передних колес
            steering_command = Float64MultiArray()
            steering_command.data = [steering_angle]  # Для 2 передних колес
            self.steering_pub.publish(steering_command)
            
            # Обновление одометрии (только если робот движется)
            if dt > 0 and abs(linear_speed) > 0.01:
                if abs(steering_angle) > 0.01:  # Поворот
                    turn_radius = self.wheel_base / math.tan(steering_angle)
                    angular_velocity = linear_speed / turn_radius
                    delta_th = angular_velocity * dt
                    delta_x = turn_radius * (math.sin(self.th + delta_th) - math.sin(self.th))
                    delta_y = -turn_radius * (math.cos(self.th + delta_th) - math.cos(self.th))
                else:  # Движение прямо
                    delta_th = 0.0
                    delta_x = linear_speed * math.cos(self.th) * dt
                    delta_y = linear_speed * math.sin(self.th) * dt
                
                self.x -= delta_x
                self.y -= delta_y
                self.th += delta_th
                
                # Нормализация угла
                self.th = math.atan2(math.sin(self.th), math.cos(self.th))
                
                # Публикация одометрии
                self.publish_odometry(current_time)
            
            self.last_time = current_time
            
        except Exception as e:
            self.get_logger().error(f"Error in joy_callback: {str(e)}")

    def publish_odometry(self, current_time):
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th / 2)
        odom.pose.pose.orientation.w = math.cos(self.th / 2)
        
        odom.twist.twist.linear.x = self.last_linear_speed
        odom.twist.twist.angular.z = (self.last_linear_speed * math.tan(self.last_steering_angle) / 
                                     self.wheel_base if abs(self.last_steering_angle) > 0.01 else 0.0)
        
        self.odom_pub.publish(odom)
        
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.th / 2)
        t.transform.rotation.w = math.cos(self.th / 2)
        self.tf_broadcaster.sendTransform(t)

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