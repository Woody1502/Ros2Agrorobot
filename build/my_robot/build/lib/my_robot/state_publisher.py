import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState

class StaticTractorPublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')
        
        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        
        # Параметры для статичного трактора
        self.joint_state = JointState()
        self.joint_state.name = [
            'world_to_base',
            'joint_f_r', 'joint_f_r_w', 
            'joint_f_l', 'joint_f_l_w',
            'joint_l_w', 'joint_r_w'
        ]
        
        # Все суставы в нулевом положении (кроме колес, если нужно)
        self.joint_state.position = [0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Публикуем состояние суставов с частотой 10 Гц
        self.timer = self.create_timer(0.1, self.publish_states)
        
    def publish_states(self):
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_pub.publish(self.joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = StaticTractorPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()