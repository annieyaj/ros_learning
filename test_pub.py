import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import random

class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, '/seven_dim_state', 10)
        self.timer = self.create_timer(1.0, self.publish_state)

    def publish_state(self):
        msg = Float64MultiArray()
        # Randomly generate a 7-dimensional state [x, y, z, ẋ, ẏ, ż, ψ]
        msg.data = [random.uniform(0, 10) for _ in range(7)]
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
