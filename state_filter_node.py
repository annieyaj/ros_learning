import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class StateFilterNode(Node):
    def __init__(self):
        super().__init__('state_filter_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/six_dim_state', 10)
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/seven_dim_state',
            self.callback,
            10
        )
        self.get_logger().info('StateFilterNode started.')

    def callback(self, msg):
        # Remove the yaw (ψ), which is the 7th element in the state
        filtered_msg = Float64MultiArray()
        filtered_msg.data = msg.data[:6]  # Keep only [x, y, z, ẋ, ẏ, ż]
        self.publisher_.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StateFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
