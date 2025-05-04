import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class StateFilterNode(Node):
    def __init__(self):
        super().__init__('state_filter_node')
        self.sub = self.create_subscription(
            Float64MultiArray, '/state7d', self.listener_callback, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/state6d', 10)

    def listener_callback(self, msg):
        # Extract first 6 elements (x, y, z, dx, dy, dz)
        filtered_msg = Float64MultiArray()
        filtered_msg.data = msg.data[:6]
        self.pub.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StateFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()