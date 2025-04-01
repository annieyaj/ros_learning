import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')

        # Publisher to send velocity commands
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Timer to call the control loop at regular intervals (0.1s = 10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Time tracking for control function
        self.start_time = self.get_clock().now()

    def control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9  # Convert to seconds

        # Define your custom control function here
        # Example: circular motion
        linear = 2.0
        angular = 1.0 * math.sin(elapsed)  # angular speed varies with time

        # Construct and publish Twist message
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)

        self.get_logger().info(f'Sent cmd_vel: linear={linear:.2f}, angular={angular:.2f}')

def main():
    rclpy.init()
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
