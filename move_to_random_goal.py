import rclpy
import random
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt

class MoveToGoal(Node):
    def __init__(self):
        super().__init__('move_to_goal')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(10.0, self.update_goal)
        self.pose = None
        self.goal = None

    def pose_callback(self, msg):
        self.pose = msg

    def update_goal(self):
        self.goal = (random.uniform(1, 10), random.uniform(1, 10))
        self.get_logger().info(f"New goal: {self.goal}")
        self.move_to_goal()

    def move_to_goal(self):
        if self.pose is None or self.goal is None:
            return

        msg = Twist()
        dx = self.goal[0] - self.pose.x
        dy = self.goal[1] - self.pose.y
        distance = sqrt(dx**2 + dy**2)
        angle = atan2(dy, dx)

        msg.linear.x = min(distance, 2.0)
        msg.angular.z = angle - self.pose.theta
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoal()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
