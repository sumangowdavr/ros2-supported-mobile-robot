"""Follow a supplied odom-frame point. Does not detect humans from images."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from .control import follow_command

class FollowTarget(Node):
    def __init__(self):
        super().__init__('follow_target')
        self.declare_parameter('standoff', 1.0)
        self.target = self.odom = None
        self.target_time = self.odom_time = -float('inf')
        self.pub = self.create_publisher(Twist, '/cmd_vel_input', 10)
        self.create_subscription(PointStamped, '/target_point', self.on_target, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_timer(0.05, self.tick)
    def on_target(self, msg):
        if msg.header.frame_id != 'odom':
            self.get_logger().warning('Target must use frame_id: odom')
            return
        self.target, self.target_time = msg.point, time.monotonic()
    def on_odom(self, msg):
        self.odom, self.odom_time = msg.pose.pose, time.monotonic()
    def tick(self):
        cmd, now = Twist(), time.monotonic()
        if (self.target is not None and self.odom is not None and
                now-self.target_time < 1.0 and now-self.odom_time < 0.5):
            q = self.odom.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            cmd.linear.x, cmd.angular.z = follow_command(
                self.odom.position.x, self.odom.position.y, yaw,
                self.target.x, self.target.y,
                self.get_parameter('standoff').value)
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = FollowTarget()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
