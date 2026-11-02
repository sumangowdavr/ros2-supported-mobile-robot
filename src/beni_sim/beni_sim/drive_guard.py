import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from .control import guarded_command

class DriveGuard(Node):
    def __init__(self):
        super().__init__('drive_guard')
        self.command = Twist()
        self.scan = None
        self.command_time = self.scan_time = -float('inf')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel_input', self.on_command, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.05, self.tick)
    def on_command(self, msg):
        self.command, self.command_time = msg, time.monotonic()
    def on_scan(self, msg):
        self.scan, self.scan_time = msg, time.monotonic()
    def tick(self):
        result = Twist()
        if self.scan is not None:
            s, now = self.scan, time.monotonic()
            result.linear.x, result.angular.z = guarded_command(
                self.command.linear.x, self.command.angular.z,
                now-self.command_time, now-self.scan_time, s.ranges,
                s.angle_min, s.angle_increment, s.range_min, s.range_max)
        self.pub.publish(result)

def main(args=None):
    rclpy.init(args=args)
    node = DriveGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Humble's signal handler may already have shut down the ROS context.
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
