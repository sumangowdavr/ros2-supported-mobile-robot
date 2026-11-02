"""Actual Gazebo integration test. Run in a fresh world, with no teleop/follower."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, LaserScan
from rosgraph_msgs.msg import Clock

def yaw(pose):
    q = pose.orientation
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

class SmokeTest(Node):
    def __init__(self):
        super().__init__('beni_smoke_test')
        self.messages, self.received = {}, {}
        self.pub = self.create_publisher(Twist, '/cmd_vel_input', 10)
        for topic, msg in [('/odom', Odometry), ('/scan', LaserScan),
                           ('/imu', Imu), ('/camera/image_raw', Image), ('/clock', Clock)]:
            self.create_subscription(msg, topic,
                lambda data, name=topic: self.record(name, data), qos_profile_sensor_data)
    def record(self, topic, msg):
        self.messages[topic], self.received[topic] = msg, time.monotonic()
    def pump(self, seconds, linear=0.0, angular=0.0, publish=True):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            if publish:
                cmd = Twist(); cmd.linear.x = linear; cmd.angular.z = angular
                self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
    def sim_time(self):
        c = self.messages['/clock'].clock
        return c.sec+c.nanosec*1e-9
    def check(self):
        deadline = time.monotonic()+45
        while time.monotonic() < deadline and len(self.messages) < 5:
            self.pump(0.1)
        missing = sorted(set(['/odom','/scan','/imu','/camera/image_raw','/clock'])-self.messages.keys())
        if missing: raise AssertionError('Missing topics: '+', '.join(missing))
        self.pump(1.0)
        image = self.messages['/camera/image_raw']
        assert image.width == 640 and image.height == 480 and len(image.data) > 0, 'Invalid camera output'
        scan = self.messages['/scan']
        assert len(scan.ranges) >= 300, 'Invalid lidar output'
        q = self.messages['/imu'].orientation
        assert abs(q.x) < 0.15 and abs(q.y) < 0.15, 'Robot not upright'
        t0 = self.sim_time()
        p0 = self.messages['/odom'].pose.pose.position
        self.pump(4.0, linear=0.2)
        p1 = self.messages['/odom'].pose.pose.position
        elapsed = self.sim_time()-t0
        distance = math.hypot(p1.x-p0.x, p1.y-p0.y)
        assert elapsed > 0.5, 'Simulation clock did not advance sufficiently'
        assert 0.08*elapsed < distance < 0.35*elapsed, f'Wrong travel: {distance:.3f}m in {elapsed:.3f}s'
        assert p1.x > p0.x, 'Forward command moved backward'
        self.pump(1.0)
        before = yaw(self.messages['/odom'].pose.pose)
        t0 = self.sim_time()
        self.pump(2.0, angular=0.6)
        turned = yaw(self.messages['/odom'].pose.pose)-before
        turned = math.atan2(math.sin(turned), math.cos(turned))
        assert turned > 0.15, f'Positive turn failed: {turned:.3f}rad'
        # Stop publishing; guard must command zero after its 0.5s timeout.
        self.pump(2.0, publish=False)
        twist = self.messages['/odom'].twist.twist
        assert abs(twist.linear.x) < 0.03 and abs(twist.angular.z) < 0.05, 'Watchdog failed to stop'
        now = time.monotonic()
        stale = [topic for topic, stamp in self.received.items() if now-stamp > 2.0]
        assert not stale, f'Stale output topics: {stale}'
        return {'status':'PASS', 'distance_m':round(distance,3),
                'forward_sim_seconds':round(elapsed,3), 'turn_radians':round(turned,3),
                'checked':['clock','odom','camera','lidar','imu','forward','turn','watchdog_stop']}

def main(args=None):
    rclpy.init(args=args)
    node = SmokeTest()
    code = 0
    try:
        print(json.dumps(node.check(), indent=2))
    except Exception as exc:
        print(json.dumps({'status':'FAIL','error':str(exc)}, indent=2))
        code = 1
    finally:
        for _ in range(5):
            node.pub.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    raise SystemExit(code)
