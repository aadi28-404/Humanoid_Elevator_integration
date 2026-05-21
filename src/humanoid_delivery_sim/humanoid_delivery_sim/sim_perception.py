import math
import time

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node

from humanoid_delivery_interfaces.msg import HumanDetection

from .qos import SENSOR_QOS


class SimPerception(Node):
    def __init__(self):
        super().__init__("sim_perception")
        self.pub = self.create_publisher(HumanDetection, "/human_detection", SENSOR_QOS)
        self.phase = 0.0
        self.robot_x = 0.85
        self.robot_y = 0.0
        self.delivery_stop_until = 0.0
        self.delivery_stop_consumed = False
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_timer(0.2, self.publish_detection)

    def _odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def publish_detection(self):
        self.phase += 0.2
        guest_x = 2.1 + math.sin(self.phase) * 0.04
        guest_y = 0.7
        distance = math.hypot(guest_x - self.robot_x, guest_y - self.robot_y)
        now = time.monotonic()
        if (
            not self.delivery_stop_consumed
            and self.robot_x > 2.8
            and self.robot_y > 0.55
            and distance < 1.75
        ):
            self.delivery_stop_until = now + 3.0
            self.delivery_stop_consumed = True

        msg = HumanDetection()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.human_id = "guest_01"
        msg.pose = Pose()
        msg.pose.position.x = guest_x - self.robot_x
        msg.pose.position.y = guest_y - self.robot_y
        msg.pose.orientation.w = 1.0
        msg.distance_m = distance
        msg.confidence = 0.91
        msg.intent = "crossing" if now < self.delivery_stop_until else "standing"
        msg.in_safety_zone = now < self.delivery_stop_until
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SimPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
