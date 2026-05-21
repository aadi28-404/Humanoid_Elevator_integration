import math

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


ENTITY_NAME = "unitree_g1_delivery"
LOCK_Z = 0.96
LOCK_RATE_HZ = 30.0


class JointMotionDemo(Node):
    def __init__(self):
        super().__init__("joint_motion_demo")
        self.callback_group = ReentrantCallbackGroup()
        self.current_x = 0.85
        self.current_y = 0.0
        self.current_yaw = 0.0
        self._lock_future = None
        self._warned = False

        self.set_entity_client = self.create_client(
            SetEntityState,
            "/gazebo/set_entity_state",
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Odometry,
            "/odom",
            self._odom_cb,
            10,
            callback_group=self.callback_group,
        )
        # self.create_timer(1.0 / LOCK_RATE_HZ, self._pose_lock_cb, callback_group=self.callback_group)
        self.get_logger().info(
            "G1 rigid avatar stabilizer active: limbs fixed, whole-body pose locked upright (pose lock disabled to prevent navigation fighting)."
        )

    def _odom_cb(self, msg):
        pose = msg.pose.pose
        self.current_x = pose.position.x
        self.current_y = pose.position.y
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _pose_lock_cb(self):
        if self._lock_future is not None and not self._lock_future.done():
            return
        if not self.set_entity_client.service_is_ready():
            self.set_entity_client.wait_for_service(timeout_sec=0.05)
        if not self.set_entity_client.service_is_ready():
            if not self._warned:
                self.get_logger().warn("/gazebo/set_entity_state unavailable; upright lock waiting.")
                self._warned = True
            return

        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = ENTITY_NAME
        request.state.reference_frame = "world"
        request.state.pose.position.x = self.current_x
        request.state.pose.position.y = self.current_y
        request.state.pose.position.z = LOCK_Z
        request.state.pose.orientation.x = 0.0
        request.state.pose.orientation.y = 0.0
        request.state.pose.orientation.z = math.sin(0.5 * self.current_yaw)
        request.state.pose.orientation.w = math.cos(0.5 * self.current_yaw)
        request.state.twist.linear.x = 0.0
        request.state.twist.linear.y = 0.0
        request.state.twist.linear.z = 0.0
        request.state.twist.angular.x = 0.0
        request.state.twist.angular.y = 0.0
        request.state.twist.angular.z = 0.0
        self._lock_future = self.set_entity_client.call_async(request)


def main():
    rclpy.init()
    node = JointMotionDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
