import math

import rclpy
from gazebo_msgs.srv import SetModelConfiguration
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_LIMITS = {
    "left_hip_pitch_joint": (-2.5307, 2.8798),
    "left_hip_roll_joint": (-0.5236, 2.9671),
    "left_hip_yaw_joint": (-2.7576, 2.7576),
    "left_knee_joint": (-0.087267, 2.8798),
    "left_ankle_pitch_joint": (-0.87267, 0.5236),
    "left_ankle_roll_joint": (-0.2618, 0.2618),
    "right_hip_pitch_joint": (-2.5307, 2.8798),
    "right_hip_roll_joint": (-2.9671, 0.5236),
    "right_hip_yaw_joint": (-2.7576, 2.7576),
    "right_knee_joint": (-0.087267, 2.8798),
    "right_ankle_pitch_joint": (-0.87267, 0.5236),
    "right_ankle_roll_joint": (-0.2618, 0.2618),
    "waist_yaw_joint": (-2.618, 2.618),
    "left_shoulder_pitch_joint": (-3.0892, 2.6704),
    "left_shoulder_roll_joint": (-1.5882, 2.2515),
    "left_shoulder_yaw_joint": (-2.618, 2.618),
    "left_elbow_joint": (-1.0472, 2.0944),
    "left_wrist_roll_joint": (-1.972222054, 1.972222054),
    "right_shoulder_pitch_joint": (-3.0892, 2.6704),
    "right_shoulder_roll_joint": (-2.2515, 1.5882),
    "right_shoulder_yaw_joint": (-2.618, 2.618),
    "right_elbow_joint": (-1.0472, 2.0944),
    "right_wrist_roll_joint": (-1.972222054, 1.972222054),
}

JOINT_NAMES = list(JOINT_LIMITS.keys())

# Conservative display pose. This is intentionally less dramatic than a real
# walking controller because the robot is moved by a planar Gazebo base plugin.
STANDING_POSE = {
    "left_hip_pitch_joint": -0.04,
    "left_hip_roll_joint": 0.0,
    "left_hip_yaw_joint": 0.0,
    "left_knee_joint": 0.14,
    "left_ankle_pitch_joint": -0.08,
    "left_ankle_roll_joint": 0.0,
    "right_hip_pitch_joint": -0.04,
    "right_hip_roll_joint": 0.0,
    "right_hip_yaw_joint": 0.0,
    "right_knee_joint": 0.14,
    "right_ankle_pitch_joint": -0.08,
    "right_ankle_roll_joint": 0.0,
    "waist_yaw_joint": 0.0,
    "left_shoulder_pitch_joint": 0.02,
    "left_shoulder_roll_joint": 0.08,
    "left_shoulder_yaw_joint": 0.0,
    "left_elbow_joint": 0.18,
    "left_wrist_roll_joint": 0.0,
    "right_shoulder_pitch_joint": 0.02,
    "right_shoulder_roll_joint": -0.08,
    "right_shoulder_yaw_joint": 0.0,
    "right_elbow_joint": 0.18,
    "right_wrist_roll_joint": 0.0,
}

# Extra display limits keep the avatar from reaching the large official ranges
# that are valid mechanically but ugly for this planar delivery demo.
DEMO_LIMITS = {
    "left_hip_pitch_joint": (-0.13, 0.08),
    "right_hip_pitch_joint": (-0.13, 0.08),
    "left_knee_joint": (0.10, 0.24),
    "right_knee_joint": (0.10, 0.24),
    "left_ankle_pitch_joint": (-0.14, -0.02),
    "right_ankle_pitch_joint": (-0.14, -0.02),
    "left_hip_roll_joint": (-0.02, 0.02),
    "right_hip_roll_joint": (-0.02, 0.02),
    "left_hip_yaw_joint": (-0.02, 0.02),
    "right_hip_yaw_joint": (-0.02, 0.02),
    "left_ankle_roll_joint": (-0.02, 0.02),
    "right_ankle_roll_joint": (-0.02, 0.02),
    "waist_yaw_joint": (-0.03, 0.03),
    "left_shoulder_pitch_joint": (-0.03, 0.08),
    "right_shoulder_pitch_joint": (-0.03, 0.08),
    "left_shoulder_roll_joint": (0.04, 0.10),
    "right_shoulder_roll_joint": (-0.10, -0.04),
    "left_shoulder_yaw_joint": (-0.02, 0.02),
    "right_shoulder_yaw_joint": (-0.02, 0.02),
    "left_elbow_joint": (0.14, 0.24),
    "right_elbow_joint": (0.14, 0.24),
    "left_wrist_roll_joint": (-0.05, 0.05),
    "right_wrist_roll_joint": (-0.05, 0.05),
}

PUBLISH_RATE_HZ = 30.0
MODEL_APPLY_RATE_HZ = 12.0
GAIT_FREQUENCY = 0.65
HIP_PITCH_AMP = 0.045
KNEE_AMP = 0.035
ANKLE_AMP = 0.025
SHOULDER_AMP = 0.025
ELBOW_AMP = 0.02
MOVING_THRESHOLD = 0.04


class G1GaitController(Node):
    """Small-amplitude visual gait controller for the planar G1 avatar."""

    def __init__(self):
        super().__init__("g1_gait_controller")
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.config_client = self.create_client(SetModelConfiguration, "/gazebo/set_model_configuration")
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)

        self.cmd_vel_x = 0.0
        self.cmd_vel_y = 0.0
        self.gait_phase = 0.0
        self.last_positions = self._positions_from_pose(STANDING_POSE)
        self.pending_config = None
        self.configured_once = False

        self.publish_timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_joint_state)
        self.model_timer = self.create_timer(1.0 / MODEL_APPLY_RATE_HZ, self.apply_model_configuration)
        self.get_logger().info("G1 visual gait controller ready: conservative demo limits enabled.")

    def on_cmd_vel(self, msg):
        self.cmd_vel_x = msg.linear.x
        self.cmd_vel_y = msg.linear.y

    def _clamp(self, name, value):
        official_lo, official_hi = JOINT_LIMITS[name]
        demo_lo, demo_hi = DEMO_LIMITS.get(name, (official_lo, official_hi))
        lower = max(official_lo, demo_lo)
        upper = min(official_hi, demo_hi)
        return max(lower, min(upper, value))

    def _positions_from_pose(self, pose):
        return [self._clamp(name, pose[name]) for name in JOINT_NAMES]

    def _is_moving(self):
        return abs(self.cmd_vel_x) > MOVING_THRESHOLD or abs(self.cmd_vel_y) > MOVING_THRESHOLD

    def _compute_pose(self):
        pose = dict(STANDING_POSE)
        if not self._is_moving():
            self.gait_phase = 0.0
            return pose

        self.gait_phase += 2.0 * math.pi * GAIT_FREQUENCY / PUBLISH_RATE_HZ
        self.gait_phase = math.fmod(self.gait_phase, 2.0 * math.pi)

        left_phase = self.gait_phase
        right_phase = self.gait_phase + math.pi
        left_swing = math.sin(left_phase)
        right_swing = math.sin(right_phase)
        left_lift = max(0.0, math.sin(left_phase))
        right_lift = max(0.0, math.sin(right_phase))

        pose["left_hip_pitch_joint"] += HIP_PITCH_AMP * left_swing
        pose["left_knee_joint"] += KNEE_AMP * left_lift
        pose["left_ankle_pitch_joint"] -= ANKLE_AMP * left_swing

        pose["right_hip_pitch_joint"] += HIP_PITCH_AMP * right_swing
        pose["right_knee_joint"] += KNEE_AMP * right_lift
        pose["right_ankle_pitch_joint"] -= ANKLE_AMP * right_swing

        pose["left_shoulder_pitch_joint"] -= SHOULDER_AMP * right_swing
        pose["left_elbow_joint"] += ELBOW_AMP * right_lift
        pose["right_shoulder_pitch_joint"] -= SHOULDER_AMP * left_swing
        pose["right_elbow_joint"] += ELBOW_AMP * left_lift

        return pose

    def publish_joint_state(self):
        self.last_positions = self._positions_from_pose(self._compute_pose())
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = self.last_positions
        self.joint_pub.publish(msg)

    def apply_model_configuration(self):
        if self.pending_config is not None:
            if self.pending_config.done():
                self.pending_config = None
                self.configured_once = True
            else:
                return
        if not self.config_client.service_is_ready():
            return

        request = SetModelConfiguration.Request()
        request.model_name = "unitree_g1_delivery"
        request.urdf_param_name = "robot_description"
        request.joint_names = JOINT_NAMES
        request.joint_positions = self.last_positions
        self.pending_config = self.config_client.call_async(request)


def main():
    rclpy.init()
    node = G1GaitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
