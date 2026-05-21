import csv
import json
import os
from datetime import datetime
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


class MotionRecorder(Node):
    """Records robot joint, link transform, and odometry motion for debugging."""

    def __init__(self):
        super().__init__("motion_recorder")
        default_root = os.environ.get(
            "HUMANOID_DELIVERY_MOTION_LOG_DIR",
            str(Path.cwd() / "debug" / "robot_motion_records"),
        )
        self.output_root = Path(self.declare_parameter("output_dir", default_root).value).expanduser()
        self.flush_every_n = int(self.declare_parameter("flush_every_n", 25).value)
        self.tf_frame_filter = str(self.declare_parameter("tf_frame_filter", "").value)
        self.write_count = 0

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = self.output_root / f"run_{stamp}"
        self.run_dir.mkdir(parents=True, exist_ok=True)

        self.joint_file = open(self.run_dir / "joint_states.csv", "w", newline="", encoding="utf-8")
        self.odom_file = open(self.run_dir / "odom.csv", "w", newline="", encoding="utf-8")
        self.tf_file = open(self.run_dir / "link_transforms.jsonl", "w", encoding="utf-8")
        self.metadata_file = self.run_dir / "metadata.json"

        self.joint_writer = csv.writer(self.joint_file)
        self.odom_writer = csv.writer(self.odom_file)
        self.joint_writer.writerow(
            [
                "recv_time_sec",
                "msg_time_sec",
                "joint_name",
                "position_rad",
                "velocity_rad_s",
                "effort",
            ]
        )
        self.odom_writer.writerow(
            [
                "recv_time_sec",
                "msg_time_sec",
                "position_x",
                "position_y",
                "position_z",
                "orientation_x",
                "orientation_y",
                "orientation_z",
                "orientation_w",
                "linear_x",
                "linear_y",
                "linear_z",
                "angular_x",
                "angular_y",
                "angular_z",
            ]
        )
        self.write_metadata()

        tf_qos = QoSProfile(depth=100)
        tf_static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 100)
        self.create_subscription(Odometry, "/odom", self.on_odom, 50)
        self.create_subscription(TFMessage, "/tf", self.on_tf, tf_qos)
        self.create_subscription(TFMessage, "/tf_static", self.on_tf, tf_static_qos)
        self.create_timer(1.0, self.flush)

        self.get_logger().info(f"Motion recorder writing robot debug traces to {self.run_dir}")

    def write_metadata(self):
        metadata = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "node": "motion_recorder",
            "topics": {
                "joint_states": "/joint_states",
                "odom": "/odom",
                "tf": "/tf",
                "tf_static": "/tf_static",
            },
            "files": {
                "joint_states": "joint_states.csv",
                "odom": "odom.csv",
                "link_transforms": "link_transforms.jsonl",
            },
            "notes": [
                "joint_states.csv has one row per joint sample.",
                "link_transforms.jsonl has one JSON object per TF transform.",
                "Use child_frame_id to inspect each robot link or sensor frame.",
            ],
        }
        self.metadata_file.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    def on_joint_states(self, msg):
        recv_time = self.now_sec()
        msg_time = self.stamp_sec(msg.header.stamp)
        for index, name in enumerate(msg.name):
            position = msg.position[index] if index < len(msg.position) else ""
            velocity = msg.velocity[index] if index < len(msg.velocity) else ""
            effort = msg.effort[index] if index < len(msg.effort) else ""
            self.joint_writer.writerow([recv_time, msg_time, name, position, velocity, effort])
            self.write_count += 1
        self.flush_if_needed()

    def on_odom(self, msg):
        recv_time = self.now_sec()
        msg_time = self.stamp_sec(msg.header.stamp)
        pose = msg.pose.pose
        twist = msg.twist.twist
        self.odom_writer.writerow(
            [
                recv_time,
                msg_time,
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
                twist.angular.x,
                twist.angular.y,
                twist.angular.z,
            ]
        )
        self.write_count += 1
        self.flush_if_needed()

    def on_tf(self, msg):
        recv_time = self.now_sec()
        for transform in msg.transforms:
            child = transform.child_frame_id
            parent = transform.header.frame_id
            if self.tf_frame_filter and self.tf_frame_filter not in child and self.tf_frame_filter not in parent:
                continue
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            record = {
                "recv_time_sec": recv_time,
                "msg_time_sec": self.stamp_sec(transform.header.stamp),
                "parent_frame": parent,
                "child_frame": child,
                "translation": {
                    "x": translation.x,
                    "y": translation.y,
                    "z": translation.z,
                },
                "rotation": {
                    "x": rotation.x,
                    "y": rotation.y,
                    "z": rotation.z,
                    "w": rotation.w,
                },
            }
            self.tf_file.write(json.dumps(record, separators=(",", ":")) + "\n")
            self.write_count += 1
        self.flush_if_needed()

    def flush_if_needed(self):
        if self.write_count >= self.flush_every_n:
            self.flush()

    def flush(self):
        self.joint_file.flush()
        self.odom_file.flush()
        self.tf_file.flush()
        self.write_count = 0

    def destroy_node(self):
        self.flush()
        self.joint_file.close()
        self.odom_file.close()
        self.tf_file.close()
        super().destroy_node()

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def stamp_sec(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main():
    rclpy.init()
    node = MotionRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
