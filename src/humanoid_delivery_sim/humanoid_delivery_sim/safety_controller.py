import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

from humanoid_delivery_interfaces.msg import HumanDetection, RobotState

from .qos import CONTROL_QOS, SENSOR_QOS, STATE_QOS


SIM_MODE = True


class SafetyController(Node):
    def __init__(self):
        super().__init__("safety_controller")
        self.robot_id = self.declare_parameter("robot_id", "humanoid_1").value
        self.estop = False
        self._sim_human_stop_logged = False
        self.last_state = RobotState()
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", CONTROL_QOS)
        self.estop_pub = self.create_publisher(Bool, "/emergency_stop", STATE_QOS)
        self.create_subscription(HumanDetection, "/human_detection", self.on_human, SENSOR_QOS)
        self.create_subscription(RobotState, "/robot_state", self.on_state, STATE_QOS)
        self.create_timer(0.5, self.publish_estop)

    def on_human(self, msg):
        should_stop = msg.in_safety_zone
        if should_stop:
            self.cmd_pub.publish(Twist())
            if SIM_MODE:
                if not self._sim_human_stop_logged:
                    self.get_logger().warn("Human inside delivery safety zone: simulated stop active. [SIM]")
                    self._sim_human_stop_logged = True
                self.estop = True
            else:
                self.get_logger().warn("Human inside safety zone: publishing zero /cmd_vel and asserting /emergency_stop.")
                self.estop = True
        else:
            if SIM_MODE and self._sim_human_stop_logged:
                self.get_logger().info("Human safety zone cleared: simulated navigation may resume. [SIM]")
                self._sim_human_stop_logged = False
            self.estop = False

    def on_state(self, msg):
        self.last_state = msg
        if msg.fault_code:
            self.estop = True

    def publish_estop(self):
        msg = Bool()
        msg.data = self.estop
        self.estop_pub.publish(msg)


def main():
    rclpy.init()
    node = SafetyController()
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
