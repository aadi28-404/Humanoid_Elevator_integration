import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from humanoid_delivery_interfaces.srv import MissionAssignment

from .qos import EVENT_QOS


class SpeechLlmInterface(Node):
    """Simple speech/LLM boundary: converts operator utterances into missions."""

    def __init__(self):
        super().__init__("speech_llm_interface")
        self.pub = self.create_publisher(String, "/speech_events", EVENT_QOS)
        self.client = self.create_client(MissionAssignment, "/assign_mission")
        self.create_timer(8.0, self.demo_utterance)
        self.sent = False

    def demo_utterance(self):
        if self.sent or not self.client.service_is_ready():
            return
        utterance = String()
        utterance.data = "Deliver medication kit from room 101 to room 302."
        self.pub.publish(utterance)
        req = MissionAssignment.Request()
        req.robot_id = "humanoid_1"
        req.pickup_room = "101"
        req.delivery_room = "302"
        req.pickup_floor = 1
        req.delivery_floor = 3
        req.item_id = "med_kit"
        req.priority = 8
        self.client.call_async(req)
        self.sent = True
        self.get_logger().info("LLM parsed utterance and submitted a mission assignment.")


def main():
    rclpy.init()
    node = SpeechLlmInterface()
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
