import os
import threading
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import EntityState, ModelState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node
from std_msgs.msg import Bool

from humanoid_delivery_interfaces.action import DeliverItem, PickObject, RetryMission
from humanoid_delivery_interfaces.msg import LiftStatus, RobotState, Task
from humanoid_delivery_interfaces.srv import CallLift, DoorOpenVerification

from .qos import EVENT_QOS, STATE_QOS


NAV_TIMEOUT_SECONDS = 120.0


class RobotExecutive(Node):
    def __init__(self):
        super().__init__("robot_executive")
        self.robot_id = self.declare_parameter("robot_id", "humanoid_1").value
        self.floor = 1
        self.mode = "idle"
        self.mission_id = ""
        self.battery = 96.0
        self.estop = False
        self.shutting_down = False
        self.holding_item = False
        self.fault_code = ""
        self.lift = LiftStatus()
        self.room_poses = self.load_room_poses()

        self.state_pub = self.create_publisher(RobotState, "/robot_state", STATE_QOS)
        self.model_state_pub = self.create_publisher(ModelState, "/gazebo/set_model_state", 10)
        self.create_subscription(Task, "/task_queue", self.on_task, EVENT_QOS)
        self.create_subscription(LiftStatus, "/lift_status", self.on_lift, STATE_QOS)
        self.create_subscription(Bool, "/emergency_stop", self.on_estop, STATE_QOS)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.pick_client = ActionClient(self, PickObject, "pick_object")
        self.deliver_client = ActionClient(self, DeliverItem, "deliver_item")
        self.retry_client = ActionClient(self, RetryMission, "retry_mission")
        self.lift_client = self.create_client(CallLift, "/call_lift")
        self.door_client = self.create_client(DoorOpenVerification, "/verify_lift_door_open")
        self.set_entity_client = self.create_client(SetEntityState, "/gazebo/set_entity_state")

        self.retry_server = ActionServer(self, RetryMission, "retry_mission", self.execute_retry_action)

        self.create_timer(1.0, self.publish_state)
        self.get_logger().info("Robot executive ready: task planner, delivery, retry, lift, and nav clients online.")

    def load_room_poses(self):
        pkg_share = get_package_share_directory("humanoid_delivery_sim")
        poses_path = os.path.join(pkg_share, "config", "room_poses.yaml")
        with open(poses_path, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        return data["rooms"]

    def on_task(self, msg):
        if self.shutting_down:
            return
        if msg.robot_id and msg.robot_id != self.robot_id:
            return
        if self.mission_id and self.mode != "idle":
            self.get_logger().warn(f"Busy; ignoring mission {msg.mission_id} in demo executive.")
            return
        threading.Thread(target=self.execute_task, args=(msg,), daemon=True).start()

    def on_lift(self, msg):
        self.lift = msg

    def on_estop(self, msg):
        self.estop = msg.data
        if self.estop:
            self.mode = "paused_for_safety"
        elif self.mode == "paused_for_safety" and not self.mission_id:
            self.mode = "idle"

    def execute_task(self, task):
        if self.shutting_down or not rclpy.ok():
            return
        self.mission_id = task.mission_id
        self.mode = "navigate_to_pickup"
        if not self.navigate_to_room("pickup"):
            return
        self.get_logger().info("Reached pickup zone")

        self.wait_while_estopped()
        if self.shutting_down or not rclpy.ok():
            return
        time.sleep(0.5)
        self.mode = "pick_item"
        item_id = task.item_id or "med_kit"
        self.get_logger().info(f"Pick {item_id} requested")
        if not self.pick_object(item_id, task.pickup_room):
            return
        self.holding_item = True
        self.get_logger().info("Pick complete — navigating to elevator")

        if task.delivery_floor != self.floor:
            self.mode = "use_elevator"
            if not self.use_lift(task.delivery_floor, task.priority):
                return
            # Teleport the Gazebo body to the elevator exit on the delivery floor
            self.teleport_robot(0.85, 0.0, 0.96)
            time.sleep(0.5)  # let odom catch up

        self.wait_while_estopped()
        if self.shutting_down or not rclpy.ok():
            return
        self.mode = "navigate_to_delivery"
        if not self.navigate_to_room(task.delivery_room):
            return

        self.mode = "handover"
        self.wait_while_estopped()
        if self.shutting_down or not rclpy.ok():
            return
        if not self.deliver_item(item_id, task.delivery_room, task.delivery_floor):
            return
        self.holding_item = False
        self.mode = "idle"
        self.mission_id = ""
        self.get_logger().info(f"Mission {task.mission_id} complete.")

    def wait_while_estopped(self):
        while self.estop and rclpy.ok() and not self.shutting_down:
            time.sleep(0.5)

    def teleport_robot(self, x, y, z):
        """Teleport the Gazebo robot body to (x, y, z) after elevator floor change.

        Uses /gazebo/set_entity_state which works at the physics level and overrides
        libgazebo_ros_planar_move's internal odometry. Retries up to 10 times (5 s).
        """
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = "unitree_g1_delivery"
        req.state.reference_frame = "world"
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = z
        req.state.pose.orientation.w = 1.0
        req.state.twist.linear.x = 0.0
        req.state.twist.linear.y = 0.0
        req.state.twist.linear.z = 0.0
        req.state.twist.angular.x = 0.0
        req.state.twist.angular.y = 0.0
        req.state.twist.angular.z = 0.0

        for attempt in range(10):
            if not rclpy.ok() or self.shutting_down:
                return
            if self.set_entity_client.wait_for_service(timeout_sec=0.5):
                future = self.set_entity_client.call_async(req)
                if self.wait_future(future, 2.0) and future.result() and future.result().success:
                    self.get_logger().info(
                        f"Robot teleported to elevator exit ({x:.2f}, {y:.2f}, {z:.2f}) "
                        f"on floor {self.floor} (attempt {attempt + 1})."
                    )
                    return
            time.sleep(0.5)

        self.get_logger().warn(
            f"Teleport to ({x:.2f}, {y:.2f}, {z:.2f}) failed after 10 attempts — "
            "navigation will proceed from current position."
        )

    def navigate_to_room(self, room):
        if self.shutting_down or not rclpy.ok():
            return False
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.fault_code = "NAV_SERVER_TIMEOUT"
            return False
        room_cfg = self.room_poses.get(room, self.room_poses["elevator"])
        goal_name = str(room_cfg.get("chain_key", self.goal_name_for_room(room)))
        for attempt in range(1, 3):
            status = self.send_navigation_goal(room, room_cfg, goal_name, NAV_TIMEOUT_SECONDS)
            if status == "success":
                return True
            if status == "timeout" and attempt == 1:
                self.get_logger().warn(f"Navigation timeout for goal '{goal_name}' — retrying")
                continue
            if status == "timeout":
                self.mode = "recover"
                self.fault_code = "NAV_TIMEOUT"
                self.request_retry_mission()
                return False
            return False
        return False

    def send_navigation_goal(self, room, room_cfg, goal_name, timeout_sec):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(room_cfg["x"])
        pose.pose.position.y = float(room_cfg["y"])
        pose.pose.orientation.w = 1.0
        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.behavior_tree = goal_name
        future = self.nav_client.send_goal_async(goal)
        if not self.wait_future(future, 12.0):
            self.fault_code = "NAV_SERVER_TIMEOUT"
            return "timeout"
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.fault_code = "NAV_GOAL_REJECTED"
            return "rejected"
        result_future = goal_handle.get_result_async()
        if not self.wait_future(result_future, timeout_sec):
            cancel_future = goal_handle.cancel_goal_async()
            self.wait_future(cancel_future, 2.0)
            self.fault_code = "NAV_TIMEOUT"
            return "timeout"
        result = result_future.result()
        if result and result.status == GoalStatus.STATUS_SUCCEEDED:
            return "success"
        self.fault_code = "NAV_TIMEOUT"
        return "failed"

    @staticmethod
    def goal_name_for_room(room):
        mapping = {
            "pickup": "pickup",
            "elevator": "elevator",
            "101": "room_101",
            "102": "room_102",
            "201": "room_201",
            "302": "room_302",
            "start": "start",
        }
        return mapping.get(str(room), "elevator")

    def pick_object(self, item_id, room):
        if self.shutting_down or not rclpy.ok():
            return False
        if not self.pick_client.wait_for_server(timeout_sec=10.0):
            self.fault_code = "PICK_SERVER_TIMEOUT"
            return False
        goal = PickObject.Goal()
        goal.object_id = item_id
        goal.source_room = room
        goal.grasp_hint.orientation.w = 1.0
        future = self.pick_client.send_goal_async(goal)
        if not self.wait_future(future, 10.0):
            self.fault_code = "PICK_SERVER_TIMEOUT"
            return False
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.fault_code = "PICK_REJECTED"
            return False
        result_future = goal_handle.get_result_async()
        if not self.wait_future(result_future, 10.0):
            self.fault_code = "PICK_TIMEOUT"
            return False
        result = result_future.result()
        if (
            result
            and result.status == GoalStatus.STATUS_SUCCEEDED
            and result.result
            and result.result.success
        ):
            return True
        if result and result.result and result.result.message:
            self.get_logger().warn(f"Pick failed: {result.result.message}")
        self.fault_code = "PICK_FAILED"
        return False

    def deliver_item(self, item_id, room, floor):
        if self.shutting_down or not rclpy.ok():
            return False
        if not self.deliver_client.wait_for_server(timeout_sec=10.0):
            self.fault_code = "DELIVER_SERVER_TIMEOUT"
            return False
        goal = DeliverItem.Goal()
        goal.mission_id = self.mission_id
        goal.item_id = item_id
        goal.destination_room = room
        goal.destination_floor = floor
        self.get_logger().info(f"Deliver {item_id} requested for room {room}")
        future = self.deliver_client.send_goal_async(goal)
        if not self.wait_future(future, 10.0):
            self.fault_code = "DELIVER_SERVER_TIMEOUT"
            return False
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.fault_code = "DELIVER_REJECTED"
            return False
        result_future = goal_handle.get_result_async()
        if not self.wait_future(result_future, 12.0):
            self.fault_code = "DELIVER_TIMEOUT"
            return False
        result = result_future.result()
        if (
            result
            and result.status == GoalStatus.STATUS_SUCCEEDED
            and result.result
            and result.result.success
        ):
            self.get_logger().info(f"Delivered {item_id} to room {room}")
            return True
        if result and result.result and result.result.message:
            self.get_logger().warn(f"Delivery failed: {result.result.message}")
        self.fault_code = "DELIVER_FAILED"
        return False

    def request_retry_mission(self):
        if self.shutting_down or not rclpy.ok():
            return False
        if not self.retry_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Retry mission action unavailable during navigation recovery.")
            return False
        goal = RetryMission.Goal()
        goal.mission_id = self.mission_id
        goal.max_retries = 1
        future = self.retry_client.send_goal_async(goal)
        if not self.wait_future(future, 2.0):
            return False
        goal_handle = future.result()
        return bool(goal_handle and goal_handle.accepted)

    def use_lift(self, target_floor, priority):
        if self.shutting_down or not rclpy.ok():
            return False
        if not self.navigate_to_room("elevator"):
            return False
        if not self.lift_client.wait_for_service(timeout_sec=10.0):
            self.fault_code = "LIFT_SERVICE_TIMEOUT"
            return False
        req = CallLift.Request()
        req.robot_id = self.robot_id
        req.lift_id = "lift_A"
        req.from_floor = self.floor
        req.to_floor = target_floor
        req.priority = priority
        future = self.lift_client.call_async(req)
        if not self.wait_future(future, 5.0):
            self.fault_code = "LIFT_SERVICE_TIMEOUT"
            return False
        if not future.result() or not future.result().accepted:
            self.fault_code = "LIFT_UNAVAILABLE"
            return False

        now = lambda: self.get_clock().now().nanoseconds * 1e-9
        deadline = now() + 45.0
        while now() < deadline and rclpy.ok() and not self.shutting_down:
            if self.lift.reserved_by == self.robot_id and self.lift.current_floor == self.floor and self.lift.door_state == "open":
                if self.verify_door(self.floor):
                    break
            time.sleep(0.5)
        else:
            self.fault_code = "LIFT_TIMEOUT_PICKUP"
            return False

        self.mode = "boarding_lift"
        time.sleep(1.0)
        deadline = now() + 45.0
        while now() < deadline and rclpy.ok() and not self.shutting_down:
            if self.lift.reserved_by == self.robot_id and self.lift.current_floor == target_floor and self.lift.door_state == "open":
                if self.verify_door(target_floor):
                    self.floor = target_floor
                    return True
            time.sleep(0.5)
        self.fault_code = "LIFT_TIMEOUT_DROPOFF"
        return False

    def verify_door(self, floor):
        if self.shutting_down or not rclpy.ok():
            return False
        if not self.door_client.wait_for_service(timeout_sec=5.0):
            return False
        req = DoorOpenVerification.Request()
        req.lift_id = "lift_A"
        req.floor = floor
        req.min_width_m = 0.9
        future = self.door_client.call_async(req)
        if not self.wait_future(future, 3.0):
            return False
        result = future.result()
        return bool(result and result.is_open)

    def wait_future(self, future, timeout_sec):
        now = lambda: self.get_clock().now().nanoseconds * 1e-9
        deadline = now() + timeout_sec
        while rclpy.ok() and not self.shutting_down and not future.done() and now() < deadline:
            time.sleep(0.02)
        return future.done()

    def execute_retry_action(self, goal_handle):
        feedback = RetryMission.Feedback()
        max_retries = max(1, goal_handle.request.max_retries)
        for attempt in range(1, max_retries + 1):
            feedback.attempt = attempt
            feedback.recovery_action = "clear_costmaps_relocalize_reassign_lift"
            goal_handle.publish_feedback(feedback)
            self.fault_code = ""
            time.sleep(0.5)
        goal_handle.succeed()
        result = RetryMission.Result()
        result.success = True
        result.final_state = "ready"
        return result

    def publish_state(self):
        self.battery = max(0.0, self.battery - 0.01)
        msg = RobotState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_id = self.robot_id
        msg.mode = self.mode
        msg.mission_id = self.mission_id
        msg.floor = self.floor
        msg.battery_percent = self.battery
        msg.emergency_stop = self.estop
        msg.localized = True
        msg.holding_item = self.holding_item
        msg.fault_code = self.fault_code
        self.state_pub.publish(msg)


def main():
    rclpy.init()
    node = RobotExecutive()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutting_down = True
        try:
            executor.shutdown()
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
