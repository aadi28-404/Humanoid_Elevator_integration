import math
import time

from gazebo_msgs.msg import EntityState, ModelState
from gazebo_msgs.srv import SetEntityState, SetModelState
import rclpy
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from humanoid_delivery_interfaces.action import DeliverItem, PickObject

from .qos import CONTROL_QOS


IDLE = "IDLE"
PICKING = "PICKING"
CARRYING = "CARRYING"
DELIVERING = "DELIVERING"
DONE = "DONE"

BOX_MODEL_NAME = "medical_kit_pickup"
CARRY_RATE_HZ = 20.0
CARRY_OFFSET_X = 0.31
CARRY_OFFSET_Y = -0.16
CARRY_OFFSET_Z = 0.16
PICKUP_TABLE_POSE = (2.0, -2.55, 0.96)
DELIVERY_TABLE_POSES = {
    "302": (5.2, 2.55, 0.96),
    "201": (2.0, 2.55, 0.96),
}


class ManipulationServer(Node):
    def __init__(self):
        super().__init__("manipulation_server")
        self.callback_group = ReentrantCallbackGroup()
        self.state = IDLE
        self.carrying = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0
        self.robot_yaw = 0.0
        self._service_warned = False
        self._model_service_warned = False
        self._attached_logged = False
        self._box_future = None
        self._model_box_future = None
        self._box_failures = 0
        self._box_move_confirmed = False

        self.arm_pub = self.create_publisher(PickObject.Feedback, "/arm_controller", CONTROL_QOS)
        self.model_state_pub = self.create_publisher(ModelState, "/gazebo/set_model_state", CONTROL_QOS)
        self.set_entity_client = self.create_client(
            SetEntityState,
            "/gazebo/set_entity_state",
            callback_group=self.callback_group,
        )
        self.set_model_client = self.create_client(
            SetModelState,
            "/gazebo/set_model_state",
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Odometry,
            "/odom",
            self._odom_cb,
            10,
            callback_group=self.callback_group,
        )
        self.pick_server = ActionServer(
            self,
            PickObject,
            "pick_object",
            self._execute_pick,
            callback_group=self.callback_group,
        )
        self.deliver_server = ActionServer(
            self,
            DeliverItem,
            "deliver_item",
            self._execute_deliver,
            callback_group=self.callback_group,
        )
        self.create_timer(1.0 / CARRY_RATE_HZ, self._carry_box_tick, callback_group=self.callback_group)
        self.get_logger().info("PickObject and DeliverItem action servers ready; box carry controller online.")

    def _odom_cb(self, msg):
        pose = msg.pose.pose
        self.robot_x = pose.position.x
        self.robot_y = pose.position.y
        self.robot_z = pose.position.z
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _execute_pick(self, goal_handle):
        object_id = goal_handle.request.object_id or "med_kit"
        self.get_logger().info(f"Pick {object_id}: reach_pregrasp")
        self.state = PICKING

        feedback = PickObject.Feedback()
        feedback.phase = "lift_and_verify"
        feedback.progress = 0.5
        goal_handle.publish_feedback(feedback)
        self.arm_pub.publish(feedback)

        time.sleep(0.6)
        self.get_logger().info(f"Pick {object_id}: close_hand")
        self._animate_box_transfer(PICKUP_TABLE_POSE, self._carry_pose(), 1.4)
        self.get_logger().info(f"Pick {object_id}: lift_and_verify")
        self.carrying = True
        self.state = CARRYING
        self._attached_logged = False
        self._box_move_confirmed = False

        attach_deadline = time.monotonic() + 3.0
        while rclpy.ok() and time.monotonic() < attach_deadline and not self._box_move_confirmed:
            self._set_box_pose(*self._carry_pose(), force=True)
            time.sleep(0.1)

        if not self._box_move_confirmed:
            self.carrying = False
            self.state = IDLE
            goal_handle.abort()
            result = PickObject.Result()
            result.success = False
            result.message = (
                f"{object_id} pick aborted: Gazebo did not confirm blue box carry pose."
            )
            self.get_logger().error(result.message)
            return result

        self.get_logger().info(f"Pick {object_id}: complete")
        feedback.phase = "carrying"
        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)
        self.arm_pub.publish(feedback)
        goal_handle.succeed()

        result = PickObject.Result()
        result.success = True
        result.message = f"{object_id} picked and carried."
        return result

    def _execute_deliver(self, goal_handle):
        room_id = goal_handle.request.destination_room
        self.get_logger().info(f"Delivering to room {room_id}")
        self.state = DELIVERING

        feedback = DeliverItem.Feedback()
        feedback.phase = "handover"
        feedback.progress = 0.5
        goal_handle.publish_feedback(feedback)

        time.sleep(0.5)
        delivery_pose = DELIVERY_TABLE_POSES.get(room_id, DELIVERY_TABLE_POSES["302"])
        self.carrying = False
        self._animate_box_transfer(self._carry_pose(), delivery_pose, 1.4)
        self._set_box_pose(*delivery_pose, force=True)
        self.get_logger().info(f"Delivery to room {room_id}: complete")
        self.state = DONE

        feedback.phase = "complete"
        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()

        result = DeliverItem.Result()
        result.success = True
        result.message = f"Delivered to room {room_id}."
        return result

    def _carry_box_tick(self):
        if not self.carrying:
            return
        self._set_box_pose(*self._carry_pose())

    def _carry_pose(self):
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)
        x = self.robot_x + CARRY_OFFSET_X * cos_yaw - CARRY_OFFSET_Y * sin_yaw
        y = self.robot_y + CARRY_OFFSET_X * sin_yaw + CARRY_OFFSET_Y * cos_yaw
        z = self.robot_z + CARRY_OFFSET_Z
        return x, y, z, self.robot_yaw

    def _animate_box_transfer(self, start, finish, duration_sec):
        steps = max(1, int(duration_sec * CARRY_RATE_HZ))
        start_x, start_y, start_z = start[:3]
        finish_x, finish_y, finish_z, *finish_yaw = finish
        yaw = finish_yaw[0] if finish_yaw else self.robot_yaw
        for step in range(1, steps + 1):
            alpha = step / steps
            lift = 0.08 * math.sin(math.pi * alpha)
            self._set_box_pose(
                start_x + alpha * (finish_x - start_x),
                start_y + alpha * (finish_y - start_y),
                start_z + alpha * (finish_z - start_z) + lift,
                yaw=yaw,
                force=True,
            )
            time.sleep(1.0 / CARRY_RATE_HZ)

    def _set_box_pose(self, x, y, z, yaw=0.0, force=False, hide=False):
        self._publish_model_state(x, y, z, yaw)
        sent = False
        try:
            if (
                self._box_future is None or self._box_future.done() or force
            ) and (
                self.set_entity_client.service_is_ready()
                or self.set_entity_client.wait_for_service(timeout_sec=0.01)
            ):
                request = SetEntityState.Request()
                request.state = EntityState()
                self._fill_entity_state(request.state, x, y, z, yaw)
                self._box_future = self.set_entity_client.call_async(request)
                self._box_future.add_done_callback(self._on_box_pose_done)
                sent = True
            elif not self._service_warned and not self._box_move_confirmed:
                self.get_logger().warn("/gazebo/set_entity_state unavailable; using model-state topic fallback.")
                self._service_warned = True

            if (
                self._model_box_future is None or self._model_box_future.done() or force
            ) and self.set_model_client.service_is_ready():
                model_request = SetModelState.Request()
                model_request.model_state = self._make_model_state(x, y, z, yaw)
                self._model_box_future = self.set_model_client.call_async(model_request)
                self._model_box_future.add_done_callback(self._on_model_box_pose_done)
                sent = True
            elif (
                not sent
                and not self.set_model_client.service_is_ready()
                and not self._model_service_warned
                and not self._box_move_confirmed
            ):
                self.get_logger().warn("/gazebo/set_model_state service unavailable; publishing topic fallback only.")
                self._model_service_warned = True

            if not sent and force and not hide:
                self.get_logger().warn("Blue pickup box state request is waiting for Gazebo state services.")
            return sent
        except Exception as exc:
            self.get_logger().warn(f"Failed to move {BOX_MODEL_NAME}: {exc}")
            return False

    def _fill_entity_state(self, state, x, y, z, yaw):
        state.name = BOX_MODEL_NAME
        state.reference_frame = "world"
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation.x = 0.0
        state.pose.orientation.y = 0.0
        state.pose.orientation.z = math.sin(0.5 * yaw)
        state.pose.orientation.w = math.cos(0.5 * yaw)
        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0
        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0

    def _make_model_state(self, x, y, z, yaw):
        state = ModelState()
        state.model_name = BOX_MODEL_NAME
        state.reference_frame = "world"
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation.x = 0.0
        state.pose.orientation.y = 0.0
        state.pose.orientation.z = math.sin(0.5 * yaw)
        state.pose.orientation.w = math.cos(0.5 * yaw)
        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0
        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0
        return state

    def _publish_model_state(self, x, y, z, yaw):
        self.model_state_pub.publish(self._make_model_state(x, y, z, yaw))

    def _on_box_pose_done(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self._box_failures += 1
            if self._box_failures <= 3 or self._box_failures % 20 == 0:
                self.get_logger().warn(f"Failed to move {BOX_MODEL_NAME}: {exc}")
            return
        if response and response.success:
            self._on_confirmed_box_move("entity-state service")
            return
        self._box_failures += 1
        status = response.status_message if response else "no response"
        if self._box_failures <= 3 or self._box_failures % 20 == 0:
            self.get_logger().warn(f"Gazebo rejected {BOX_MODEL_NAME} pose update: {status}")

    def _on_model_box_pose_done(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self._box_failures += 1
            if self._box_failures <= 3 or self._box_failures % 20 == 0:
                self.get_logger().warn(
                    f"Failed to move {BOX_MODEL_NAME} through model-state service: {exc}"
                )
            return
        if response and response.success:
            self._on_confirmed_box_move("model-state service")
            return
        self._box_failures += 1
        status = response.status_message if response else "no response"
        if self._box_failures <= 3 or self._box_failures % 20 == 0:
            self.get_logger().warn(f"Gazebo rejected {BOX_MODEL_NAME} model pose update: {status}")

    def _on_confirmed_box_move(self, source):
        self._box_failures = 0
        if self.carrying:
            self._box_move_confirmed = True
            if not self._attached_logged:
                self.get_logger().info(f"Blue pickup box attached to robot carry pose via {source}.")
                self._attached_logged = True


def main():
    rclpy.init()
    node = ManipulationServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
