import heapq
import time
import uuid
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node

from humanoid_delivery_interfaces.msg import LiftStatus, Task
from humanoid_delivery_interfaces.srv import (
    CallLift,
    ChargingRequest,
    DoorOpenVerification,
    MissionAssignment,
    RecoveryReset,
)

from .qos import EVENT_QOS, STATE_QOS


@dataclass(order=True)
class LiftReservation:
    sort_key: tuple
    reservation_id: str = field(compare=False)
    robot_id: str = field(compare=False)
    from_floor: int = field(compare=False)
    to_floor: int = field(compare=False)
    priority: int = field(compare=False)
    created_at: float = field(compare=False)


class FleetOrchestrator(Node):
    """RMF-like central arbiter for missions, lifts, charging, and recovery."""

    def __init__(self):
        super().__init__("fleet_orchestrator")
        self.robot_ids = ["humanoid_1", "humanoid_2"]
        self.lift_floor = 1
        self.lift_target = 1
        self.door_state = "open"
        self.motion_state = "idle"
        self.active_reservation = None
        self.reservations = []
        self.sequence = 0
        self._complete_hold = 0  # extra publish cycles to hold reservation after drop-off

        self.task_pub = self.create_publisher(Task, "/task_queue", EVENT_QOS)
        self.lift_pub = self.create_publisher(LiftStatus, "/lift_status", STATE_QOS)

        self.create_service(CallLift, "/call_lift", self.call_lift)
        self.create_service(DoorOpenVerification, "/verify_lift_door_open", self.verify_door)
        self.create_service(MissionAssignment, "/assign_mission", self.assign_mission)
        self.create_service(ChargingRequest, "/request_charging", self.request_charging)
        self.create_service(RecoveryReset, "/recovery_reset", self.recovery_reset)

        self.create_timer(1.0, self.tick)
        self.create_timer(0.4, self.publish_lift_status)
        self.get_logger().info("Fleet orchestrator ready: lift reservation and mission APIs online.")

    def assign_mission(self, request, response):
        mission_id = f"mission-{uuid.uuid4().hex[:8]}"
        task = Task()
        task.header.stamp = self.get_clock().now().to_msg()
        task.mission_id = mission_id
        task.robot_id = request.robot_id or self.robot_ids[0]
        task.pickup_room = request.pickup_room
        task.delivery_room = request.delivery_room
        task.pickup_floor = request.pickup_floor
        task.delivery_floor = request.delivery_floor
        task.item_id = request.item_id
        task.priority = request.priority
        task.status = "queued"
        self.task_pub.publish(task)
        response.accepted = True
        response.mission_id = mission_id
        response.message = "Mission queued and published on /task_queue."
        return response

    def call_lift(self, request, response):
        self.sequence += 1
        reservation_id = f"lift-{uuid.uuid4().hex[:8]}"
        now = self.get_clock().now().nanoseconds * 1e-9
        reservation = LiftReservation(
            sort_key=(-int(request.priority), self.sequence, now),
            reservation_id=reservation_id,
            robot_id=request.robot_id,
            from_floor=request.from_floor,
            to_floor=request.to_floor,
            priority=request.priority,
            created_at=now,
        )
        heapq.heappush(self.reservations, reservation)
        response.accepted = True
        response.reservation_id = reservation_id
        response.eta_sec = 5 + (len(self.reservations) * 8)
        response.message = "Reservation accepted. Queue is priority then FIFO; one lift token prevents deadlock."
        self.get_logger().info(
            f"Lift request queued: {request.robot_id} floor {request.from_floor}->{request.to_floor} "
            f"priority={request.priority} reservation={reservation_id}"
        )
        return response

    def verify_door(self, request, response):
        response.is_open = self.door_state == "open" and self.lift_floor == request.floor
        response.measured_width_m = 1.05 if response.is_open else 0.0
        response.evidence = "simulated depth-camera doorway segmentation"
        return response

    def request_charging(self, request, response):
        response.accepted = True
        response.charger_id = "dock_A"
        response.dock_pose_name = "charging_dock_floor_1"
        response.eta_sec = 45
        self.get_logger().warn(f"Charging requested by {request.robot_id} at {request.battery_percent:.1f}%")
        return response

    def recovery_reset(self, request, response):
        response.reset = True
        response.next_state = "resume_or_replan"
        response.message = f"Fault {request.fault_code} acknowledged for {request.robot_id}."
        self.get_logger().warn(response.message)
        return response

    def tick(self):
        if self.active_reservation is None and self.reservations:
            self.active_reservation = heapq.heappop(self.reservations)
            self.lift_target = self.active_reservation.from_floor
            self.motion_state = "moving_to_pickup"
            self.door_state = "closed"
            self.get_logger().info(
                f"Lift token granted to {self.active_reservation.robot_id} "
                f"({self.active_reservation.reservation_id})"
            )
            return

        if self.active_reservation is None:
            self.motion_state = "idle"
            self.door_state = "open"
            return

        reservation = self.active_reservation
        if self.lift_floor != self.lift_target:
            self.lift_floor += 1 if self.lift_target > self.lift_floor else -1
            return

        if self.motion_state == "moving_to_pickup":
            self.door_state = "open"
            self.motion_state = "boarding"
            self.lift_target = reservation.to_floor
            return

        if self.motion_state == "boarding":
            self.door_state = "closed"
            self.motion_state = "moving_to_dropoff"
            return

        if self.motion_state == "moving_to_dropoff":
            self.door_state = "open"
            self.motion_state = "complete"
            return

        if self.motion_state == "complete":
            if self._complete_hold < 6:
                # Hold the reservation visible for 6 extra ticks (~6 s) so the
                # robot executive can see reserved_by + door_open at target floor.
                self._complete_hold += 1
                return
            self.get_logger().info(f"Lift reservation complete: {reservation.reservation_id}")
            self.active_reservation = None
            self._complete_hold = 0

    def publish_lift_status(self):
        msg = LiftStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.lift_id = "lift_A"
        msg.current_floor = self.lift_floor
        msg.target_floor = self.lift_target
        msg.door_state = self.door_state
        msg.motion_state = self.motion_state
        msg.reserved_by = self.active_reservation.robot_id if self.active_reservation else ""
        msg.mode = "autonomous"
        msg.waiting_robot_ids = [reservation.robot_id for reservation in sorted(self.reservations)]
        self.lift_pub.publish(msg)


def main():
    rclpy.init()
    node = FleetOrchestrator()
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
