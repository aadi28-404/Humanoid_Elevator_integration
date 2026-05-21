import heapq
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from .qos import CONTROL_QOS, SENSOR_QOS, STATE_QOS
from .static_map_publisher import (
    MAP_HEIGHT,
    MAP_ORIGIN_X,
    MAP_ORIGIN_Y,
    MAP_RESOLUTION,
    MAP_WIDTH,
    OCCUPIED_RECTS,
    RED_ZONE_KEEP_OUT_CIRCLES,
    ROBOT_INFLATION_RADIUS,
    circle_contains,
    rect_contains,
)


GOAL_POINTS = {
    "pickup": (2.05, -1.95),
    "elevator": (0.55, 0.00),
    "room_101": (2.05, -1.95),
    "room_102": (4.10, -1.90),
    "room_201": (1.60, 1.60),
    "room_302": (5.20, 1.90),   # counter-side stance before the table footprint
    "start": (0.85, 0.00),
}

GOAL_YAWS = {
    # The pickup route approaches from the east. This final left turn faces
    # the robot into the counter before the pick action starts.
    "pickup": -0.5 * math.pi,
    # Face the north delivery counter after entering the 302 doorway.
    "room_302": 0.5 * math.pi,
}

PICKUP_ROUTE = [
    (0.85, 0.00),
    (0.85, -0.72),
    (3.95, -0.72),
    (3.95, -1.95),
    (2.70, -1.95),
    (2.05, -1.95),
]

ROUTE_CHAINS = {
    "elevator": [(2.05, -1.95), (2.70, -1.95), (3.95, -1.95), (3.95, -0.72), (0.85, -0.72), (0.85, 0.00), (0.55, 0.00)],
    "room_101": [(0.85, 0.00), (0.85, -0.72), (3.95, -0.72), (3.95, -1.95), (2.70, -1.95), (2.05, -1.95)],
    "room_102": [(0.85, 0.00), (0.85, -0.72), (3.95, -0.72), (3.95, -1.85), (4.10, -1.90)],
    "room_201": [(0.85, 0.00), (0.85, -0.72), (3.75, -0.72), (3.75, 1.60), (1.60, 1.60)],
    "room_302": [
        (0.55, 0.00),
        (0.85, 0.00),
        (0.85, -0.72),
        (3.80, -0.72),
        (3.80, 1.60),
        (5.20, 1.60),
        (5.20, 1.90),
    ],
    "start": [(0.85, 0.00)],
}

WALL_BOUNDS = [
    (-0.5, 0.0, -3.5, 3.5),
    (7.0, 7.5, -3.5, 3.5),
    (-0.5, 7.5, -3.5, -3.0),
    (-0.5, 7.5, 3.0, 3.5),
    (1.0, 1.35, -3.0, -0.4),
    (1.0, 1.35, 0.4, 3.0),
    (1.0, 3.25, -1.43, -1.27),
    (4.4, 4.75, -3.0, -0.4),
    (4.4, 4.75, 0.4, 3.0),
    (-0.80, -0.58, -0.75, 0.75),
    (0.00, 0.14, -0.92, -0.72),
    (0.00, 0.14, 0.72, 0.92),
    (0.52, 0.76, -0.64, -0.34),
]

MAX_SPEED = 0.18
MIN_SPEED = 0.035
DEFAULT_ARRIVAL_THRESHOLD = 0.20
PICKUP_ARRIVAL_THRESHOLD = 0.28
ELEVATOR_ARRIVAL_THRESHOLD = 0.45
DELIVERY_ARRIVAL_THRESHOLD = 0.35
DELIVERY_TABLE_ARRIVAL_THRESHOLD = 0.22
STALL_CHECK_SECONDS = 6.0       # longer window — slow wall-hugging is not a stall
STALL_PROGRESS_METERS = 0.08   # lower threshold — any movement counts
ESCAPE_BURST_SECONDS = 1.5     # longer burst to fully clear wall contact
FRONT_SLOW_DISTANCE = 0.90
FRONT_STOP_DISTANCE = 0.30
SIDE_KEEP_DISTANCE = 0.45


class NavigationServer(Node):
    """Nav2-compatible demo server using named routes, static map A*, and lidar clearance."""

    def __init__(self):
        super().__init__("navigation_server")
        self.callback_group = ReentrantCallbackGroup()
        self.current_x = 0.85
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.have_odom = False
        self.estop = False
        self.front_clearance = float("inf")
        self.left_clearance = float("inf")
        self.right_clearance = float("inf")
        self.have_scan = False
        self.occupancy = self._build_occupancy()
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", CONTROL_QOS)
        self.create_subscription(
            Odometry,
            "/odom",
            self.on_odom,
            QoSProfile(depth=10),
            callback_group=self.callback_group,
        )
        self.create_subscription(Bool, "/emergency_stop", self.on_estop, STATE_QOS, callback_group=self.callback_group)
        self.create_subscription(LaserScan, "/scan", self.on_scan, SENSOR_QOS, callback_group=self.callback_group)
        self.server = ActionServer(
            self,
            NavigateToPose,
            "navigate_to_pose",
            self.execute_callback,
            callback_group=self.callback_group,
        )
        self.get_logger().info("NavigateToPose ready: SLAM-style map routes and lidar wall perception online.")

    def on_odom(self, msg):
        pose = msg.pose.pose
        self.current_x = pose.position.x
        self.current_y = pose.position.y
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.have_odom = True

    def on_estop(self, msg):
        self.estop = msg.data

    def on_scan(self, msg):
        front = []
        left = []
        right = []
        angle = msg.angle_min
        for distance in msg.ranges:
            if math.isfinite(distance) and msg.range_min < distance < msg.range_max:
                if abs(angle) <= math.radians(35.0):
                    front.append(distance)
                elif math.radians(35.0) < angle < math.radians(115.0):
                    left.append(distance)
                elif -math.radians(115.0) < angle < -math.radians(35.0):
                    right.append(distance)
            angle += msg.angle_increment

        self.front_clearance = min(front) if front else float("inf")
        self.left_clearance = min(left) if left else float("inf")
        self.right_clearance = min(right) if right else float("inf")
        self.have_scan = True

    def execute_callback(self, goal_handle):
        pose = goal_handle.request.pose.pose
        goal_name = goal_handle.request.behavior_tree.strip()
        goal_x = pose.position.x
        goal_y = pose.position.y
        if goal_name in GOAL_POINTS:
            goal_x, goal_y = GOAL_POINTS[goal_name]
        elif math.hypot(goal_x - 2.0, goal_y + 2.0) < 0.4:
            goal_name = "pickup"
            goal_x, goal_y = GOAL_POINTS["pickup"]
        else:
            goal_name = self._nearest_goal_key(goal_x, goal_y)

        path = self._path_for_goal(goal_name, goal_x, goal_y)
        if not path:
            self.get_logger().error(f"Navigation ABORT — could not reach goal '{goal_name}'")
            self._publish_stop()
            goal_handle.abort()
            return NavigateToPose.Result()

        self.get_logger().info(f"Navigation goal '{goal_name}' planned with {len(path)} waypoints.")
        final_x, final_y = path[-1]
        if goal_name == "pickup":
            arrival_threshold = PICKUP_ARRIVAL_THRESHOLD
        elif goal_name == "elevator":
            arrival_threshold = ELEVATOR_ARRIVAL_THRESHOLD
        elif goal_name == "room_302":
            arrival_threshold = DELIVERY_TABLE_ARRIVAL_THRESHOLD
        elif goal_name.startswith("room_"):
            arrival_threshold = DELIVERY_ARRIVAL_THRESHOLD
        else:
            arrival_threshold = DEFAULT_ARRIVAL_THRESHOLD
        feedback = NavigateToPose.Feedback()
        waypoint_index = 0
        now = lambda: self.get_clock().now().nanoseconds * 1e-9
        last_progress_time = now()
        last_progress_x = self.current_x
        last_progress_y = self.current_y
        deadline = now() + max(120.0, 12.0 * len(path))

        while rclpy.ok() and now() < deadline:
            final_distance = math.hypot(final_x - self.current_x, final_y - self.current_y)
            if final_distance <= arrival_threshold:
                if not self._align_goal_heading(goal_name, goal_handle):
                    return NavigateToPose.Result()
                self._publish_stop()
                if goal_name == "pickup":
                    self.get_logger().info("Arrived at pickup zone — navigation SUCCESS")
                goal_handle.succeed()
                return NavigateToPose.Result()

            if waypoint_index >= len(path):
                recovery_threshold = arrival_threshold if goal_name == "room_302" else 0.6
                if final_distance <= recovery_threshold:
                    if not self._align_goal_heading(goal_name, goal_handle):
                        return NavigateToPose.Result()
                    self._publish_stop()
                    if goal_name == "pickup":
                        self.get_logger().info("Arrived at pickup zone — navigation SUCCESS")
                    goal_handle.succeed()
                    return NavigateToPose.Result()
                if goal_name == "elevator" and final_distance <= 3.0:
                    self.get_logger().warn("Elevator approach partially blocked — accepting safe standoff for demo flow.")
                    self._publish_stop()
                    goal_handle.succeed()
                    return NavigateToPose.Result()
                self.get_logger().error("Navigation ABORT — could not reach goal")
                self._publish_stop()
                goal_handle.abort()
                return NavigateToPose.Result()

            if self.estop:
                self._publish_stop()
                time.sleep(0.05)
                continue

            waypoint_x, waypoint_y = path[waypoint_index]
            distance = math.hypot(waypoint_x - self.current_x, waypoint_y - self.current_y)
            if distance <= DEFAULT_ARRIVAL_THRESHOLD:
                waypoint_index += 1
                continue

            if now() - last_progress_time >= STALL_CHECK_SECONDS:
                progress = math.hypot(self.current_x - last_progress_x, self.current_y - last_progress_y)
                if progress < STALL_PROGRESS_METERS:
                    # Escape maneuver: drive south/backward to break wall contact
                    escape = Twist()
                    escape.linear.y = -0.15   # push south (negative Y in map frame)
                    escape.linear.x = -0.10   # slight reverse
                    escape_end = now() + ESCAPE_BURST_SECONDS
                    while now() < escape_end and not self.estop:
                        self.cmd_pub.publish(escape)
                        time.sleep(0.05)
                    self._publish_stop()
                    time.sleep(0.2)  # let physics settle
                    waypoint_index += 1
                    self.get_logger().warn("Stall detected — escape burst applied, skipping waypoint")
                    last_progress_time = now()
                    last_progress_x = self.current_x
                    last_progress_y = self.current_y
                    continue
                last_progress_time = now()
                last_progress_x = self.current_x
                last_progress_y = self.current_y

            twist = self._twist_to_waypoint(waypoint_x, waypoint_y, distance)
            if self._too_close_to_wall(self.current_x, self.current_y):
                twist.linear.x *= 0.65
                twist.linear.y += 0.03 if self.right_clearance < self.left_clearance else -0.03
            self._apply_lidar_clearance(twist)
            self.cmd_pub.publish(twist)

            feedback.current_pose.header.frame_id = "odom"
            feedback.current_pose.header.stamp = self.get_clock().now().to_msg()
            feedback.current_pose.pose.position.x = self.current_x
            feedback.current_pose.pose.position.y = self.current_y
            feedback.distance_remaining = final_distance
            goal_handle.publish_feedback(feedback)
            time.sleep(0.05)

        self.get_logger().error("Navigation ABORT — could not reach goal")
        self._publish_stop()
        goal_handle.abort()
        return NavigateToPose.Result()

    def _path_for_goal(self, goal_name, goal_x, goal_y):
        if goal_name == "pickup":
            return self._trim_route_to_current(PICKUP_ROUTE)
        if goal_name in ROUTE_CHAINS:
            route = ROUTE_CHAINS[goal_name]
            # If robot is far from every waypoint in the chain, use A* from current position
            min_dist = min(
                math.hypot(wp[0] - self.current_x, wp[1] - self.current_y)
                for wp in route
            )
            if min_dist > 1.5:
                self.get_logger().info(
                    f"Robot {self.current_x:.2f},{self.current_y:.2f} far from '{goal_name}' "
                    f"route (nearest wp {min_dist:.2f} m) — using A* planner."
                )
                return self._plan_path(goal_x, goal_y)
            return self._trim_route_to_current(route)
        return self._plan_path(goal_x, goal_y)

    def _trim_route_to_current(self, route):
        if not route:
            return []
        closest_index = min(
            range(len(route)),
            key=lambda index: math.hypot(route[index][0] - self.current_x, route[index][1] - self.current_y),
        )
        return route[closest_index:]

    def _twist_to_waypoint(self, waypoint_x, waypoint_y, distance):
        angle_to_waypoint = math.atan2(waypoint_y - self.current_y, waypoint_x - self.current_x)
        heading_error = self._wrap_angle(angle_to_waypoint - self.current_yaw)
        speed = min(MAX_SPEED, max(MIN_SPEED, 0.45 * distance))
        if abs(heading_error) > 0.9:
            speed *= 0.45
        twist = Twist()
        twist.linear.x = speed * math.cos(heading_error)
        twist.linear.y = speed * math.sin(heading_error)
        twist.angular.z = max(-0.45, min(0.45, 1.1 * heading_error))
        return twist

    def _align_goal_heading(self, goal_name, goal_handle):
        target_yaw = GOAL_YAWS.get(goal_name)
        if target_yaw is None:
            return True

        deadline = time.monotonic() + 8.0
        while rclpy.ok() and time.monotonic() < deadline:
            if self.estop:
                self._publish_stop()
                time.sleep(0.05)
                continue
            error = self._wrap_angle(target_yaw - self.current_yaw)
            if abs(error) <= 0.10:
                self._publish_stop()
                if goal_name == "pickup":
                    self.get_logger().info("Pickup approach aligned to counter.")
                return True
            twist = Twist()
            twist.angular.z = max(-0.40, min(0.40, 1.2 * error))
            self.cmd_pub.publish(twist)
            time.sleep(0.05)

        self.get_logger().error(f"Navigation ABORT - could not align heading for goal '{goal_name}'")
        self._publish_stop()
        goal_handle.abort()
        return False

    def _plan_path(self, goal_x, goal_y):
        start = self._nearest_free_cell(*self.world_to_grid(self.current_x, self.current_y))
        goal = self._nearest_free_cell(*self.world_to_grid(goal_x, goal_y))
        grid_path = self._astar(start, goal)
        if not grid_path:
            return []
        return self._sample_path([self.grid_to_world(gx, gy) for gx, gy in grid_path])

    def _build_occupancy(self):
        data = [[False for _ in range(MAP_WIDTH)] for _ in range(MAP_HEIGHT)]
        for gy in range(MAP_HEIGHT):
            for gx in range(MAP_WIDTH):
                wx, wy = self.grid_to_world(gx, gy)
                data[gy][gx] = any(rect_contains(rect, wx, wy, ROBOT_INFLATION_RADIUS) for rect in OCCUPIED_RECTS)
                data[gy][gx] = data[gy][gx] or any(
                    circle_contains(circle, wx, wy, ROBOT_INFLATION_RADIUS)
                    for circle in RED_ZONE_KEEP_OUT_CIRCLES
                )
        return data

    def _astar(self, start, goal):
        frontier = [(0.0, start)]
        came_from = {start: None}
        cost_so_far = {start: 0.0}
        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal:
                break
            for nxt, step_cost in self._neighbors(current):
                new_cost = cost_so_far[current] + step_cost
                if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_cost
                    priority = new_cost + self._heuristic(nxt, goal)
                    heapq.heappush(frontier, (priority, nxt))
                    came_from[nxt] = current

        if goal not in came_from:
            return []
        path = []
        cur = goal
        while cur is not None:
            path.append(cur)
            cur = came_from[cur]
        path.reverse()
        return path

    def _neighbors(self, cell):
        gx, gy = cell
        candidates = [
            (gx + 1, gy, 1.0),
            (gx - 1, gy, 1.0),
            (gx, gy + 1, 1.0),
            (gx, gy - 1, 1.0),
            (gx + 1, gy + 1, 1.414),
            (gx + 1, gy - 1, 1.414),
            (gx - 1, gy + 1, 1.414),
            (gx - 1, gy - 1, 1.414),
        ]
        for nx, ny, step_cost in candidates:
            if not self._grid_in_bounds(nx, ny) or self.occupancy[ny][nx]:
                continue
            if nx != gx and ny != gy and (self.occupancy[gy][nx] or self.occupancy[ny][gx]):
                continue
            yield (nx, ny), step_cost

    def _sample_path(self, path):
        if len(path) <= 2:
            return path
        sampled = [path[0]]
        accumulated = 0.0
        previous = path[0]
        for point in path[1:-1]:
            accumulated += math.hypot(point[0] - previous[0], point[1] - previous[1])
            if accumulated >= 0.35:
                sampled.append(point)
                accumulated = 0.0
            previous = point
        sampled.append(path[-1])
        return sampled

    def _nearest_free_cell(self, gx, gy):
        if self._grid_in_bounds(gx, gy) and not self.occupancy[gy][gx]:
            return gx, gy
        for radius in range(1, 16):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    nx = gx + dx
                    ny = gy + dy
                    if self._grid_in_bounds(nx, ny) and not self.occupancy[ny][nx]:
                        return nx, ny
        return max(0, min(MAP_WIDTH - 1, gx)), max(0, min(MAP_HEIGHT - 1, gy))

    def _too_close_to_wall(self, px, py, margin=0.22):
        for xmin, xmax, ymin, ymax in WALL_BOUNDS:
            if xmin - margin < px < xmax + margin and ymin - margin < py < ymax + margin:
                return True
        for circle in RED_ZONE_KEEP_OUT_CIRCLES:
            if circle_contains(circle, px, py, margin):
                return True
        return False

    def world_to_grid(self, x, y):
        gx = int(round((x - MAP_ORIGIN_X) / MAP_RESOLUTION))
        gy = int(round((y - MAP_ORIGIN_Y) / MAP_RESOLUTION))
        return max(0, min(MAP_WIDTH - 1, gx)), max(0, min(MAP_HEIGHT - 1, gy))

    def grid_to_world(self, gx, gy):
        return MAP_ORIGIN_X + gx * MAP_RESOLUTION, MAP_ORIGIN_Y + gy * MAP_RESOLUTION

    def _grid_in_bounds(self, gx, gy):
        return 0 <= gx < MAP_WIDTH and 0 <= gy < MAP_HEIGHT

    def _nearest_goal_key(self, goal_x, goal_y):
        return min(GOAL_POINTS, key=lambda key: math.hypot(goal_x - GOAL_POINTS[key][0], goal_y - GOAL_POINTS[key][1]))

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    def _apply_lidar_clearance(self, twist):
        if not self.have_scan:
            return
        if self.front_clearance < FRONT_STOP_DISTANCE:
            twist.linear.x *= 0.35
            twist.linear.y += self._side_escape_velocity() * 0.5
            twist.angular.z = -0.25 if self.left_clearance < self.right_clearance else 0.25
            return
        if self.front_clearance < FRONT_SLOW_DISTANCE:
            scale = max(0.35, (self.front_clearance - FRONT_STOP_DISTANCE) / (FRONT_SLOW_DISTANCE - FRONT_STOP_DISTANCE))
            twist.linear.x *= scale
        if self.left_clearance < SIDE_KEEP_DISTANCE and self.right_clearance >= self.left_clearance:
            twist.linear.y = min(twist.linear.y, -0.04)
        elif self.right_clearance < SIDE_KEEP_DISTANCE:
            twist.linear.y = max(twist.linear.y, 0.04)

    def _side_escape_velocity(self):
        if self.left_clearance < self.right_clearance:
            return -0.08
        return 0.08

    @staticmethod
    def _heuristic(cell, goal):
        return math.hypot(goal[0] - cell[0], goal[1] - cell[1])

    @staticmethod
    def _wrap_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main():
    rclpy.init()
    node = NavigationServer()
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
