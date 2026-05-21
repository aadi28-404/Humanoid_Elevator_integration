import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

from .qos import STATE_QOS


MAP_RESOLUTION = 0.10
MAP_ORIGIN_X = -2.8
MAP_ORIGIN_Y = -4.0
MAP_WIDTH = 112
MAP_HEIGHT = 80
ROBOT_INFLATION_RADIUS = 0.38

OCCUPIED_RECTS = [
    # Outer walls
    (-2.56, 7.96, -3.76, -3.50),
    (-2.56, 7.96, 3.50, 3.76),
    (-2.56, -2.30, -3.76, 3.76),
    (7.70, 7.96, -3.76, 3.76),
    # Room doorway wall segments
    (1.00, 3.25, -1.47, -1.23),
    (4.30, 4.75, -1.47, -1.23),
    (5.60, 6.40, -1.47, -1.23),
    (4.30, 4.75, 1.23, 1.47),
    (5.60, 6.40, 1.23, 1.47),
    # Elevator fixtures
    (-0.80, -0.58, -0.75, 0.75),
    (0.00, 0.14, -0.92, -0.72),
    (0.00, 0.14, 0.72, 0.92),
    (0.52, 0.76, -0.64, -0.34),
    # Counters and large props
    (1.45, 2.55, -2.85, -2.15),
    (4.65, 5.75, 2.15, 2.85),
]

RED_ZONE_KEEP_OUT_CIRCLES = [
    # Matches the red safety marker and the standing guest in the world.
    (2.10, 0.70, 0.90),
]


def world_to_grid(x, y):
    gx = int(round((x - MAP_ORIGIN_X) / MAP_RESOLUTION))
    gy = int(round((y - MAP_ORIGIN_Y) / MAP_RESOLUTION))
    return gx, gy


def rect_contains(rect, x, y, inflation=0.0):
    xmin, xmax, ymin, ymax = rect
    return xmin - inflation <= x <= xmax + inflation and ymin - inflation <= y <= ymax + inflation


def circle_contains(circle, x, y, inflation=0.0):
    center_x, center_y, radius = circle
    return (x - center_x) ** 2 + (y - center_y) ** 2 <= (radius + inflation) ** 2


class StaticMapPublisher(Node):
    """Publishes a hotel occupancy map used by the demo planner."""

    def __init__(self):
        super().__init__("static_map_publisher")
        self.pub = self.create_publisher(OccupancyGrid, "/map", STATE_QOS)
        self.map_data = self._build_map()
        self.create_timer(1.0, self.publish_map)
        self.get_logger().info("Static SLAM-style occupancy map ready on /map.")

    def _build_map(self):
        data = [0] * (MAP_WIDTH * MAP_HEIGHT)
        for gy in range(MAP_HEIGHT):
            for gx in range(MAP_WIDTH):
                wx = MAP_ORIGIN_X + gx * MAP_RESOLUTION
                wy = MAP_ORIGIN_Y + gy * MAP_RESOLUTION
                occupied = any(rect_contains(rect, wx, wy, ROBOT_INFLATION_RADIUS) for rect in OCCUPIED_RECTS)
                occupied = occupied or any(
                    circle_contains(circle, wx, wy, ROBOT_INFLATION_RADIUS)
                    for circle in RED_ZONE_KEEP_OUT_CIRCLES
                )
                if occupied:
                    data[gy * MAP_WIDTH + gx] = 100
        return data

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = MAP_RESOLUTION
        msg.info.width = MAP_WIDTH
        msg.info.height = MAP_HEIGHT
        msg.info.origin.position.x = MAP_ORIGIN_X
        msg.info.origin.position.y = MAP_ORIGIN_Y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.map_data
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = StaticMapPublisher()
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
