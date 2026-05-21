from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="humanoid_delivery_sim", executable="fleet_orchestrator", output="screen"),
            Node(package="humanoid_delivery_sim", executable="navigation_server", output="screen"),
            Node(package="humanoid_delivery_sim", executable="manipulation_server", output="screen"),
            Node(package="humanoid_delivery_sim", executable="sim_perception", output="screen"),
            Node(package="humanoid_delivery_sim", executable="safety_controller", output="screen"),
            Node(
                package="humanoid_delivery_sim",
                executable="robot_executive",
                parameters=[{"robot_id": "humanoid_1"}],
                output="screen",
            ),
            Node(
                package="humanoid_delivery_sim",
                executable="robot_executive",
                name="robot_executive_2",
                parameters=[{"robot_id": "humanoid_2"}],
                output="screen",
            ),
            TimerAction(
                period=2.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "service",
                            "call",
                            "/assign_mission",
                            "humanoid_delivery_interfaces/srv/MissionAssignment",
                            "{robot_id: humanoid_1, pickup_room: '101', delivery_room: '302', pickup_floor: 1, delivery_floor: 3, item_id: med_kit, priority: 8}",
                        ],
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=2.5,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "service",
                            "call",
                            "/assign_mission",
                            "humanoid_delivery_interfaces/srv/MissionAssignment",
                            "{robot_id: humanoid_2, pickup_room: '102', delivery_room: '201', pickup_floor: 1, delivery_floor: 2, item_id: linens, priority: 4}",
                        ],
                        output="screen",
                    )
                ],
            ),
        ]
    )
