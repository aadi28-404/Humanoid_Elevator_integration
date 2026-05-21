from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="humanoid_delivery_sim", executable="fleet_orchestrator", output="screen"),
            Node(package="humanoid_delivery_sim", executable="navigation_server", output="screen"),
            Node(package="humanoid_delivery_sim", executable="manipulation_server", output="screen"),
            Node(package="humanoid_delivery_sim", executable="sim_perception", output="screen"),
            Node(package="humanoid_delivery_sim", executable="safety_controller", output="screen"),
            Node(package="humanoid_delivery_sim", executable="robot_executive", output="screen"),
            Node(package="humanoid_delivery_sim", executable="speech_llm_interface", output="screen"),
        ]
    )
