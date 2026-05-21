import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory("humanoid_delivery_sim")
    world = os.path.join(pkg, "worlds", "hotel_healthcare.world")

    # Proper Unitree G1 23-DOF humanoid model
    urdf = os.path.join(pkg, "urdf", "unitree_g1_23dof_delivery.urdf.xacro")
    robot_description = ParameterValue(Command(["xacro ", urdf]), value_type=str)
    gui = LaunchConfiguration("gui")

    gzserver = ExecuteProcess(
        cmd=[
            "gzserver",
            world,
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
            "-s", "libgazebo_ros_force_system.so",
        ],
        output="screen",
    )

    gzclient = ExecuteProcess(
        cmd=["gzclient"],
        condition=IfCondition(gui),
        output="screen",
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "unitree_g1_delivery",
            "-x", "0.85",
            "-y", "0.0",
            "-z", "0.96",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true", description="Set false to run gzserver headless."),
            SetEnvironmentVariable("QT_X11_NO_MITSHM", "1"),
            SetEnvironmentVariable("OGRE_RTT_MODE", "Copy"),
            SetEnvironmentVariable("__GL_THREADED_OPTIMIZATIONS", "0"),
            gzserver,
            # gzclient after 5s — enough time for the server to open a display connection
            TimerAction(period=5.0, actions=[gzclient]),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description, "use_sim_time": True}],
                output="screen",
            ),
            # Spawn the robot after 10s — Gazebo OGRE rendering scene must be fully
            # initialised before spawn or it crashes: Assertion `px != 0' failed (SIGABRT)
            TimerAction(period=10.0, actions=[spawn_robot]),
            Node(package="humanoid_delivery_sim", executable="fleet_orchestrator",   parameters=[{"use_sim_time": True}], output="screen"),
            Node(package="humanoid_delivery_sim", executable="navigation_server",      parameters=[{"use_sim_time": True}], output="screen"),
            Node(package="humanoid_delivery_sim", executable="manipulation_server",  parameters=[{"use_sim_time": True}], output="screen"),
            Node(package="humanoid_delivery_sim", executable="sim_perception",       parameters=[{"use_sim_time": True}], output="screen"),
            Node(package="humanoid_delivery_sim", executable="safety_controller",    parameters=[{"use_sim_time": True}], output="screen"),
            Node(package="humanoid_delivery_sim", executable="robot_executive",      parameters=[{"use_sim_time": True}], output="screen"),
            Node(package="humanoid_delivery_sim", executable="static_map_publisher", parameters=[{"use_sim_time": True}], output="screen"),
            Node(package="humanoid_delivery_sim", executable="motion_recorder",      parameters=[{"use_sim_time": True}], output="screen"),
            # Mission trigger: start the demo run after 25s (15s after robot spawns)
            TimerAction(period=25.0, actions=[
                Node(package="humanoid_delivery_sim", executable="speech_llm_interface",
                     parameters=[{"use_sim_time": True}], output="screen"),
            ]),
        ]
    )
