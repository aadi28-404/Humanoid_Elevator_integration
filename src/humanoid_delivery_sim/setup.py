import glob as _glob

from setuptools import setup

package_name = "humanoid_delivery_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", _glob.glob("launch/*.py")),
        (f"share/{package_name}/worlds", _glob.glob("worlds/*.world")),
        (f"share/{package_name}/config", _glob.glob("config/*.yaml")),
        (f"share/{package_name}/urdf", _glob.glob("urdf/*.xacro") + _glob.glob("urdf/*.urdf")),
        (
            f"share/{package_name}/meshes/unitree_g1",
            _glob.glob("meshes/unitree_g1/*.STL")
            + _glob.glob("meshes/unitree_g1/*.stl")
            + _glob.glob("meshes/unitree_g1/*.dae"),
        ),
        (f"share/{package_name}/third_party/unitree_ros", _glob.glob("third_party/unitree_ros/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Candidate",
    maintainer_email="candidate@example.com",
    description="Humanoid hotel delivery robot Gazebo simulation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fleet_orchestrator = humanoid_delivery_sim.fleet_orchestrator:main",
            "robot_executive = humanoid_delivery_sim.robot_executive:main",
            "sim_perception = humanoid_delivery_sim.sim_perception:main",
            "safety_controller = humanoid_delivery_sim.safety_controller:main",
            "manipulation_server = humanoid_delivery_sim.manipulation_server:main",
            "speech_llm_interface = humanoid_delivery_sim.speech_llm_interface:main",
            "navigation_server = humanoid_delivery_sim.navigation_server:main",
            "static_map_publisher = humanoid_delivery_sim.static_map_publisher:main",
            "joint_motion_demo = humanoid_delivery_sim.joint_motion_demo:main",
            "g1_gait_controller = humanoid_delivery_sim.g1_gait_controller:main",
            "motion_recorder = humanoid_delivery_sim.motion_recorder:main",
        ],
    },
)
