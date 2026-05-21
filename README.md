# Humanoid Delivery Robot ROS2/Gazebo Assessment

This workspace implements a runnable ROS2 Humble simulation for a Unitree G1 humanoid delivery robot deployed in hotel and healthcare corridors.

## Build

```bash
./scripts/build.sh --clean
```

## Run The Gazebo Demo

```bash
./scripts/launch_sim.sh --gui
```

If Gazebo GUI is heavy on your machine, run the same simulation headless:

```bash
./scripts/launch_sim.sh --headless
```

Stop stale Gazebo/ROS demo processes:

```bash
./scripts/stop_sim.sh
```

## What The Simulation Is Supposed To Show

The visible mission is:

1. The robot starts near the elevator lobby.
2. A speech/LLM node creates a task: pick up `med_kit` from room `101`.
3. The Unitree G1 model drives to the green pickup marker and performs a simulated VLA grasp.
4. The robot requests elevator `lift_A`; the fleet orchestrator reserves the lift with priority/FIFO scheduling.
5. The robot verifies the elevator door, boards/exits in the state machine, and changes its logical floor.
6. The robot drives to the blue delivery marker at room `302` and completes the handoff.
7. If the simulated human is inside the safety zone, `/emergency_stop` is asserted and `/cmd_vel` is forced to zero.

The demo is intentionally a systems assessment simulation, not a production humanoid physics model. It proves the ROS2 architecture, topic/service/action contracts, elevator arbitration, mission execution, and safety watchdogs. The Gazebo scene gives a visual trace of that flow.

The default Gazebo model is the vendored Unitree G1 23-DoF mesh model with fixed standing joints, simplified Gazebo collision, and planar `/cmd_vel` motion for assessment use.

The launch file starts Gazebo, spawns the Unitree G1 humanoid model with LiDAR and depth camera simulation, and runs the core ROS2 stack:

- `/cmd_vel`, `/tf`, `/map`, `/robot_state`, `/human_detection`, `/task_queue`, `/lift_status`, `/arm_controller`, `/emergency_stop`
- services: `/call_lift`, `/verify_lift_door_open`, `/assign_mission`, `/request_charging`, `/recovery_reset`
- actions: `/navigate_to_pose`, `/pick_object`, `/deliver_item`, `/retry_mission`

The speech/LLM interface injects a demo mission after startup. You can also submit one manually:

```bash
ros2 service call /assign_mission humanoid_delivery_interfaces/srv/MissionAssignment \
"{robot_id: humanoid_1, pickup_room: '101', delivery_room: '302', pickup_floor: 1, delivery_floor: 3, item_id: med_kit, priority: 8}"
```

## Run Without Gazebo

```bash
ros2 launch humanoid_delivery_sim core_nodes.launch.py
```

## Two-Robot Elevator Arbitration Demo

```bash
ros2 launch humanoid_delivery_sim two_robot_lift_demo.launch.py
```

This runs two robot executives against the same `/call_lift` service. The fleet orchestrator grants a single lift token using priority then FIFO ordering, preventing two robots from entering the same elevator reservation at once.

## Architecture

See [docs/architecture.md](docs/architecture.md).

## Repository Notes

ROS 2 build output, Gazebo logs, Python caches, and motion recorder traces are generated locally and are excluded from version control. Vendored Unitree assets under `src/humanoid_delivery_sim/third_party/` keep their own license files.
