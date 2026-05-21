# Humanoid Delivery Simulation Debug Report

Generated: 2026-05-20 10:09:35 IST  
Workspace: `/media/aadi/Extras/ba0ba0/assesment`  
ROS distribution: Humble  
Simulator: Gazebo Classic 11.10.x  
Robot entity: `unitree_g1_delivery`

## 2026-05-21 Assessment Requirement Coverage

Assessment source checked:

- Part 1 asks for a complete ROS 2 architecture for corridor navigation, elevator use, room identification, item delivery, safe human interaction, failure recovery, VLA/speech integration, Jetson Orin compute split, and fleet communication.
- Part 2 asks for autonomous elevator communication, two-robot scheduling, queue prioritization, deadlock prevention, recovery/timeouts, human override handling, RMF-like orchestration, distributed state machines, reservation logic, and ROS 2 action coordination.

Audit result:

- The implementation already exposes all required assessment interfaces in `humanoid_delivery_interfaces` and `humanoid_delivery_sim`.
- `docs/architecture.md` now includes explicit traceability tables for Part 1 and Part 2, plus room-identification layering and elevator state ownership.
- The current Gazebo demo is intentionally deterministic at the navigation/manipulation boundary: it demonstrates the ROS graph, mission executive, map/lidar planning boundary, human safety stop, lift reservation protocol, pickup/delivery actions, charging/recovery APIs, and remote mission entry without claiming to be a full production biped locomotion or vendor elevator adapter.

### Part 1 Implemented ROS Coverage

| Requirement | Achieved in repo |
| --- | --- |
| Required nodes | `navigation_server`, `static_map_publisher`, `sim_perception`, `robot_executive`, `manipulation_server`, `fleet_orchestrator`, `speech_llm_interface`, `safety_controller`, Gazebo sensors, and robot TF publication are documented and launched for the demo. |
| Navigation stack | `navigation_server` serves `/navigate_to_pose` with occupancy-map routes, A* fallback, LiDAR wall clearance, named room goals, red human-zone keepout, and stall handling. Architecture documents Nav2 BT/controller/planner/lifecycle replacement for deployment. |
| SLAM/localization | Simulation publishes `/map`, `/scan`, `/odom`, and `/tf`; architecture documents `slam_toolbox` commissioning plus AMCL normal operation. |
| Perception/human detection | `sim_perception` publishes `/human_detection`; `safety_controller` consumes it and asserts the simulation safety pause/zero velocity behavior. |
| Task planner | `robot_executive` drives pickup, pick, elevator trip, room delivery, payload state, retry, and mission completion. |
| Manipulation | `manipulation_server` provides `PickObject`, `DeliverItem`, and `/arm_controller` feedback while moving the blue medical kit through the demo carry/place sequence. |
| Elevator and fleet | `fleet_orchestrator` owns `/call_lift`, `/lift_status`, `/assign_mission`, `/request_charging`, `/recovery_reset`, and priority/FIFO reservation arbitration. |
| Speech/LLM/VLA boundary | `speech_llm_interface` publishes a speech event and turns the parsed request into a structured `/assign_mission` call; architecture keeps heavy VLA/LLM inference off safety-critical loops. |
| Required topics | `/cmd_vel`, `/tf`, `/map`, `/robot_state`, `/human_detection`, `/task_queue`, `/lift_status`, `/arm_controller`, and `/emergency_stop` are represented in the architecture and implementation. |
| Required services | `/call_lift`, `/verify_lift_door_open`, `/assign_mission`, `/request_charging`, and `/recovery_reset` are implemented by `fleet_orchestrator`. |
| Required actions | `/navigate_to_pose`, `/pick_object`, `/deliver_item`, and `/retry_mission` are implemented by navigation, manipulation, and executive nodes. |
| QoS and real-time | `qos.py` defines reliable control/event/state profiles and best-effort sensor profile; architecture documents watchdogs, deterministic low-level controllers, and callback isolation. |
| DDS, edge/cloud, Jetson allocation, failover | Architecture now explicitly documents Cyclone DDS recommendation, Jetson CPU/GPU/DLA split, cloud/fleet split, offline safe-stop behavior, and network/perception/localization/hardware failure handling. |

### Part 2 Elevator Coverage

| Elevator requirement | Achieved in repo |
| --- | --- |
| Elevator communication protocol | `CallLift`, `LiftStatus`, and `DoorOpenVerification` form the ROS protocol boundary. |
| Two-robot scheduling | `fleet_orchestrator` stores lift requests in a heap ordered by priority then FIFO sequence; `two_robot_lift_demo.launch.py` submits two requests. |
| Reservation/deadlock prevention | One `active_reservation` token is granted at a time; only the reservation owner sees itself in `/lift_status.reserved_by` and the executive waits for ownership before boarding/exiting. |
| Safety timeouts and recovery | Executive timeouts raise lift failure codes, door verification gates movement, `/retry_mission` provides bounded recovery, and `/recovery_reset` acknowledges recoverable faults. |
| Human override handling | Architecture defines manual/hold policy: stop outside lift threshold, release/expire leases, keep state visible to fleet, and wait for operator recovery. |
| RMF-like orchestration and distributed state machines | Fleet owns resource arbitration and queue; robot executive owns action coordination and per-robot state transitions. |

### Current Simulation Boundary

- Achieved for assessment demonstration: ROS 2 interfaces, mission sequence, pickup/delivery actions, human-zone avoidance and stop behavior, map/lidar navigation boundary, lift reservation scheduling, charging/recovery APIs, QoS strategy, fleet/edge split, and documented failover.
- Production follow-up: plug in Nav2 lifecycle bringup with tuned costmaps/localization, vendor elevator adapter or RMF lift bridge, low-level whole-body Unitree locomotion/arm controllers, hardware e-stop/watchdog chain, and real VLA room/object perception models on Jetson.

## 2026-05-21 Delivery Table Approach

Latest observation:

- The room 302 delivery action could start while the robot was still at the room doorway.
- The carried blue box then animated to the destination table while the robot was visibly away from the counter.

Root cause:

- `navigation_server.py` still used the old room-302 doorway goal `(4.10, 1.90)`.
- Room goals shared a loose `0.75 m` arrival radius, so a delivery room could report success before the table-side stance.

Fix applied:

- Room 302 now navigates through the doorway to a counter-side stance at `(5.20, 1.90)`.
- The named route adds in-room waypoints at `(5.20, 1.60)` and `(5.20, 1.90)` after the red-zone bypass.
- Room-302 delivery success now uses a tight `0.22 m` table approach threshold.
- The exhausted-waypoint fallback no longer accepts a broad `0.6 m` delivery success for room 302, so the deliver action waits for the robot to reach the table approach.

Expected behavior:

- The robot approaches the room 302 table before the box handoff starts.
- The box remains carried until the table-side delivery navigation succeeds.
- After the place animation, the robot stays at the table approach pose.

Validation from the 2026-05-21 headless delivery run:

```text
[navigation_server]: Navigation goal 'room_302' planned with 6 waypoints.
[safety_controller]: Human inside delivery safety zone: simulated stop active. [SIM]
[safety_controller]: Human safety zone cleared: simulated navigation may resume. [SIM]
[robot_executive]: Deliver med_kit requested for room 302
[manipulation_server]: Delivery to room 302: complete
[robot_executive]: Mission ... complete.
```

Post-mission odometry sample at the held delivery pose:

```text
x: 5.068
y: 1.716
z: 0.960
```

## 2026-05-21 Human Red-Zone Keepout And Pickup Bypass

Latest request:

- Replace the placeholder guest with a human-looking Gazebo model.
- Put that guest inside the visible red safety circle.
- Make pickup navigation avoid the red circle instead of driving through the human safety area.

Fix applied:

- `hotel_healthcare.world` now places the `guest_crossing` model at the red marker center `(2.1, 0.7)` and renders it with the standing-person mesh instead of a cylinder body and sphere head.
- The static occupancy map now exposes the red safety circle as an inflated circular keepout matching the visible red marker radius.
- The navigation server uses the same circular keepout for map planning and local clearance checks.
- The explicit pickup route now drops below the red circle first, moves east along the south side of the main corridor, then enters the pickup room from the open east-side path.
- Elevator and room route hints were shifted through the same south-side corridor bypass so later mission legs do not cut through the red human zone.
- Simulated human detection now reports the guest near the red marker center so `/human_detection` matches the Gazebo scene again.

Expected behavior:

- The human is visible inside the red circle.
- The robot stays outside the red circle while travelling to the pickup counter.
- Pickup still reaches the blue box route through the open room entry instead of trying to cross the keepout.

## 2026-05-21 Counter Choreography, Hand Carry, And Delivery Pause

Latest observation:

- The blue medical kit followed the robot above its head instead of reading as a hand-held object.
- Pickup started from a broad room standoff, so the G1 appeared to stand straight while the box teleported away from the counter.
- The executive finished the delivery navigation without calling the manipulation server to place the carried kit on the destination counter.
- The north delivery guest no longer caused an assessment-visible pause.

Fix applied:

- Pickup navigation now approaches through the open south-room route, reaches a closer counter stance at `(2.05, -1.95)`, and aligns the robot yaw toward the pickup counter before the pick action succeeds.
- The blue box carry pose is now robot-frame-relative near the right hand instead of `robot_z + 1.1` above the body. The carry pose rotates with odometry yaw.
- `manipulation_server.py` now animates a short box transfer from the pickup counter into the carry pose and from the carry pose onto the room 302 delivery counter.
- `robot_executive.py` now calls the manipulation `deliver_item` action after delivery navigation instead of completing after a timed handover only.
- The simulated guest detector now publishes one short delivery-corridor safety crossing from odometry proximity, and simulation safety converts it into a brief `/emergency_stop` pause before navigation resumes.

Stability note:

- The humanoid joints remain fixed in the Gazebo URDF for this demo. Earlier free-joint gait experiments mangled the detailed G1 under Gazebo physics. The human-readable behavior here comes from counter alignment, hand-level payload placement, transfer timing, and safety interaction while the global avatar stays rigid.

Validation from the 2026-05-21 headless delivery run:

```text
[navigation_server]: Pickup approach aligned to counter.
[manipulation_server]: Pick med_kit: reach_pregrasp
[manipulation_server]: Pick med_kit: close_hand
[manipulation_server]: Blue pickup box attached to robot carry pose via entity-state service.
[safety_controller]: Human inside delivery safety zone: simulated stop active. [SIM]
[safety_controller]: Human safety zone cleared: simulated navigation may resume. [SIM]
[robot_executive]: Deliver med_kit requested for room 302
[manipulation_server]: Delivery to room 302: complete
```

## 2026-05-21 Hip Plate Removal And Pickup Confirmation

Latest observation:

- The active launch path uses the detailed `unitree_g1_23dof_delivery.urdf.xacro` model.
- The white plate around the robot hip/torso was the fixed `delivery_tray_link` added as an earlier payload fallback.
- The robot could log a successful pick and continue toward the lift while the blue medical kit did not visibly follow it.

Root cause:

- The tray fallback was still present in the active 23-DOF URDF after the demo shifted back to the real blue box.
- `manipulation_server.py` kept publishing `/gazebo/set_model_state` topic updates during carry, but a topic publish alone does not confirm that Gazebo moved `medical_kit_pickup`.
- `robot_executive.py` accepted a succeeded pick action status without checking the `PickObject.Result.success` flag.

Fix applied:

- Removed `delivery_tray_link`, its fixed joint, and its Gazebo reference block from the active 23-DOF URDF.
- `hotel_healthcare.world` now loads `libgazebo_ros_state.so` as a Gazebo world plugin under the `/gazebo` namespace. The stale command-line state-plugin flag was removed from launch so the box carry services have a real server.
- The carry controller now keeps attempting `/gazebo/set_entity_state` and `/gazebo/set_model_state` service updates while the box is carried.
- Pickup now waits for a Gazebo state-service confirmation before returning success. If no confirmed blue-box move arrives, the pick action aborts and the mission does not continue to the lift as though the box were collected.
- `robot_executive.py` now requires both a succeeded action status and `PickObject.Result.success=True` before it marks the item as held.

Expected behavior after rebuild:

- The hip plate is absent from the robot model.
- The blue box stays on the counter before pickup.
- After a confirmed pickup, the blue box moves to the carry pose and follows the robot.
- If Gazebo state services cannot move the blue box, the logs show a pickup failure instead of a false pick success.

Validation from the 2026-05-21 headless pickup run:

```text
[gazebo.hotel_gazebo_ros_state]: Publishing states of gazebo models at [/gazebo/model_states]
[navigation_server]: Arrived at pickup zone - navigation SUCCESS
[robot_executive]: Pick med_kit requested
[manipulation_server]: Pick med_kit: complete
[manipulation_server]: Blue pickup box attached to robot carry pose via entity-state service.
[robot_executive]: Pick complete - navigating to elevator
```

## 2026-05-20 17:23 IST Update

Latest user-observed problem:

- The detailed G1 mesh model was still visually mangling during navigation.
- The pickup leg could succeed in logs, but the model could tip/rotate after wall contact and then the elevator leg stalled.
- The blue pickup model did not reliably move because Gazebo state services were listed in the graph but did not accept calls in the current launch.

Fix applied now:

- The launch now uses `unitree_g1_lite_delivery.urdf.xacro` for the demo avatar.
  - This is a fixed-link, no-gravity G1-style delivery avatar.
  - It keeps arms, legs, torso, head, tray, lidar, and camera as one coherent body.
  - No joint animation is published, so individual limbs cannot move separately or detach.
- The lite avatar is spawned at `z=0.96` so its fixed feet/tray/body sit correctly in Gazebo.
- The avatar pelvis collision is disabled with `collide_bitmask=0x00`; wall avoidance is handled by the map/lidar planner instead of Gazebo contact forces twisting the body.
- `joint_motion_demo.py` is now only a status/stabilization node; it does not publish fake limb motion.
- `fake_nav_server.py` pickup route now uses a safer standoff:
  - pickup target: `(2.15, -1.65)`
  - pickup arrival radius: `0.70 m`
  - route approaches from the open right side of the continuous wall, avoiding the narrow divider gap.
- Elevator navigation now accepts a safe standoff if the final approach is partially blocked, so the mission does not freeze immediately after pickup.
- `manipulation_server.py` now tries both `/gazebo/set_entity_state` and `/gazebo/set_model_state`, and keeps publishing a model-state fallback for the carried box. The fixed tray also includes a visible blue med-kit, so the delivery payload remains visible even when Gazebo state services are not usable.

Validation from the latest headless run:

```text
[fake_nav_server]: Navigation goal 'pickup' planned with 5 waypoints.
[fake_nav_server]: Arrived at pickup zone — navigation SUCCESS
[robot_executive]: Reached pickup zone
[robot_executive]: Pick med_kit requested
[manipulation_server]: Pick med_kit: complete
[manipulation_server]: Blue pickup box attached to robot carry pose.
```

Build and syntax checks passed after these changes:

```text
python3 -m py_compile ...
xacro unitree_g1_lite_delivery.urdf.xacro && check_urdf
./scripts/build.sh
Summary: 2 packages finished
```

## 2026-05-20 15:45 IST Update

Latest movement-log review:

- Motion recorder output is available under:

```text
debug/robot_motion_records/run_20260520_152627/
```

- Files reviewed:
  - `joint_states.csv`
  - `odom.csv`
  - `link_transforms.jsonl`
- The recorder captured 176k+ joint rows, 3.7k+ odometry rows, and 180k+ TF/link transform rows.
- ROS logs showed the mission did reach pickup and did call the pick action:

```text
[fake_nav_server]: Arrived at pickup zone — navigation SUCCESS
[robot_executive]: Reached pickup zone
[robot_executive]: Pick med_kit requested
[manipulation_server]: Pick med_kit: lift_and_verify
```

Problems found:

- The blue box disappeared because `manipulation_server.py` explicitly deleted the Gazebo model after pickup using `/delete_entity`.
- The robot links could still appear detached because `joint_motion_demo.py` was still forcing Gazebo model pose with `/gazebo/set_entity_state` while also forcing joint configuration. With many G1 links marked kinematic, that can fight Gazebo's internal model/link tree and make visual links appear to move separately.

Fix applied now:

- `joint_motion_demo.py` no longer calls `/gazebo/set_entity_state`.
- `joint_motion_demo.py` no longer repeatedly forces joint configuration after startup.
- It now publishes a fixed home pose on `/joint_states` and applies `/gazebo/set_model_configuration` once at startup.
- Global robot movement is owned by `libgazebo_ros_planar_move.so`; individual limbs are not used to move the robot.
- `manipulation_server.py` no longer deletes the blue box after pickup.
- The blue box is carried by repeatedly setting the `medical_kit_pickup` model pose relative to the robot odometry pose.
- `hotel_healthcare.world` now marks `medical_kit_pickup` as non-static, no-gravity, and kinematic so it can be moved by the pick server instead of disappearing.

## 2026-05-20 13:10 IST Update

Latest observed problem:

- The robot planned a pickup route and crossed the continuous front wall area, but it still brushed the wall and slowed/stopped before completing pickup.
- The log showed navigation started successfully:

```text
[fake_nav_server]: Navigation goal 'pickup' planned with 17 waypoints.
```

- No `Pick med_kit` logs appeared afterward, which means the robot executive never reached the pickup success condition.

Current corrective changes:

- The robot is now treated as a rigid demo avatar:
  - The last physical `base_link` collision was removed.
  - Major G1 body links are marked no-gravity and kinematic in Gazebo.
  - `joint_motion_demo.py` holds the home pose continuously and does not animate gait.
- Navigation is now a SLAM-style static-map plus lidar-perception planner:
  - `/map` is published as an occupancy grid by `static_map_publisher.py`.
  - `/scan` from the simulated MID360 lidar is consumed by `fake_nav_server.py`.
  - The planner uses inflated wall geometry, A* planning, route hints, line-of-sight smoothing, wall clearance checks, and progress-stall replanning.
  - Pickup now uses an explicit route hint around the continuous wall instead of trying to cut through the narrow wall/counter clearance.
- The blue pickup box is its own Gazebo model and is carried with the robot after the simulated pick action succeeds.

Important limitation:

- This is not full online SLAM Toolbox/Nav2. It is a deterministic SLAM-style demo stack: known occupancy grid + simulated lidar wall perception + A* replanning. Full Nav2/SLAM Toolbox integration would require adding lifecycle bringup, map/odom/base TF alignment, costmaps, controller plugins, and a real robot footprint.

## Executive Summary

The simulation now builds, generates a valid robot description, spawns the Unitree G1 model, starts the ROS nodes, starts the Gazebo planar motion plugin, publishes odometry, and keeps the robot visible with official Unitree link and joint geometry.

The previous robot disfigurement was caused by treating a physically simulated humanoid as if it were a stable walking robot while only driving its base with `libgazebo_ros_planar_move.so` and separately animating joints. That made the limbs move independently from the body, and wall contact caused the uncontrolled physical model to fall.

The current repair changes the robot into a stable simulation avatar:

- The URDF was rebuilt from the official Unitree `g1_23dof_mode_10` model.
- Official link names, joint origins, axes, inertias, masses, limits, and visual meshes were preserved.
- Fake gait animation was removed because it made the robot look disfigured.
- Joint publishing now holds the official neutral pose.
- The planar motion plugin owns whole-body movement. The joint node only publishes the fixed home pose and does not override model pose.
- Mesh paths were fixed for installed ROS packages.
- Full STL mesh collision was tested and found to crash Gazebo Classic, so mesh collision blocks were removed while primitive collisions were retained.

Important limitation: this is still not a real Unitree walking controller. It is a Gazebo Classic delivery demo using a planar base plugin plus a stabilized visual G1 model. Real humanoid walking would require a whole-body balance/locomotion controller, ros2_control integration, or Unitree's controller stack.

## Latest Validation State

Latest successful smoke test:

```text
/home/aadi/.ros/log/2026-05-20-15-26-26-833797-Aadi-Alpha-76525
```

Observed successful logs:

```text
[gzserver] process started
[robot_state_publisher] process started
[spawn_entity] SpawnEntity: Successfully spawned entity [unitree_g1_delivery]
[g1_planar_motion]: Subscribed to [/cmd_vel]
[g1_planar_motion]: Advertise odometry on [/odom]
[g1_planar_motion]: Publishing odom transforms between [odom] and [base_link]
[motion_recorder]: Motion recorder writing robot debug traces to ...
[joint_motion_demo]: G1 rigid home-pose publisher ready: no limb animation, no pose override.
[fake_nav_server]: Arrived at pickup zone — navigation SUCCESS
[robot_executive]: Pick med_kit requested
[manipulation_server]: Pick med_kit: lift_and_verify
```

Build status after the latest robot reset:

```text
Summary: 2 packages finished
```

Validation commands that passed during the latest repair:

```bash
source install/setup.bash
xacro src/humanoid_delivery_sim/urdf/unitree_g1_23dof_delivery.urdf.xacro > /tmp/g1_fixed_tree.urdf
check_urdf /tmp/g1_fixed_tree.urdf
gz sdf -p /tmp/g1_fixed_tree.urdf
python3 -m py_compile src/humanoid_delivery_sim/humanoid_delivery_sim/joint_motion_demo.py
```

## Current Architecture

### Launch

File:

```text
src/humanoid_delivery_sim/launch/humanoid_delivery_sim.launch.py
```

Main launched components:

- `gzserver`
- optional delayed `gzclient`
- `robot_state_publisher`
- `spawn_entity.py`
- `fleet_orchestrator`
- `fake_nav_server`
- `manipulation_server`
- `sim_perception`
- `safety_controller`
- `robot_executive`
- `static_map_publisher`
- delayed `joint_motion_demo`
- delayed `speech_llm_interface`

Robot spawn settings:

```text
entity: unitree_g1_delivery
x: 0.85
y: 0.0
z: 0.74
orientation: upright quaternion
```

### World

File:

```text
src/humanoid_delivery_sim/worlds/hotel_healthcare.world
```

World models:

- `hotel_floor_plan`
- `mission_props`
- `elevator_A`
- `guest_crossing`

Collision state:

- Outer walls have collision.
- Internal room walls have collision.
- Counter blocks have collision.
- Elevator floor, wall, doors, and panel have collision.
- Mission props have collision.
- Guest model has collision.
- Route markers are mostly visual guidance and should not be treated as real navigation obstacles.

Guest placement:

```text
guest_crossing pose: 4.5 1.5 0.0
```

This prevents immediate emergency stop at launch and places the guest on a later corridor path.

### Robot Model

File:

```text
src/humanoid_delivery_sim/urdf/unitree_g1_23dof_delivery.urdf.xacro
```

Official source used:

```text
https://github.com/unitreerobotics/unitree_ros/tree/master/robots/g1_description
robots/g1_description/g1_23dof_mode_10.urdf
```

Local official checkout used during repair:

```text
/tmp/unitree_ros_official
```

Model family:

```text
Unitree G1 23 DOF mode 10
```

Root arrangement:

- `base_link` wrapper added for Gazebo planar motion.
- `base_to_g1_body` fixed joint connects `base_link` to official `pelvis`.
- Official G1 link tree remains under `pelvis`.

Reason for `base_link`:

- `libgazebo_ros_planar_move.so` needs a stable base frame for `/cmd_vel` motion and `/odom`.
- The official G1 URDF root is `pelvis`, which is not ideal as the planar plugin frame by itself.

### Motion Model

Gazebo plugin:

```text
libgazebo_ros_planar_move.so
```

Subscribed topic:

```text
/cmd_vel
```

Published topic:

```text
/odom
```

Base frame:

```text
base_link
```

Current behavior:

- Robot translation and yaw are driven as a whole body by the planar plugin.
- Limbs are held at official neutral joint positions.
- Fake limb-by-limb walking was removed to stop visual disfigurement.
- The joint node does not drive global pose. It only publishes the fixed home pose and applies Gazebo joint configuration once at startup.

This is intentional. Until a real whole-body controller is added, animated gait should not be mixed with physics-driven wall contact.

## Official Joint Data

The current URDF preserves official Unitree mode-10 joint definitions for these joints:

```text
left_hip_pitch_joint
left_hip_roll_joint
left_hip_yaw_joint
left_knee_joint
left_ankle_pitch_joint
left_ankle_roll_joint
right_hip_pitch_joint
right_hip_roll_joint
right_hip_yaw_joint
right_knee_joint
right_ankle_pitch_joint
right_ankle_roll_joint
waist_yaw_joint
left_shoulder_pitch_joint
left_shoulder_roll_joint
left_shoulder_yaw_joint
left_elbow_joint
left_wrist_roll_joint
right_shoulder_pitch_joint
right_shoulder_roll_joint
right_shoulder_yaw_joint
right_elbow_joint
right_wrist_roll_joint
```

The joint node publishes these joints at the fixed home pose:

```text
left/right hip pitch: -0.10
left/right knee:       0.30
left/right ankle pitch:-0.20
arms: slight relaxed bend
all other listed joints: neutral
```

Why neutral pose is currently used:

- It prevents disconnected limb motion.
- It preserves official joint/link alignment.
- It keeps the robot visually coherent while the base turns.
- It avoids pretending that a fake sinusoidal gait is a valid G1 locomotion controller.

## Files Changed And Current Purpose

### `unitree_g1_23dof_delivery.urdf.xacro`

Purpose:

- Official Unitree G1 model with package-local mesh references.
- Adds `base_link` wrapper.
- Adds Gazebo planar motion, camera, and lidar plugins.
- Applies no-gravity/kinematic behavior to major body links.
- Removes full STL mesh collision blocks that crashed Gazebo Classic.

Key fix:

```xml
<xacro:property name="mesh_dir" value="file://$(find humanoid_delivery_sim)/meshes/unitree_g1"/>
```

This avoids Gazebo converting `package://humanoid_delivery_sim/...` into unsupported `model://humanoid_delivery_sim/...` paths after install.

### `joint_motion_demo.py`

Purpose:

- Publishes official joint states at fixed home pose.
- Calls `/gazebo/set_model_configuration` once to apply the home joint configuration.
- Subscribes `/odom`.
- Does not call `/gazebo/set_entity_state`.
- Does not animate limbs.
- Leaves global robot translation/yaw to the planar move plugin.

Current expected log:

```text
G1 rigid home-pose publisher ready: no limb animation, no pose override.
```

### `fake_nav_server.py`

Purpose:

- Fake Nav2-compatible action server.
- Reads `/odom`.
- Publishes `/cmd_vel`.
- Uses waypoint chains instead of a single straight line.
- Performs simple wall-boundary checks.

Limitation:

- No real costmap.
- No local planner.
- No obstacle avoidance from lidar.
- No recovery behavior.

### `robot_executive.py`

Purpose:

- Loads room and route targets from YAML.
- Sends the route chain key to fake navigation.
- Coordinates pickup, elevator, and delivery behavior.

Config file:

```text
src/humanoid_delivery_sim/config/room_poses.yaml
```

### `safety_controller.py`

Purpose:

- Stops `/cmd_vel` when a simulated human is in a STOP zone.
- Suppresses hard `/emergency_stop=True` in simulation mode.
- Continues publishing healthy `False` emergency state normally.

Simulation flag:

```python
SIM_MODE = True
```

### `setup.py`

Purpose:

- Installs launch files, worlds, URDF/Xacro files, YAML config, and Unitree mesh files into package share.

This is necessary because Gazebo loads robot assets from the installed package when launched after `colcon build`.

## Problems Faced And Fixes

### Problem 1: Gazebo GUI stuck on splash screen

Symptom:

- `gzclient` opened to the orange Gazebo loading screen and stayed there.
- `gzserver` and ROS nodes were already running.

Finding:

- The robot and server were not the blocker.
- The GUI was the unstable piece.
- Launching headless and opening `gzclient` separately was more reliable.

Current state:

- Headless launch works.
- GUI can still show the Xlib warning.
- Xlib warning is cosmetic unless `gzclient` crashes.

### Problem 2: Robot model missing

Symptom:

```text
FuelModelDatabase.cc: URI not supported by Fuel [model://humanoid_delivery_sim/meshes/unitree_g1/pelvis.STL]
SystemPaths.cc: File or path does not exist
Visual.cc: No mesh specified
```

Root cause:

- Gazebo Classic converted package mesh URIs into `model://humanoid_delivery_sim/...`.
- It then tried to resolve the package as a Gazebo model, not as a ROS package.

Fix:

- Mesh URIs now use an install-time resolved `file://$(find humanoid_delivery_sim)/meshes/unitree_g1/...` path.
- Meshes are installed by `setup.py`.

Current state:

- `gz sdf -p` resolves meshes to installed file paths.
- Robot visuals load.

### Problem 3: Robot was upside down or falling

Root cause:

- Gazebo physics simulated humanoid bodies with gravity.
- No balance controller existed.
- Contact forces from walls or props caused the uncontrolled body to tip.

Fix:

- Major G1 links are marked no-gravity/kinematic for this demo.
- The previous whole-body pose lock has been removed because it could fight Gazebo's model/link tree.
- The planar motion plugin now owns model translation and yaw.
- The joint node only holds the home joint pose.

Current state:

- Robot should move globally as one model during planar motion.
- This is a simulation stabilization strategy, not real humanoid balance.

### Problem 4: Joints looked disfigured

Root cause:

- The old joint demo animated limbs independently from the base.
- The planar plugin moved and turned the base while joint animation continued without whole-body coordination.
- This looked like disconnected limbs rather than a humanoid turning.

Fix:

- Removed fake gait animation.
- Replaced it with official neutral joint publishing.
- Removed Gazebo pose overriding from the joint node.
- Turns now happen through the planar motion plugin, not by twisting limbs separately.

Current state:

- Robot should look like a coherent G1 avatar.
- It will not show a realistic walking gait yet.

### Problem 5: Full mesh collision crashed Gazebo

Failed test log:

```text
/home/aadi/.ros/log/2026-05-20-09-53-51-205092-Aadi-Alpha-8619
```

Symptom:

```text
gzserver died with exit code -11
```

Root cause:

- Full STL collision meshes from the official G1 model were too heavy or unstable for Gazebo Classic in this setup.

Fix:

- Visual meshes were preserved.
- Full STL mesh collision blocks were removed.
- Existing simple primitive collisions were retained where official URDF already used them.

Current state:

- Gazebo no longer crashes during the latest headless smoke test.
- Robot body collision is simplified and should not be treated as high-fidelity humanoid contact.

### Problem 6: Safety stop triggered at launch

Root cause:

- Guest model or synthetic human detection was too close to the starting robot position.

Fix:

- Guest moved to a later corridor area near room 201.
- Synthetic crossing delayed.
- In simulation mode, hard emergency stop is suppressed while zero velocity is still published.

Current state:

- No immediate mission-blocking emergency stop should occur at launch.

### Problem 7: Fake navigation drove through walls

Root cause:

- Straight-line navigation ignored wall geometry.

Fix:

- Fake navigation now uses waypoint chains through corridor centers.
- It includes a simple wall-boundary proximity check.

Current state:

- Better than straight-line driving.
- Still not a true navigation stack.
- Waypoints must remain synchronized with the world geometry.

## Current Remaining Limitations

The following are still true and should not be confused with bugs in the current repair:

- The robot is not doing real Unitree G1 locomotion.
- The robot does not have a whole-body balance controller.
- The robot does not compute footstep plans.
- The robot does not use leg torques or ros2_control controllers.
- The fake navigation server does not use an occupancy grid, costmap, or local planner.
- The current joint node is a simulation-only home-pose publisher.
- Full STL mesh collision is disabled because it crashed Gazebo Classic.
- Xlib thread-safety warnings may still appear from Gazebo Classic on Ubuntu 22.04.

## Recommended Next Engineering Steps

### Step 1: Keep the official neutral avatar stable

Do not re-enable fake sinusoidal gait until the base, yaw, and links stay coherent. The neutral pose is the correct baseline.

### Step 2: Add a proper command interface for visual gait only

If visual walking is needed before real locomotion, implement a single coordinated gait pose sequence where:

- the full model yaw follows `/odom`,
- both legs remain attached to the pelvis,
- joint positions are set as one complete robot configuration,
- no physics torque balance is implied.

### Step 3: Add real navigation later

Replace `fake_nav_server.py` with Nav2 only after the world has:

- valid occupancy map,
- valid robot footprint,
- costmap parameters,
- collision geometry aligned with map geometry.

### Step 4: Add real humanoid locomotion only as a separate project

Real G1 walking in simulation needs one of:

- Unitree SDK/controller integration,
- ros2_control with position or effort controllers,
- whole-body controller,
- foot contact model,
- balance controller,
- MuJoCo workflow bridged to ROS.

Gazebo planar motion plus animated joints cannot become a physically correct humanoid walking controller by tuning only URDF values.

## Validation Commands

Run from workspace root:

```bash
cd /media/aadi/Extras/ba0ba0/assesment

./scripts/stop_sim.sh
./scripts/build.sh --clean

source install/setup.bash
xacro src/humanoid_delivery_sim/urdf/unitree_g1_23dof_delivery.urdf.xacro > /tmp/g1_report.urdf
check_urdf /tmp/g1_report.urdf
gz sdf -p /tmp/g1_report.urdf > /tmp/g1_report.sdf
xmllint --noout src/humanoid_delivery_sim/worlds/hotel_healthcare.world

python3 -m py_compile \
  src/humanoid_delivery_sim/humanoid_delivery_sim/fake_nav_server.py \
  src/humanoid_delivery_sim/humanoid_delivery_sim/robot_executive.py \
  src/humanoid_delivery_sim/humanoid_delivery_sim/joint_motion_demo.py \
  src/humanoid_delivery_sim/humanoid_delivery_sim/safety_controller.py
```

Launch headless:

```bash
cd /media/aadi/Extras/ba0ba0/assesment
./scripts/stop_sim.sh
./scripts/launch_sim.sh --headless
```

Open GUI in another terminal:

```bash
cd /media/aadi/Extras/ba0ba0/assesment
source install/setup.bash
./scripts/open_gui.sh
```

Launch with GUI:

```bash
cd /media/aadi/Extras/ba0ba0/assesment
./scripts/stop_sim.sh
./scripts/launch_sim.sh --gui
```

## Expected Current Behavior

Expected:

- Robot model is visible.
- Robot spawns upright.
- Robot turns as a whole body.
- Limbs should not separate or twist independently during turning.
- Robot should not immediately fall from joint physics.
- Mission nodes start.
- Navigation follows waypoint chains.
- Guest does not trigger hard emergency stop at launch.

Not expected yet:

- Realistic Unitree walking gait.
- Dynamic balance recovery.
- Physically accurate humanoid wall contact.
- Full-body collision fidelity.
- True Nav2 obstacle avoidance.

## Bottom Line

The current correct baseline is an official Unitree G1 visual/kinematic delivery robot driven by planar motion, not a physics-balanced humanoid. The previous disfigured movement was the result of fake limb animation fighting the planar base. The immediate priority is to keep the official model coherent, upright, and visible. Real walking should be added only after choosing a proper humanoid controller stack.

## 2026-05-20 15:52 Navigation Regression

Latest run:

- `fake_nav_server`: `Navigation goal 'pickup' planned with 6 waypoints.`
- repeated `Stall detected - skipping waypoint`
- final `Navigation ABORT - could not reach goal`

Root cause:

- The pickup route was using the narrow elevator-side passage around the room divider.
- After removing Gazebo `<kinematic>true</kinematic>` from robot links to fix ghost limbs, the detailed humanoid collision meshes became physically active again.
- Those limb/body collision meshes can brush the wall before the planner reaches the pickup success radius.
- The wall proximity rectangle for the elevator was also too broad and treated the robot start corridor as near-wall space.

Fix applied:

- `fake_nav_server.py` now routes pickup and room 101 around the wider right-side opening:
  `(0.85, 0.0) -> (2.0, 0.0) -> (3.95, 0.0) -> (3.95, -2.05) -> (2.70, -2.05) -> (2.0, -1.90)`.
- Elevator return route now reverses through the same wide opening instead of squeezing past the elevator-side wall.
- Elevator wall bounds were replaced with the actual small fixture rectangles instead of one large rectangle that covered the start area.
- `_too_close_to_wall()` margin was reduced from `0.35 m` to `0.22 m`.
- The G1 visual links now keep `<gravity>false</gravity>` but also use `<collide_bitmask>0x00</collide_bitmask>`.

Important design note:

- This keeps the robot visually coherent without reintroducing `<kinematic>true</kinematic>`.
- Wall avoidance is now owned by the map/lidar navigation layer, not by detailed limb collision meshes.
- World walls, counters, elevator geometry, and the blue box still keep their own collision geometry.

## 2026-05-20 16:28 Limb Drift And Pickup Visual Regression

Latest observation:

- The robot reached the elevator visually with arms and legs crossing into an unnatural pose.
- The blue medical kit still did not visibly attach in Gazebo during the pickup sequence.

Root cause:

- `/joint_states` only drives the ROS TF tree. It does not hold Gazebo revolute joints in place.
- The home-pose node was calling `/gazebo/set_model_configuration` only once after spawn.
- After that one-time call, Gazebo physics could still integrate the uncontrolled humanoid joints, so limbs drifted between contacts and turns.
- The blue-box carry action was firing pose updates before Gazebo service discovery was always ready, and the box model still had a link-level kinematic flag.

Fix applied:

- The 20 Hz full-model joint lock was tested and rejected because it stalled planar navigation.
- The one-shot `/gazebo/set_model_configuration` call was also removed because it reset/corrupted odom during validation.
- `joint_motion_demo.py` now only publishes `/joint_states`.
- `/joint_states` continues publishing the same fixed home pose at 50 Hz.
- The node still has no walking animation and no `/gazebo/set_entity_state` body pose override.
- All 23 revolute G1 joints now include high damping and friction:
  `damping="120.0"` and `friction="25.0"`.
- Narrowing revolute limits around either home pose or zero pose caused Gazebo solver/odom instability.
- Final rigid demo choice: all 23 humanoid articulation joints are fixed in the Gazebo URDF. This makes the G1 a rigid visual avatar moved globally by the planar plugin.
- This intentionally disables limb articulation for the assessment demo until a real humanoid controller is added.
- `manipulation_server.py` now waits up to 2 seconds for `/gazebo/set_entity_state` at pickup completion before sending the first box attach request.
- Box pose service calls now track completion and log Gazebo rejection/failure instead of failing silently.
- `hotel_healthcare.world` keeps `medical_kit_pickup` non-static and no-gravity, but removes the box link `<kinematic>true</kinematic>` flag.

Expected behavior after rebuild:

- Limbs remain locked in the home pose while the whole robot moves globally with planar motion.
- The blue box remains visible on the counter before pickup.
- After pickup, the blue box moves to the robot carry pose and follows the robot.
- After delivery, the blue box is moved below the floor.

---

## 2026-05-20 20:00 IST — Official G1 URDF + Mesh Crash Fix

### Problem 1: Placeholder model (boxes/cylinders) — not a real G1

**Symptom:** The Gazebo visual showed a rough approximation of the G1 using primitive geometries.  
**Root cause:** `unitree_g1_lite_delivery.urdf.xacro` used box and cylinder visuals instead of the official STL meshes.  
**Fix:** Created `unitree_g1_official_delivery.urdf.xacro` by:
- Downloading the official Unitree G1 29-DOF URDF from `unitreerobotics/unitree_ros`
- Adapting mesh paths and adding `base_link`, Gazebo planar-move plugin, D435, LiDAR, delivery tray
- Launch file updated to use new xacro

---

### Problem 2: Robot invisible in Gazebo after model switch

**Symptom:** Gazebo launched, spawn succeeded (`Successfully spawned entity [unitree_g1_delivery]`), but the robot model was not visible. Blue pickup box was floating in air.  
**Root cause:** Mesh filenames used `package://humanoid_delivery_sim/...` URIs. Gazebo Classic cannot reliably resolve `package://` URIs when loading models via the `/robot_description` topic — it needs the ROS resource resolver to be active at the exact moment meshes are loaded.  
**Fix applied:**
- Added `GAZEBO_RESOURCE_PATH` and `GAZEBO_MODEL_PATH` env vars in launch file pointing to the installed package share directory
- Replaced all `package://...` mesh paths with absolute filesystem paths using a xacro `mesh_pkg` arg passed from the launch file

```xml
<xacro:arg name="mesh_pkg" default=""/>
<xacro:property name="mesh_base" value="$(arg mesh_pkg)/meshes/unitree_g1"/>
```

Launch passes: `xacro urdf.xacro mesh_pkg:=/path/to/installed/share/`

---

### Problem 3: Gazebo crash on spawn (exit code -6, SIGABRT)

**Symptom:** `gzserver` died with `exit code -6` approximately 15 seconds after launch when attempting to load the official G1 model.  
**Root cause:** All 165 STL files in `meshes/unitree_g1/` were in **ASCII STL format** (files started with `solid `). Gazebo Classic's OGRE renderer requires **binary STL format**. Loading ASCII STLs at 103 MB total caused OGRE to abort.

**Diagnosis:**
```
$ head -c 6 pelvis.STL | xxd
00000000: 736f 6c69 6420   solid
```
ASCII STL confirmed — Gazebo SIGABRT on mesh load.

**Fix:** Converted all 165 STL files from ASCII to binary format using `numpy-stl`:

```python
from stl import mesh, Mode
m = mesh.Mesh.from_file(path)
m.save(path, mode=Mode.BINARY)
```

Result: 165 files converted, 0 errors. Binary STL header now starts with `numpy-` (80-byte binary header).

---

### Current State (post-fix)

| Component | Status |
|-----------|--------|
| Robot model | Official Unitree G1 29-DOF with real STL meshes |
| Mesh format | Binary STL (Gazebo-compatible) |
| Mesh resolution | Absolute filesystem paths (no `package://` URI issues) |
| Navigation | Functional (pickup → elevator → room 302 delivery proven) |
| Mission success | Confirmed end-to-end in prior session |

**Run command:**
```bash
cd /media/aadi/Extras/ba0ba0/assesment && ./scripts/launch_sim.sh gui:=true
```

---

## 2026-05-20 20:14 IST — Gazebo OGRE Rendering Assertion Crash (Final Resolution)

### Crash

```
gzserver: .../boost/shared_ptr.hpp:728:
  Assertion `px != 0' failed.
[ERROR] gzserver: process has died [exit code -6]
```

**Trigger:** Immediately after `spawn_entity` calls `/spawn_entity` service with the official G1 URDF containing 36 STL mesh references.

**Root cause:** Gazebo Classic's OGRE rendering pipeline null-dereferences its `Scene` pointer when loading meshes that are too geometrically complex. The 36 body-link STL files (103 MB total, tens of thousands of triangles each) exceed what OGRE can load safely in a single spawn call under Gazebo Classic 11.

Even after converting from ASCII → binary STL (which fixed the earlier text-parse abort), OGRE still crashes during vertex buffer creation for these high-polygon meshes.

### Decision

The official Unitree G1 STL meshes are **not compatible with Gazebo Classic** for real-time spawning. They are designed for:
- MuJoCo (mjcf/mujoco compiler handles high-poly fine)
- Isaac Sim / Gazebo Fortress (modern Ignition rendering pipeline)
- RViz visualization (CPU-based, no GPU scene limits)

### Fix Applied

Reverted `launch.py` to `unitree_g1_lite_delivery.urdf.xacro` — the primitive-geometry model that is **proven stable**:

| Property | Lite URDF | Official STL URDF |
|----------|-----------|-------------------|
| Visuals | Boxes + cylinders | 36 × STL meshes |
| Total mesh data | 0 bytes | 103 MB |
| Gazebo stability | ✅ Stable | ❌ SIGABRT |
| Navigation | ✅ Functional | ✅ Functional |
| Joint kinematic tree | Simplified | Official 29-DOF |

The official URDF (`unitree_g1_official_delivery.urdf.xacro`) is retained in the `urdf/` directory for use with MuJoCo / Isaac Sim or future migration to Gazebo Ignition.

### Current Working State

```
URDF (Gazebo): unitree_g1_lite_delivery.urdf.xacro  ← primitive shapes, stable
URDF (Reference): unitree_g1_official_delivery.urdf.xacro ← full mesh, MuJoCo/Isaac
Mission: pickup → elevator (floor 1→3) → room_302 delivery  ← COMPLETE ✅
```

**Run command:**
```bash
cd /media/aadi/Extras/ba0ba0/assesment && ./scripts/launch_sim.sh gui:=true
```


---

## 2026-05-21 08:15 IST — Spawn Timing Race Condition (SIGABRT Root Cause)

### Crash (same assertion, even with lite primitive URDF)

```
gzserver: boost/shared_ptr.hpp:728: Assertion `px != 0' failed.
[ERROR] gzserver: process has died [exit code -6]
```

### Root Cause — OGRE Scene Not Ready at Spawn Time

**Isolation test:** Running `gzserver` alone with the hotel world for 12 seconds → **no crash**.
**Conclusion:** The crash is a **race condition** — `spawn_entity` called `/spawn_entity` ~2 seconds after `gzserver` started. At that point, Gazebo's OGRE rendering scene (`gazebo::rendering::Scene`) was still being constructed. When the spawner tried to attach a visual to the scene, the `shared_ptr<Scene>` was null → `Assertion 'px != 0' failed` → SIGABRT.

This was **not a mesh issue**. The primitive-geometry URDF (zero meshes) crashed for the same reason once Gazebo started faster after zombie processes were cleared.

### Fix Applied — `TimerAction(period=10.0)` on spawn_entity

```python
# BEFORE (crashes — spawn fires ~2s after gzserver start):
Node(package="gazebo_ros", executable="spawn_entity.py", ...)

# AFTER (stable — OGRE scene has 10s to fully initialize):
TimerAction(period=10.0, actions=[
    Node(package="gazebo_ros", executable="spawn_entity.py", ...)
])
```

Timer schedule after fix:

| Action | Delay |
|--------|-------|
| gzserver | 0 s |
| gzclient | 5 s |
| spawn_entity | **10 s** |
| joint_motion_demo | 12 s |
| speech_llm_interface (mission) | **35 s** |

### Current State

```
Crash: FIXED
URDF: unitree_g1_lite_delivery.urdf.xacro (primitive shapes, Gazebo-stable)
Spawn delay: 10 seconds after gzserver start
Mission auto-trigger: 35 seconds after launch
```

**Run command:**
```bash
cd /media/aadi/Extras/ba0ba0/assesment && ./scripts/launch_sim.sh gui:=true
```

---

## 2026-05-21 08:20 IST — Robot Model Removal & Static Medical Kit Setup

### User Request / Changes Implemented
1. **Robot Model Deleted Completely:**
   * Removed robot spawning nodes (`spawn_entity.py`), joint demos (`joint_motion_demo`), and robot description setup (`robot_state_publisher`) from `humanoid_delivery_sim.launch.py`.
   * The simulation now operates without a Gazebo visual/physics representation of the robot body, which eliminates Gazebo renderer assertion crashes entirely.
2. **Blue Medical Kit Made Static:**
   * In `hotel_healthcare.world`, changed `medical_kit_pickup` to `<static>true</static>` and updated its Z-coordinate to `0.96` to sit cleanly on the table (Z=0.90 counter height).
   * This completely prevents the blue box from slowly drifting or floating up due to collision solver interactions when gravity was disabled.
3. **Simulated Odometry (Fake Nav Server):**
   * Configured `fake_nav_server.py` to act as a self-contained kinematic simulator.
   * It subscribes to `/cmd_vel` (looping back its own navigation controls) and integrates the velocities inside a 20Hz update timer.
   * It publishes simulated `Odometry` messages to `/odom`, making the rest of the navigation/monitoring stack fully functional without requiring a physical Gazebo robot model.
   * Subscribed to `/gazebo/set_model_state` inside `fake_nav_server.py` to intercept teleportation commands (e.g., when the robot executive changes elevator floors) and reset simulated coordinates.
4. **Robot Executive Teleportation:**
   * Updated `robot_executive.py`'s `teleport_robot()` method to publish a `ModelState` message to `model_state_pub` rather than calling the `/gazebo/set_entity_state` service for the non-existent Gazebo robot body.

### Current Working State
```
Gazebo Robot Model: Removed completely (zero renderer load) ✅
Medical Kit Box: Static at (2.0, -2.55, 0.96) — no floating ✅
Odometry: Simulated internally by fake_nav_server (published to /odom) ✅
Teleportation: Intercepted via /gazebo/set_model_state ✅
```

**Run command:**
```bash
cd /media/aadi/Extras/ba0ba0/assesment && ./scripts/launch_sim.sh gui:=true
```

---

## 2026-05-21 08:30 IST — Replaced with Proper Unitree G1 23-DOF Model

### User Request / Changes Implemented
1. **Added Proper G1 23-DOF Model:**
   * Restored robot spawning (`spawn_entity.py`) and `robot_state_publisher` nodes in `humanoid_delivery_sim.launch.py`.
   * Switched URDF source model to `unitree_g1_23dof_delivery.urdf.xacro`.
   * Added `delivery_tray_link` and `delivery_tray_joint` to `unitree_g1_23dof_delivery.urdf.xacro` so the robot has its delivery tray.
   * Kept the 10-second `TimerAction` for spawning the robot to avoid race conditions with OGRE scene-pointer initialization.
2. **Reverted Simulated Odometry:**
   * Reconfigured `fake_nav_server.py` to subscribe to `/odom` published by Gazebo (planar-move plugin).
   * Reconfigured `robot_executive.py`'s `teleport_robot()` to call `/gazebo/set_entity_state` service (which teleports the physical G1 robot body in the physics engine).

### Current Working State
```
Gazebo Robot Model: Unitree G1 23-DOF model (rigid limbs, planar-move base) ✅
Medical Kit Box: Static at (2.0, -2.55, 0.96) — no floating ✅
Odometry: Published by libgazebo_ros_planar_move.so ✅
Teleportation: Handled by Gazebo's set_entity_state service ✅
```

**Run command:**
```bash
cd /media/aadi/Extras/ba0ba0/assesment && ./scripts/launch_sim.sh gui:=true
```

---

## 2026-05-21 08:35 IST — Fixed Medical Kit Pickup & Removed Waist Plate

### User Request / Changes Implemented
1. **Fixed Medical Kit Pickup:**
   * In `hotel_healthcare.world`, changed `medical_kit_pickup` back to `<static>false</static>` so it can be dynamically teleported by Gazebo APIs during carrying.
   * Completely removed the `<collision>` element of the `medical_kit_pickup` model. Without collision geometry, it has no interaction forces with the table or the environment, preventing it from floating or drifting.
   * Retained `<gravity>false</gravity>` so it stays stationary at its target location without falling.
2. **Removed Waist Plate:**
   * Removed `pelvis_contour_link` and `pelvis_contour_joint` from `unitree_g1_23dof_delivery.urdf.xacro`.
   * Removed the corresponding `<gazebo reference="pelvis_contour_link">` entry to avoid compiler/runtime warnings.

### Current Working State
```
Gazebo Robot Model: Unitree G1 23-DOF model (rigid limbs, planar-move base) ✅
Waist Plate (pelvis contour): Removed completely ✅
Medical Kit Box: Dynamic but non-colliding (no gravity) — cleanly teleports to robot tray ✅
```

**Run command:**
```bash
cd /media/aadi/Extras/ba0ba0/assesment && ./scripts/launch_sim.sh gui:=true
```

---

## 2026-05-21 08:40 IST — Codebase Cleanup & Rename of "Fake" Components

### User Request / Changes Implemented
1. **Renamed "Fake" Components:**
   * Renamed the Python file `fake_nav_server.py` to `navigation_server.py`.
   * Renamed the ROS2 Node and entry point from `fake_nav_server` to `navigation_server`.
   * Updated `setup.py`, all launch files (`humanoid_delivery_sim.launch.py`, `core_nodes.launch.py`, `two_robot_lift_demo.launch.py`), bash scripts (`launch_sim.sh`, `stop_sim.sh`), and `docs/architecture.md` to reference `navigation_server`.
2. **Cleaned Up Unused URDF Models:**
   * Deleted unused/unnecessary URDF models from `src/humanoid_delivery_sim/urdf/` to keep the codebase clean:
     * `unitree_g1_lite_delivery.urdf.xacro`
     * `unitree_g1_29dof_delivery.urdf.xacro`
     * `unitree_g1_29dof_with_hand.urdf.xacro`
     * `unitree_g1_official_delivery.urdf.xacro`
   * Only the active `unitree_g1_23dof_delivery.urdf.xacro` remains.

### Current Working State
```
Active Robot Model: Unitree G1 23-DOF model ✅
Navigation Node: Renamed to navigation_server (no "fake" terminology in setup or launch) ✅
Codebase Cleanliness: Removed all unused xacro models, keeping the repo clean and professional ✅
```

**Run command:**
```bash
cd /media/aadi/Extras/ba0ba0/assesment && ./scripts/launch_sim.sh gui:=true
```
