# ROS2 Architecture

## Node Graph

| Node | Responsibility | Key Interfaces |
| --- | --- | --- |
| `robot_state_publisher` | Publishes the vendored Unitree G1 23-DoF URDF tree, simulated sensors, hands, and base frame. | `/tf`, `/robot_description` |
| `navigation_server` | Nav2-compatible local demo server. In production this is replaced by Nav2 `bt_navigator`, planner, controller, smoother, behavior server, AMCL, and lifecycle manager. | `/navigate_to_pose`, `/cmd_vel` |
| `static_map_publisher` | Simulation map/localization stand-in that publishes the known hotel occupancy grid for planner evaluation. Production replaces it with a commissioned map plus SLAM/localization lifecycle nodes. | `/map` |
| `fleet_orchestrator` | RMF-like mission, elevator, charging, and recovery coordinator. | `/task_queue`, `/lift_status`, `/call_lift`, `/assign_mission`, `/request_charging`, `/recovery_reset` |
| `robot_executive` | Task planner and mission state machine for pickup, elevator use, delivery, and retry. | `/robot_state`, `/task_queue`, `/navigate_to_pose`, `/pick_object`, `/deliver_item`, `/retry_mission` |
| `sim_perception` | Simulated human perception from depth/LiDAR/VLA perception stack. | `/human_detection` |
| `safety_controller` | Safety watchdog, human proximity stop, fault stop, zero velocity override. | `/human_detection`, `/robot_state`, `/emergency_stop`, `/cmd_vel` |
| `manipulation_server` | Humanoid arm/hand grasp action abstraction. | `/pick_object`, `/arm_controller` |
| `speech_llm_interface` | Speech/LLM boundary that turns operator utterances into structured missions. | `/speech_events`, `/assign_mission` |
| `motion_recorder` | Simulation observability for odometry, TF, and joints when diagnosing body motion, manipulation, and navigation regressions. | `/odom`, `/tf`, `/tf_static`, `/joint_states` |
| `gazebo` | Physics, Unitree G1 visual model, planar delivery motion, sensors, corridor/elevator world. | `/scan`, `/camera/*`, `/odom` |

## Navigation, SLAM, And Localization

Production stack:

- `slam_toolbox` online async mapping for commissioning and map updates.
- Nav2 AMCL for normal hotel operation with `/map`, `/tf`, `/scan`, and odometry.
- Nav2 behavior tree for `NavigateToPose`, recovery behaviors, clear costmaps, assisted teleop fallback, and docking route.
- Local controller tuned for humanoid constraints: low acceleration, larger personal-space inflation near humans, and a stable zero-velocity hold.

The demo includes a Nav2-compatible `/navigate_to_pose` action server so the rest of the assessment can run without map tuning.

In this assessment simulation the official Unitree G1 geometry is kept in a fixed neutral pose and moved through Gazebo planar motion. This is deliberate: the goal is to evaluate the ROS2 delivery architecture, elevator orchestration, perception/safety interfaces, and mission state machines without requiring a full biped gait controller.

Room identification is split across layers:

- Perception/VLA detects door plates, room labels, open counter scenes, payload affordances, and ambiguous human requests.
- The local task executive converts a validated room ID into a named navigation target and expected floor.
- Fleet configuration owns hotel room metadata, elevator connectivity, charging docks, and restricted areas so a room-layout update is not hidden inside the locomotion controller.

## Required Topics

- `/cmd_vel`: reliable control command path; safety controller can publish zero velocity.
- `/tf`: base, sensor, and hand transforms from robot state publisher and odometry.
- `/map`: occupancy grid for localization and planning.
- `/robot_state`: mission mode, floor, battery, localization, payload, and fault state.
- `/human_detection`: detected human pose, distance, intent, confidence, and safety-zone flag.
- `/task_queue`: structured fleet mission assignments.
- `/lift_status`: elevator floor, door, motion, reservation owner, and waiting robots.
- `/arm_controller`: grasp phase feedback for the humanoid manipulation controller.
- `/emergency_stop`: latched safety stop state.

## Required Services

- `/call_lift`: reserves an elevator trip from floor A to B.
- `/verify_lift_door_open`: validates door opening using depth camera evidence before boarding/exiting.
- `/assign_mission`: fleet mission assignment API.
- `/request_charging`: charger reservation when battery falls below threshold.
- `/recovery_reset`: clears recoverable faults and returns the robot to replan/resume.

## Required Actions

- `/navigate_to_pose`: Nav2 action for corridor navigation.
- `/pick_object`: VLA-guided reach, grasp, lift, and verification.
- `/deliver_item`: item handover at destination room.
- `/retry_mission`: bounded recovery action for clear-costmap, relocalize, recall lift, or return-to-safe-zone.

## Elevator Integration

The elevator interface is a reservation protocol:

1. Robot calls `/call_lift` with robot ID, source floor, target floor, and priority.
2. Fleet orchestrator inserts the request into a priority/FIFO queue.
3. A single active reservation token is granted for each lift.
4. `/lift_status` publishes current floor, door state, motion state, reservation owner, and waiting IDs.
5. Robot boards only when `reserved_by == robot_id`, current floor matches, door is open, and `/verify_lift_door_open` succeeds.
6. Robot exits under the same ownership and verification rules.

Two robots requesting the same lift cannot deadlock because only the token holder is allowed to board. Queue ordering is priority first, then FIFO. A production system would extend this to RMF traffic lanes and elevator sessions, with a lease timeout and heartbeat from the robot.

State ownership is deliberately distributed:

- `fleet_orchestrator` owns queue ordering, reservation token issue, lift status publication, and cross-robot scheduling.
- `robot_executive` owns the per-robot elevator sequence: navigate to lift, wait for token, verify door, board, wait for target floor, verify door again, exit, and retry on bounded failure.
- Elevator communication can be bridged to BACnet, REST/gRPC, vendor MQTT, or RMF adapters behind `/call_lift`, `/lift_status`, and `/verify_lift_door_open`; the robot behavior tree does not depend on that transport.
- Human override moves the lift integration to a manual/hold policy: robots stop outside the threshold, release or expire leases, keep `/robot_state` visible to fleet, and wait for operator recovery rather than entering an uncontrolled car.

## Failure And Recovery

- Lift unavailable: retry reservation, route to alternate lift, or notify remote operator.
- Door not open: wait, verify again, then call recovery reset or human override.
- Human override: elevator mode changes to `manual`; robots release reservations and hold outside the door.
- Timeout: state machine raises `LIFT_TIMEOUT_PICKUP` or `LIFT_TIMEOUT_DROPOFF`, then `/retry_mission` can relocalize and reacquire a reservation.
- Safety stop: `/emergency_stop` becomes true and `/cmd_vel` is forced to zero until the safety zone clears.

## QoS Strategy

- Sensor streams (`/scan`, depth, `/human_detection`): best effort, small queue, low latency.
- Control and safety (`/cmd_vel`, `/emergency_stop`): reliable, small queue.
- Fleet state (`/robot_state`, `/lift_status`): reliable + transient local so late subscribers get current state.
- Mission events (`/task_queue`, service/action traffic): reliable with moderate depth.

## Real-Time And Compute

- Jetson Orin CPU: ROS2 executors, behavior trees, lifecycle management, fleet protocol, watchdogs, DDS threads.
- Jetson Orin GPU/DLA: VLA perception, human detection, room/door recognition, grasp affordance inference, depth segmentation.
- Hard real-time loops are isolated in the base controller, arm controller, and safety PLC/microcontroller. ROS2 carries commands and supervision, while low-level joint limits, force limits, and e-stop remain local and deterministic.
- Use callback groups and multithreaded executors for action/service concurrency; keep perception inference in separate processes to avoid blocking safety callbacks.

## DDS Middleware

Recommended default: Cyclone DDS for deterministic discovery behavior, good ROS2 Humble support, and straightforward fleet network configuration. Fast DDS is also viable when the deployment already standardizes on eProsima tooling. For hotels/hospitals, configure domain IDs, static discovery or discovery server, transport allow-lists, and QoS overrides per site.

## Edge/Cloud Split

On robot:

- Safety, navigation, localization, manipulation control, human detection, elevator state machine, local mission execution.

Cloud/fleet:

- Mission assignment, analytics, OTA configuration, remote assistance, logs, cross-robot scheduling, map distribution, LLM-heavy reasoning when latency is not safety-critical.

The robot must complete safe stop, recovery hold, and local replan without cloud connectivity.

## Failover Strategy

- Fleet orchestrator failure: robot continues current safe segment, then holds at the next safe waypoint and retries service discovery.
- Network failure: local executive preserves mission and avoids elevator entry unless it owns a valid active reservation.
- Perception failure: reduce speed, increase inflation radius, request remote assistance.
- Localization loss: stop, relocalize using AMCL/visual landmarks, then retry mission.
- Hardware fault: assert e-stop, publish `/robot_state`, and require `/recovery_reset` or technician intervention.

## Assessment Traceability

### Part 1 Architecture Challenge

| Assessment requirement | Current architecture coverage |
| --- | --- |
| Navigation stack | `navigation_server` implements the demo `NavigateToPose` boundary; production mapping is Nav2 planner/controller/BT/recovery/lifecycle nodes. |
| SLAM/localization | `/map`, `/scan`, `/tf`, odometry, `static_map_publisher`, and the production `slam_toolbox` + AMCL plan are defined above. |
| Perception and human detection | Depth/LiDAR/VLA perception boundary is represented by `sim_perception` and `/human_detection`; safety consumes the safety-zone flag. |
| Task planner | `robot_executive` drives pickup, elevator, room delivery, retry, and payload state. |
| Manipulation controller | `manipulation_server` implements `/pick_object`, `/deliver_item`, and `/arm_controller` feedback. |
| Elevator interface | `fleet_orchestrator`, `/call_lift`, `/lift_status`, and `/verify_lift_door_open`. |
| Speech/LLM interface | `speech_llm_interface` converts a speech event into `/assign_mission`; VLA/LLM reasoning stays outside hard safety loops. |
| Safety controller/watchdogs | `safety_controller`, `/emergency_stop`, `/cmd_vel` zero override, local low-level e-stop/watchdog guidance, and recovery reset. |
| Fleet communication | `fleet_orchestrator` mission, charging, recovery, lift scheduling services plus cloud/edge split. |
| Required topics/services/actions | Listed in the dedicated sections and implemented by the interface package and simulation nodes. |
| QoS, real-time, DDS, failover, compute split | Covered in the QoS, Real-Time And Compute, DDS Middleware, Edge/Cloud Split, and Failover Strategy sections. |

### Part 2 Elevator Challenge

| Elevator requirement | Current architecture coverage |
| --- | --- |
| Communication protocol | ROS service/status/action boundary isolates vendor lift transport behind fleet integration. |
| Two-robot scheduling | Priority/FIFO lift reservations are queued centrally and exposed through `/lift_status.waiting_robot_ids`. |
| Deadlock prevention | Only one active reservation token can board or exit a lift car at a time. |
| Fleet orchestration | RMF-like central arbitration is implemented by `fleet_orchestrator`; the two-robot launch demonstrates simultaneous requests. |
| Recovery and safety timeouts | Door verification, pickup/drop-off timeouts, retry action, recovery reset, and hold behavior are defined. |
| Human override handling | Manual/hold policy keeps robots outside lift thresholds until override clears or operator recovery completes. |
| Distributed state machines | Fleet owns lift/resource state; robot executive owns per-robot action coordination and mission progression. |
