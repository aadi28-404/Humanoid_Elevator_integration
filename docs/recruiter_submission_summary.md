# Humanoid Robotics Engineer Technical Assessment Submission Summary

Candidate: Aadi  
Assessment domain: ROS 2 humanoid delivery robot for hotel and healthcare environments  
Simulation environment: ROS 2 Humble, Gazebo Classic 11, Unitree G1-based delivery robot simulation

## Submission Note

Thank you for the opportunity to work on this technical assessment.

The assessment contained three major parts:

1. System architecture design for a humanoid delivery robot.
2. Elevator integration and multi-robot orchestration.
3. Vision-Language-Action training and deployment design for complex hotel manipulation tasks.

Within the available time, I was able to complete and demonstrate the core work for the first two parts:

- A ROS 2 architecture and simulation stack for autonomous hotel delivery.
- An elevator reservation and mission flow with lift scheduling, door verification, timeout handling, and two-robot arbitration logic.

I was not able to complete the third part to the same level of detail and implementation quality within the assessment time. I also did not fully achieve physically correct humanoid gait and joint control in Gazebo for the Unitree G1 model. That gap came from the short assessment window, limited available controller resources/specifications, and my limited direct experience with production G1 locomotion and VLA training pipelines. I attempted several gait and joint-control approaches, but without a full verified low-level locomotion controller, tuned joint controller gains, contact/balance control, and enough time to validate real Unitree joint behavior in simulation, the resulting motion became visually unstable or unrealistic. I therefore chose a stable simulation baseline that demonstrates the ROS 2 delivery architecture, navigation, elevator orchestration, human safety behavior, and manipulation action flow without claiming a production-quality biped gait controller.

This report summarizes what was achieved, how it was achieved, the major problems encountered, the fixes applied, and the specifications or resources still missing for a production-grade humanoid system.

## Demo And Error Images

The final demo and key mission artifact is:

- Final demo video: `media/demo.webm`

The remaining media files are error images captured during development:

- Global joint-axis / transform mismatch (earlier iteration): `media/global_joint_error.png`
- Joint limit / pose tuning error 1: `media/joint_limit_error_1.png`
- Joint limit / pose tuning error 2: `media/joint_limit_error_2.png`
- Wall collision/property error from an earlier simulation iteration: `media/wall_stuck_error.png`

## Original Assessment Scope

The original goal was to design and implement core components for a humanoid robot that can:

- Navigate hotel and healthcare corridors.
- Use elevators autonomously.
- Identify target rooms.
- Deliver items.
- Interact safely with humans.
- Recover from failures.
- Integrate ROS 2, Jetson Orin, depth cameras, LiDAR, humanoid arms/hands, VLA methods, and remote fleet management.

## Completion Status

| Assessment part | Status | Notes |
| --- | --- | --- |
| Part 1: System Architecture Challenge | Completed | ROS 2 node graph, interfaces, QoS, DDS, real-time considerations, safety, edge/cloud split, compute allocation, failover, navigation/perception/manipulation/fleet design documented and represented in the repo. |
| Part 2: Elevator Integration Challenge | Completed | Elevator protocol boundary, reservation service, queue prioritization, single-token deadlock prevention, lift status publication, door verification, robot executive lift sequence, timeout/recovery handling, and two-robot lift demo logic implemented. |
| Part 3: VLA Challenge | Not fully completed | A production design direction is understood, but data collection, training pipeline, action tokenization, diffusion/VLA policy design, edge inference benchmarking, and deformable-object manipulation validation were not completed in the available time. |

## What Was Achieved

### 1. ROS 2 System Architecture

A ROS 2 architecture was designed and implemented around the required system boundaries.

Implemented or documented nodes include:

- `navigation_server`
- `static_map_publisher`
- `robot_executive`
- `fleet_orchestrator`
- `manipulation_server`
- `sim_perception`
- `safety_controller`
- `speech_llm_interface`
- `robot_state_publisher`
- Gazebo sensor and world integration
- `motion_recorder` for debugging robot movement and transforms

The architecture covers:

- Navigation stack boundaries.
- SLAM/localization production plan.
- Perception and simulated human detection.
- Task planning and mission execution.
- Manipulation action interfaces.
- Elevator integration.
- Speech/LLM mission entry.
- Safety controller and emergency-stop behavior.
- Fleet communication and recovery interfaces.

### 2. Required ROS Interfaces

The required topics were represented in the system:

| Topic | Purpose |
| --- | --- |
| `/cmd_vel` | Robot motion command path. |
| `/tf` | Robot and sensor frame transforms. |
| `/map` | Occupancy map for navigation. |
| `/robot_state` | Mission mode, floor, battery, payload, and fault state. |
| `/human_detection` | Human detection and safety-zone information. |
| `/task_queue` | Mission assignments from fleet orchestration. |
| `/lift_status` | Lift floor, door, motion, reservation owner, and waiting robots. |
| `/arm_controller` | Manipulation action feedback. |
| `/emergency_stop` | Safety stop state. |

The required services were implemented:

| Service | Purpose |
| --- | --- |
| `/call_lift` | Request and reserve a lift trip. |
| `/verify_lift_door_open` | Verify lift door opening before boarding/exiting. |
| `/assign_mission` | Submit structured delivery missions. |
| `/request_charging` | Request charger/dock assignment. |
| `/recovery_reset` | Acknowledge and reset recoverable faults. |

The required actions were implemented:

| Action | Purpose |
| --- | --- |
| `/navigate_to_pose` | Corridor and room navigation interface. |
| `/pick_object` | Pickup action for the medical kit. |
| `/deliver_item` | Destination handover/place action. |
| `/retry_mission` | Bounded mission recovery flow. |

### 3. Navigation and Delivery Demonstration

The simulation demonstrates:

- Named room goals.
- Corridor routing.
- Map-based navigation behavior.
- Wall and obstacle avoidance logic.
- LiDAR clearance checks.
- Red human safety-zone keepout routing.
- Pickup navigation to a blue medical kit.
- Elevator navigation.
- Delivery-room approach to the destination table.
- Final item delivery action.

The final navigation behavior uses:

- An occupancy grid published on `/map`.
- Route hints for narrow hotel geometry.
- A* fallback planning behavior.
- Simulated LiDAR clearance checks.
- Arrival thresholds calibrated for counters and table-side approaches.

### 4. Elevator Integration and Fleet Scheduling

The elevator integration covers the main requirements of Part 2.

Implemented behavior:

- Lift request through `/call_lift`.
- Lift status publication through `/lift_status`.
- Door verification through `/verify_lift_door_open`.
- One active lift reservation token at a time.
- Priority-first and FIFO-second queue ordering.
- Multi-robot waiting list publication.
- Robot executive logic that waits for ownership before boarding or exiting.
- Timeout handling for pickup-floor and dropoff-floor lift stages.
- Retry/recovery action support.

The implementation follows an RMF-like resource-arbitration idea:

- Fleet owns shared lift state and scheduling.
- Each robot owns its own mission state machine and action coordination.
- Lift reservation ownership prevents two robots from entering conflicting lift states simultaneously.

### 5. Human Safety Behavior

The simulation includes:

- A human-looking standing person model in the environment.
- A visible red safety zone.
- Human detection messages.
- Navigation keepout behavior around the red safety circle.
- Safety-controller zero-velocity stop behavior when the simulated human safety condition is triggered.

### 6. Manipulation Action Flow

The manipulation flow demonstrates:

- Pickup request from the executive.
- Pick action feedback phases.
- Blue medical kit transfer into a carried pose.
- Payload following the robot during the mission.
- Delivery action at the destination room table.

This is a simulation-level action flow and not yet a real contact-rich dexterous grasp controller.

## How the Work Was Achieved

### Architecture and Interfaces

Custom ROS 2 interfaces were created for:

- Mission tasks.
- Robot state.
- Human detections.
- Lift status.
- Elevator and recovery services.
- Pick, deliver, navigate, and retry actions.

The architecture documentation explains:

- QoS choices.
- DDS middleware recommendation.
- Jetson Orin CPU/GPU/DLA allocation.
- Edge/cloud split.
- Real-time control boundaries.
- Safety watchdog responsibilities.
- Failover behavior.

### Mission Execution

The mission executive coordinates:

1. Navigate to pickup.
2. Trigger `PickObject`.
3. Navigate to elevator.
4. Request and reserve lift.
5. Verify lift door.
6. Move to delivery floor.
7. Navigate to room and destination table.
8. Trigger `DeliverItem`.
9. Publish final robot mission state.

### Elevator Scheduling

The lift scheduler uses:

- A reservation queue.
- Priority ordering.
- FIFO ordering for equal-priority requests.
- A single active reservation token.
- Lift state publication for robot executives.

### Debugging and Validation

The simulation was iteratively debugged using:

- Gazebo logs.
- ROS 2 terminal logs.
- Robot odometry.
- TF and joint motion recording.
- Static map and route inspection.
- Headless smoke runs.
- Build, syntax, XML, URDF, and launch validation.

## Major Problems Encountered and Solutions

| Problem | Cause | Solution |
| --- | --- | --- |
| Robot passed through, climbed, or became stuck on walls | Early wall collision/property setup and obstacle representation did not constrain the robot against the world geometry correctly. Evidence: `media/wall_stuck_error.png`. | Corrected wall and obstacle collision behavior, then added occupancy map behavior, wall bounds, route calibration, red-zone keepout, LiDAR clearance logic, safer approach routes, and tighter table/counter arrival behavior. |
| Robot fell or became visually mangled during humanoid motion attempts | A planar base motion demo was combined with physically simulated uncontrolled humanoid joints and experimental joint animation without a real balance controller. | Returned to a stable fixed-pose demo baseline for the humanoid body and separated the architecture demonstration from unsupported full biped gait control. |
| Global joint-axis mismatch / detached limb visuals (earlier iteration) | The robot body heading and the joint visuals diverged due to conflicting transform sources (pose overrides vs. joint publishing) and Gazebo link handling during kinematic/joint experimentation. Evidence: `media/global_joint_error.png`. | Stabilized the model by reducing conflicting pose/joint forcing and converging on a single consistent joint-state publishing path for the demo. |
| Joint limit / pose tuning errors (unresolved) | After addressing the global joint mismatch, the exact joint limit and neutral pose values for a realistic G1 gait were not fully verified/tuned within the assessment time. Evidence: `media/joint_limit_error_1.png`, `media/joint_limit_error_2.png`. | Left as a known limitation: the demo prioritizes mission orchestration (nav + lift + manipulation action flow) over production-quality biped locomotion control. |
| Pickup looked false because the robot advanced without visibly carrying the box | The blue medical-kit state updates were not confirmed reliably in Gazebo. | Added Gazebo state-service based payload attachment confirmation before accepting pickup success. |
| Box jumped to the delivery table while robot was still away from it | Delivery navigation succeeded at an old doorway standoff pose with a loose success threshold. | Changed room 302 delivery route to a table-side goal and tightened the room delivery threshold before the delivery action can execute. |
| Human obstacle and planner disagreed | The visual human/safety marker was not fully represented in navigation occupancy behavior. | Placed the human inside the red safety circle and made the red circle an explicit navigation keepout. |
| Lift coordination needed multi-robot arbitration | Shared elevator resources can deadlock or conflict without ownership. | Added priority/FIFO lift reservations and one-token lift ownership with published lift status. |

## Important Limitations

### Humanoid Gait and Joint Control

The most important limitation is that I did not complete a physically correct Unitree G1 gait controller in the assessment time.

What is missing for that work:

- Verified Unitree G1 low-level locomotion controller integration.
- Full actuator control mode selection for simulation: effort, position, velocity, or ros2_control command interfaces.
- Tuned joint gains and damping values.
- Contact-aware foot control.
- Balance and recovery control such as ZMP, MPC, whole-body control, or manufacturer locomotion stack integration.
- Stable ground-contact and collision tuning for Gazebo Classic.
- Validation of joint limits, inertias, friction, and controller update rates under locomotion.
- A verified simulated walking policy or controller that matches real robot behavior.

Because those pieces were not available or fully validated in time, the simulation uses a stable humanoid delivery avatar for the architecture demonstration instead of claiming real biped walking.

### VLA Challenge

Part 3 was not completed to the same depth as Parts 1 and 2.

Missing VLA work includes:

- Demonstration dataset design for bed making, linen folding, deformable picking, and door opening.
- Teleoperation and VR data capture pipeline.
- Motion-capture synchronization.
- Human demonstration recording format and labeling.
- Diffusion policy training.
- Transformer/VLA architecture selection and temporal attention design.
- Action tokenization and world-model strategy.
- Deformable-object state representation.
- Quantization and Jetson edge-inference benchmarks.
- Latency and safety envelope measurements.
- Recovery behavior for uncertain VLA actions.

## Missing Production Specifications and Resources

The following information or resources would be needed to move from this assessment demo toward a deployable humanoid system:

### Robot Control

- Official Unitree G1 simulation/control package verified for the chosen robot revision.
- Joint control API and command interface documentation.
- Actuator limits, force limits, controller rates, and safe motion envelopes.
- A real locomotion/balance controller or manufacturer-provided walking stack.
- Hardware-in-the-loop validation plan.

### Sensors and Perception

- Calibrated camera intrinsics/extrinsics.
- LiDAR calibration and sensor placement verification.
- Real hotel-room label and door datasets.
- Human detection accuracy targets and personal-space policy.
- Grasp and deformable-object datasets.

### Elevator and Facility Integration

- Elevator vendor API details.
- Security/authentication model for lift control.
- Facility override protocol.
- Fire-service and emergency policy constraints.
- RMF or building-management-system integration requirements.

### Deployment and Safety

- Jetson Orin latency and thermal benchmarks.
- Safety PLC/e-stop wiring and watchdog specification.
- Robot operating-speed limits around patients and guests.
- Failure-mode and hazard analysis.
- Remote fleet monitoring requirements.
- Network QoS, DDS discovery policy, and hospital/hotel IT constraints.

## Evidence of Completed Work

The repository includes:

- Architecture documentation in `docs/architecture.md`.
- Detailed engineering and debugging history in `docs/simulation_debug_report.md`.
- Demo recording and debug screenshots in `media/` (see `media/README.md`).
- ROS 2 interfaces in `src/humanoid_delivery_interfaces`.
- Simulation nodes in `src/humanoid_delivery_sim`.
- Elevator reservation logic.
- Mission executive logic.
- Safety-controller behavior.
- Manipulation actions.
- Speech/LLM mission entry point.
- Navigation and delivery demonstration logic.

## Final Reflection

This assessment was challenging because it combines several difficult robotics domains:

- Humanoid locomotion.
- Navigation and mapping.
- Perception and VLA policy design.
- Manipulation.
- Building systems integration.
- Fleet scheduling.
- Human safety.

I completed the architecture and elevator-integration components and built a working ROS 2 simulation flow that demonstrates delivery mission orchestration from mission assignment through pickup, elevator use, human-aware navigation, and final delivery.

The main unfinished area is physically correct humanoid gait/joint control and the full VLA training/deployment challenge. With more time, official robot-control resources, validated controller parameters, and a real VLA data pipeline, I would extend the current architecture toward:

1. Proper Unitree locomotion and arm control.
2. Real Nav2/SLAM lifecycle bringup.
3. Vendor elevator or RMF lift adapter integration.
4. Demonstration collection and VLA policy training for deformable and articulated tasks.
