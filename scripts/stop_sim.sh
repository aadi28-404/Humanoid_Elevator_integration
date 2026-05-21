#!/usr/bin/env bash
set -euo pipefail

PROCESS_PATTERN='gzserver|gzclient|gazebo|ros2 launch humanoid_delivery_sim|fleet_orchestrator|navigation_server|fake_nav_server|robot_executive|speech_llm_interface|safety_controller|sim_perception|manipulation_server|static_map_publisher|motion_recorder|joint_motion_demo|spawn_entity.py|robot_state_publisher'

pkill -f "$PROCESS_PATTERN" 2>/dev/null || true
sleep 1
pkill -9 -f "$PROCESS_PATTERN" 2>/dev/null || true
