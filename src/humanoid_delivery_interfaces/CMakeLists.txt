cmake_minimum_required(VERSION 3.8)
project(humanoid_delivery_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HumanDetection.msg"
  "msg/LiftStatus.msg"
  "msg/RobotState.msg"
  "msg/Task.msg"
  "srv/CallLift.srv"
  "srv/DoorOpenVerification.srv"
  "srv/MissionAssignment.srv"
  "srv/ChargingRequest.srv"
  "srv/RecoveryReset.srv"
  "action/PickObject.action"
  "action/DeliverItem.action"
  "action/RetryMission.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs unique_identifier_msgs
)

ament_package()
