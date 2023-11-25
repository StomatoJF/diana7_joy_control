cmake_minimum_required(VERSION 3.0.2)
project(diana7_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  CartesianState.msg
  JointPositionLimit.msg
  Joint.msg
)

add_service_files(
  FILES
  SetControlMode.srv
  SetCartesianCollision.srv
  SetHandGuidingAxis.srv
  IK.srv
  FK.srv
  SetJointLimits.srv
  SetCartesianLimits.srv
  SetJointPositionLimits.srv
  MoveJ.srv
  SetJointCollision.srv
  GetBoxDigitalInput.srv
  SetBoxDigitalOutput.srv
  GetBoxAnalogInput.srv
  SetBoxAnalogOutput.srv
  SetImpedance.srv
  SetPayload.srv
  SetResultantForceCollisionThreshold.srv
  SetTCP.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS std_msgs geometry_msgs message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

