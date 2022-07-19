cmake_minimum_required(VERSION 3.5)
project(duckietown_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)




# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(msg
  ./msg/AntiInstagramThresholds.msg
  ./msg/AprilTagDetection.msg
  ./msg/AprilTagDetectionArray.msg
  ./msg/AprilTagsWithInfos.msg
  ./msg/BoolStamped.msg
  ./msg/ButtonEvent.msg
  ./msg/CarControl.msg
  ./msg/CoordinationClearance.msg
  ./msg/CoordinationSignal.msg
  ./msg/DiagnosticsCodeProfiling.msg
  ./msg/DiagnosticsCodeProfilingArray.msg
  ./msg/DiagnosticsRosLink.msg
  ./msg/DiagnosticsRosLinkArray.msg
  ./msg/DiagnosticsRosNode.msg
  ./msg/DiagnosticsRosParameterArray.msg
  ./msg/DiagnosticsRosTopic.msg
  ./msg/DiagnosticsRosTopicArray.msg
  ./msg/DisplayFragment.msg
  ./msg/DroneControl.msg
  ./msg/DroneMode.msg
  ./msg/NodeParameter.msg
  ./msg/DuckiebotLED.msg
  ./msg/EncoderStamped.msg
  ./msg/EpisodeStart.msg
  ./msg/FSMState.msg
  ./msg/IntersectionPose.msg
  ./msg/IntersectionPoseImg.msg
  ./msg/IntersectionPoseImgDebug.msg
  ./msg/KinematicsParameters.msg
  ./msg/KinematicsWeights.msg
  ./msg/LanePose.msg
  ./msg/LEDDetection.msg
  ./msg/LEDDetectionArray.msg
  ./msg/LEDDetectionDebugInfo.msg
  ./msg/LEDInterpreter.msg
  ./msg/LEDPattern.msg
  ./msg/LightSensor.msg
  ./msg/LineFollowerStamped.msg
  ./msg/MaintenanceState.msg
  ./msg/ObstacleImageDetection.msg
  ./msg/ObstacleImageDetectionList.msg
  ./msg/ObstacleProjectedDetection.msg
  ./msg/ObstacleProjectedDetectionList.msg
  ./msg/ObstacleType.msg
  ./msg/ParamTuner.msg
  ./msg/Pixel.msg
  ./msg/Pose2DStamped.msg
  ./msg/Rect.msg
  ./msg/Rects.msg
  ./msg/SceneSegments.msg
  ./msg/Segment.msg
  ./msg/SegmentList.msg
  ./msg/SignalsDetection.msg
  ./msg/SignalsDetectionETHZ17.msg
  ./msg/SourceTargetNodes.msg
  ./msg/StopLineReading.msg
  ./msg/TagInfo.msg
  ./msg/ThetaDotSample.msg
  ./msg/Trajectory.msg
  ./msg/TurnIDandType.msg
  ./msg/Twist2DStamped.msg
  ./msg/Vector2D.msg
  ./msg/VehicleCorners.msg
  ./msg/VehiclePose.msg
  ./msg/Vsample.msg
  ./msg/WheelEncoderStamped.msg
  ./msg/WheelsCmd.msg
  ./msg/WheelsCmdStamped.msg
  ./msg/WheelsCmdDBV2Stamped.msg
)

set(srv
  ./srv/ChangePattern.srv
  ./srv/GetVariable.srv
  ./srv/IMUstatus.srv
  ./srv/LFstatus.srv
  ./srv/NodeGetParamsList.srv
  ./srv/NodeRequestParamsUpdate.srv
  ./srv/SensorsStatus.srv
  ./srv/SetCustomLEDPattern.srv
  ./srv/SetFSMState.srv
  ./srv/SetValue.srv
  ./srv/SetVariable.srv
  ./srv/ToFstatus.srv
)


rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg}
  ${srv}
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

ament_package()
