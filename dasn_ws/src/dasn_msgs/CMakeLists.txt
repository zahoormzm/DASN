cmake_minimum_required(VERSION 3.8)
project(dasn_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ZoneSensorData.msg"
  "msg/ThreatLevel.msg"
  "msg/DispatchCommand.msg"
  "msg/BotStatus.msg"
  "msg/FaceDetection.msg"
  "msg/FaceIdentity.msg"
  "msg/SoundEvent.msg"
  "msg/GasEvent.msg"
  "msg/AuthEvent.msg"
  "msg/AlarmEvent.msg"
  "msg/DepthGrid.msg"
  "msg/EncoderTicks.msg"
  "msg/StallEvent.msg"
  "msg/AudioChunk.msg"
  "msg/PoseAnalysis.msg"
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    sensor_msgs
)

ament_package()
