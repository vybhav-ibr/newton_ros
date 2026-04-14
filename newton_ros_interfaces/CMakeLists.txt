cmake_minimum_required(VERSION 3.16)
project(newton_ros_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(simulation_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(rosidl_default_runtime REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  #sensor data types  
  "msg/Contact.msg"
  "msg/ContactForce.msg"
  #robot_options
  "msg/RobotOptions.msg"
  "msg/JointProperty.msg"
  #sensor_options
  "msg/RigidSensorOptions.msg"
  "msg/NoisySensorOptions.msg"
  "msg/GeneralSensorOptions.msg"
  "msg/RosSensorOptions.msg"
  "msg/CameraOptions.msg"
  "msg/LidarOptions.msg"
  "msg/ImuOptions.msg"
  "msg/ContactOptions.msg"
  "msg/ContactForceOptions.msg"
  #sensor_patterns
  "msg/GridRayCasterPattern.msg"
  "msg/SphericalRayCasterPattern.msg"
  "msg/DepthCameraRayCasterPattern.msg"
  #services
  "srv/SetEntityPose.srv"
  "srv/GetEntityPose.srv"
  "srv/PathPlanTarget.srv"
  "srv/IKTarget.srv"
  "srv/FKTarget.srv"
  "srv/SuctionSwitch.srv"
  "srv/JoinEntities.srv"
  "srv/StartRecording.srv"
  "srv/PauseRecording.srv"
  "srv/StopRecording.srv"
  "srv/SetPhysicsAttributes.srv"
  #osi related services
  "srv/BuildSimulator.srv"
  "srv/GetRobotOptions.srv"
  "srv/SetRobotOptions.srv"
  "srv/GetSensorOptions.srv"
  "srv/SetSensorOptions.srv"
  DEPENDENCIES geometry_msgs std_msgs simulation_interfaces
)

install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
