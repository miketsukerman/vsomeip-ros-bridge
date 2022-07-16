cmake_minimum_required(VERSION 3.10)

project(gnss_ros_lib VERSION 0.0.1)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "src/GpsData.msg"
  "src/Dop.msg"
  "src/Fix.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
