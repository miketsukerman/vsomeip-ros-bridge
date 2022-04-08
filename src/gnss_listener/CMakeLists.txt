cmake_minimum_required(VERSION 3.8)

project(gnss_listener VERSION 0.0.1)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gnss_ros_lib REQUIRED)

set(GNSS_LISTENER_TARGET gnss-listener)

add_executable(${GNSS_LISTENER_TARGET} src/main.cpp)

ament_target_dependencies(${GNSS_LISTENER_TARGET} rclcpp gnss_ros_lib)

target_compile_features(${GNSS_LISTENER_TARGET} PRIVATE cxx_std_17)

install(TARGETS ${GNSS_LISTENER_TARGET} DESTINATION lib/${PROJECT_NAME})

ament_package()
