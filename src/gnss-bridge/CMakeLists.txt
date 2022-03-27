cmake_minimum_required(VERSION 3.10)

project(gnss_bridge VERSION 0.0.1)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)

find_package(Threads REQUIRED)
find_package(Boost COMPONENTS log system CONFIG REQUIRED)
find_package(CommonAPI REQUIRED)
find_package(CommonAPI-SomeIP REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(gnss REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(CLIENT_TARGET_NAME gnss-client)
add_executable(${CLIENT_TARGET_NAME} 
    src/main.cpp
)
target_compile_features(${CLIENT_TARGET_NAME} PRIVATE cxx_std_17)
target_compile_definitions(${CLIENT_TARGET_NAME} PRIVATE -DGNSS_CLIENT_VERSION="${PROJECT_VERSION}")


target_include_directories(${CLIENT_TARGET_NAME} PRIVATE 
    ${COMMONAPI_INCLUDE_DIRS}
)

target_link_libraries(${CLIENT_TARGET_NAME} PRIVATE
    CommonAPI
    CommonAPI-SomeIP
    Threads::Threads
    Boost::log
    Boost::system
    gnss::gnss
)

install(TARGETS ${CLIENT_TARGET_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()