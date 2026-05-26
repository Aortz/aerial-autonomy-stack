cmake_minimum_required(VERSION 3.10)
project(airsim_gst_bridge)

set(CMAKE_CXX_STANDARD 17)

# ROS 2 (sourced env provides these on CMAKE_PREFIX_PATH). Built standalone into
# build/ like comms/gz_gst_bridge -- no ament_package()/colcon install needed.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

add_executable(airsim_gst_bridge airsim_gst_bridge.cpp)

target_include_directories(airsim_gst_bridge PUBLIC
    ${GST_INCLUDE_DIRS}
)

ament_target_dependencies(airsim_gst_bridge rclcpp sensor_msgs)
target_link_libraries(airsim_gst_bridge
    ${GST_LIBRARIES}
)
