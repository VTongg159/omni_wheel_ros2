# ✅ SAU (Humble-compatible)
cmake_minimum_required(VERSION 3.8)
project(slam_gmapping)

# Humble yêu cầu C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(openslam_gmapping REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(
  include
  ${openslam_gmapping_INCLUDE_DIRS}
)

add_executable(slam_gmapping
  src/slam_gmapping.cpp
  
)

# Humble: dùng target_link_libraries thay vì ament_target_dependencies cho Boost
ament_target_dependencies(slam_gmapping
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  message_filters
)

# Link openslam và Boost riêng
target_link_libraries(slam_gmapping
  ${openslam_gmapping_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS slam_gmapping
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()