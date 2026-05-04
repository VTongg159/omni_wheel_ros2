cmake_minimum_required(VERSION 3.8)
project(hector_slam_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# Find dependencies
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# ============================================================
# hector_slam_lib headers (algorithm core – header-only, pure C++)
# Cloned from tu-darmstadt-ros-pkg/hector_slam (noetic branch).
# We only need the include directory – no ROS1 dependency.
#
#   Path after setup_hector_lib.sh runs:
#     <ws>/src/my_robot/hector_slam_ros2/hector_slam_lib/
#                └── include/hector_slam_lib/
# ============================================================
set(HECTOR_LIB_INCLUDE
  ${CMAKE_CURRENT_SOURCE_DIR}/hector_slam_lib/include
)

if(NOT EXISTS ${HECTOR_LIB_INCLUDE})
  message(FATAL_ERROR
    "\n[hector_slam_ros2] Algorithm headers not found at:\n"
    "  ${HECTOR_LIB_INCLUDE}\n"
    "Ensure the hector_slam_lib subdirectory exists.\n"
  )
endif()

# ============================================================
# Build the wrapper node
# ============================================================
add_executable(hector_mapping_node
  src/hector_mapping_node.cpp
  src/hector_mapping_ros2.cpp
)

target_include_directories(hector_mapping_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${HECTOR_LIB_INCLUDE}
    ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(hector_mapping_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
)

target_link_libraries(hector_mapping_node Eigen3::Eigen)

# ============================================================
# Install
# ============================================================
install(TARGETS hector_mapping_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# ============================================================
# Tests
# ============================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()