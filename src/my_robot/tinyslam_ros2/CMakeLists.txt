cmake_minimum_required(VERSION 3.8)
project(tinyslam_ros2)

# ---------------------------------------------------------------
# Compiler options
# ---------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# ---------------------------------------------------------------
# find_package
# ---------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ---------------------------------------------------------------
# TinySLAM core library (pure C — compiled as C99)
# ---------------------------------------------------------------
# The .c files must be cloned from:
#   https://github.com/OpenSLAM-org/openslam_tinyslam
# into tinyslam_lib/ before building.
#
# Files needed:
#   CoreSLAM.c
#   CoreSLAM_ext.c
#   CoreSLAM_random.c
#   CoreSLAM_state.c
#   CoreSLAM_loop_closing.c  (optional — safe to include)
# ---------------------------------------------------------------

file(GLOB TINYSLAM_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/tinyslam_lib/*.c")

add_library(coreSlam STATIC ${TINYSLAM_SOURCES})
target_include_directories(coreSlam PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/tinyslam_lib>
)
set_target_properties(coreSlam PROPERTIES
  C_STANDARD 99
  POSITION_INDEPENDENT_CODE ON
)
target_compile_options(coreSlam PRIVATE -Wno-unused-result)

# ---------------------------------------------------------------
# ROS 2 wrapper node (C++)
# ---------------------------------------------------------------
add_executable(tinyslam_node
  src/tinyslam_node.cpp
)

target_include_directories(tinyslam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/tinyslam_lib>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(tinyslam_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(tinyslam_node coreSlam m)

set_target_properties(tinyslam_node PROPERTIES
  CXX_STANDARD 17
)

# ---------------------------------------------------------------
# Install
# ---------------------------------------------------------------
install(TARGETS tinyslam_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# ---------------------------------------------------------------
# Testing
# ---------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
