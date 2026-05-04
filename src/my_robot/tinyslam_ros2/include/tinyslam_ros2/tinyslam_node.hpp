#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

extern "C" {
#include "CoreSLAM.h"
}

namespace tinyslam_ros2
{

class TinySlamNode : public rclcpp::Node
{
public:
  explicit TinySlamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  ~TinySlamNode();

private:
  /* ---- ROS callbacks -------------------------------------------- */
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  /* ---- helpers --------------------------------------------------- */
  void initTinySlam();
  void laserScanToSensorData(const sensor_msgs::msg::LaserScan::SharedPtr & msg,
                             ts_sensor_data_t & sd);
  void publishMap(const rclcpp::Time & stamp);
  void publishTF(const rclcpp::Time & stamp);

  /* ---- ROS interfaces -------------------------------------------- */
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr   map_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster>               tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer>                             tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>                  tf_listener_;

  /* ---- TinySLAM state -------------------------------------------- */
  ts_map_t   *ts_map_{nullptr};
  ts_state_t  ts_state_{};
  bool        slam_initialized_{false};
  unsigned int scan_count_{0};

  /* ---- parameters ------------------------------------------------ */
  std::string map_frame_;
  std::string odom_frame_;       // ← THÊM: frame trung gian map→odom
  std::string base_frame_;
  std::string laser_frame_;

  double sigma_xy_;
  double sigma_theta_;
  int    hole_width_;
  double map_resolution_;
  double laser_offset_;
  double max_range_;
  int    update_map_every_n_scans_;
};

} // namespace tinyslam_ros2