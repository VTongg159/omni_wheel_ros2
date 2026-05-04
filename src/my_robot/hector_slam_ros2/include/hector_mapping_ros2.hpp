#pragma once

// ============================================================
//  hector_mapping_ros2.hpp
//  ROS2 Humble wrapper for the Hector SLAM algorithm.
//
//  Design principle:
//    - Include hector_slam_lib headers (pure C++ / Eigen) directly.
//    - Replace every ros:: / tf:: call with its rclcpp / tf2_ros
//      equivalent.  The algorithm is UNTOUCHED.
// ============================================================

#include <rclcpp/rclcpp.hpp>

// TF2
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Messages
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

// ─── Hector SLAM algorithm headers (pure C++) ─────────────────────────────
// These come from the original hector_slam repo's
//   hector_mapping/include/hector_slam_lib/
// They have NO ROS1 dependency – only Eigen.
#include <hector_slam_lib/slam_main/HectorSlamProcessor.h>
#include <hector_slam_lib/map/GridMap.h>
#include <hector_slam_lib/scan/DataPointContainer.h>
// ──────────────────────────────────────────────────────────────────────────

#include <memory>
#include <mutex>
#include <string>

namespace hector_slam_ros2
{

// ── Thin mutex shim so HectorSlamProcessor can lock map tiles ──────────────
class MapMutex : public MapLockerInterface
{
public:
  void lockMap()   override { mutex_.lock(); }
  void unlockMap() override { mutex_.unlock(); }

private:
  std::mutex mutex_;
};

// ── Main ROS2 node ─────────────────────────────────────────────────────────
class HectorMappingRos2 : public rclcpp::Node
{
public:
  explicit HectorMappingRos2(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  ~HectorMappingRos2() override = default;

private:
  // ── ROS2 I/O ─────────────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr   map_pub_;
  rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr     map_meta_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_with_cov_pub_;

  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr map_service_;
  rclcpp::TimerBase::SharedPtr                      map_pub_timer_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer>               tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;

  // ── Algorithm core ────────────────────────────────────────────────────────
  std::unique_ptr<hectorslam::HectorSlamProcessor> slam_processor_;
  std::unique_ptr<MapMutex>                        map_mutex_;

  // ── Parameters ────────────────────────────────────────────────────────────
  std::string p_map_frame_       {"map"};
  std::string p_base_frame_      {"base_link"};
  std::string p_odom_frame_      {"odom"};
  std::string p_scan_topic_      {"/scan"};

  double p_map_resolution_       {0.05};
  int    p_map_size_             {1024};
  double p_map_start_x_          {0.5};
  double p_map_start_y_          {0.5};
  int    p_map_multi_res_levels_ {3};

  double p_update_factor_free_     {0.4};
  double p_update_factor_occupied_ {0.9};
  double p_map_update_dist_thresh_ {0.4};
  double p_map_update_angle_thresh_{0.9};

  double p_map_pub_period_       {2.0};   // seconds

  double p_laser_min_dist_       {0.4};
  double p_laser_max_dist_       {30.0};
  double p_laser_z_min_          {-1.0};
  double p_laser_z_max_          {1.0};

  bool   p_pub_map_odom_tf_      {false};  // Hector does NOT use odom
  bool   p_pub_map_scanmatch_tf_ {true};
  bool   p_use_sim_time_         {false};

  // ── Internal state ────────────────────────────────────────────────────────
  nav_msgs::msg::OccupancyGrid map_msg_;
  std::mutex                   map_msg_mutex_;

  Eigen::Vector3f last_published_pose_{0.0f, 0.0f, 0.0f};

  // ── Callbacks & helpers ───────────────────────────────────────────────────
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  void mapPubTimerCallback();
  void mapServiceCallback(
    const std::shared_ptr<nav_msgs::srv::GetMap::Request>  req,
    std::shared_ptr<nav_msgs::srv::GetMap::Response>       res);

  void publishMap(const rclcpp::Time & stamp);
  void publishPose(const Eigen::Vector3f & pose, const rclcpp::Time & stamp);
  void publishTf(const Eigen::Vector3f & pose, const rclcpp::Time & stamp);

  bool laserScanToDataContainer(
    const sensor_msgs::msg::LaserScan & scan,
    hectorslam::DataContainer          & container,
    float                               scale_to_map);

  void declareParameters();
  void readParameters();
};

}  // namespace hector_slam_ros2