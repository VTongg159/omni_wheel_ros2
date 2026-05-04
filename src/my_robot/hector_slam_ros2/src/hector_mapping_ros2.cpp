// ============================================================
//  hector_mapping_ros2.cpp
//
//  ROS2 Humble wrapper for hector_slam algorithm.
//
//  Key design decisions
//  ─────────────────────
//  • hectorslam::HectorSlamProcessor is used exactly as-is.
//    Zero algorithm changes.
//  • ALL ros:: / tf:: calls replaced with rclcpp / tf2_ros.
//  • Hector SLAM does NOT use odometry – we publish
//      map → base_link   directly from the scan-matcher output.
//  • Map publishes on a timer (p_map_pub_period_) AND after
//    each scan that triggers a map update.
// ============================================================

#include "hector_mapping_ros2.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <rclcpp/create_timer.hpp>

#include <cmath>
#include <cstring>

namespace hector_slam_ros2
{

// ── Constructor ───────────────────────────────────────────────────────────
HectorMappingRos2::HectorMappingRos2(const rclcpp::NodeOptions & options)
: Node("hector_mapping", options)
{
  declareParameters();
  readParameters();

  // ── TF2 ────────────────────────────────────────────────────────────────
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_buffer_      = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── Create SLAM processor ───────────────────────────────────────────────
  // Arguments mirror the original HectorMappingRos constructor.
  slam_processor_ = std::make_unique<hectorslam::HectorSlamProcessor>(
    static_cast<float>(p_map_resolution_),
    p_map_size_, p_map_size_,
    Eigen::Vector2f(
      static_cast<float>(p_map_start_x_),
      static_cast<float>(p_map_start_y_)),
    p_map_multi_res_levels_,
    nullptr,   // hectorDrawings  – not needed in ROS2 wrapper
    nullptr);  // debugInfoProvider

  slam_processor_->setUpdateFactorFree(
    static_cast<float>(p_update_factor_free_));
  slam_processor_->setUpdateFactorOccupied(
    static_cast<float>(p_update_factor_occupied_));
  slam_processor_->setMapUpdateMinDistDiff(
    static_cast<float>(p_map_update_dist_thresh_));
  slam_processor_->setMapUpdateMinAngleDiff(
    static_cast<float>(p_map_update_angle_thresh_));

  // Mutex for level-0 map tile
  map_mutex_ = std::make_unique<MapMutex>();
  slam_processor_->addMapMutex(0, map_mutex_.get());

  // ── Publishers ─────────────────────────────────────────────────────────
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).transient_local());

  map_meta_pub_ = this->create_publisher<nav_msgs::msg::MapMetaData>(
    "/map_metadata", rclcpp::QoS(1).transient_local());

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "slam_out_pose", 5);

  pose_with_cov_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "poseupdate", 5);

  // ── Subscriber ──────────────────────────────────────────────────────────
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    p_scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&HectorMappingRos2::scanCallback, this, std::placeholders::_1));

  // ── Map service ─────────────────────────────────────────────────────────
  map_service_ = this->create_service<nav_msgs::srv::GetMap>(
    "dynamic_map",
    std::bind(&HectorMappingRos2::mapServiceCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  // ── Periodic map publisher ───────────────────────────────────────────────
  auto period_ms = std::chrono::milliseconds(
    static_cast<int>(p_map_pub_period_ * 1000.0));
  map_pub_timer_ = rclcpp::create_timer(
    this, this->get_clock(), period_ms,
    std::bind(&HectorMappingRos2::mapPubTimerCallback, this));

  RCLCPP_INFO(this->get_logger(),
    "[HectorSLAM-ROS2] Ready.\n"
    "  map_frame      : %s\n"
    "  base_frame     : %s\n"
    "  scan_topic     : %s\n"
    "  map_resolution : %.3f m/px\n"
    "  map_size       : %d px\n"
    "  multi_res_levels: %d",
    p_map_frame_.c_str(), p_base_frame_.c_str(),
    p_scan_topic_.c_str(),
    p_map_resolution_, p_map_size_, p_map_multi_res_levels_);
}

// ── Parameter helpers ─────────────────────────────────────────────────────
void HectorMappingRos2::declareParameters()
{
  this->declare_parameter("map_frame",              p_map_frame_);
  this->declare_parameter("base_frame",             p_base_frame_);
  this->declare_parameter("odom_frame",             p_odom_frame_);
  this->declare_parameter("scan_topic",             p_scan_topic_);
  this->declare_parameter("map_resolution",         p_map_resolution_);
  this->declare_parameter("map_size",               p_map_size_);
  this->declare_parameter("map_start_x",            p_map_start_x_);
  this->declare_parameter("map_start_y",            p_map_start_y_);
  this->declare_parameter("map_multi_res_levels",   p_map_multi_res_levels_);
  this->declare_parameter("update_factor_free",     p_update_factor_free_);
  this->declare_parameter("update_factor_occupied", p_update_factor_occupied_);
  this->declare_parameter("map_update_dist_thresh", p_map_update_dist_thresh_);
  this->declare_parameter("map_update_angle_thresh",p_map_update_angle_thresh_);
  this->declare_parameter("map_pub_period",         p_map_pub_period_);
  this->declare_parameter("laser_min_dist",         p_laser_min_dist_);
  this->declare_parameter("laser_max_dist",         p_laser_max_dist_);
  this->declare_parameter("laser_z_min",            p_laser_z_min_);
  this->declare_parameter("laser_z_max",            p_laser_z_max_);
  this->declare_parameter("pub_map_odom_tf",        p_pub_map_odom_tf_);
  this->declare_parameter("pub_map_scanmatch_tf",   p_pub_map_scanmatch_tf_);
}

void HectorMappingRos2::readParameters()
{
  this->get_parameter("map_frame",               p_map_frame_);
  this->get_parameter("base_frame",              p_base_frame_);
  this->get_parameter("odom_frame",              p_odom_frame_);
  this->get_parameter("scan_topic",              p_scan_topic_);
  this->get_parameter("map_resolution",          p_map_resolution_);
  this->get_parameter("map_size",                p_map_size_);
  this->get_parameter("map_start_x",             p_map_start_x_);
  this->get_parameter("map_start_y",             p_map_start_y_);
  this->get_parameter("map_multi_res_levels",    p_map_multi_res_levels_);
  this->get_parameter("update_factor_free",      p_update_factor_free_);
  this->get_parameter("update_factor_occupied",  p_update_factor_occupied_);
  this->get_parameter("map_update_dist_thresh",  p_map_update_dist_thresh_);
  this->get_parameter("map_update_angle_thresh", p_map_update_angle_thresh_);
  this->get_parameter("map_pub_period",          p_map_pub_period_);
  this->get_parameter("laser_min_dist",          p_laser_min_dist_);
  this->get_parameter("laser_max_dist",          p_laser_max_dist_);
  this->get_parameter("laser_z_min",             p_laser_z_min_);
  this->get_parameter("laser_z_max",             p_laser_z_max_);
  this->get_parameter("pub_map_odom_tf",         p_pub_map_odom_tf_);
  this->get_parameter("pub_map_scanmatch_tf",    p_pub_map_scanmatch_tf_);
  this->get_parameter("use_sim_time",            p_use_sim_time_);
}

// ── Scan callback – core update loop ──────────────────────────────────────
void HectorMappingRos2::scanCallback(
  const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  // Scale factor: map pixels per metre
  float scale_to_map = 1.0f / static_cast<float>(p_map_resolution_);

  // Convert LaserScan → hectorslam::DataContainer
  hectorslam::DataContainer data_container;
  if (!laserScanToDataContainer(*scan, data_container, scale_to_map)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "laserScanToDataContainer returned no valid points – skipping scan.");
    return;
  }

  // ── Run Hector SLAM update (algorithm untouched) ──────────────────────
  slam_processor_->update(data_container, slam_processor_->getLastScanMatchPose());
  Eigen::Vector3f slam_pose = slam_processor_->getLastScanMatchPose();

  rclcpp::Time stamp = scan->header.stamp;

  // ── Publish pose & TF ─────────────────────────────────────────────────
  publishPose(slam_pose, stamp);
  publishTf(slam_pose, stamp);
}

// ── Periodic map publisher ────────────────────────────────────────────────
void HectorMappingRos2::mapPubTimerCallback()
{
  publishMap(this->now());
}

// ── Map service handler ───────────────────────────────────────────────────
void HectorMappingRos2::mapServiceCallback(
  const std::shared_ptr<nav_msgs::srv::GetMap::Request>  /*req*/,
  std::shared_ptr<nav_msgs::srv::GetMap::Response>       res)
{
  std::lock_guard<std::mutex> lock(map_msg_mutex_);
  res->map = map_msg_;
}

// ── Publish OccupancyGrid ─────────────────────────────────────────────────
void HectorMappingRos2::publishMap(const rclcpp::Time & stamp)
{
  // Retrieve level-0 grid map from SLAM processor
  const hectorslam::GridMap & grid_map =
    slam_processor_->getGridMap(0);

  int size_x = grid_map.getSizeX();
  int size_y = grid_map.getSizeY();
  int size   = size_x * size_y;

  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = p_map_frame_;
  msg.header.stamp    = stamp;

  msg.info.resolution = static_cast<float>(p_map_resolution_);
  msg.info.width      = static_cast<uint32_t>(size_x);
  msg.info.height     = static_cast<uint32_t>(size_y);

  // Map origin in world coordinates
  Eigen::Vector2f map_origin =
    grid_map.getWorldCoords(Eigen::Vector2f(0.0f, 0.0f));

  msg.info.origin.position.x =
    static_cast<double>(map_origin.x()) - p_map_resolution_ * 0.5;
  msg.info.origin.position.y =
    static_cast<double>(map_origin.y()) - p_map_resolution_ * 0.5;
  msg.info.origin.orientation.w = 1.0;

  // ── Fill cell data ────────────────────────────────────────────────────
  // Original hector_mapping uses the same logic below.
  msg.data.resize(static_cast<size_t>(size));

  map_mutex_->lockMap();

  for (int i = 0; i < size; ++i) {
    if (grid_map.isFree(i)) {
      msg.data[static_cast<size_t>(i)] = 0;           // free
    } else if (grid_map.isOccupied(i)) {
      msg.data[static_cast<size_t>(i)] = 100;          // occupied
    } else {
      msg.data[static_cast<size_t>(i)] = -1;           // unknown
    }
  }

  map_mutex_->unlockMap();

  {
    std::lock_guard<std::mutex> lock(map_msg_mutex_);
    map_msg_ = msg;
  }

  map_pub_->publish(msg);
  map_meta_pub_->publish(msg.info);
}

// ── Publish pose messages ─────────────────────────────────────────────────
void HectorMappingRos2::publishPose(
  const Eigen::Vector3f & pose, const rclcpp::Time & stamp)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, static_cast<double>(pose[2]));
  q.normalize();

  // PoseStamped
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id       = p_map_frame_;
  ps.header.stamp          = stamp;
  ps.pose.position.x       = static_cast<double>(pose[0]);
  ps.pose.position.y       = static_cast<double>(pose[1]);
  ps.pose.orientation.x    = q.x();
  ps.pose.orientation.y    = q.y();
  ps.pose.orientation.z    = q.z();
  ps.pose.orientation.w    = q.w();
  pose_pub_->publish(ps);

  // PoseWithCovarianceStamped (diagonal covariance, rough estimate)
  geometry_msgs::msg::PoseWithCovarianceStamped pcs;
  pcs.header           = ps.header;
  pcs.pose.pose        = ps.pose;
  pcs.pose.covariance[0]  = 0.1;   // x
  pcs.pose.covariance[7]  = 0.1;   // y
  pcs.pose.covariance[35] = 0.05;  // yaw
  pose_with_cov_pub_->publish(pcs);
}

// ── Publish TF: map → base_link (and optionally map → odom) ──────────────
void HectorMappingRos2::publishTf(
  const Eigen::Vector3f & pose, const rclcpp::Time & stamp)
{
  if (!p_pub_map_scanmatch_tf_) { return; }

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, static_cast<double>(pose[2]));
  q.normalize();

  geometry_msgs::msg::TransformStamped ts;
  ts.header.stamp            = stamp;
  ts.header.frame_id         = p_map_frame_;       // parent
  ts.child_frame_id          = p_base_frame_;      // child
  ts.transform.translation.x = static_cast<double>(pose[0]);
  ts.transform.translation.y = static_cast<double>(pose[1]);
  ts.transform.translation.z = 0.0;
  ts.transform.rotation.x    = q.x();
  ts.transform.rotation.y    = q.y();
  ts.transform.rotation.z    = q.z();
  ts.transform.rotation.w    = q.w();

  tf_broadcaster_->sendTransform(ts);

  // ── Optional map → odom TF ────────────────────────────────────────────
  // Only useful if you want Nav2 / move_base to keep the odom chain intact.
  // Hector SLAM itself does NOT need odometry.
  if (p_pub_map_odom_tf_) {
    try {
      // Look up odom → base_link
      geometry_msgs::msg::TransformStamped odom_to_base =
        tf_buffer_->lookupTransform(
          p_odom_frame_, p_base_frame_, stamp,
          rclcpp::Duration::from_seconds(0.05));

      // map_to_odom = map_to_base * odom_to_base^{-1}
      tf2::Transform tf_map_to_base;
      tf2::fromMsg(ts.transform, tf_map_to_base);

      tf2::Transform tf_odom_to_base;
      tf2::fromMsg(odom_to_base.transform, tf_odom_to_base);

      tf2::Transform tf_map_to_odom = tf_map_to_base * tf_odom_to_base.inverse();

      geometry_msgs::msg::TransformStamped map_to_odom_ts;
      map_to_odom_ts.header.stamp    = stamp;
      map_to_odom_ts.header.frame_id = p_map_frame_;
      map_to_odom_ts.child_frame_id  = p_odom_frame_;
      map_to_odom_ts.transform       = tf2::toMsg(tf_map_to_odom);

      tf_broadcaster_->sendTransform(map_to_odom_ts);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "map→odom TF failed: %s", ex.what());
    }
  }
}

// ── Convert LaserScan → DataContainer (algorithm expects scaled coords) ───
bool HectorMappingRos2::laserScanToDataContainer(
  const sensor_msgs::msg::LaserScan & scan,
  hectorslam::DataContainer          & container,
  float                               scale_to_map)
{
  size_t n = scan.ranges.size();
  container.clear();
  container.setOrigo(Eigen::Vector2f::Zero());

  float angle = scan.angle_min;
  int   valid = 0;

  for (size_t i = 0; i < n; ++i, angle += scan.angle_increment) {
    float dist = scan.ranges[i];

    // Range filter
    if (!std::isfinite(dist) ||
        dist < static_cast<float>(p_laser_min_dist_) ||
        dist > static_cast<float>(p_laser_max_dist_)) {
      continue;
    }

    // Convert polar → Cartesian, scale to map pixels
    float x = std::cos(angle) * dist * scale_to_map;
    float y = std::sin(angle) * dist * scale_to_map;

    container.add(Eigen::Vector2f(x, y));
    ++valid;
  }

  return valid > 0;
}

}  // namespace hector_slam_ros2