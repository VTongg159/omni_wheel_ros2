/*
 * tinyslam_node.cpp
 *
 * ROS 2 wrapper for TinySLAM (CoreSLAM).
 *
 * TF tree published: map -> odom -> base_link -> laser
 *   - Node publishes:       map  -> odom
 *   - launch static_tf:     odom -> base_link  (nếu không có odometry thật)
 *   - robot_state_publisher: base_link -> laser
 */

#include "tinyslam_ros2/tinyslam_node.hpp"

#include <cmath>
#include <cstring>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace tinyslam_ros2
{

/* ================================================================
   Constructor / Destructor
   ================================================================ */
TinySlamNode::TinySlamNode(const rclcpp::NodeOptions & options)
: Node("tinyslam_node", options)
{
  /* ---------- declare parameters --------------------------------- */
  declare_parameter("map_frame",                "map");
  declare_parameter("odom_frame",               "odom");       // ← THÊM
  declare_parameter("base_frame",               "base_link");
  declare_parameter("laser_frame",              "laser");
  declare_parameter("use_sim_time",              false);
  declare_parameter("sigma_xy",                 100.0);
  declare_parameter("sigma_theta",              0.1745);
  declare_parameter("hole_width",               600);
  declare_parameter("map_resolution",           0.1);
  declare_parameter("laser_offset",             0.0);
  declare_parameter("max_range",                8.0);
  declare_parameter("update_map_every_n_scans", 1);

  /* ---------- read parameters ------------------------------------ */
  map_frame_    = get_parameter("map_frame").as_string();
  odom_frame_   = get_parameter("odom_frame").as_string();     // ← THÊM
  base_frame_   = get_parameter("base_frame").as_string();
  laser_frame_  = get_parameter("laser_frame").as_string();
  sigma_xy_     = get_parameter("sigma_xy").as_double();
  sigma_theta_  = get_parameter("sigma_theta").as_double() * (180.0 / M_PI);
  hole_width_   = get_parameter("hole_width").as_int();
  map_resolution_ = get_parameter("map_resolution").as_double();
  laser_offset_ = get_parameter("laser_offset").as_double();
  max_range_    = get_parameter("max_range").as_double();
  update_map_every_n_scans_ = get_parameter("update_map_every_n_scans").as_int();

  /* ---------- TF ------------------------------------------------- */
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_      = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  /* ---------- publishers / subscribers --------------------------- */
  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(1).transient_local());

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(),
    std::bind(&TinySlamNode::scanCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
    "TinySLAM node started. TF: %s → %s → %s. Waiting for /scan …",
    map_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
}

TinySlamNode::~TinySlamNode()
{
  delete ts_map_;
}

/* ================================================================
   First-scan initialisation
   ================================================================ */
void TinySlamNode::initTinySlam()
{
  ts_map_ = new ts_map_t;
  ts_map_init(ts_map_);

  ts_robot_parameters_t robot_params{};
  robot_params.r     = 1.0;
  robot_params.R     = 1.0;
  robot_params.inc   = 1;
  robot_params.ratio = 1.0;

  ts_position_t init_pos{};
  init_pos.x     = TS_MAP_SIZE * 500.0 * TS_MAP_SCALE;
  init_pos.y     = TS_MAP_SIZE * 500.0 * TS_MAP_SCALE;
  init_pos.theta = 0.0;

  ts_state_init(&ts_state_, ts_map_, &robot_params,
                &ts_state_.laser_params,
                &init_pos,
                sigma_xy_, sigma_theta_,
                hole_width_, TS_DIRECTION_FORWARD);

  slam_initialized_ = true;
  RCLCPP_INFO(get_logger(),
    "TinySLAM initialised. Map: %d×%d px, %.3f m/px",
    TS_MAP_SIZE, TS_MAP_SIZE, TS_MAP_SCALE);
}

/* ================================================================
   LaserScan → ts_sensor_data_t
   ================================================================ */
void TinySlamNode::laserScanToSensorData(
  const sensor_msgs::msg::LaserScan::SharedPtr & msg,
  ts_sensor_data_t & sd)
{
  const int n    = static_cast<int>(msg->ranges.size());
  const int npts = std::min(n, TS_SCAN_SIZE);

  for (int i = 0; i < npts; ++i) {
    double r = msg->ranges[i];
    if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max || r > max_range_) {
      sd.d[i] = static_cast<int>(ts_state_.laser_params.distance_no_detection);
    } else {
      sd.d[i] = static_cast<int>(r * 1000.0);
    }
  }

  sd.timestamp = static_cast<unsigned int>(
    msg->header.stamp.sec * 1000u + msg->header.stamp.nanosec / 1000000u);
  sd.q1 = sd.q2 = 0;
  sd.v  = sd.psidot = 0.0;
}

/* ================================================================
   Main scan callback
   ================================================================ */
void TinySlamNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  /* --- on first scan: fill laser params & init SLAM -------------- */
  if (!slam_initialized_) {
    const int n = static_cast<int>(msg->ranges.size());

    ts_laser_parameters_t & lp = ts_state_.laser_params;
    lp.scan_size             = std::min(n, TS_SCAN_SIZE);
    lp.angle_min             = static_cast<int>(msg->angle_min * 180.0 / M_PI);
    lp.angle_max             = static_cast<int>(msg->angle_max * 180.0 / M_PI);
    lp.detection_margin      = 0;
    lp.distance_no_detection = max_range_ * 1000.0;
    lp.offset                = laser_offset_;

    initTinySlam();
  }

  const std::string scan_frame = msg->header.frame_id.empty()
    ? laser_frame_
    : msg->header.frame_id;

  if (scan_frame.empty()) {
    RCLCPP_WARN(get_logger(),
      "LaserScan frame_id is empty and laser_frame param is not set. Skipping.");
    return;
  }

  const rclcpp::Time stamp = msg->header.stamp;

  /* ---------------------------------------------------------------
   * FIX: Dùng tf2::TimePointZero thay vì timestamp chính xác.
   * Tránh lỗi timing khi use_sim_time=true (clock chưa sync kịp).
   * --------------------------------------------------------------- */
  geometry_msgs::msg::TransformStamped laser_to_base;
  try {
    laser_to_base = tf_buffer_->lookupTransform(
      base_frame_, scan_frame,
      tf2::TimePointZero);          // ← FIX: không dùng stamp cụ thể
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "TF lookup failed '%s' → '%s': %s",
      scan_frame.c_str(), base_frame_.c_str(), ex.what());
    return;
  }

  /* --- áp dụng yaw offset nếu laser không nằm trên base_link ----- */
  const bool need_transform = (scan_frame != base_frame_);
  sensor_msgs::msg::LaserScan scan_in_base = *msg;

  if (need_transform) {
    const tf2::Quaternion q(
      laser_to_base.transform.rotation.x,
      laser_to_base.transform.rotation.y,
      laser_to_base.transform.rotation.z,
      laser_to_base.transform.rotation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    if (std::abs(roll) > 1e-3 || std::abs(pitch) > 1e-3) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Laser→base transform has roll/pitch (%.3f, %.3f rad); only yaw supported.",
        roll, pitch);
    }

    const double y_off = laser_to_base.transform.translation.y;
    const double z_off = laser_to_base.transform.translation.z;
    if (std::abs(y_off) > 1e-3 || std::abs(z_off) > 1e-3) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Laser offset has y=%.3f m, z=%.3f m; only x supported cleanly.", y_off, z_off);
    }

    scan_in_base.header.frame_id = base_frame_;
    scan_in_base.angle_min      += yaw;
    scan_in_base.angle_max      += yaw;
    scan_in_base.header.stamp    = stamp;
  }

  /* --- chuyển đổi và chạy SLAM ----------------------------------- */
  ts_sensor_data_t sd{};
  laserScanToSensorData(
    std::make_shared<sensor_msgs::msg::LaserScan>(scan_in_base), sd);

  ts_iterative_map_building(&sd, &ts_state_);
  ++scan_count_;

  /* --- publish --------------------------------------------------- */
  publishTF(stamp);

  if (scan_count_ % static_cast<unsigned int>(update_map_every_n_scans_) == 0) {
    publishMap(stamp);
  }
}

/* ================================================================
   Publish TF: map → odom
   (odom → base_link được publish bởi static_transform_publisher
    trong launch file, hoặc odometry node thật)
   ================================================================ */
void TinySlamNode::publishTF(const rclcpp::Time & stamp)
{
  const ts_position_t & pos = ts_state_.position;

  const double map_origin_mm = TS_MAP_SIZE * 500.0 * TS_MAP_SCALE;
  const double x_m  = (pos.x - map_origin_mm) / 1000.0;
  const double y_m  = (pos.y - map_origin_mm) / 1000.0;
  const double yaw  = pos.theta * M_PI / 180.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp            = stamp;
  tf_msg.header.frame_id         = map_frame_;
  tf_msg.child_frame_id          = odom_frame_;   // ← FIX: map → odom (không phải base_link)
  tf_msg.transform.translation.x = x_m;
  tf_msg.transform.translation.y = y_m;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation      = tf2::toMsg(q);

  tf_broadcaster_->sendTransform(tf_msg);
}

/* ================================================================
   Publish nav_msgs/OccupancyGrid
   ================================================================ */
void TinySlamNode::publishMap(const rclcpp::Time & stamp)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp    = stamp;
  grid.header.frame_id = map_frame_;

  grid.info.resolution        = static_cast<float>(TS_MAP_SCALE);
  grid.info.width             = TS_MAP_SIZE;
  grid.info.height            = TS_MAP_SIZE;
  grid.info.origin.position.x = -(TS_MAP_SIZE * TS_MAP_SCALE) / 2.0;
  grid.info.origin.position.y = -(TS_MAP_SIZE * TS_MAP_SCALE) / 2.0;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  const int npix = TS_MAP_SIZE * TS_MAP_SIZE;
  grid.data.resize(npix);

  const ts_map_pixel_t * src = ts_map_->map;
  for (int i = 0; i < npix; ++i) {
    const int v = src[i];
    if      (v >= 65400) grid.data[i] = 0;    /* free     */
    else if (v <=   100) grid.data[i] = 100;  /* occupied */
    else                 grid.data[i] = -1;   /* unknown  */
  }

  map_pub_->publish(grid);
}

} // namespace tinyslam_ros2

/* ================================================================
   main
   ================================================================ */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  rclcpp::spin(std::make_shared<tinyslam_ros2::TinySlamNode>(options));
  rclcpp::shutdown();
  return 0;
}