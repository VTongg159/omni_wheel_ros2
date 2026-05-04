#include <rclcpp/rclcpp.hpp>
#include "hector_mapping_ros2.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hector_slam_ros2::HectorMappingRos2>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
