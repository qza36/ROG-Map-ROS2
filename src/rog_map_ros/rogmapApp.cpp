#include "rog_map_ros/rog_map_ros2.hpp"

int main(int argc, char** argv)
{
   rclcpp::init(argc, argv);
   rclcpp::NodeOptions options;
   auto node = std::make_shared<rclcpp::Node>("rog_map_node", options);
   auto rog_map = std::make_shared<rog_map::ROGMapROS>(node, "/ros2_ws/src/COD_NAV_NEXT/cnn_perception/rog_map/config/config.yaml");
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}