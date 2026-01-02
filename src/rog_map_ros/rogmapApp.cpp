#include "rog_map_ros/rog_map_ros2.hpp"

#define CONFIG_FILE_DIR(name) (std::string(ROOT_DIR) + "config/" + (name))

int main(int argc, char** argv)
{
   rclcpp::init(argc, argv);
   auto node = std::make_shared<rclcpp::Node>("rog_map_node");

   // 从参数获取配置文件名
   std::string cfg_name = "config.yaml";
   node->declare_parameter("config_name", cfg_name);
   node->get_parameter("config_name", cfg_name);
   std::string cfg_path = CONFIG_FILE_DIR(cfg_name);

   auto rog_map = std::make_shared<rog_map::ROGMapROS>(node, cfg_path);

   // 使用多线程执行器以支持回调组并发
   rclcpp::executors::MultiThreadedExecutor executor;
   executor.add_node(node);
   executor.spin();

   rclcpp::shutdown();
   return 0;
}