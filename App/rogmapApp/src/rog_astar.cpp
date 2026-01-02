#include "rogmapApp/astar/rog_astar.hpp"
#include "rog_map_ros/rog_map_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>

#define CONFIG_FILE_DIR(name) (std::string(ROOT_DIR) + "config/" + (name))

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("rog_astar_node");

    RCLCPP_INFO(node->get_logger(), "A-Star node started.");

    // 从参数获取配置文件名
    std::string cfg_name = "config.yaml";
    node->declare_parameter("config_name", cfg_name);
    node->get_parameter("config_name", cfg_name);
    std::string cfg_path = CONFIG_FILE_DIR(cfg_name);

    try {
        auto rog_map_ros = std::make_shared<rog_map::ROGMapROS>(node, cfg_path);

        // 4. Create the AStar planner instance
        auto astar_planner = std::make_shared<rog_astar::AStar>(node, rog_map_ros);

        RCLCPP_INFO(node->get_logger(), "Running A-Star example...");

        // 5. Run the example path search
        if (astar_planner->runExample()) {
            RCLCPP_INFO(node->get_logger(), "A-Star example finished successfully.");
            rog_map::vec_Vec3f path = astar_planner->getPath();
            RCLCPP_INFO(node->get_logger(), "Path found with %zu points.", path.size());
        } else {
            RCLCPP_ERROR(node->get_logger(), "A-Star example failed.");
        }

    } catch (const std::exception& e) {
        RCLCPP_FATAL(node->get_logger(), "A-Star node failed to initialize: %s", e.what());
    }

    // 6. Wait a bit for visualization messages to be sent and then shut down.
    RCLCPP_INFO(node->get_logger(), "Execution finished. Shutting down in 5 seconds...");
    rclcpp::sleep_for(std::chrono::seconds(5));

    // 7. Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}
