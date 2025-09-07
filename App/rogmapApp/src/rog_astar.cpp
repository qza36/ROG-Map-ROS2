#include "rogmapApp/astar/rog_astar.hpp"
#include "rog_map_ros/rog_map_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>

int main(int argc, char** argv)
{
    // 1. Initialize ROS 2
    rclcpp::init(argc, argv);

    // 2. Create the ROS 2 node
    auto node = std::make_shared<rclcpp::Node>("rog_astar_node");

    RCLCPP_INFO(node->get_logger(), "A-Star node started.");

    // 3. Create the map object from the ROS wrapper
    // The ROGMapROS class inherits from ROGMap, so a shared_ptr to it can be used
    // where a shared_ptr to ROGMap is expected.
    // Using the same hardcoded config path as the other executable in the project.
    try {
        auto rog_map_ros = std::make_shared<rog_map::ROGMapROS>(node, "/ros2_ws/src/COD_NAV_NEXT/cnn_perception/rog_map/config/config.yaml");

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
