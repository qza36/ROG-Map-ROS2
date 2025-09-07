#ifndef ROG_ASTAR_CONFIG_HPP
#define ROG_ASTAR_CONFIG_HPP

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <vector>
#include <exception>
#include <string> // Add for std::string

namespace rog_astar
{
    using std::string;
    using std::vector;
    using Vec3i = Eigen::Vector3i;
    typedef pcl::PointXYZINormal PclPoint;
    typedef pcl::PointCloud<PclPoint> PointCloud;

    class Config
    {
    private:
        rclcpp::Node::SharedPtr node_;

        template<class T>
        bool LoadParam(const std::string& param_name, T& param_value,
                      T default_value = T{}, bool required = false)
        {
            try
            {
                // 声明参数
                if (!node_->has_parameter(param_name)) {
                    node_->declare_parameter<T>(param_name, default_value);
                }

                // 获取参数值
                param_value = node_->get_parameter(param_name).get_value<T>();
                printf("\033[0;32m Load param %s success: \033[0;0m",
                       (std::string(node_->get_fully_qualified_name()) + "/" + param_name).c_str());
                std::cout << param_value << std::endl;
                return true;
            }
            catch (const std::exception& e)
            {
                // 其他异常情况
                printf("\033[0;33m Load param %s failed with error: %s, use default value: \033[0;0m",
                       (std::string(node_->get_fully_qualified_name()) + "/" + param_name).c_str(),
                       e.what());
                param_value = default_value;
                std::cout << param_value << std::endl;
                if (required) {
                    throw std::invalid_argument(
                        std::string("Required param ") + node_->get_fully_qualified_name() +
                                   "/" + param_name + " not found");
                }
                return false;
            }
        }

        // 特化版本：处理 vector 类型参数
        template<class T>
        bool LoadParam(const std::string& param_name, std::vector<T>& param_value,
                      std::vector<T> default_value = std::vector<T>{}, bool required = false)
        {
            try
            {
                // 声明参数（如果不存在则创建，如果存在则获取）
                if (!node_->has_parameter(param_name)) {
                    node_->declare_parameter<std::vector<T>>(param_name, default_value);
                }

                // 获取参数值 - 使用正确的方法
                param_value = node_->get_parameter(param_name).get_value<std::vector<T>>();
                printf("\033[0;32m Load param %s success: \033[0;0m",
                       (std::string(node_->get_fully_qualified_name()) + "/" + param_name).c_str());
                for (const auto& val : param_value) {
                    std::cout << val << " ";
                }
                std::cout << std::endl;
                return true;
            }
            catch (const std::exception& e)
            {
                // 其他异常情况
                printf("\033[0;33m Load param %s failed with error: %s, use default value: \033[0;0m",
                       (std::string(node_->get_fully_qualified_name()) + "/" + param_name).c_str(),
                       e.what());
                param_value = default_value;
                for (const auto& val : param_value) {
                    std::cout << val << " ";
                }
                std::cout << std::endl;
                if (required) {
                    throw std::invalid_argument(
                        std::string("Required param ") + node_->get_fully_qualified_name() +
                                   "/" + param_name + " not found");
                }
                return false;
            }
        }

    public:
        bool visualize_process_en;
        bool allow_diag;
        Vec3i map_voxel_num, map_size_i;
        int heu_type;

        Eigen::Vector3d example_start;
        Eigen::Vector3d example_goal;

        explicit Config(const rclcpp::Node::SharedPtr& node) : node_(node)
        {
            // 加载布尔参数
            LoadParam("astar/visualize_process_en", visualize_process_en, false);
            LoadParam("astar/allow_diag", allow_diag, false);

            // 加载整数参数
            LoadParam("astar/heu_type", heu_type, 0);

            vector<long> vox;
            LoadParam("astar/map_voxel_num", vox, vector<long>{100, 100, 100});
            if(vox.size() == 3) {
                map_voxel_num = Vec3i(vox[0], vox[1], vox[2]);
            } else {
                RCLCPP_WARN(node_->get_logger(),
                           "map_voxel_num should have 3 elements, using default [100,100,100]");
                map_voxel_num = Vec3i(100, 100, 100);
            }

            // 加载起点坐标
            vector<double> tmp;
            LoadParam("astar/example_start", tmp, vector<double>{0.0, 0.0, 0.0});
            if(tmp.size() == 3) {
                example_start = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]);
            } else {
                RCLCPP_WARN(node_->get_logger(),
                           "example_start should have 3 elements, using default [0,0,0]");
                example_start = Eigen::Vector3d(0.0, 0.0, 0.0);
            }

            // 加载终点坐标
            tmp.clear();
            LoadParam("astar/example_goal", tmp, vector<double>{10.0, 10.0, 10.0});
            if(tmp.size() == 3) {
                example_goal = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]);
            } else {
                RCLCPP_WARN(node_->get_logger(),
                           "example_goal should have 3 elements, using default [10,10,10]");
                example_goal = Eigen::Vector3d(10.0, 10.0, 10.0);
            }

            // 确保地图体素数为奇数（可能是为了中心对称）
            // 注意：这会覆盖用户配置的值
            map_size_i = map_voxel_num / 2;
            map_voxel_num = map_size_i * 2 + Vec3i::Constant(1);

            RCLCPP_INFO(node_->get_logger(),
                       "Final map_voxel_num: [%d, %d, %d]",
                       map_voxel_num.x(), map_voxel_num.y(), map_voxel_num.z());
        }
    };
}

#endif // ROG_ASTAR_CONFIG_HPP