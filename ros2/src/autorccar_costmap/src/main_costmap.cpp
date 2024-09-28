#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "config_reader.h"
#include "costmap.h"
#include "costmap_ros_wrapper.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    std::string config = argv[1];

    auto config_reader = std::make_shared<ConfigReader>(config);
    auto occupancy_grid = std::make_shared<Costmap>(config_reader);

    auto node = std::make_shared<CostmapWrapper>(occupancy_grid);
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
