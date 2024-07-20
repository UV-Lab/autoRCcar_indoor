#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "costmap.h"
#include "costmap_config_manager.h"
#include "costmap_ros_wrapper.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    std::string config = argv[1];

    ConfigManager config_manager(config);

    Costmap occupancy_grid(&config_manager);

    auto node = std::make_shared<CostmapWrapper>(&occupancy_grid, &config_manager);
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
