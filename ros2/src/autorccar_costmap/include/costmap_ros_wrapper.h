#ifndef COSTMAP_ROS_WRAPPER_H
#define COSTMAP_ROS_WRAPPER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <eigen3/Eigen/Dense>

#include "autorccar_interfaces/msg/nav_state.hpp"
#include "costmap.h"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"

typedef pcl::PointXYZI PointType;

class CostmapWrapper : public rclcpp::Node {
   public:
    CostmapWrapper(Costmap* pCostmap);

   private:
    Costmap* mpCostmap;

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr point_cloud_subscriber;  // livox lidar
    rclcpp::Subscription<autorccar_interfaces::msg::NavState>::SharedPtr nav_state_subscriber;  // navigation output
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr costmap_save_cmd_subscriber;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher;

    void PointCloudCallback(const livox_ros_driver2::msg::CustomMsg& msg);
    void NavStateCallback(const autorccar_interfaces::msg::NavState& msg);
    void CostmapSaveCmdCallback(const std_msgs::msg::Bool& msg);

    void CostmapToRosOccupancyGridMsg(bool save_pgm);

    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_in;
    Eigen::Matrix<double, 4, 4> transformation;
};

#endif
