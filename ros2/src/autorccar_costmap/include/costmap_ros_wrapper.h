#ifndef AUTORCCAR_COSTMAP__COSTMAP_ROS_WRAPPER_H
#define AUTORCCAR_COSTMAP__COSTMAP_ROS_WRAPPER_H

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

class CostmapWrapper : public rclcpp::Node {
   public:
    CostmapWrapper(Costmap* pCostmap);

   private:
    Costmap* mpCostmap_;

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr point_cloud_subscriber_;  // livox lidar
    rclcpp::Subscription<autorccar_interfaces::msg::NavState>::SharedPtr nav_state_subscriber_;  // navigation output
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr costmap_save_cmd_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_occupancy_grid_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_occupancy_grid_publisher_;

    void PointCloudCallback(const livox_ros_driver2::msg::CustomMsg& msg);
    void NavStateCallback(const autorccar_interfaces::msg::NavState& msg);
    void CostmapSaveCmdCallback(const std_msgs::msg::Bool& msg);

    void PublishGlobalOccupancyGrid(bool save_pgm);
    void PublishLocalOccupancyGrid();
    void SaveCostmapAsPgm(const nav_msgs::msg::OccupancyGrid& msg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_in_;
    Eigen::Matrix<double, 4, 4> transformation_ = Eigen::Matrix4d::Zero();
};

#endif
