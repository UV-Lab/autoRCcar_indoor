#include "costmap_ros_wrapper.h"

#include "costmap.h"
#include "rclcpp/rclcpp.hpp"

CostmapWrapper::CostmapWrapper(Costmap* pCostmap) : Node("costmap"), mpCostmap(pCostmap) {
    point_cloud_subscriber = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        "livox/lidar", 10, std::bind(&CostmapWrapper::PointCloudCallback, this, std::placeholders::_1));

    nav_state_subscriber = this->create_subscription<autorccar_interfaces::msg::NavState>(
        "/nav_topic", 10, std::bind(&CostmapWrapper::NavStateCallback, this, std::placeholders::_1));

    costmap_save_cmd_subscriber = this->create_subscription<std_msgs::msg::Bool>(
        "/costmap/save", 10, std::bind(&CostmapWrapper::CostmapSaveCmdCallback, this, std::placeholders::_1));

    occupancy_grid_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 10);

    // initialization
    point_cloud_in.reset(new pcl::PointCloud<pcl::PointXYZI>());
    point_cloud_in->clear();
    transformation.setIdentity();
}

void CostmapWrapper::PointCloudCallback(const livox_ros_driver2::msg::CustomMsg& msg) {
    // convert livox msg to pcl msg
    point_cloud_in->clear();  // Removes all points in a cloud and sets the width and height to 0
    point_cloud_in->reserve(msg.point_num);
    point_cloud_in->header.frame_id = msg.header.frame_id;
    point_cloud_in->header.stamp = (uint64_t)((msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec) / 1000);

    pcl::PointXYZI point;
    for (uint i = 0; i < msg.point_num - 1; i++) {
        point.x = msg.points[i].x;
        point.y = msg.points[i].y;
        point.z = msg.points[i].z;
        point.intensity = msg.points[i].reflectivity;
        point_cloud_in->push_back(point);
    }

    // update pointcloud
    mpCostmap->UpdatePointCloud(point_cloud_in);

    // update costmap & publish occupancy grid
    if (mpCostmap->costmap_flag) {
        mpCostmap->UpdateCostmap();
        CostmapToRosOccupancyGridMsg(false);
    }
}

void CostmapWrapper::NavStateCallback(const autorccar_interfaces::msg::NavState& msg) {
    transformation.block<3, 3>(0, 0) =
        Eigen::Quaterniond(msg.quaternion.w, msg.quaternion.x, msg.quaternion.y, msg.quaternion.z).toRotationMatrix();
    transformation(0, 3) = msg.position.x;
    transformation(1, 3) = msg.position.y;
    transformation(2, 3) = msg.position.z;

    // update pose
    mpCostmap->UpdatePose(transformation);
}

void CostmapWrapper::CostmapSaveCmdCallback(const std_msgs::msg::Bool& msg) {
    if (msg.data) {
        CostmapToRosOccupancyGridMsg(true);
    }
}

void CostmapWrapper::CostmapToRosOccupancyGridMsg(bool save_pgm) {
    struct CostmapInfo info = mpCostmap->GetCostmapInfo();

    nav_msgs::msg::OccupancyGrid occupancy_grid_map;
    occupancy_grid_map.header.stamp = this->get_clock()->now();
    occupancy_grid_map.header.frame_id = "map";
    occupancy_grid_map.info.width = info.size_x;
    occupancy_grid_map.info.height = info.size_y;
    occupancy_grid_map.info.resolution = static_cast<float>(info.resolution);
    occupancy_grid_map.info.origin.position.x = static_cast<float>(info.origin_pos_x);
    occupancy_grid_map.info.origin.position.y = static_cast<float>(info.origin_pos_y);

    for (size_t i = 0; i < info.size_x * info.size_y; i++) {
        // convert log odds to probability(0~1)
        double logOdds = info.costmap->data()[i];
        double prob = std::exp(logOdds) / (1 + std::exp(logOdds));

        if (prob == 0.5) {
            occupancy_grid_map.data.push_back(-1);
        } else {
            occupancy_grid_map.data.push_back(prob * 100);
        }
    }

    occupancy_grid_publisher->publish(occupancy_grid_map);

    // save as pgm (https://github.com/ros-planning/navigation/blob/noetic-devel/map_server/src/map_saver.cpp)
    if (save_pgm) {
        std::string package_share_dir = ament_index_cpp::get_package_share_directory("autorccar_costmap");
        std::string map_data_file = package_share_dir + "/map.pgm";
        FILE* out = fopen(map_data_file.c_str(), "w");
        if (!out) {
            RCLCPP_INFO(this->get_logger(), "Couldn't save map file");
            return;
        }

        fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n", occupancy_grid_map.info.resolution,
                occupancy_grid_map.info.width, occupancy_grid_map.info.height);

        for (unsigned int y = 0; y < occupancy_grid_map.info.height; y++) {
            for (unsigned int x = 0; x < occupancy_grid_map.info.width; x++) {
                unsigned int i = x + (occupancy_grid_map.info.height - y - 1) * occupancy_grid_map.info.width;
                if (occupancy_grid_map.data[i] >= 0 && occupancy_grid_map.data[i] <= 25) {  // [0,free)
                    fputc(254, out);
                } else if (occupancy_grid_map.data[i] >= 65) {  // (occ,255]
                    fputc(000, out);
                } else {  // occ [0.25,0.65]
                    fputc(205, out);
                }
            }
        }

        fclose(out);

        RCLCPP_INFO(this->get_logger(), "Saved map occupancy data to %s", map_data_file.c_str());
    }
}
