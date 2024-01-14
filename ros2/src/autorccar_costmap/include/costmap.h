#ifndef COSTMAP_H
#define COSTMAP_H

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

struct CostmapInfo {
    double resolution;    // map resolution [m/cell]
    unsigned int size_x;  // map width [cell]
    unsigned int size_y;  // map height [cell]
    double origin_pos_x;  // real-world pose of the cell (0,0) in the map.
    double origin_pos_y;
    Eigen::MatrixXd *costmap;
};

class Costmap {
   public:
    Costmap(const std::string &config_file_path);

    bool costmap_flag;  // noticify it's time to update the costmap

    void UpdatePointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr pcl);
    void UpdatePose(Eigen::Matrix<double, 4, 4> &trans);
    void UpdateCostmap();

    struct CostmapInfo GetGlobalCostmapInfo();
    struct CostmapInfo GetLocalCostmapInfo();

   private:
    std::string config;
    unsigned int cnt_iter;
    unsigned int cnt_limit;

    // global costmap
    Eigen::MatrixXd global_costmap;
    double resolution;  // costmap resolution [m/cell]
    int global_width;   // costmap width and height [m]
    int global_height;
    int global_size_x;  // number of cell
    int global_size_y;
    int global_center_x;  // origin cell index
    int global_center_y;

    // local costmap
    Eigen::MatrixXd local_costmap;
    int local_width;
    int local_height;
    int local_size_x;  // number of cell
    int local_size_y;
    int local_center_x;  // origin cell index
    int local_center_y;

    double P_UNKNOWN = 0.5;
    double P_FREE = 0.4;
    double P_OCCUPIED = 0.6;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_point_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_point_cloud_transformed;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_point_cloud_filtered;
    Eigen::Matrix<double, 4, 4> cur_pose;

    void LoadConfig(const std::string &config);
    void InitCostmap(Eigen::MatrixXd &costmap, int size_x, int size_y);
    void UpdateCostmapFlag();
    bool IsInBounds(int x, int y);
    void BresenhamLine(int sx, int sy, int ex, int ey);
    void UpdateCell(int x, int y, double p);
    double ProbabilityToLogOdds(double p);
    double LogOddsToProbability(double l);
};

#endif
