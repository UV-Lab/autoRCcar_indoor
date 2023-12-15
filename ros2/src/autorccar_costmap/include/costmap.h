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

    bool pcl_flag;
    bool pose_flag;

    void UpdatePointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr pcl);
    void UpdatePose(Eigen::Matrix<double, 4, 4> &trans);
    void UpdateCostmap();

    struct CostmapInfo GetCostmapInfo();

   private:
    std::string config;
    unsigned int width;  // costmap width and height [m]
    unsigned int height;
    double resolution;  // costmap resolution [m/cell]

    Eigen::MatrixXd costmap;
    unsigned int size_x;  // number of cell
    unsigned int size_y;
    unsigned int center_x;  // origin cell index
    unsigned int center_y;

    double P_UNKNOWN = 0.5;
    double P_FREE = 0.4;
    double P_OCCUPIED = 0.6;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_point_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_point_cloud_transformed;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_point_cloud_filtered;
    Eigen::Matrix<double, 4, 4> cur_pose;

    void LoadConfig(const std::string &config);
    void InitCostmap(unsigned int size_x, unsigned int size_y);
    bool IsInBounds(int x, int y);
    void BresenhamLine(int sx, int sy, int ex, int ey);
    void UpdateCell(int x, int y, double p);
    double ProbabilityToLogOdds(double p);
    double LogOddsToProbability(double l);
};

#endif
