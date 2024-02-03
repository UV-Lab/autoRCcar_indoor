#ifndef AUTORCCAR_COSTMAP__COSTMAP_H_
#define AUTORCCAR_COSTMAP__COSTMAP_H_

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using PointType = pcl::PointXYZI;

struct CostmapInfo {
    double resolution = 0.0;    // map resolution [m/cell]
    unsigned int size_x = 0;    // map width [cell]
    unsigned int size_y = 0;    // map height [cell]
    double origin_pos_x = 0.0;  // real-world pose of the cell (0,0) in the map.
    double origin_pos_y = 0.0;
    Eigen::MatrixXd* ptr_costmap;
};

class Costmap {
   public:
    Costmap(const std::string& config_file_path);

    bool costmap_flag_ = false;  // noticify it's time to update the costmap

    void UpdatePointCloud(const pcl::PointCloud<PointType>::Ptr pcl);
    void UpdatePose(Eigen::Matrix<double, 4, 4>& trans);
    void UpdateCostmap();

    struct CostmapInfo GetGlobalCostmapInfo();
    struct CostmapInfo GetLocalCostmapInfo();

   private:
    unsigned int cnt_iter_ = 0;
    unsigned int cnt_limit_ = 0;

    // global costmap
    Eigen::MatrixXd global_costmap_;
    double resolution_ = 0.0;  // costmap resolution [m/cell]
    int global_width_ = 0;     // costmap width and height [m]
    int global_height_ = 0;
    Eigen::Vector2i global_size_ = Eigen::Vector2i::Zero();    // number of cell
    Eigen::Vector2i global_center_ = Eigen::Vector2i::Zero();  // origin cell index

    // local costmap
    Eigen::MatrixXd local_costmap_;
    int local_width_ = 0;
    int local_height_ = 0;
    Eigen::Vector2i local_size_ = Eigen::Vector2i::Zero();    // number of cell
    Eigen::Vector2i local_center_ = Eigen::Vector2i::Zero();  // origin cell index

    static constexpr double P_UNKNOWN = 0.5;
    static constexpr double P_FREE = 0.4;
    static constexpr double P_OCCUPIED = 0.6;

    pcl::PointCloud<PointType>::Ptr cur_point_cloud_;
    pcl::PointCloud<PointType>::Ptr cur_point_cloud_transformed_;
    pcl::PointCloud<PointType>::Ptr cur_point_cloud_filtered_;
    Eigen::Matrix<double, 4, 4> cur_pose_ = Eigen::Matrix4d::Zero();

    void LoadConfig(const std::string& config);
    void InitCostmap(Eigen::MatrixXd& costmap, int size_x, int size_y);
    void UpdateCostmapFlag();
    bool IsInBounds(int x, int y) const;
    void UpdateCellsOnBresenhamLine(int sx, int sy, int ex, int ey);
    void UpdateCell(int x, int y, double p);
    double ProbabilityToLogOdds(double p) const;
    double LogOddsToProbability(double l) const;
};

#endif
