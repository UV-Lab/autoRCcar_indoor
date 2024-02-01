#include "costmap.h"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>

Costmap::Costmap(const std::string& config_file_path) : costmap_flag_(false), cnt_iter_(0) {
    LoadConfig(config_file_path);

    // Initialize global costmap
    global_size_ = {static_cast<int>(global_width_ / resolution_), static_cast<int>(global_height_ / resolution_)};
    global_center_ = {global_size_.x() / 2, global_size_.y() / 2};
    InitCostmap(global_costmap_, global_size_.x(), global_size_.y());

    // Initialize local costmap
    local_size_ = {static_cast<int>(local_width_ / resolution_), static_cast<int>(local_height_ / resolution_)};
    local_center_ = {local_size_.x() / 2, local_size_.y() / 2};
    InitCostmap(local_costmap_, local_size_.x(), local_size_.y());

    // Allocate memory
    cur_point_cloud_.reset(new pcl::PointCloud<PointType>());
    cur_point_cloud_transformed_.reset(new pcl::PointCloud<PointType>());
    cur_point_cloud_filtered_.reset(new pcl::PointCloud<PointType>());
}

void Costmap::LoadConfig(const std::string& config) {
    std::ifstream fin(config);
    if (!fin) {
        std::cerr << "Failed to open file: " << config << std::endl;
        exit(-1);
    }
    fin.close();

    YAML::Node cfg = YAML::LoadFile(config);

    resolution_ = cfg["global"]["resolution"].as<double>();
    cnt_limit_ = cfg["global"]["updateEveryNthLidar"].as<unsigned int>();

    global_width_ = cfg["global"]["width"].as<int>();
    global_height_ = cfg["global"]["height"].as<int>();

    local_width_ = cfg["local"]["width"].as<int>();
    local_height_ = cfg["local"]["height"].as<int>();

    std::cout << "Successfully loaded the config" << std::endl;
}

void Costmap::InitCostmap(Eigen::MatrixXd& costmap, int size_x, int size_y) {
    costmap.resize(size_x, size_y);
    costmap.setConstant(ProbabilityToLogOdds(P_UNKNOWN));
}

void Costmap::UpdatePointCloud(const pcl::PointCloud<PointType>::Ptr pcl) {
    cur_point_cloud_->clear();
    cur_point_cloud_transformed_->clear();
    cur_point_cloud_filtered_->clear();

    pcl::copyPointCloud(*pcl, *cur_point_cloud_);

    UpdateCostmapFlag();
}

void Costmap::UpdatePose(Eigen::Matrix<double, 4, 4>& trans) { cur_pose_ = trans; }

void Costmap::UpdateCostmapFlag() {
    if (cnt_iter_++ >= cnt_limit_) {
        costmap_flag_ = true;
        cnt_iter_ = 0;
    }
}

void Costmap::UpdateCostmap() {
    // Get cell indices of current pose. This will be used as the start point of the bresenham line.
    // assume (global_center_x, global_center_y) = (0,0)
    const int sx = std::floor(cur_pose_(0, 3) / resolution_) + global_center_.x();
    const int sy = std::floor(cur_pose_(1, 3) / resolution_) + global_center_.y();

    // Transform point cloud from body to world frame.
    pcl::transformPointCloud(*cur_point_cloud_, *cur_point_cloud_transformed_, cur_pose_);

    // Filtering
    pcl::PassThrough<PointType> zFilter;
    zFilter.setInputCloud(cur_point_cloud_transformed_);
    zFilter.setFilterFieldName("z");
    zFilter.setFilterLimits(0.0, 1.0);
    zFilter.filter(*cur_point_cloud_filtered_);

    for (const auto& pt : *cur_point_cloud_filtered_) {
        // Get cell indices of the pointcloud
        int x = std::floor(pt.x / resolution_) + global_center_.x();
        int y = std::floor(pt.y / resolution_) + global_center_.y();

        // The cell where the lidar point located is occupied.
        UpdateCell(x, y, P_OCCUPIED);

        // The cells on the bresenham line are free.
        UpdateCellsOnBresenhamLine(sx, sy, x, y);
    }

    costmap_flag_ = false;
}

void Costmap::UpdateCellsOnBresenhamLine(int sx, int sy, int ex, int ey) {
    const int dx = std::abs(ex - sx);
    const int dy = std::abs(ey - sy);

    // discriminant
    int d;

    // current point
    int x = sx;
    int y = sy;

    std::vector<Eigen::Vector2i> line;

    if (dx > dy) {
        // when 0 < |slope| < 1
        d = 2 * dy - dx;
        for (int i = 0; i < dx; i++) {
            if (d < 0) {
                d = d + 2 * dy;
            } else {
                if (ey > sy) {
                    y = y + 1;
                } else {
                    y = y - 1;
                }
                d = d + 2 * (dy - dx);
            }
            line.push_back({x, y});

            if (ex > sx) {
                x = x + 1;
            } else {
                x = x - 1;
            }
        }
    } else {
        // when |slope| > 1
        d = 2 * dx - dy;
        for (int i = 0; i < dy; i++) {
            if (d < 0) {
                d = d + 2 * dx;
            } else {
                if (ex > sx) {
                    x = x + 1;
                } else {
                    x = x - 1;
                }
                d = d + 2 * (dx - dy);
            }
            line.push_back({x, y});

            if (ey > sy) {
                y = y + 1;
            } else {
                y = y - 1;
            }
        }
    }

    // Set costmap.
    for (unsigned int i = 0; i < line.size(); i++) {
        UpdateCell(line[i].x(), line[i].y(), P_FREE);
    }
}

bool Costmap::IsInBounds(int x, int y) const {
    if (!(x >= 0 && x < global_size_.x())) return false;
    if (!(y >= 0 && y < global_size_.y())) return false;
    return true;
}

void Costmap::UpdateCell(int x, int y, double p) {
    if (IsInBounds(x, y)) {
        global_costmap_(x, y) = global_costmap_(x, y) + ProbabilityToLogOdds(p) - ProbabilityToLogOdds(P_UNKNOWN);
    }
}

double Costmap::ProbabilityToLogOdds(double p) const { return std::log(p / (1.0 - p)); }

double Costmap::LogOddsToProbability(double l) const { return std::exp(l) / (1.0 + std::exp(l)); }

struct CostmapInfo Costmap::GetGlobalCostmapInfo() {
    struct CostmapInfo info;

    info.size_x = static_cast<unsigned int>(global_size_.x());
    info.size_y = static_cast<unsigned int>(global_size_.y());
    info.resolution = resolution_;
    info.origin_pos_x = global_center_.x() * resolution_ * (-1);  // assume (global_center_x, global_center_y) = (0,0)
    info.origin_pos_y = global_center_.y() * resolution_ * (-1);
    info.costmap = &global_costmap_;

    return info;
}

struct CostmapInfo Costmap::GetLocalCostmapInfo() {
    struct CostmapInfo info;

    // initialize local costmap.
    local_costmap_.setConstant(ProbabilityToLogOdds(P_UNKNOWN));

    // Get cell indices of current pose.
    const int px = std::floor(cur_pose_(0, 3) / resolution_) + global_center_.x();
    const int py = std::floor(cur_pose_(1, 3) / resolution_) + global_center_.y();

    // Get local costmap from global costmap.
    // Local_costmap is a rectangle with center at (px, py) and width local_size.
    if (IsInBounds(px - local_center_.x(), py - local_center_.y()) &&
        IsInBounds(px + local_center_.x(), py + local_center_.y())) {
        local_costmap_ =
            global_costmap_.block(px - local_center_.x(), py - local_center_.y(), local_size_.x(), local_size_.y());
    }

    info.size_x = static_cast<unsigned int>(local_size_.x());
    info.size_y = static_cast<unsigned int>(local_size_.y());
    info.resolution = resolution_;
    info.origin_pos_x = (px - global_center_.x() - local_center_.x()) * resolution_;
    info.origin_pos_y = (py - global_center_.y() - local_center_.y()) * resolution_;
    info.costmap = &local_costmap_;

    return info;
}
