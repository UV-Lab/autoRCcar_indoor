#include "costmap.h"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>

Costmap::Costmap(const std::string& config_file_path) : costmap_flag(false), cnt_iter(0) {
    LoadConfig(config_file_path);

    // Initialize global costmap
    global_size_x = (int)global_width / resolution;
    global_size_y = (int)global_height / resolution;
    global_center_x = global_size_x / 2;
    global_center_y = global_size_y / 2;
    InitCostmap(global_costmap, global_size_x, global_size_y);

    // Initialize local costmap
    local_size_x = (int)local_width / resolution;
    local_size_y = (int)local_height / resolution;
    local_center_x = local_size_x / 2;
    local_center_y = local_size_y / 2;
    InitCostmap(local_costmap, local_size_x, local_size_y);

    // Allocate memory
    cur_point_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    cur_point_cloud_transformed.reset(new pcl::PointCloud<pcl::PointXYZI>());
    cur_point_cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZI>());
}

void Costmap::LoadConfig(const std::string& config) {
    std::ifstream fin(config);
    if (!fin) {
        std::cerr << "Failed to open file: " << config << std::endl;
        exit(-1);
    }
    fin.close();

    YAML::Node cfg = YAML::LoadFile(config);

    resolution = cfg["global"]["resolution"].as<double>();
    cnt_limit = cfg["global"]["updateEveryNthLidar"].as<unsigned int>();

    global_width = cfg["global"]["width"].as<int>();
    global_height = cfg["global"]["height"].as<int>();

    local_width = cfg["local"]["width"].as<int>();
    local_height = cfg["local"]["height"].as<int>();

    std::cout << "Successfully loaded the config" << std::endl;
}

void Costmap::InitCostmap(Eigen::MatrixXd& costmap, int size_x, int size_y) {
    costmap.resize(size_x, size_y);
    costmap.setConstant(ProbabilityToLogOdds(P_UNKNOWN));
}

void Costmap::UpdatePointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr pcl) {
    cur_point_cloud->clear();
    cur_point_cloud_transformed->clear();
    cur_point_cloud_filtered->clear();

    pcl::copyPointCloud(*pcl, *cur_point_cloud);

    UpdateCostmapFlag();
}

void Costmap::UpdatePose(Eigen::Matrix<double, 4, 4>& trans) { cur_pose = trans; }

void Costmap::UpdateCostmapFlag() {
    cnt_iter += 1;

    if (cnt_iter >= cnt_limit) {
        costmap_flag = true;
        cnt_iter = 0;
    }
}

void Costmap::UpdateCostmap() {
    // Get cell indices of current pose. This will be used as the start point of the bresenham line.
    int sx = floor(cur_pose(0, 3) / resolution) + global_center_x;  // assume (global_center_x, global_center_y) = (0,0)
    int sy = floor(cur_pose(1, 3) / resolution) + global_center_y;

    // Transform point cloud from body to world frame.
    pcl::transformPointCloud(*cur_point_cloud, *cur_point_cloud_transformed, cur_pose);

    // Filtering
    pcl::PassThrough<pcl::PointXYZI> zFilter;
    zFilter.setInputCloud(cur_point_cloud_transformed);
    zFilter.setFilterFieldName("z");
    zFilter.setFilterLimits(0.0, 1.0);
    zFilter.filter(*cur_point_cloud_filtered);

    for (const auto& pt : *cur_point_cloud_filtered) {
        // Get cell indices of the pointcloud
        int x = floor(pt.x / resolution) + global_center_x;
        int y = floor(pt.y / resolution) + global_center_y;

        // The cell where the lidar point located is occupied.
        UpdateCell(x, y, P_OCCUPIED);

        // The cells on the bresenham line are free.
        BresenhamLine(sx, sy, x, y);
    }

    costmap_flag = false;
}

void Costmap::BresenhamLine(int sx, int sy, int ex, int ey) {
    int dx = std::abs(ex - sx);
    int dy = std::abs(ey - sy);

    // discriminant
    int d;

    // current point
    int x = sx;
    int y = sy;

    std::vector<std::pair<int, int>> line;

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
            line.push_back(std::make_pair(x, y));

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
            line.push_back(std::make_pair(x, y));

            if (ey > sy) {
                y = y + 1;
            } else {
                y = y - 1;
            }
        }
    }

    // Set costmap.
    for (unsigned int i = 0; i < line.size(); i++) {
        UpdateCell(line[i].first, line[i].second, P_FREE);
    }
}

bool Costmap::IsInBounds(int x, int y) {
    if (!(x >= 0 && x < global_size_x)) return false;
    if (!(y >= 0 && y < global_size_y)) return false;
    return true;
}

void Costmap::UpdateCell(int x, int y, double p) {
    if (IsInBounds(x, y)) {
        global_costmap(x, y) = global_costmap(x, y) + ProbabilityToLogOdds(p) - ProbabilityToLogOdds(P_UNKNOWN);
    }
}

double Costmap::ProbabilityToLogOdds(double p) { return std::log(p / (1.0 - p)); }

double Costmap::LogOddsToProbability(double l) { return std::exp(l) / (1.0 + std::exp(l)); }

struct CostmapInfo Costmap::GetGlobalCostmapInfo() {
    struct CostmapInfo info;

    info.size_x = static_cast<unsigned int>(global_size_x);
    info.size_y = static_cast<unsigned int>(global_size_y);
    info.resolution = resolution;
    info.origin_pos_x = global_center_x * resolution * (-1);  // assume (global_center_x, global_center_y) = (0,0)
    info.origin_pos_y = global_center_y * resolution * (-1);
    info.costmap = &global_costmap;

    return info;
}

struct CostmapInfo Costmap::GetLocalCostmapInfo() {
    struct CostmapInfo info;

    // initialize local costmap.
    local_costmap.setConstant(ProbabilityToLogOdds(P_UNKNOWN));

    // Get cell indices of current pose.
    int px = floor(cur_pose(0, 3) / resolution) + global_center_x;
    int py = floor(cur_pose(1, 3) / resolution) + global_center_y;

    // Get local costmap from global costmap.
    // Local_costmap is a rectangle with center at (px, py) and width local_size.
    if (IsInBounds(px - local_center_x, py - local_center_y) && IsInBounds(px + local_center_x, py + local_center_y)) {
        local_costmap = global_costmap.block(px - local_center_x, py - local_center_y, local_size_x, local_size_y);
    }

    info.size_x = static_cast<unsigned int>(local_size_x);
    info.size_y = static_cast<unsigned int>(local_size_y);
    info.resolution = resolution;
    info.origin_pos_x = (px - global_center_x - local_center_x) * resolution;
    info.origin_pos_y = (py - global_center_y - local_center_y) * resolution;
    info.costmap = &local_costmap;

    return info;
}
