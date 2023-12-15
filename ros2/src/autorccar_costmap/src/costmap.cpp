#include "costmap.h"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>

Costmap::Costmap(const std::string& config_file_path) : pcl_flag(false), pose_flag(false) {
    LoadConfig(config_file_path);

    size_x = (unsigned int)width / resolution;
    size_y = (unsigned int)height / resolution;
    center_x = size_x / 2;
    center_y = size_y / 2;
    InitCostmap(size_x, size_y);

    // allocate memory
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

    width = cfg["width"].as<unsigned int>();
    height = cfg["height"].as<unsigned int>();
    resolution = cfg["resolution"].as<double>();

    std::cout << "Successfully loaded the config" << std::endl;
}

void Costmap::InitCostmap(unsigned int size_x, unsigned int size_y) {
    costmap.resize(size_x, size_y);
    costmap.setConstant(ProbabilityToLogOdds(P_UNKNOWN));

    std::cout << "Successfully initialized the costmap" << std::endl;
}

void Costmap::UpdatePointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr pcl) {
    cur_point_cloud->clear();
    cur_point_cloud_transformed->clear();
    cur_point_cloud_filtered->clear();

    pcl::copyPointCloud(*pcl, *cur_point_cloud);
    pcl_flag = true;
}

void Costmap::UpdatePose(Eigen::Matrix<double, 4, 4>& trans) {
    cur_pose = trans;
    pose_flag = true;
}

void Costmap::UpdateCostmap() {
    // Get cell indices of current pose. This will be used as the start point of the bresenham line.
    int sx = floor(cur_pose(0, 3) / resolution) + center_x;
    int sy = floor(cur_pose(1, 3) / resolution) + center_y;

    // curPointCloudNew (body -> global frame)
    pcl::transformPointCloud(*cur_point_cloud, *cur_point_cloud_transformed, cur_pose);

    // filtering
    pcl::PassThrough<pcl::PointXYZI> zFilter;
    zFilter.setInputCloud(cur_point_cloud_transformed);
    zFilter.setFilterFieldName("z");
    zFilter.setFilterLimits(0.0, 1.0);
    zFilter.filter(*cur_point_cloud_filtered);

    for (const auto& pt : *cur_point_cloud_filtered) {
        // Get cell indices of the pointcloud
        int x = floor(pt.x / resolution) + center_x;
        int y = floor(pt.y / resolution) + center_y;

        // The cell where the lidar point located is occupied.
        UpdateCell(x, y, P_OCCUPIED);

        // The cells on the bresenham line are free.
        BresenhamLine(sx, sy, x, y);
    }

    // set pcl_flag
    pcl_flag = false;
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

    // set costmap
    for (unsigned int i = 0; i < line.size(); i++) {
        UpdateCell(line[i].first, line[i].second, P_FREE);
    }
}

bool Costmap::IsInBounds(int x, int y) {
    if (!(x >= 0 && static_cast<unsigned int>(x) < size_x)) return false;
    if (!(y >= 0 && static_cast<unsigned int>(y) < size_y)) return false;
    return true;
}

void Costmap::UpdateCell(int x, int y, double p) {
    if (IsInBounds(x, y)) {
        costmap(x, y) = costmap(x, y) + ProbabilityToLogOdds(p) - ProbabilityToLogOdds(P_UNKNOWN);
    }
}

double Costmap::ProbabilityToLogOdds(double p) { return std::log(p / (1.0 - p)); }

double Costmap::LogOddsToProbability(double l) { return std::exp(l) / (1.0 + std::exp(l)); }

struct CostmapInfo Costmap::GetCostmapInfo() {
    struct CostmapInfo info;

    info.size_x = size_x;
    info.size_y = size_y;
    info.resolution = resolution;
    info.origin_pos_x = static_cast<int>(center_x) * resolution * (-1);
    info.origin_pos_y = static_cast<int>(center_y) * resolution * (-1);
    info.costmap = &costmap;

    return info;
}
