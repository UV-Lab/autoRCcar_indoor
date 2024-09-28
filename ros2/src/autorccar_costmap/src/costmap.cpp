#include "costmap.h"

#include <fstream>
#include <iostream>

Costmap::Costmap(std::shared_ptr<ConfigReader> config_reader)
    : costmap_flag_(false), config_reader_(config_reader), cnt_iter_(0), cnt_limit_(0) {
    // Get Yaml configs
    Config config = config_reader_->GetConfig();
    resolution_ = config.global_resolution;
    cnt_limit_ = config.global_update_per_lidar;
    global_width_ = config.global_width;
    global_height_ = config.global_height;
    local_width_ = config.local_width;
    local_height_ = config.local_height;
    dbscan_eps_ = config.dbscan_eps;
    dbscan_min_samples_ = config.dbscan_min_samples;

    // Initialize global costmap
    global_size_ = {static_cast<int>(global_width_ / resolution_), static_cast<int>(global_height_ / resolution_)};
    global_center_ = {global_size_.x() / 2, global_size_.y() / 2};
    InitCostmap(global_costmap_, global_size_.x(), global_size_.y());

    // Initialize global costmap info
    global_costmap_info_.size_x = static_cast<unsigned int>(global_size_.x());
    global_costmap_info_.size_y = static_cast<unsigned int>(global_size_.y());
    global_costmap_info_.resolution = resolution_;
    global_costmap_info_.origin_pos_x =
        global_center_.x() * resolution_ * (-1);  // assume (global_center_x, global_center_y) = (0,0)
    global_costmap_info_.origin_pos_y = global_center_.y() * resolution_ * (-1);
    global_costmap_info_.ptr_costmap = &global_costmap_;

    // Initialize local costmap
    local_size_ = {static_cast<int>(local_width_ / resolution_), static_cast<int>(local_height_ / resolution_)};
    local_center_ = {local_size_.x() / 2, local_size_.y() / 2};
    InitCostmap(local_costmap_, local_size_.x(), local_size_.y());

    // Initialize local costmap info
    const int px = std::floor(cur_pose_(0, 3) / resolution_) + global_center_.x();
    const int py = std::floor(cur_pose_(1, 3) / resolution_) + global_center_.y();
    local_costmap_info_.size_x = static_cast<unsigned int>(local_size_.x());
    local_costmap_info_.size_y = static_cast<unsigned int>(local_size_.y());
    local_costmap_info_.resolution = resolution_;
    local_costmap_info_.origin_pos_x = (px - global_center_.x() - local_center_.x()) * resolution_;
    local_costmap_info_.origin_pos_y = (py - global_center_.y() - local_center_.y()) * resolution_;
    local_costmap_info_.ptr_costmap = &local_costmap_;

    // Allocate memory
    cur_point_cloud_.reset(new pcl::PointCloud<PointType>());
    cur_point_cloud_transformed_.reset(new pcl::PointCloud<PointType>());
    cur_point_cloud_filtered_.reset(new pcl::PointCloud<PointType>());
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

    // Update Global Costmap
    for (const auto& pt : *cur_point_cloud_filtered_) {
        // Get cell indices of the pointcloud
        int x = std::floor(pt.x / resolution_) + global_center_.x();
        int y = std::floor(pt.y / resolution_) + global_center_.y();

        // The cell where the lidar point located is occupied.
        UpdateCell(x, y, P_OCCUPIED);

        // The cells on the bresenham line are free.
        UpdateCellsOnBresenhamLine(sx, sy, x, y);
    }

    UpdateLocalCostmap();
    CalculateBoundingBoxes();

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
    line.reserve(global_size_.x() + global_size_.y());

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

    // Set global costmap.
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

void Costmap::UpdateLocalCostmap() {
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

    local_costmap_info_.origin_pos_x = (px - global_center_.x() - local_center_.x()) * resolution_;
    local_costmap_info_.origin_pos_y = (py - global_center_.y() - local_center_.y()) * resolution_;
}

void Costmap::CalculateBoundingBoxes() {
    bounding_boxes_.clear();

    // local costmap에서 클러스터링 할 포인트 추출
    pcl::PointCloud<PointType>::Ptr cluster_points(new pcl::PointCloud<PointType>);

    for (int i = 0; i < local_costmap_.rows(); ++i) {
        for (int j = 0; j < local_costmap_.cols(); ++j) {
            if (local_costmap_(i, j) >= P_OCCUPIED) {
                pcl::PointXYZI point;
                point.x = local_costmap_info_.origin_pos_x + resolution_ * i;
                point.y = local_costmap_info_.origin_pos_y + resolution_ * j;
                point.z = 0;
                cluster_points->points.push_back(point);
            }
        }
    }

    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> dbscan;

    if (cluster_points->size() > 0) {
        // KdTree 객체 생성
        tree->setInputCloud(cluster_points);

        // DBSCAN
        dbscan.setClusterTolerance(dbscan_eps_);
        dbscan.setMinClusterSize(dbscan_min_samples_);
        dbscan.setSearchMethod(tree);
        dbscan.setInputCloud(cluster_points);
        dbscan.extract(cluster_indices);
    }

    // 클러스터링 결과로부터 Bounding Box 추출
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end();
         ++it) {
        // 각 클러스터의 포인트 추가
        pcl::PointCloud<PointType>::Ptr cluster(new pcl::PointCloud<PointType>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cluster->points.push_back(cluster_points->points[*pit]);

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        // Min, Max 포인트 추출
        PointType minPt, maxPt;
        pcl::getMinMax3D(*cluster, minPt, maxPt);

        // Bounding Box 계산
        BoundingBox box;
        box.size_x = maxPt.x - minPt.x;
        box.size_y = maxPt.y - minPt.y;
        box.center_x = (minPt.x + maxPt.x) / 2.0;
        box.center_y = (minPt.y + maxPt.y) / 2.0;
        bounding_boxes_.push_back(box);
    }
}

struct CostmapInfo Costmap::GetGlobalCostmapInfo() { return global_costmap_info_; }

struct CostmapInfo Costmap::GetLocalCostmapInfo() { return local_costmap_info_; }

const BoundingBoxArr& Costmap::GetBoundingBoxes() { return bounding_boxes_; }

std::shared_ptr<ConfigReader> Costmap::GetConfigReaderPtr() { return config_reader_; }
