#include "GroundSegment.h"

#include <algorithm>
#include <numeric>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>

using namespace grid_map;

GroundSegment::GroundSegment(GroundSegmentParams::SharedPtr params_ptr)
    : m_params_ptr(params_ptr) {}
GroundSegment::~GroundSegment() {}
int GroundSegment::init() {
    m_non_ground_cloud_ptr.reset(new CloudT());
    m_ground_cloud_ptr.reset(new CloudT());

    return 0;
}

int GroundSegment::groundSegment(CloudT::Ptr src_cloud_ptr) {
    if (nullptr == src_cloud_ptr || true == src_cloud_ptr->empty()) {
        return -1;
    }

    CloudT::Ptr aux_cloud_ptr(new CloudT());
    pcl::copyPointCloud(*src_cloud_ptr, *aux_cloud_ptr);

    // sort the point cloud by ascend order.
    // std::sort(cur_cloud_ptr->points.begin(), cur_cloud_ptr->points.end(),
    //           [](const PointT &a, const PointT &b) { return a.z < b.z; });

    preprocessCloud(aux_cloud_ptr);
    auto seed_cloud_ptr = extractInitSeeds(aux_cloud_ptr);
    if (nullptr == seed_cloud_ptr) {
        std::cout << "The seed cloud is empty!\n";
        return -2;
    }

    m_ground_cloud_ptr = seed_cloud_ptr;
    bool is_use_height = false;
    std::vector<bool> is_ground_idx_vec(aux_cloud_ptr->points.size(), false);
    for (int it_idx = 0, it_cnt = m_params_ptr->num_iter; it_idx < it_cnt;
         ++it_idx) {
        auto rc = estimatePlane();
        m_ground_cloud_ptr->clear();
        m_non_ground_cloud_ptr->clear();
        if (0 != rc) {
            is_use_height = true;
            break;
        }
        for (const auto &pnt : aux_cloud_ptr->points) {
            auto val = m_plane_abcd[0] * pnt.x + m_plane_abcd[1] * pnt.y +
                       m_plane_abcd[2] * pnt.z + m_plane_abcd[3];
            if (0 < val) {
                m_non_ground_cloud_ptr->points.push_back(pnt);
            } else {
                m_ground_cloud_ptr->points.push_back(pnt);
            }
        }
    }

    if (true == is_use_height) {
        std::cout << "use the height directly.\n";
        for (const auto &pnt : aux_cloud_ptr->points) {
            if (pnt.z < m_params_ptr->seg_height_th) {
                m_ground_cloud_ptr->points.push_back(pnt);
            } else {
                m_non_ground_cloud_ptr->points.push_back(pnt);
            }
        }
    } else {
        for (const auto &pnt : src_cloud_ptr->points) {
            auto val = m_plane_abcd[0] * pnt.x + m_plane_abcd[1] * pnt.y +
                       m_plane_abcd[2] * pnt.z + m_plane_abcd[3];
            if (0 < val) {
                m_non_ground_cloud_ptr->points.push_back(pnt);
            } else {
                m_ground_cloud_ptr->points.push_back(pnt);
            }
        }
    }

    return 0;
}

int GroundSegment::groundSegment(CloudT::Ptr src_cloud_ptr,
                                 CloudT::Ptr auxiliary_src_cloud_ptr) {

    if (nullptr == src_cloud_ptr || true == src_cloud_ptr->empty() ||
        nullptr == auxiliary_src_cloud_ptr ||
        true == auxiliary_src_cloud_ptr->empty()) {
        return -1;
    }

    CloudT::Ptr aux_cloud_ptr(new CloudT());
    pcl::copyPointCloud(*auxiliary_src_cloud_ptr, *aux_cloud_ptr);

    // sort the point cloud by ascend order.
    // std::sort(cur_cloud_ptr->points.begin(), cur_cloud_ptr->points.end(),
    //           [](const PointT &a, const PointT &b) { return a.z < b.z; });

    // extract the points near the ground plane by min_height and max_height.
    preprocessCloud(aux_cloud_ptr);
    auto seed_cloud_ptr = extractInitSeeds(aux_cloud_ptr);
    if (nullptr == seed_cloud_ptr) {
        std::cout << "The seed cloud is empty!\n";
        return -2;
    }

    m_ground_cloud_ptr = seed_cloud_ptr;
    bool is_use_height = false;
    std::vector<bool> is_ground_idx_vec(aux_cloud_ptr->points.size(), false);

    

    for (int it_idx = 0, it_cnt = m_params_ptr->num_iter; it_idx < it_cnt;
         ++it_idx) {
        auto rc = estimatePlane();
        
        m_ground_cloud_ptr.reset(new CloudT());
        m_non_ground_cloud_ptr.reset(new CloudT());
        std::vector<int> non_ground_point_cloud_indices;
        std::vector<int> ground_point_cloud_indices;
        if (0 != rc) {
            is_use_height = true;
            break;
        }
        for (size_t i = 0; i < aux_cloud_ptr->points.size(); ++i) {
            const auto &pnt = aux_cloud_ptr->points[i];
            auto val = m_plane_abcd[0] * pnt.x + m_plane_abcd[1] * pnt.y +
                       m_plane_abcd[2] * pnt.z + m_plane_abcd[3];
            if (0 < val) {
                //m_non_ground_cloud_ptr->points.push_back(pnt);
                non_ground_point_cloud_indices.push_back(static_cast<int>(i));
            } else {
                ground_point_cloud_indices.push_back(static_cast<int>(i));
            }
        }
        pcl::copyPointCloud(*aux_cloud_ptr, non_ground_point_cloud_indices,
                            *m_non_ground_cloud_ptr);
        pcl::copyPointCloud(*aux_cloud_ptr, ground_point_cloud_indices,
                            *m_ground_cloud_ptr);
    }
    std::cout << "is_use_height: " << is_use_height << std::endl;
    //m_ground_cloud_ptr->clear();
    //m_non_ground_cloud_ptr->clear();

    m_ground_cloud_ptr.reset(new CloudT());
    m_non_ground_cloud_ptr.reset(new CloudT());
    std::vector<int> non_ground_point_cloud_indices;
    std::vector<int> ground_point_cloud_indices;
    if (false == is_use_height) {
        for (size_t i = 0; i < src_cloud_ptr->points.size(); ++i) {
            const auto &pnt = src_cloud_ptr->points[i];
            auto val = m_plane_abcd[0] * pnt.x + m_plane_abcd[1] * pnt.y +
                       m_plane_abcd[2] * pnt.z + m_plane_abcd[3];
            if (0 < val) {
                //m_non_ground_cloud_ptr->points.push_back(pnt);
                non_ground_point_cloud_indices.push_back(static_cast<int>(i));
            } else {
                //m_ground_cloud_ptr->points.push_back(pnt);
                ground_point_cloud_indices.push_back(static_cast<int>(i));
            }
        }
        pcl::copyPointCloud(*src_cloud_ptr, non_ground_point_cloud_indices,
                            *m_non_ground_cloud_ptr);
        pcl::copyPointCloud(*src_cloud_ptr, ground_point_cloud_indices,
                            *m_ground_cloud_ptr);

    } else {
        std::cout << "use the height directly.\n";
        for (size_t i = 0; i < src_cloud_ptr->points.size(); ++i) {
            const auto &pnt = src_cloud_ptr->points[i];
            if (pnt.z < m_params_ptr->seg_height_th) {
                //m_ground_cloud_ptr->points.push_back(pnt);
                ground_point_cloud_indices.push_back(static_cast<int>(i));
            } else {
                //m_non_ground_cloud_ptr->points.push_back(pnt);
                non_ground_point_cloud_indices.push_back(static_cast<int>(i));
            }
        }
        pcl::copyPointCloud(*src_cloud_ptr, non_ground_point_cloud_indices,
                            *m_non_ground_cloud_ptr);
        pcl::copyPointCloud(*src_cloud_ptr, ground_point_cloud_indices,
                            *m_ground_cloud_ptr);

    }

    return 0;
}

CloudT::Ptr GroundSegment::getNonGroundCloud() {
    return m_non_ground_cloud_ptr;
}
CloudT::Ptr GroundSegment::getGroundCloud() { return m_ground_cloud_ptr; }

// CloudT::Ptr GroundSegment::getGSCloud() { return nullptr; }

int GroundSegment::preprocessCloud(CloudT::Ptr cloud_ptr) {
    // sort the point cloud by ascend order.
    std::sort(cloud_ptr->points.begin(), cloud_ptr->points.end(),
              [](const PointT &a, const PointT &b) { return a.z < b.z; });

    // trim the point cloud by z.
    auto &pnts = cloud_ptr->points;
    auto it_lower = std::lower_bound(
            pnts.begin(), pnts.end(), m_params_ptr->min_height,
            [](const PointT &pnt, double value) -> bool { return pnt.z < value; });
    if (pnts.end() != it_lower) {
        pnts.erase(pnts.begin(), it_lower);
    }
    auto it_upper = std::upper_bound(
            pnts.begin(), pnts.end(), m_params_ptr->max_height,
            [](double value, const PointT &pnt) -> bool { return value < pnt.z; });
    if (pnts.end() != it_upper) {
        pnts.erase(it_upper, pnts.end());
    }

    return 0;
}
CloudT::Ptr GroundSegment::extractInitSeeds(CloudT::Ptr sorted_cloud_ptr) {
    float z_sum = 0.f;
    size_t pnt_cnt = m_params_ptr->num_lpr;
    if (sorted_cloud_ptr->points.size() < pnt_cnt) {
        pnt_cnt = sorted_cloud_ptr->points.size();
    }
    for (size_t pnt_idx = 0; pnt_idx != pnt_cnt; ++pnt_idx) {
        z_sum += sorted_cloud_ptr->points[pnt_idx].z;
        // ++pnt_cnt;
    }
    float lpr_height = z_sum / float(pnt_cnt);

    CloudT::Ptr seeds_ptr(new CloudT());
    // auto it_upper = std::upper_bound(
    //     sorted_cloud_ptr->points.begin(), sorted_cloud_ptr->points.end(),
    //     lpr_height + m_params_ptr->seed_th,
    //     [](double value, const PointT &pnt) -> bool { return value < pnt.z; });
    // if (sorted_cloud_ptr->points.end() != it_upper) {
    //   seeds_ptr->points.assign(sorted_cloud_ptr->points.begin(), it_upper);
    // }
    float seed_z_max = lpr_height + m_params_ptr->seed_th;
    for (const auto &ele_pnt : sorted_cloud_ptr->points) {
        if (ele_pnt.z < seed_z_max) {
            seeds_ptr->push_back(ele_pnt);
        } else {
            break;
        }
    }

    return seeds_ptr;
}
int GroundSegment::estimatePlane() {
    // Create covarian matrix in single pass.
    // TODO: compare the efficiency.
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*m_ground_cloud_ptr, cov, pc_mean);
    // Singular Value Decomposition: SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(
            cov, Eigen::DecompositionOptions::ComputeFullU);
    // use the least singular vector as normal
    Eigen::Vector3f plane_n = (svd.matrixU().col(2));
    plane_n.normalize();
    // Judge the angle between p_normal and the z axis.
    Eigen::Vector3f z_axis(0, 0, 1);
    float dot_val = plane_n.dot(z_axis);
    if (dot_val < 0) {
        plane_n = -plane_n;
        dot_val *= -1.0;
    }
    float angle_rad = std::acos(dot_val);
    if (angle_rad > m_params_ptr->slope_angle_rad_th) {
        m_plane_abcd = {0, 0, 1, float(m_params_ptr->seg_height_th)};
        return 1;
    }

    m_plane_abcd = {plane_n(0), plane_n(1), plane_n(2), 0.f};
    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();
    // according to normal.T*[x,y,z] = -d
    float plane_d = -(plane_n.transpose() * seeds_mean)(0, 0);
    float seg_plane_d = plane_d - m_params_ptr->dist_th;
    m_plane_abcd[3] = seg_plane_d;

    return 0;
}
