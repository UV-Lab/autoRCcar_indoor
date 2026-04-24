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


int GroundSegment::groundSegment(CloudT::Ptr src_cloud_ptr,
                                 CloudT::Ptr auxiliary_src_cloud_ptr) {

    if (nullptr == src_cloud_ptr || true == src_cloud_ptr->empty() ||
        nullptr == auxiliary_src_cloud_ptr ||
        true == auxiliary_src_cloud_ptr->empty()) {
        return -1;
    }

     auto min_z_point = std::min_element(auxiliary_src_cloud_ptr->points.begin(), auxiliary_src_cloud_ptr->points.end(),
                         [](const PointT &a, const PointT &b) { return a.z < b.z; });
        std::cout << "min_z_point:" << min_z_point->x << "," << min_z_point->y << "," << min_z_point->z << std::endl;
        auto max_z_point = std::max_element(auxiliary_src_cloud_ptr->points.begin(), auxiliary_src_cloud_ptr->points.end(),
                         [](const PointT &a, const PointT &b) { return a.z < b.z; });
        std::cout << "max_z_point:" << max_z_point->x << "," << max_z_point->y << "," << max_z_point->z << std::endl;


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

    std::cout << "fist: m_ground_cloud_ptr.size:" << m_ground_cloud_ptr->points.size() << std::endl;
    bool is_use_height = false;
    //std::vector<bool> is_ground_idx_vec(aux_cloud_ptr->points.size(), false);

   
//     在这段代码中，如果在执行 114行 extractInit的时候，会从数组的最下方取200个点（第264行），作为样本，然后再向上扩展0.1米的高度（第287行）；
// 但有个问题，假如在地面下有一些噪声点，导致由噪声点推断出的平面方程（第137-156行）偏低 z轴偏低，不能正确反应地面点云位置；
// 所以，在重新划分src_cloud_ptr的点的判断，全部都在平面上访（第177-199行）；

// 我准备做两处改动来直接抑制这个问题：
// 一是在种子提取时忽略最低一小段离群点并加下界，
// 二是在迭代划分时加“全是非地面”的保护，触发回退高度法，
// 避免整帧误判。接下来我会只改 GroundSegment.cpp。

// 你这个判断是对的：当前算法把最低一批点直接当成初始地面种子，地下噪声会把 LPR 均值和后续平面整体拉低，最后出现“几乎所有点都在平面上方”的误分割。

// 我已经在代码里做了两层修复，并落地到 GroundSegment.cpp：

// 种子提取改为鲁棒版本
// 在 GroundSegment.cpp:270：

// 增加空点云保护，避免异常输入。

// 计算 LPR 高度时，不再用最底部全部点，而是丢弃最低 10% 后再求均值（抑制地下离群点影响），见 GroundSegment.cpp:281。

// 选种子时增加下界和上界区间，不再无条件从最底部往上吃点，见 GroundSegment.cpp:308。

// 迭代分割增加“平面过低”保护
// 在主循环中（GroundSegment.cpp:141）新增判据：

// 如果地面点数量过少（<10）或地面占比 <1%，认为平面明显偏低。

// 立即触发回退到高度分割（is_use_height = true），避免后续整帧被判成非地面。

// 这两层结合后，能显著降低“地下噪声导致整帧全非地面”的概率。


    for (int it_idx = 0, it_cnt = m_params_ptr->num_iter; it_idx < it_cnt;
         ++it_idx) {
        auto rc = estimatePlane(); // 里面计算的还是种子点的协方差矩阵，得到平面参数abcd
        
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

        // If almost all points are above the plane, the plane is likely biased too low.
        if (ground_point_cloud_indices.size() < 10 ||
            ground_point_cloud_indices.size() * 100 < aux_cloud_ptr->points.size()) {
            std::cout << "The plane is likely biased too low. Switch to height-based segmentation.\n" ;
            std::cout << "ground_point_cloud_indices.size: " << ground_point_cloud_indices.size() << ", total points: " << aux_cloud_ptr->points.size() << std::endl;
            is_use_height = true;
            break;
        }

        pcl::copyPointCloud(*aux_cloud_ptr, non_ground_point_cloud_indices,
                            *m_non_ground_cloud_ptr);
        pcl::copyPointCloud(*aux_cloud_ptr, ground_point_cloud_indices,
                            *m_ground_cloud_ptr);
    }
    std::cout << "is_use_height: " << is_use_height << std::endl;
    std::cout << "m_non_ground_cloud_ptr.size: "<< m_non_ground_cloud_ptr->points.size()<< std::endl;
    std::cout << "second: m_ground_cloud_ptr.size: "<< m_ground_cloud_ptr->points.size()<< std::endl;
    //m_ground_cloud_ptr->clear();
    //m_non_ground_cloud_ptr->clear();

    m_ground_cloud_ptr.reset(new CloudT());
    m_non_ground_cloud_ptr.reset(new CloudT());
    std::vector<int> non_ground_point_cloud_indices;
    std::vector<int> ground_point_cloud_indices;
    if (false == is_use_height) {
        std::cout << "is_use_height:"<< is_use_height<< std::endl;

        auto min_z_point = std::min_element(src_cloud_ptr->points.begin(), src_cloud_ptr->points.end(),
                         [](const PointT &a, const PointT &b) { return a.z < b.z; });
        std::cout << "min_z_point:" << min_z_point->x << "," << min_z_point->y << "," << min_z_point->z << std::endl;
        auto max_z_point = std::max_element(src_cloud_ptr->points.begin(), src_cloud_ptr->points.end(),
                         [](const PointT &a, const PointT &b) { return a.z < b.z; });
        std::cout << "max_z_point:" << max_z_point->x << "," << max_z_point->y << "," << max_z_point->z << std::endl;

        for (size_t i = 0; i < src_cloud_ptr->points.size(); ++i) {
            const auto &pnt = src_cloud_ptr->points[i];
            auto val = m_plane_abcd[0] * pnt.x + m_plane_abcd[1] * pnt.y +
                       m_plane_abcd[2] * pnt.z + m_plane_abcd[3];
            if (0 < val) {
                non_ground_point_cloud_indices.push_back(static_cast<int>(i));
            } else {
                ground_point_cloud_indices.push_back(static_cast<int>(i));
            }
        }
        if (non_ground_point_cloud_indices.size()>0){
            std::cout << "non_ground_point_cloud_indices:"<< non_ground_point_cloud_indices.size()<< std::endl;
            if (non_ground_point_cloud_indices.size() == src_cloud_ptr->points.size()){
                non_ground_point_cloud_indices.pop_back();
            }
            pcl::copyPointCloud(*src_cloud_ptr, non_ground_point_cloud_indices,
                            *m_non_ground_cloud_ptr);
        }
        if(ground_point_cloud_indices.size()>0){
            std::cout << "ground_point_cloud_indices:"<< ground_point_cloud_indices.size()<< std::endl;
            pcl::copyPointCloud(*src_cloud_ptr, ground_point_cloud_indices,
                            *m_ground_cloud_ptr);
        }
        
        

    } else {
        

        std::cout << "use the height directly.\n";
        for (size_t i = 0; i < src_cloud_ptr->points.size(); ++i) {
            const auto &pnt = src_cloud_ptr->points[i];
            if (pnt.z < m_params_ptr->seg_height_th) {
                ground_point_cloud_indices.push_back(static_cast<int>(i));
            } else {
                non_ground_point_cloud_indices.push_back(static_cast<int>(i));
            }
        }
        if (non_ground_point_cloud_indices.size()>0){
            std::cout << "non_ground_point_cloud_indices:"<< non_ground_point_cloud_indices.size()<< std::endl;
            if (non_ground_point_cloud_indices.size() == src_cloud_ptr->points.size()){
                non_ground_point_cloud_indices.pop_back();
            }
            pcl::copyPointCloud(*src_cloud_ptr, non_ground_point_cloud_indices,
                            *m_non_ground_cloud_ptr);
        }
        if(ground_point_cloud_indices.size()>0){
            std::cout << "ground_point_cloud_indices:"<< ground_point_cloud_indices.size()<< std::endl;
            pcl::copyPointCloud(*src_cloud_ptr, ground_point_cloud_indices,
                            *m_ground_cloud_ptr);
        }

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

    std::cout << "before size:" << cloud_ptr->points.size() << std::endl;
    std::cout << "lower:" << cloud_ptr->points.front().z << ", hight:" << cloud_ptr->points.back().z << std::endl;

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
    std::cout << "after size:" << cloud_ptr->points.size() << std::endl;

    return 0;
}
CloudT::Ptr GroundSegment::extractInitSeeds(CloudT::Ptr sorted_cloud_ptr) {
    if (nullptr == sorted_cloud_ptr || sorted_cloud_ptr->points.empty()) {
        return nullptr;
    }

    float z_sum = 0.f;
    size_t pnt_cnt = m_params_ptr->num_lpr; //200个点
    if (sorted_cloud_ptr->points.size() < pnt_cnt) {
        pnt_cnt = sorted_cloud_ptr->points.size();
    }

    // Ignore the lowest 10% in the LPR window to reduce below-ground outlier bias.
    size_t start_idx = pnt_cnt / 10;
    if (start_idx >= pnt_cnt) {
        start_idx = 0;
    }

    size_t test_num = 0;
    for (size_t pnt_idx = start_idx; pnt_idx != pnt_cnt; ++pnt_idx) {
        z_sum += sorted_cloud_ptr->points[pnt_idx].z;
        test_num ++;
    }
    if (0 == test_num) {
        return nullptr;
    }

    std::cout << "test_num:" << test_num << std::endl;
    float lpr_height = z_sum / float(test_num);
    std::cout << "lpr_height:" << lpr_height << std::endl;

    CloudT::Ptr seeds_ptr(new CloudT());
    // auto it_upper = std::upper_bound(
    //     sorted_cloud_ptr->points.begin(), sorted_cloud_ptr->points.end(),
    //     lpr_height + m_params_ptr->seed_th,
    //     [](double value, const PointT &pnt) -> bool { return value < pnt.z; });
    // if (sorted_cloud_ptr->points.end() != it_upper) {
    //   seeds_ptr->points.assign(sorted_cloud_ptr->points.begin(), it_upper);
    // }
    const float seed_th = static_cast<float>(m_params_ptr->seed_th);
    const float min_seed_band = std::max(0.05f, 0.5f * seed_th);
    float seed_z_min = lpr_height - min_seed_band;
    float seed_z_max = lpr_height + seed_th;
    for (const auto &ele_pnt : sorted_cloud_ptr->points) {
        if (ele_pnt.z < seed_z_min) {
            continue;
        }
        if (ele_pnt.z < seed_z_max) {
            seeds_ptr->push_back(ele_pnt);
        } else {
            break;
        }
    }
    std::cout << "seeds_ptr.size:" << seeds_ptr->points.size() << std::endl;
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

    std::cout << "m_plane_abcd:" << m_plane_abcd[0] 
        <<  m_plane_abcd[0]
        << "," << m_plane_abcd[1]
        << "," << m_plane_abcd[2]
        << "," << m_plane_abcd[3]
        <<std::endl;

    return 0;
}
