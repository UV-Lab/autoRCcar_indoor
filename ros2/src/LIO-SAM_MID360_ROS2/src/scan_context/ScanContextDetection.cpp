#include "ScanContextDetection.h"

#include <fstream>

#include <rob_common/log_config/RobLogger.h>

using namespace msf_loc;
using namespace rob_common;
using namespace Eigen;
using namespace nanoflann;

// using std::cout;
// using std::endl;
// using std::make_pair;

// using std::atan2;
// using std::cos;
// using std::sin;


ScanContextDetection::ScanContextDetection(
        ScanContextDetectionParams::SharedPtr params_ptr)
    : m_params_ptr(params_ptr) {
    LIDAR_HEIGHT = m_params_ptr->lidar_height;
    PC_NUM_RING = m_params_ptr->pc_num_ring;
    PC_NUM_SECTOR = m_params_ptr->pc_num_sector;
    PC_MAX_RADIUS = m_params_ptr->pc_max_radius;
    PC_UNIT_SECTOR_ANGLE = m_params_ptr->pc_sector_angle_range_deg / m_params_ptr->pc_num_sector;
    PC_UNIT_RINGGAP = m_params_ptr->pc_max_radius / double(m_params_ptr->pc_num_ring);
    NUM_CANDIDATES = m_params_ptr->num_candidates;
    SEARCH_RATIO = m_params_ptr->search_ratio;
    SC_DIST_THRES = m_params_ptr->sc_dist_thres;
}

ScanContextDetection::~ScanContextDetection() {
}

int ScanContextDetection::init() {
    return 0;
}

void ScanContextDetection::makeAndSaveScancontextAndKeys(
        pcl::PointCloud<SCPointType> &_scan_down) {
    Eigen::MatrixXd sc = makeScancontext(_scan_down);// v1
    Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(sc);
    Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext(sc);
    std::vector<float> polarcontext_invkey_vec = eig2stdvec(ringkey);

    m_polarcontexts.push_back(sc);
    m_polarcontext_invkeys.push_back(ringkey);
    m_polarcontext_vkeys.push_back(sectorkey);
    m_polarcontext_invkeys_mat.push_back(polarcontext_invkey_vec);
}

void ScanContextDetection::constructRingKeyTree() {
    m_polarcontext_invkeys_to_search.clear();
    m_polarcontext_invkeys_to_search.assign(m_polarcontext_invkeys_mat.begin(), m_polarcontext_invkeys_mat.end());
    m_polarcontext_tree.reset();
    m_polarcontext_tree = std::make_unique<InvKeyTree>(PC_NUM_RING /* dim */, m_polarcontext_invkeys_to_search, 10 /* max leaf */);
}

int ScanContextDetection::detectLoopClosureCandidates(
        pcl::PointCloud<SCPointType> &scan_down,
        std::vector<std::pair<int, float>> &candidates_vec) {
    // make the scan context for the scan_down
    Eigen::MatrixXd sc = makeScancontext(scan_down);// v1
    Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(sc);
    Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext(sc);
    std::vector<float> polarcontext_invkey_vec = eig2stdvec(ringkey);


    auto curr_key = polarcontext_invkey_vec;
    auto curr_desc = sc;

    /**********************Another trial *****************/
    struct SCIndexDistAlign {
        int idx;
        double dist;
        double align_deg;
    };

    // 这里使用了暴力搜索，但其实可以使用FLANN来进行快速搜索最相机的N帧候选帧，然后再旋转对齐。
    std::vector<SCIndexDistAlign> sc_idx_dist_align_vec;
    for (int sc_idx = 0, sc_cnt = m_polarcontexts.size(); sc_idx != sc_cnt; ++sc_idx) {
        std::pair<double, int> sc_dist_align = distanceBtnScanContext(curr_desc, m_polarcontexts.at(sc_idx));
        sc_idx_dist_align_vec.push_back({sc_idx, sc_dist_align.first, sc_dist_align.second * PC_UNIT_SECTOR_ANGLE});
    }
    std::sort(sc_idx_dist_align_vec.begin(), sc_idx_dist_align_vec.end(),
              [](const SCIndexDistAlign &a, const SCIndexDistAlign &b) {
                  return a.dist < b.dist;
              });
    // print the result.
    // std::cout << "The sc registration result: \n";
    // for (int idx = 0, cnt = sc_idx_dist_align_vec.size(); idx != cnt; ++idx) {
    //     const auto &ele = sc_idx_dist_align_vec.at(idx);
    //     std::cout << "(" << ele.idx << ", " << ele.dist << ", " << ele.align_deg << ") ";
    //     if (idx % 10 == 0)
    //         std::cout << std::endl;
    // }

    /*****************************************************/
    candidates_vec.clear();
    for (int idx = 0, cnt = NUM_CANDIDATES; idx != cnt; ++idx) {
        const auto &ele = sc_idx_dist_align_vec.at(idx);
        candidates_vec.push_back({ele.idx, deg2rad(ele.align_deg)});
    }


    return 0;
}

int ScanContextDetection::saveSCData(const std::string &sc_data_file) {
    return saveMatrixs(sc_data_file, m_polarcontexts);
}
int ScanContextDetection::loadSCData(const std::string &sc_data_file) {
    auto rc = loadMatrixs(sc_data_file, m_polarcontexts);
    if (0 != rc) {
        ROB_LOG_WARN("Fail to load the scan context data.");
        return -1;
    }
    for (auto &sc : m_polarcontexts) {
        Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(sc);
        Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext(sc);
        std::vector<float> polarcontext_invkey_vec = eig2stdvec(ringkey);

        m_polarcontext_invkeys.push_back(ringkey);
        m_polarcontext_vkeys.push_back(sectorkey);
        m_polarcontext_invkeys_mat.push_back(polarcontext_invkey_vec);
    }
    return 0;
}

int ScanContextDetection::saveMatrixs(
        const std::string &data_file,
        const std::vector<Eigen::MatrixXd> &mat_vec) {

    const Eigen::IOFormat the_format(4, Eigen::DontAlignCols, " ", "\n");
    std::ofstream out_file(data_file);

    if (!out_file.is_open()) {
        // std::cout << "Cannot open file for writing: " + data_file << std::endl;
        ROB_LOG_WARN("Cannot open file for writing: {}.", data_file);
        return -1;
    }

    // Write the number of matrices
    out_file << mat_vec.size() << "\n";

    // Write each matrix
    for (const auto &matrix : mat_vec) {
        // Write matrix dimensions
        out_file << matrix.rows() << " " << matrix.cols() << "\n";
        // Write matrix data
        out_file << matrix.format(the_format) << "\n";
    }

    out_file.close();
    return 0;
}

int ScanContextDetection::loadMatrixs(
        const std::string &data_file,
        std::vector<Eigen::MatrixXd> &mat_vec) {
    std::ifstream in_file(data_file);

    if (!in_file.is_open()) {
        // std::cout << "Cannot open file for reading: " + data_file << std::endl;
        ROB_LOG_WARN("Cannot open file for reading: {}.", data_file);
        return -1;
    }

    mat_vec.clear();

    // Read number of matrices
    size_t num_matrices;
    in_file >> num_matrices;

    // Read each matrix
    for (size_t i = 0; i < num_matrices; ++i) {
        // Read matrix dimensions
        Eigen::Index rows, cols;
        in_file >> rows >> cols;

        // Create matrix with appropriate size
        Eigen::MatrixXd matrix(rows, cols);

        // Read matrix data
        for (Eigen::Index r = 0; r < rows; ++r) {
            for (Eigen::Index c = 0; c < cols; ++c) {
                in_file >> matrix(r, c);
            }
        }

        mat_vec.push_back(matrix);
    }

    in_file.close();
    return 0;
}


Eigen::MatrixXd ScanContextDetection::makeScancontext(pcl::PointCloud<SCPointType> &_scan_down) {
    TicToc t_making_desc;

    int num_pts_scan_down = _scan_down.points.size();

    // main
    const int NO_POINT = -1000;
    MatrixXd desc = NO_POINT * MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);

    SCPointType pt;
    float azim_angle, azim_range;// wihtin 2d plane
    int ring_idx, sctor_idx;
    for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++) {
        pt.x = _scan_down.points[pt_idx].x;
        pt.y = _scan_down.points[pt_idx].y;
        pt.z = _scan_down.points[pt_idx].z + LIDAR_HEIGHT;// naive adding is ok (all points should be > 0).

        // xyz to ring, sector
        azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
        azim_angle = xy2theta(pt.x, pt.y);

        // if range is out of roi, pass
        if (azim_range > PC_MAX_RADIUS)
            continue;

        ring_idx = std::max(std::min(PC_NUM_RING, int(ceil((azim_range / PC_MAX_RADIUS) * PC_NUM_RING))), 1);
        sctor_idx = std::max(std::min(PC_NUM_SECTOR, int(ceil((azim_angle / 360.0) * PC_NUM_SECTOR))), 1);

        // taking maximum z
        if (desc(ring_idx - 1, sctor_idx - 1) < pt.z)// -1 means cpp starts from 0
            desc(ring_idx - 1, sctor_idx - 1) = pt.z;// update for taking maximum value at that bin
    }

    // reset no points to zero (for cosine dist later)
    for (int row_idx = 0; row_idx < desc.rows(); row_idx++)
        for (int col_idx = 0; col_idx < desc.cols(); col_idx++)
            if (desc(row_idx, col_idx) == NO_POINT)
                desc(row_idx, col_idx) = 0;

    t_making_desc.toc("PolarContext making");

    return desc;
}
Eigen::MatrixXd ScanContextDetection::makeRingkeyFromScancontext(Eigen::MatrixXd &_desc) {
    /* 
     * summary: rowwise mean vector
    */
    Eigen::MatrixXd invariant_key(_desc.rows(), 1);
    for (int row_idx = 0; row_idx < _desc.rows(); row_idx++) {
        Eigen::MatrixXd curr_row = _desc.row(row_idx);
        invariant_key(row_idx, 0) = curr_row.mean();
    }

    return invariant_key;
}
Eigen::MatrixXd ScanContextDetection::makeSectorkeyFromScancontext(Eigen::MatrixXd &_desc) {
    /* 
     * summary: columnwise mean vector
    */
    Eigen::MatrixXd variant_key(1, _desc.cols());
    for (int col_idx = 0; col_idx < _desc.cols(); col_idx++) {
        Eigen::MatrixXd curr_col = _desc.col(col_idx);
        variant_key(0, col_idx) = curr_col.mean();
    }

    return variant_key;
}

int ScanContextDetection::fastAlignUsingVkey(Eigen::MatrixXd &_vkey1, Eigen::MatrixXd &_vkey2) {
    int argmin_vkey_shift = 0;
    double min_veky_diff_norm = 10000000;
    for (int shift_idx = 0; shift_idx < _vkey1.cols(); shift_idx++) {
        MatrixXd vkey2_shifted = circshift(_vkey2, shift_idx);

        MatrixXd vkey_diff = _vkey1 - vkey2_shifted;

        double cur_diff_norm = vkey_diff.norm();
        if (cur_diff_norm < min_veky_diff_norm) {
            argmin_vkey_shift = shift_idx;
            min_veky_diff_norm = cur_diff_norm;
        }
    }

    return argmin_vkey_shift;
}
double ScanContextDetection::distDirectSC(Eigen::MatrixXd &_sc1, Eigen::MatrixXd &_sc2) {
    int num_eff_cols = 0;// i.e., to exclude all-nonzero sector
    double sum_sector_similarity = 0;
    for (int col_idx = 0; col_idx < _sc1.cols(); col_idx++) {
        VectorXd col_sc1 = _sc1.col(col_idx);
        VectorXd col_sc2 = _sc2.col(col_idx);

        if ((col_sc1.norm() == 0) | (col_sc2.norm() == 0))
            continue;// don't count this sector pair.

        double sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());

        sum_sector_similarity = sum_sector_similarity + sector_similarity;
        num_eff_cols = num_eff_cols + 1;
    }

    double sc_sim = sum_sector_similarity / num_eff_cols;
    return 1.0 - sc_sim;
}
std::pair<double, int> ScanContextDetection::distanceBtnScanContext(Eigen::MatrixXd &_sc1, Eigen::MatrixXd &_sc2) {
    // 1. fast align using variant key (not in original IROS18)
    MatrixXd vkey_sc1 = makeSectorkeyFromScancontext(_sc1);
    MatrixXd vkey_sc2 = makeSectorkeyFromScancontext(_sc2);
    int argmin_vkey_shift = fastAlignUsingVkey(vkey_sc1, vkey_sc2);

    const int SEARCH_RADIUS = round(0.5 * SEARCH_RATIO * _sc1.cols());// a half of search range
    std::vector<int> shift_idx_search_space{argmin_vkey_shift};
    for (int ii = 1; ii < SEARCH_RADIUS + 1; ii++) {
        shift_idx_search_space.push_back((argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols());
        shift_idx_search_space.push_back((argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols());
    }
    std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

    // 2. fast columnwise diff
    int argmin_shift = 0;
    double min_sc_dist = 10000000;
    for (int num_shift : shift_idx_search_space) {
        MatrixXd sc2_shifted = circshift(_sc2, num_shift);
        double cur_sc_dist = distDirectSC(_sc1, sc2_shifted);
        if (cur_sc_dist < min_sc_dist) {
            argmin_shift = num_shift;
            min_sc_dist = cur_sc_dist;
        }
    }

    return std::make_pair(min_sc_dist, argmin_shift);
}

float ScanContextDetection::xy2theta(const float &_x, const float &_y) {
    if ((_x >= 0) & (_y >= 0))
        return (180 / M_PI) * atan(_y / _x);

    if ((_x < 0) & (_y >= 0))
        return 180 - ((180 / M_PI) * atan(_y / (-_x)));

    if ((_x < 0) & (_y < 0))
        return 180 + ((180 / M_PI) * atan(_y / _x));

    if ((_x >= 0) & (_y < 0))
        return 360 - ((180 / M_PI) * atan((-_y) / _x));
}
Eigen::MatrixXd ScanContextDetection::circshift(Eigen::MatrixXd &_mat, int _num_shift) {
    // shift columns to right direction
    assert(_num_shift >= 0);

    if (_num_shift == 0) {
        MatrixXd shifted_mat(_mat);
        return shifted_mat;// Early return
    }

    MatrixXd shifted_mat = MatrixXd::Zero(_mat.rows(), _mat.cols());
    for (int col_idx = 0; col_idx < _mat.cols(); col_idx++) {
        int new_location = (col_idx + _num_shift) % _mat.cols();
        shifted_mat.col(new_location) = _mat.col(col_idx);
    }

    return shifted_mat;
}
std::vector<float> ScanContextDetection::eig2stdvec(Eigen::MatrixXd _eigmat) {
    std::vector<float> vec(_eigmat.data(), _eigmat.data() + _eigmat.size());
    return vec;
}

float ScanContextDetection::rad2deg(float radians) {
    return radians * 180.0 / M_PI;
}
float ScanContextDetection::deg2rad(float degrees) {
    return degrees * M_PI / 180.0;
}