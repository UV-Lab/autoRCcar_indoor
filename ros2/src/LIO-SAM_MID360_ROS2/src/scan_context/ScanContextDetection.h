#ifndef F6D35A3D_5AB1_40E9_B1A1_8A6C85AFC47D
#define F6D35A3D_5AB1_40E9_B1A1_8A6C85AFC47D

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Dense>

// #include <opencv2/core/eigen.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/opencv.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>

#include"SensorDataType.h"

#include "KDTreeVectorOfVectorsAdaptor.h"
#include "nanoflann.h"
#include "tictoc.h"

namespace msf_loc {


using SCPointType = PointT;// using xyz only. but a user can exchange the original bin encoding function (i.e., max hegiht) to max intensity (for detail, refer 20 ICRA Intensity Scan Context)
using KeyMat = std::vector<std::vector<float>>;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor<KeyMat, float>;

class ScanContextDetectionParams {
public:
    using SharedPtr = std::shared_ptr<ScanContextDetectionParams>;
    double lidar_height;
    int pc_num_ring;
    int pc_num_sector;
    double pc_max_radius;
    double pc_sector_angle_range_deg;
    // double pc_unit_ring_gap;
    int num_candidates;
    double search_ratio;
    double sc_dist_thres;
};


class ScanContextDetection {
public:
    ScanContextDetection(ScanContextDetectionParams::SharedPtr params_ptr);
    virtual ~ScanContextDetection();
    int init();

    void makeAndSaveScancontextAndKeys(pcl::PointCloud<SCPointType> &_scan_down);
    void constructRingKeyTree();
    // detectLoopClosureID for the special key frame cloud.
    // std::pair<int, float> detectLoopClosureID(pcl::PointCloud<SCPointType> &_scan_down);
    int detectLoopClosureCandidates(pcl::PointCloud<SCPointType> &scan_down,
                                    std::vector<std::pair<int, float>> &candidates_vec);

    int saveSCData(const std::string &sc_data_file);
    int loadSCData(const std::string &sc_data_file);

    // FIXME: delete it later.
    const std::vector<Eigen::MatrixXd> &getScanContext() const {
        return m_polarcontexts;
    }

private:
    Eigen::MatrixXd makeScancontext(pcl::PointCloud<SCPointType> &_scan_down);
    Eigen::MatrixXd makeRingkeyFromScancontext(Eigen::MatrixXd &_desc);
    Eigen::MatrixXd makeSectorkeyFromScancontext(Eigen::MatrixXd &_desc);

    int fastAlignUsingVkey(Eigen::MatrixXd &_vkey1, Eigen::MatrixXd &_vkey2);
    double distDirectSC(Eigen::MatrixXd &_sc1, Eigen::MatrixXd &_sc2);                          // "d" (eq 5) in the original paper (IROS 18)
    std::pair<double, int> distanceBtnScanContext(Eigen::MatrixXd &_sc1, Eigen::MatrixXd &_sc2);// "D" (eq 6) in the original paper (IROS 18)

    float xy2theta(const float &_x, const float &_y);
    Eigen::MatrixXd circshift(Eigen::MatrixXd &_mat, int _num_shift);
    std::vector<float> eig2stdvec(Eigen::MatrixXd _eigmat);
    float rad2deg(float radians);
    float deg2rad(float degrees);

    int saveMatrixs(const std::string &data_file,
                    const std::vector<Eigen::MatrixXd> &mat_vec);
    int loadMatrixs(const std::string &data_file,
                    std::vector<Eigen::MatrixXd> &mat_vec);

private:
    ScanContextDetectionParams::SharedPtr m_params_ptr = nullptr;


    double LIDAR_HEIGHT = 2.0;  // lidar height : add this for simply directly using lidar scan in the lidar local coord (not robot base coord) / if you use robot-coord-transformed lidar scans, just set this as 0.
    int PC_NUM_RING = 20;       // 20 in the original paper (IROS 18)
    int PC_NUM_SECTOR = 60;     // 60 in the original paper (IROS 18)
    double PC_MAX_RADIUS = 80.0;// 80 meter max in the original paper (IROS 18)
    double PC_UNIT_SECTOR_ANGLE = 360.0 / double(PC_NUM_SECTOR);
    double PC_UNIT_RINGGAP = PC_MAX_RADIUS / double(PC_NUM_RING);

    // tree
    int NUM_EXCLUDE_RECENT = 30;// simply just keyframe gap (related with loopClosureFrequency in yaml), but node position distance-based exclusion is ok.
    int NUM_CANDIDATES = 3;     // 10 is enough. (refer the IROS 18 paper)

    // loop thres
    double SEARCH_RATIO = 0.1;// for fast comparison, no Brute-force, but search 10 % is okay. // not was in the original conf paper, but improved ver.
    // const double SC_DIST_THRES = 0.13; // empirically 0.1-0.2 is fine (rare false-alarms) for 20x60 polar context (but for 0.15 <, DCS or ICP fit score check (e.g., in LeGO-LOAM) should be required for robustness)
    double SC_DIST_THRES = 0.7;// 0.4-0.6 is good choice for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold / if not, recommend 0.1-0.15
    // const double SC_DIST_THRES = 0.7; // 0.4-0.6 is good choice for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold / if not, recommend 0.1-0.15

    // config
    // int TREE_MAKING_PERIOD_ = 10;


    std::vector<double> m_polarcontexts_timestamp;// optional.
    std::vector<Eigen::MatrixXd> m_polarcontexts;
    std::vector<Eigen::MatrixXd> m_polarcontext_invkeys;
    std::vector<Eigen::MatrixXd> m_polarcontext_vkeys;

    KeyMat m_polarcontext_invkeys_mat;
    KeyMat m_polarcontext_invkeys_to_search;
    std::unique_ptr<InvKeyTree> m_polarcontext_tree;
};


}// namespace msf_loc

#endif /* F6D35A3D_5AB1_40E9_B1A1_8A6C85AFC47D */
