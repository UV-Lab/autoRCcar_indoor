// #include <filesystem>
#include <iostream>

#include <opencv2/opencv.hpp>

#include <pcl/common/transforms.h>

#include <rob_common/log_config/RobConfig.h>
#include <rob_common/log_config/RobLogger.h>

#include "map_splitter/MapSplitter.h"
#include "scan_context/ScanContextDetection.h"

#include "map_splitter/common.h"


using namespace rob_common;
using namespace msf_loc;

int main(int argc, char **argv) {
    std::cout << "Usage: slam_map_post_processing cfg_file src_map_folder grid_size dst_folder" << std::endl;
    if (argc != 5) {
        std::cout << "Please input the cmd correctly." << std::endl;
        return -1;
    }

    std::string cfg_file_path = std::string(argv[1]);
    std::string src_map_folder = std::string(argv[2]) + "/";
    std::string src_cloud_path = src_map_folder + "GlobalMap.ply";
    float grid_size = std::atof(argv[3]);
    std::string dst_folder = argv[4];
    std::string keyframe_cloud_folder = src_map_folder + "pointCloud/";
    std::string keyframe_pose_file = keyframe_cloud_folder + "cloud.yaml";
    std::string dst_sc_file_path = dst_folder + "/scan_context.scd";
    std::string dst_keyframe_pose_file = dst_folder + "/keyframe_poses.yaml";


    // load the cfg_file.
    int rc = RobConfig::initConfig(cfg_file_path, ConfigFileType::YAML);
    if (0 != rc) {
        ROB_LOG_ERROR("Fail to init the config file.");
        return -1;
    }

    ROB_LOG_INFO("Split the global map.");
    MapSplitterParams::SharedPtr params_ptr = std::make_shared<MapSplitterParams>();
    params_ptr->grid_sz = grid_size;
    std::shared_ptr<MapSplitter> map_splitter_ptr = std::make_shared<MapSplitter>(params_ptr);
    map_splitter_ptr->init();
    PointCloudT::Ptr cloud_in_ptr(new PointCloudT());
    // auto cloud_load_rc = pcl::io::loadPCDFile(src_cloud_path, *cloud_in_ptr);
    auto cloud_load_rc = msf_util::loadPointCloudFile(src_cloud_path, *cloud_in_ptr);
    const auto &quat_base_lidar_wxyz_arr = RobConfig::getNodeArray<float>("rob_intrinsic/tf_base_lidar/quat_wxyz");
    Eigen::Quaternionf quat_base_lidar(quat_base_lidar_wxyz_arr[0],
                                       quat_base_lidar_wxyz_arr[1],
                                       quat_base_lidar_wxyz_arr[2],
                                       quat_base_lidar_wxyz_arr[3]);
    quat_base_lidar.normalize();
    Eigen::Matrix4f tf_base_lidar = Eigen::Matrix4f::Identity();
    tf_base_lidar.block<3, 3>(0, 0) = quat_base_lidar.toRotationMatrix();
    // PointCloudT::Ptr new_cloud_ptr(new PointCloudT());
    // pcl::transformPointCloud(*cloud_in_ptr, *new_cloud_ptr, tf_base_lidar);
    // *cloud_in_ptr = *new_cloud_ptr;


    if (0 != cloud_load_rc) {
        // printf("Fail to load the cloud: %s.\n", src_cloud_path.c_str());
        ROB_LOG_WARN("Fail to load the cloud: {}.", src_cloud_path);
    }
    rc = map_splitter_ptr->splitMap(cloud_in_ptr);
    if (0 != rc) {
        ROB_LOG_WARN("Fail to split the map.");
        return -1;
    }
    rc = map_splitter_ptr->saveMap(dst_folder);
    if (0 != rc) {
        ROB_LOG_WARN("Fail to save the map.");
        return -2;
    }
    ROB_LOG_INFO("Split the map successfully.");

    ROB_LOG_INFO("Generate the Scan Context data.");
    ScanContextDetectionParams::SharedPtr sc_params_ptr = std::make_shared<ScanContextDetectionParams>();
    std::string sc_root("msf_loc_module/auto_init_loc/scan_context/");
    sc_params_ptr->lidar_height = RobConfig::getNodeValue<double>(sc_root + "lidar_height");
    sc_params_ptr->pc_num_ring = RobConfig::getNodeValue<int>(sc_root + "pc_num_ring");
    sc_params_ptr->pc_num_sector = RobConfig::getNodeValue<int>(sc_root + "pc_num_sector");
    sc_params_ptr->pc_max_radius = RobConfig::getNodeValue<double>(sc_root + "pc_max_radius");
    sc_params_ptr->pc_sector_angle_range_deg = RobConfig::getNodeValue<double>(sc_root + "pc_sector_angle_range_deg");
    sc_params_ptr->num_candidates = RobConfig::getNodeValue<int>(sc_root + "num_candidates");
    sc_params_ptr->search_ratio = RobConfig::getNodeValue<double>(sc_root + "search_ratio");
    sc_params_ptr->sc_dist_thres = RobConfig::getNodeValue<double>(sc_root + "sc_dist_thres");
    std::shared_ptr<ScanContextDetection> sc_detection_ptr = std::make_shared<ScanContextDetection>(sc_params_ptr);
    rc = sc_detection_ptr->init();
    if (rc != 0) {
        ROB_LOG_WARN("Fail to init the sc detection.");
        return -3;
    }

    // Create the voxel grid.
    pcl::VoxelGrid<SCPointType> voxel_grid_filter;
    float voxel_grid_leaf_size = RobConfig::getNodeValue<float>("msf_loc_module/auto_init_loc/voxel_grid/leaf_size");
    voxel_grid_filter.setLeafSize(voxel_grid_leaf_size, voxel_grid_leaf_size, voxel_grid_leaf_size);


    // load the keyframe cloud.
    std::vector<std::string> keyframe_cloud_file_vec;
    // load the keyframe cloud.
    std::string cloud_yaml_file_path = keyframe_pose_file;
    cv::FileStorage fs;
    fs.open(cloud_yaml_file_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open file " << cloud_yaml_file_path << std::endl;
        return -2;
    }
    int kf_cloud_cnt = fs["cloud_cnt"];
    // cv::FileNode quat_vl_l_wxyz_node = fs["quat_vl_l_wxyz"];
    auto read_arr_from_cv_file = [](const ::cv::FileStorage &fs, const std::string &node_name,
                                    std::vector<float> &val_arr) -> int {
        val_arr.clear();
        cv::FileNode float_array_node = fs[node_name];
        if (false == float_array_node.isNone() &&
            true == float_array_node.isSeq()) {
            for (cv::FileNodeIterator it = float_array_node.begin();
                 it != float_array_node.end(); ++it) {
                val_arr.push_back(static_cast<float>(*it));
            }
        }
        return val_arr.size();
    };
    std::vector<float> quat_vl_l_wxyz_arr;
    int arr_size = read_arr_from_cv_file(fs, "quat_vl_l_wxyz", quat_vl_l_wxyz_arr);
    if (0 == quat_vl_l_wxyz_arr.size()) {
        std::cerr << "Fail to read the quat_vl_l_wxyz from the yaml file." << std::endl;
        fs.release();
        return -3;
    }
    Eigen::Quaternionf quat_vl_l(quat_vl_l_wxyz_arr[0], quat_vl_l_wxyz_arr[1],
                                 quat_vl_l_wxyz_arr[2], quat_vl_l_wxyz_arr[3]);
    Eigen::Matrix4f tf_vl_l = Eigen::Matrix4f::Identity();
    tf_vl_l.block(0, 0, 3, 3) = quat_vl_l.normalized().toRotationMatrix();
    Eigen::Matrix4f tf_l_vl = tf_vl_l.inverse();
    fs.release();


    for (int cloud_idx = 0; cloud_idx != kf_cloud_cnt; ++cloud_idx) {
        std::string cloud_file_path = keyframe_cloud_folder + "/" + std::to_string(cloud_idx) + ".ply";
        pcl::PointCloud<SCPointType>::Ptr vl_kf_cloud_ptr(new pcl::PointCloud<SCPointType>());
        // pcl::io::loadPCDFile(cloud_file_path, *kf_cloud_ptr);
        auto load_rc = msf_util::loadPointCloudFile(cloud_file_path, *vl_kf_cloud_ptr);
        if (0 != load_rc) {
            std::cout << "Fail to load the cloud file: " << cloud_file_path << std::endl;
            continue;
        }
        pcl::PointCloud<SCPointType>::Ptr vl_filtered_cloud_ptr(new pcl::PointCloud<SCPointType>());
        voxel_grid_filter.setInputCloud(vl_kf_cloud_ptr);
        voxel_grid_filter.filter(*vl_filtered_cloud_ptr);
        pcl::PointCloud<SCPointType>::Ptr l_filtered_cloud_ptr(new pcl::PointCloud<SCPointType>());
        pcl::transformPointCloud(*vl_filtered_cloud_ptr, *l_filtered_cloud_ptr, tf_l_vl);

        // make scancontext and keys
        sc_detection_ptr->makeAndSaveScancontextAndKeys(*l_filtered_cloud_ptr);
    }
    sc_detection_ptr->constructRingKeyTree();
    sc_detection_ptr->saveSCData(dst_sc_file_path);

    // copy the keyframe_pose_file to dst_keyframe_pose_file
    auto copy_rc = std_fs::copy_file(keyframe_pose_file, dst_keyframe_pose_file);
    if (false == copy_rc) {
        ROB_LOG_WARN("Fail to copy keyframe pose file: {} -> {}.",
                     keyframe_pose_file, dst_keyframe_pose_file);
        return -4;
    } else {
        ROB_LOG_INFO("Copy keyframe pose file: {} -> {}.",
                     keyframe_pose_file, dst_keyframe_pose_file);
    }

    ROB_LOG_INFO("The post processing is done.");


    return 0;
}