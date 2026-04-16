#include <iostream>

#include "utility.hpp"

#include <pcl/common/angles.h>
#include <pcl/io/ply_io.h>

#include <Eigen/Eigen>

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "grid_map/GridMapMaker.h"
#include "grid_map/GroundSegment.h"

#include "grid_map/CloudToScan.h"

#include "grid_map/RobLog.h"

#include "grid_map/Common.h"
#include "grid_map/GridMapMaker.h"

using namespace grid_map;

struct CloudRange {
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float min_z;
    float max_z;
};

pcl::PointCloud<PointType>::Ptr
trimPointCloud(pcl::PointCloud<PointType>::Ptr cloud_in,
               const CloudRange &range) {

    pcl::PointCloud<PointType>::Ptr cloud_out(new pcl::PointCloud<PointType>());
    for (const auto &pnt : cloud_in->points) {
        if (pnt.x < range.min_x || pnt.x > range.max_x) {
            continue;
        }
        if (pnt.y < range.min_y || pnt.y > range.max_y) {
            continue;
        }
        if (pnt.z < range.min_z || pnt.z > range.max_z) {
            continue;
        }
        cloud_out->points.push_back(pnt);
    }
    return cloud_out;
}

CloudT::Ptr extractSubMapCloud(CloudT::Ptr map_cloud_in,
                               const CloudRange &range,
                               const Eigen::Matrix4f &tf_world_base) {

    // 计算旋转立方体的参数
    Eigen::Vector3f box_translation = tf_world_base.block<3, 1>(0, 3);
    Eigen::Matrix3f rotation_matrix = tf_world_base.block<3, 3>(0, 0);
    Eigen::Vector3f box_rot_xyz = rotation_matrix.eulerAngles(0, 1, 2);

    // 计算立方体尺寸（使用原始范围参数）
    Eigen::Vector4f min_point, max_point;
    min_point << range.min_x, range.min_y, range.min_z, 0;
    max_point << range.max_x, range.max_y, range.max_z, 0;

    // 创建并配置cropBox过滤器
    pcl::CropBox<PointType> crop_box;
    crop_box.setMin(min_point);
    crop_box.setMax(max_point);
    crop_box.setInputCloud(map_cloud_in);
    crop_box.setTranslation(box_translation);
    crop_box.setRotation(box_rot_xyz);

    // 执行裁剪并返回结果
    CloudT::Ptr world_crop_cloud_ptr(new CloudT());
    crop_box.filter(*world_crop_cloud_ptr);
    CloudT::Ptr base_crop_cloud_ptr(new CloudT());
    pcl::transformPointCloud(*world_crop_cloud_ptr, *base_crop_cloud_ptr,
                             tf_world_base.inverse());
    // ROB_LOG_INFO("Save the crop map cloud.");
    // pcl::io::savePLYFileASCII("./sub_map_in_lidar.ply", *lidar_crop_cloud_ptr);
    // pcl::io::savePLYFileASCII("./sub_map_in_world.ply", *world_crop_cloud_ptr);

    return base_crop_cloud_ptr;
}

int main(int argc, char **argv) {
    std::cout
            << "cmd: grid_map_builder cloud_path cfg_file map_save_path map_name"
            << std::endl;
    if (5 != argc) {
        ROB_LOG_WARN("Please input the correct cmd.");
        return -1;
    }

    std::string cloud_path = std::string(argv[1]) + "/";
    std::string cfg_file = std::string(argv[2]);
    std::string map_save_path = std::string(argv[3]) + "/";
    std::string grid_map_name = std::string(argv[4]);

    // load the params from yaml file.
    cv::FileStorage cfg_fs(cfg_file, cv::FileStorage::READ);
    if (false == cfg_fs.isOpened()) {
        ROB_LOG_WARN("Open the cfg file %s failed.", cfg_file.c_str());
        return -1;
    }

    // load the cloud config.
    std::string cloud_yaml_path = cloud_path + "cloud.yaml";
    cv::FileStorage cloud_fs(cloud_yaml_path, cv::FileStorage::READ);
    if (false == cloud_fs.isOpened()) {
        ROB_LOG_WARN("Open the cloud yaml file %s failed.",
                     cloud_yaml_path.c_str());
        return -2;
    }
    int cloud_cnt = cloud_fs["cloud_cnt"];
    ROB_LOG_INFO("The Cloud cnt is %d.", cloud_cnt);
    std::string cloud_prefix = cloud_fs["cloud_prefix"];

    GridMapMakerParams::SharedPtr gmm_params_ptr =
            std::make_shared<GridMapMakerParams>();
    /// load the point cloud processing.
    CloudRange cloud_range;
    const auto &range_node = cfg_fs["cloud_process"]["cloud_range"];
    cloud_range.min_x = range_node["min_x"];
    cloud_range.max_x = range_node["max_x"];
    cloud_range.min_y = range_node["min_y"];
    cloud_range.max_y = range_node["max_y"];
    cloud_range.min_z = range_node["min_z"];
    cloud_range.max_z = range_node["max_z"];
    float voxel_size = cfg_fs["cloud_process"]["voxel_size"];

    ROB_LOG_INFO("cloud range: [%f, %f], [%f, %f], [%f, %f]", cloud_range.min_x,
                 cloud_range.max_x, cloud_range.min_y, cloud_range.max_y,
                 cloud_range.min_z, cloud_range.max_z);

    cv::Mat cvmat_base_lidar;
    cfg_fs["tf_base_lidar"] >> cvmat_base_lidar;
    Eigen::Matrix4f tf_base_lidar;
    cv::cv2eigen(cvmat_base_lidar, tf_base_lidar);
    Eigen::Matrix4f tf_lidar_base = tf_base_lidar.inverse();
    std::cout << "tf_base_lidar: \n"
              << tf_base_lidar << std::endl;


    gmm_params_ptr->m_gs_params_ptr = std::make_shared<GroundSegmentParams>();
    gmm_params_ptr->m_cts_params_ptr = std::make_shared<CloudToScanParams>();
    gmm_params_ptr->m_occ_map_params_ptr = std::make_shared<OccupancyMapParams>();

    // Load the ground segment params.
    //   std::string gs_node("ground_segment.");
    const auto &gs_node = cfg_fs["ground_segment"];
    auto gs_params_ptr = gmm_params_ptr->m_gs_params_ptr;
    gs_params_ptr->min_height = gs_node["min_height"];
    gs_params_ptr->max_height = gs_node["max_height"];
    gs_params_ptr->num_iter = gs_node["num_iter"];
    gs_params_ptr->num_lpr = gs_node["num_lpr"];
    gs_params_ptr->seed_th = gs_node["seed_th"];
    gs_params_ptr->dist_th = gs_node["dist_th"];
    gs_params_ptr->slope_angle_rad_th =
            gs_node["slop_deg_th"].real() / 180.0 * M_PI;
    gs_params_ptr->seg_height_th = gs_node["segment_height_th"];
    ROB_LOG_INFO("ground segment params: min_height = %f, max_height = %f, "
                 "num_iter = %d, num_lpr = %d, seed_th = %f, dist_th = %f, "
                 "slope_angle_rad_th = %f, seg_height_th = %f",
                 gs_params_ptr->min_height, gs_params_ptr->max_height,
                 gs_params_ptr->num_iter, gs_params_ptr->num_lpr,
                 gs_params_ptr->seed_th, gs_params_ptr->dist_th,
                 gs_params_ptr->slope_angle_rad_th, gs_params_ptr->seg_height_th);

    // load the cloud to scan params.
    //   std::string cts_node("cloud_to_scan.");
    const auto &cts_node = cfg_fs["cloud_to_scan"];
    auto cts_params_ptr = gmm_params_ptr->m_cts_params_ptr;
    cts_params_ptr->angle_min = cts_node["angle_min"].real() / 180.f * M_PI;
    cts_params_ptr->angle_max = cts_node["angle_max"].real() / 180.f * M_PI;
    cts_params_ptr->angle_increment = cts_node["angle_increment"];
    cts_params_ptr->time_increment = cts_node["time_increment"];
    cts_params_ptr->scan_time = cts_node["scan_time"];
    cts_params_ptr->range_min = cts_node["range_min"];
    cts_params_ptr->range_max = cts_node["range_max"];
    cts_params_ptr->z_min = cts_node["z_min"];
    cts_params_ptr->z_max = cts_node["z_max"];
    ROB_LOG_INFO("cloud to scan params: angle_min = %f, angle_max = %f, "
                 "angle_increment = %f, time_increment = %f, scan_time = %f,"
                 "range_min = %f, range_max = %f, z_min = %f, z_max = %f",
                 cts_params_ptr->angle_min, cts_params_ptr->angle_max,
                 cts_params_ptr->angle_increment, cts_params_ptr->time_increment,
                 cts_params_ptr->scan_time, cts_params_ptr->range_min,
                 cts_params_ptr->range_max, cts_params_ptr->z_min,
                 cts_params_ptr->z_max);

    // load the occupancy map params.
    //   std::string occ_map_node("occupancy_map.");
    const auto &occ_map_node = cfg_fs["occupancy_map"];
    auto occ_map_params_ptr = gmm_params_ptr->m_occ_map_params_ptr;
    occ_map_params_ptr->resolution = occ_map_node["resolution"];
    occ_map_params_ptr->free_pix_th = occ_map_node["free_pix_th"];
    occ_map_params_ptr->obstacle_pix_th = occ_map_node["obstacle_pix_th"];
    occ_map_params_ptr->grid_map_name = grid_map_name;

    cfg_fs.release();

    // the map cloud, the frame is map2.
    CloudT::Ptr world_map_ptr(new CloudT());
    // Generate the whole map without trim cloud.
    for (int idx = 0; idx != cloud_cnt; ++idx) {
        // load the lidar cloud and transform it to the base frame.
        std::string cloud_file = cloud_path + std::to_string(idx) + ".ply";
        pcl::PointCloud<PointType>::Ptr lidar_cloud_ptr(
                new pcl::PointCloud<PointType>());
        pcl::io::loadPLYFile(cloud_file, *lidar_cloud_ptr);
        pcl::PointCloud<PointType>::Ptr base_cloud_ptr(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*lidar_cloud_ptr, *base_cloud_ptr, tf_base_lidar);

        // load the tf_map1_lidar.
        std::string node_name = cloud_prefix + std::to_string(idx);
        cv::FileNode float_array_node = cloud_fs[node_name];
        std::vector<float> xyzrpy_vec;
        if (false == float_array_node.isNone() &&
            true == float_array_node.isSeq()) {
            for (cv::FileNodeIterator it = float_array_node.begin();
                 it != float_array_node.end(); ++it) {
                xyzrpy_vec.push_back(static_cast<float>(*it));
            }
        } else {
            ROB_LOG_INFO("Fail to load the pose of point cloud.");
            break;
        }
        Eigen::Affine3f cur_trans = pcl::getTransformation(
                xyzrpy_vec.at(0), xyzrpy_vec.at(1), xyzrpy_vec.at(2), xyzrpy_vec.at(3),
                xyzrpy_vec.at(4), xyzrpy_vec.at(5));
        Eigen::Matrix4f tf_map1_lidar = cur_trans.matrix();
        Eigen::Matrix4f tf_map2_base = tf_base_lidar * tf_map1_lidar * tf_lidar_base;
        pcl::PointCloud<PointType>::Ptr map2_cloud_ptr(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*base_cloud_ptr, *map2_cloud_ptr, tf_map2_base);
        *world_map_ptr += *map2_cloud_ptr;
    }

    cv::Point2f min_pnt, max_pnt;
    {
        const auto &cloud = world_map_ptr->points;
        ROB_LOG_DEBUG("world map cloud size = %d.", cloud.size());
        std::vector<float> x_ls, y_ls;
        for (const auto &ele : cloud) {
            x_ls.push_back(ele.x);
            y_ls.push_back(ele.y);
        }
        auto [x_min_it, x_max_it] = std::minmax_element(x_ls.begin(), x_ls.end());
        auto [y_min_it, y_max_it] = std::minmax_element(y_ls.begin(), y_ls.end());
        min_pnt = cv::Point2f(*x_min_it, *y_min_it);
        max_pnt = cv::Point2f(*x_max_it, *y_max_it);
    }

    ROB_LOG_INFO("cloud min_pnt=[%f, %f], max_pnt = [%f, %f]", min_pnt.x,
                 min_pnt.y, max_pnt.x, max_pnt.y);
    occ_map_params_ptr->world_min_pnt = min_pnt;
    occ_map_params_ptr->world_max_pnt = max_pnt;
    ROB_LOG_INFO(
            "occ_map_params: resolution = %f, world_min_pnt = [%f, %f], "
            "world_max_pnt = [%f, %f], free_pix_th = %d, obstacle_pix_th = %d, "
            "grid_map_name = %s",
            occ_map_params_ptr->resolution, occ_map_params_ptr->world_min_pnt.x,
            occ_map_params_ptr->world_min_pnt.y, occ_map_params_ptr->world_max_pnt.x,
            occ_map_params_ptr->world_max_pnt.y, occ_map_params_ptr->free_pix_th,
            occ_map_params_ptr->obstacle_pix_th,
            occ_map_params_ptr->grid_map_name.c_str());

    // init the grid map maker.
    std::shared_ptr<GridMapMaker> gmm_ptr =
            std::make_shared<GridMapMaker>(gmm_params_ptr);
    gmm_ptr->init();

    /************************ form the grid map. *******************************/
    for (int idx = 0; idx != cloud_cnt; ++idx) {
        
        ROB_LOG_INFO("Point idx = %d.", idx);
        std::string cloud_file = cloud_path + std::to_string(idx) + ".ply";
        pcl::PointCloud<PointType>::Ptr cur_lidar_cloud_ptr(
                new pcl::PointCloud<PointType>());
        pcl::io::loadPLYFile(cloud_file, *cur_lidar_cloud_ptr);

        ROB_LOG_INFO( "step1: read ply idx = %d ", idx);


        pcl::PointCloud<PointType>::Ptr cur_base_cloud_ptr(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*cur_lidar_cloud_ptr, *cur_base_cloud_ptr, tf_base_lidar);

        cur_base_cloud_ptr = trimPointCloud(cur_base_cloud_ptr, cloud_range);
        ROB_LOG_INFO("step2: trimPointCloud idx = %d",idx);
        // load the pose of the current cloud.
        std::string node_name = cloud_prefix + std::to_string(idx);



        cv::FileNode float_array_node = cloud_fs[node_name];
        ROB_LOG_INFO("step3: load pose idx = %s", node_name.c_str());
        ROB_LOG_INFO("step3: load pose idx = %d", float_array_node.size());
        std::vector<float> xyzrpy_vec;
        if (false == float_array_node.isNone() &&
            true == float_array_node.isSeq()) {
            for (cv::FileNodeIterator it = float_array_node.begin();
                 it != float_array_node.end(); ++it) {
                xyzrpy_vec.push_back(static_cast<float>(*it));
            }
        } else {
            ROB_LOG_INFO("Fail to load the pose of point cloud.");
            break;
        }
        ROB_LOG_INFO("step3: xyzrpy_vec:", xyzrpy_vec.size());
        Eigen::Affine3f cur_trans = pcl::getTransformation(
                xyzrpy_vec.at(0), xyzrpy_vec.at(1), xyzrpy_vec.at(2), xyzrpy_vec.at(3),
                xyzrpy_vec.at(4), xyzrpy_vec.at(5));
        Eigen::Matrix4f tf_map1_lidar = cur_trans.matrix();
        Eigen::Matrix4f tf_map2_base = tf_base_lidar * tf_map1_lidar * tf_lidar_base;
        ROB_LOG_INFO("step4: tf_map2_base:", tf_map2_base);

        auto sub_map_in_base =
                extractSubMapCloud(world_map_ptr, cloud_range, tf_map2_base);
        ROB_LOG_INFO("step5: sub_map_in_base created");


        gmm_ptr->addFrameToGridMap(tf_map2_base, cur_base_cloud_ptr, sub_map_in_base);
        ROB_LOG_INFO("step6: addFrameToGridMap idx = %d", idx);

        // gmm_ptr->addFrameToGridMap(tf_world_lidar, cur_cloud_ptr);

        // if (cloud_cnt / 2 - 1 == idx) {
        //     // FIXME: just for testing.
        //     CloudT::Ptr sub_cloud_ptr =
        //             extractSubMapCloud(world_map_ptr, cloud_range, tf_map2_base);
        //     std::string sub_cloud_file =
        //             std::string("sub_map_in_lidar_") + std::to_string(idx) + ".ply";
        //     pcl::io::savePLYFileASCII(sub_cloud_file, *sub_cloud_ptr);
        // }


    }

    ROB_LOG_INFO("Try to save the grid map.");
    gmm_ptr->saveGridMap(map_save_path);

    return 0;
}