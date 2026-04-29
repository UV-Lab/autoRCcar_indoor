#include <chrono>
#include <fstream>
#include <experimental/filesystem>
#include <iostream>
#include <thread>


// #include "utility.h"
// #include <livox_ros_driver2/CustomMsg.h>
// #include <ros/ros.h>
// #include <rosbag/bag.h>
// #include <rosbag/view.h>


#include <pcl/common/angles.h>
#include <pcl/io/ply_io.h>

#include <Eigen/Eigen>

// #include <grid_map/Common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>


#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "grid_map/GridMapMaker.h"
#include "grid_map/GroundSegment.h"

#include "grid_map/CloudToScan.h"

#include "grid_map/RobLog.h"

#include "grid_map/CloudRenderMapMaker.h"
#include "grid_map/Common.h"
#include "grid_map/GridMapMaker.h"


using namespace grid_map;

int main(int argc, char *argv[]) {


    std::cout
            << "cmd: transform_global_map cloud_path cfg_file tf_map_file map_save_path"
            << std::endl;
    if (5 != argc) {
        ROB_LOG_WARN("Please input the correct cmd.");
        return -1;
    }

    std::string cloud_path = std::string(argv[1]) + "/";
    std::string cfg_file = std::string(argv[2]);
    std::string tf_map_file = std::string(argv[3]);
    std::string map_save_path = std::string(argv[4]) + "/";

    // 检测map_save_path是否存在，若不存在，创建之
    if (false == std::experimental::filesystem::exists(map_save_path)) {
        std::experimental::filesystem::create_directory(map_save_path);
    }

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
    std::string stamp_prefix = cloud_fs["timestamp_prefix"];

    std::string tf_map_new_old_mat_file_path = tf_map_file;
    Eigen::Matrix4f tf_map_new_old_mat = Eigen::Matrix4f::Zero();
    std::cout << "tf_map_new_old_mat_file_path: " << tf_map_new_old_mat_file_path << std::endl;
    // 打开文件
    std::ifstream file(tf_map_new_old_mat_file_path);
    if (!file.is_open()) {
        std::cerr << "Fail to open the file: " << tf_map_new_old_mat_file_path << std::endl;
        return 1;
    }
    // 逐行读取文件内容并赋值到矩阵
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            float tmp_ele_val;
            if (file >> tmp_ele_val) {
                tf_map_new_old_mat(i, j) = tmp_ele_val;
            } else {
                std::cout << "Fail to read the matrix element at row: " << i << ", col: " << j << std::endl;
            }
        }
    }
    // 关闭文件
    file.close();
    // 输出矩阵以验证
    std::cout << "Read the matrix from file: " << tf_map_new_old_mat_file_path << ":\n"
              << tf_map_new_old_mat << std::endl;


    Eigen::Matrix4f tf_mt_mo = tf_map_new_old_mat;
    // cv::cv2eigen(cvmat_mt_mo, tf_mt_mo);
    Eigen::Matrix4f tf_mo_mt = tf_mt_mo.inverse();
    std::cout << "tf_mt_mo: \n"
              << tf_mt_mo << std::endl;

    Eigen::Quaternionf quat_vl_l(1, 0, 0, 0);
    std::string quat_vl_l_node_name = "quat_vl_l_wxyz";
    cv::FileNode quat_vl_l_wxyz_node = cloud_fs[quat_vl_l_node_name];
    std::vector<float> quat_vl_l_wxyz_vec;
    if (false == quat_vl_l_wxyz_node.isNone() &&
        true == quat_vl_l_wxyz_node.isSeq()) {
        for (cv::FileNodeIterator it = quat_vl_l_wxyz_node.begin();
             it != quat_vl_l_wxyz_node.end(); ++it) {
            quat_vl_l_wxyz_vec.push_back(static_cast<float>(*it));
        }
    } else {
        ROB_LOG_WARN("Fail to load the quat_vl_l_wxyz.");
        return -3;
    }
    quat_vl_l = Eigen::Quaternionf(quat_vl_l_wxyz_vec.at(0),
                                   quat_vl_l_wxyz_vec.at(1),
                                   quat_vl_l_wxyz_vec.at(2),
                                   quat_vl_l_wxyz_vec.at(3));
    Eigen::Matrix3f rot_vl_l = quat_vl_l.toRotationMatrix();
    Eigen::Matrix4f tf_vl_l = Eigen::Matrix4f::Identity();
    tf_vl_l.block<3, 3>(0, 0) = rot_vl_l;


    // Create the transformed global map.
    CloudT::Ptr world_map_ptr(new CloudT());
    for (int idx = 0; idx != cloud_cnt; ++idx) {
        // load the lidar cloud and transform it to the base frame.
        std::string cloud_file = cloud_path + std::to_string(idx) + ".ply";
        pcl::PointCloud<PointT>::Ptr vl_cloud_ptr(
                new pcl::PointCloud<PointT>());
        pcl::io::loadPLYFile(cloud_file, *vl_cloud_ptr);

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
        Eigen::Matrix4f tf_map_vl = cur_trans.matrix();
        Eigen::Matrix4f tf_mt_vl = tf_mt_mo * tf_map_vl;
        // Eigen::Matrix4f tf_map2_base = tf_base_lidar * tf_map1_lidar * tf_lidar_base;
        pcl::PointCloud<PointT>::Ptr mt_cloud_ptr(new pcl::PointCloud<PointT>());
        // pcl::transformPointCloud(*vl_cloud_ptr, *map2_cloud_ptr, tf_map_vl);
        pcl::transformPointCloud(*vl_cloud_ptr, *mt_cloud_ptr, tf_mt_vl);
        *world_map_ptr += *mt_cloud_ptr;
    }
    ROB_LOG_INFO("Save the global map!");
    std::string whole_map_path = map_save_path + "GlobalMap.ply";
    pcl::io::savePLYFileBinary(whole_map_path, *world_map_ptr);
    ROB_LOG_INFO("save the global map = %s", whole_map_path.c_str());

    // transfor the point cloud of keyframe.
    ROB_LOG_INFO("Try to transform the keyframe cloud and save it's pose to yaml.");
    std::string point_cloud_folder = map_save_path + "/pointCloud/";
    if (false == std::experimental::filesystem::exists(point_cloud_folder)) {
        std::experimental::filesystem::create_directory(point_cloud_folder);
    }

    // create the yaml file.
    std::string yaml_name = point_cloud_folder + "cloud.yaml";
    ROB_LOG_INFO("cloud yame file: %s", yaml_name.c_str());
    cv::FileStorage cloud_yaml_cfg(yaml_name, cv::FileStorage::WRITE);
    cloud_yaml_cfg << "cloud_cnt" << cloud_cnt;
    cloud_yaml_cfg << "cloud_prefix" << cloud_prefix;
    cloud_yaml_cfg << "pose_order"
                   << "x y z r p y";
    Eigen::Quaternionf quat_vl2_l(1, 0, 0, 0);
    Eigen::Matrix4f tf_vl2_l = Eigen::Matrix4f::Identity();
    for (int idx = 0; idx != cloud_cnt; ++idx) {
        // load the lidar cloud and transform it to the base frame.
        std::string cloud_file = cloud_path + std::to_string(idx) + ".ply";
        pcl::PointCloud<PointT>::Ptr vl_cloud_ptr(
                new pcl::PointCloud<PointT>());
        pcl::io::loadPLYFile(cloud_file, *vl_cloud_ptr);

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
        Eigen::Matrix4f tf_mo_vl = cur_trans.matrix();
        Eigen::Matrix4f tf_mt_vl = tf_mt_mo * tf_mo_vl;

        Eigen::Matrix4f tf_vl2_vl = Eigen::Matrix4f::Identity();
        if (0 == idx) {
            Eigen::Matrix3f rot_mt_vl = tf_mt_vl.block(0, 0, 3, 3);
            Eigen::Vector3f mt_z_vl2(0.f, 0.f, 1.f);
            Eigen::Vector3f mt_x_vl = rot_mt_vl.col(0);
            Eigen::Vector3f mt_y_vl2 = mt_z_vl2.cross(mt_x_vl).normalized();
            Eigen::Vector3f mt_x_vl2 = mt_y_vl2.cross(mt_z_vl2).normalized();
            Eigen::Matrix3f rot_mt_vl2;
            rot_mt_vl2.col(0) = mt_x_vl2;
            rot_mt_vl2.col(1) = mt_y_vl2;
            rot_mt_vl2.col(2) = mt_z_vl2;
            Eigen::Matrix3f rot_vl2_vl = rot_mt_vl2.transpose() * rot_mt_vl;
            // Eigen::Matrix4f tf_vl2_vl = Eigen::Matrix4f::Identity();
            tf_vl2_vl.block(0, 0, 3, 3) = rot_vl2_vl;
            tf_vl2_l = tf_vl2_vl * tf_vl_l;
            Eigen::Matrix3f rot_vl2_l = tf_vl2_l.block(0, 0, 3, 3);
            Eigen::Quaternionf quat_vl2_l = Eigen::Quaternionf(rot_vl2_l).normalized();
            ROB_LOG_INFO("Save the tf_vl2_l to yaml file.");
            cloud_yaml_cfg << quat_vl_l_node_name << "[" << quat_vl2_l.w() << quat_vl2_l.x() << quat_vl2_l.y() << quat_vl2_l.z() << "]";

        } else {
            tf_vl2_vl = tf_vl2_l * tf_vl_l.inverse();
        }
        // Eigen::Matrix4f tf_map2_base = tf_base_lidar * tf_map1_lidar * tf_lidar_base;
        pcl::PointCloud<PointT>::Ptr vl2_cloud_ptr(new pcl::PointCloud<PointT>());
        // pcl::transformPointCloud(*vl_cloud_ptr, *map2_cloud_ptr, tf_map_vl);
        pcl::transformPointCloud(*vl_cloud_ptr, *vl2_cloud_ptr, tf_vl2_vl);
        Eigen::Matrix4f tf_mt_vl2 = tf_mt_vl * tf_vl2_vl.inverse();
        // save the point cloud
        std::string pc_file = point_cloud_folder + std::to_string(idx) + ".ply";
        pcl::io::savePLYFile(pc_file, *vl2_cloud_ptr);
        ROB_LOG_INFO("save point cloud %s finished.", pc_file.c_str());
        // convert the tf_mt_vl2 to x y z roll pitch yaw
        Eigen::Affine3f aff_mt_vl2(tf_mt_vl2);
        float x = 0.f, y = 0.f, z = 0.f;
        float roll = 0.f, pitch = 0.f, yaw = 0.f;
        pcl::getTranslationAndEulerAngles(aff_mt_vl2, x, y, z, roll, pitch, yaw);
        cloud_yaml_cfg << node_name.c_str() << "[" << x << y << z << roll << pitch << yaw << "]";
    }
    cloud_yaml_cfg.release();
    cfg_fs.release();

    // save the tf_map_new_old_mat as txt file.
    std::string tf_mat_txt_file = map_save_path + "/" + "tf_map_new_old_mat.txt";
    std::ofstream tf_mat_txt_file_stream(tf_mat_txt_file);
    if (false == tf_mat_txt_file_stream.is_open()) {
        printf("Fail to open the file: %s \n", tf_mat_txt_file.c_str());
        return -1;
    }
    std::stringstream mat_ss;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            mat_ss << tf_map_new_old_mat(i, j) << " ";
        }
        mat_ss << std::endl;
    }
    tf_mat_txt_file_stream << mat_ss.str() << std::endl;
    tf_mat_txt_file_stream.close();


    return 0;
}