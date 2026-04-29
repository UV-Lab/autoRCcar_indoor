#include "CloudToScan.h"

#include <boost/shared_ptr.hpp>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <numeric>

#include "RobLog.h"

using namespace grid_map;

CloudToScan::CloudToScan(CloudToScanParams::SharedPtr params_ptr)
    : m_params_ptr(params_ptr) {}

CloudToScan::~CloudToScan() {}

// This function maybe useless.
sensor_msgs::msg::LaserScan::Ptr
CloudToScan::convertCloudToScan(CloudT::Ptr cloud_ptr) {
    sensor_msgs::msg::LaserScan::Ptr scan_msg(new sensor_msgs::msg::LaserScan());

    //   scan_msg->header = cloud_ptr->header;
    scan_msg->angle_min = m_params_ptr->angle_min;
    scan_msg->angle_max = m_params_ptr->angle_max;
    scan_msg->angle_increment = m_params_ptr->angle_increment;
    scan_msg->time_increment = m_params_ptr->time_increment;
    scan_msg->scan_time = m_params_ptr->scan_time;
    scan_msg->range_min = m_params_ptr->range_min;
    scan_msg->range_max = m_params_ptr->range_max;

    std::size_t ranges_size = std::ceil(
            (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

    scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::infinity());

    auto vl_cloud_ptr = cloud_ptr;

    for (const auto &ele_pnt : vl_cloud_ptr->points) {
        // printf("cur pnt: [%f, %f, %f]\n", ele_pnt.x, ele_pnt.y, ele_pnt.z);
        if (std::isnan(ele_pnt.x) || std::isnan(ele_pnt.y) ||
            std::isnan(ele_pnt.z)) {
            // std::cout << "nan point\n";
            // ROB_LOG_DEBUG("Nan point");
            continue;
        }

        if (ele_pnt.z < m_params_ptr->z_min || m_params_ptr->z_max < ele_pnt.z) {
            // std::cout << "z out of range\n";
            // ROB_LOG_DEBUG("z oiut range.");
            continue;
        }

        float range = std::hypot(ele_pnt.x, ele_pnt.y);
        if (range < scan_msg->range_min || scan_msg->range_max < range) {
            // std::cout << "range out of range\n";
            // ROB_LOG_DEBUG("Range is out of range.");
            continue;
        }

        float angle = std::atan2(ele_pnt.y, ele_pnt.x);
        if (angle < scan_msg->angle_min || scan_msg->angle_max < angle) {
            // std::cout << "angle out of range\n";
            // ROB_LOG_DEBUG("Angle is out of range.");
            continue;
        }
        std::size_t angle_idx =
                std::round((angle - scan_msg->angle_min) / scan_msg->angle_increment);
        if (range < scan_msg->ranges[angle_idx]) {
            scan_msg->ranges[angle_idx] = range;
        }
    }
    return scan_msg;
}

int CloudToScan::convertCloudToScan(const Eigen::Matrix4f &tf_world_lidar,
                                    const CloudT::Ptr lidar_cloud_ptr,
                                    std::vector<cv::Point2f> &world_pnts) {
    std::vector<float> range_ls;
    std::size_t ranges_size =
            std::ceil((m_params_ptr->angle_max - m_params_ptr->angle_min) /
                      m_params_ptr->angle_increment);

    range_ls.assign(ranges_size, std::numeric_limits<float>::infinity());
    std::vector<cv::Point2f> tmp_world_pnts;
    tmp_world_pnts.assign(ranges_size, cv::Point2f(0.f, 0.f));

    CloudT::Ptr world_cloud_ptr(new CloudT());
    pcl::transformPointCloud(*lidar_cloud_ptr, *world_cloud_ptr, tf_world_lidar);
    world_cloud_ptr->height = 1;
    world_cloud_ptr->width = world_cloud_ptr->size();
    PointT pnt_world_lidar;
    pnt_world_lidar.x = tf_world_lidar(0, 3);
    pnt_world_lidar.y = tf_world_lidar(1, 3);
    pnt_world_lidar.z = tf_world_lidar(2, 3);
    float z_min = pnt_world_lidar.z + m_params_ptr->z_min;
    float z_max = pnt_world_lidar.z + m_params_ptr->z_max;
    std::set<int> valid_idx_set;
    for (const auto &tmp_world_pnt : world_cloud_ptr->points) {
        PointT ele_pnt;
        ele_pnt.x = tmp_world_pnt.x - pnt_world_lidar.x;
        ele_pnt.y = tmp_world_pnt.y - pnt_world_lidar.y;
        ele_pnt.z = tmp_world_pnt.z - pnt_world_lidar.z;

        if (std::isnan(ele_pnt.x) || std::isnan(ele_pnt.y) ||
            std::isnan(ele_pnt.z)) {
            ROB_LOG_DEBUG("Nan point");
            continue;
        }

        if (ele_pnt.z < m_params_ptr->z_min || m_params_ptr->z_max < ele_pnt.z) {
            // std::cout << "z out of range\n";
            // ROB_LOG_DEBUG("z out of range, cur_z = %f, z_min_max= [%f ,%f]",
            //               ele_pnt.z, m_params_ptr->z_min, m_params_ptr->z_max);
            continue;
        }

        float range = std::hypot(ele_pnt.x, ele_pnt.y);
        if (range < m_params_ptr->range_min || m_params_ptr->range_max < range) {
            // std::cout << "range out of range\n";
            // ROB_LOG_DEBUG("Range is out of range.");
            continue;
        }

        float angle = std::atan2(ele_pnt.y, ele_pnt.x);
        if (angle < m_params_ptr->angle_min || m_params_ptr->angle_max < angle) {
            // std::cout << "angle out of range\n";
            // ROB_LOG_DEBUG("Angle is out of range.");
            continue;
        }
        std::size_t angle_idx = std::round((angle - m_params_ptr->angle_min) /
                                           m_params_ptr->angle_increment);
        if (range < range_ls[angle_idx]) {
            range_ls[angle_idx] = range;
            tmp_world_pnts[angle_idx] = cv::Point2f(tmp_world_pnt.x, tmp_world_pnt.y);
            valid_idx_set.insert(angle_idx);
        }
    }

    for (const auto &valid_idx : valid_idx_set) {
        world_pnts.push_back(tmp_world_pnts[valid_idx]);
    }

    return 0;
}