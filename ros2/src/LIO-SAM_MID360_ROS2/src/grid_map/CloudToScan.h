#ifndef A845E923_F128_4BA7_B699_E6BE991691B1
#define A845E923_F128_4BA7_B699_E6BE991691B1

#include <memory>

#include <opencv2/opencv.hpp>
//#include <sensor_msgs/LaserScan.h>

#include <sensor_msgs/msg/laser_scan.hpp>

#include "Common.h"

namespace grid_map {

class CloudToScanParams {
public:
    using SharedPtr = std::shared_ptr<CloudToScanParams>;
    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float scan_time;
    float range_min;
    float range_max;

    float z_min;
    float z_max;
};

class CloudToScan {
public:
    CloudToScan(CloudToScanParams::SharedPtr params_ptr);

    virtual ~CloudToScan();

    sensor_msgs::msg::LaserScan::Ptr convertCloudToScan(CloudT::Ptr cloud_ptr);

    /**
   * @brief Convert the cloud to scan, the scan is in the world frame.
   *
   * @param tf_world_lidar transform from lidar to world.
   * @param lidar_cloud_ptr the cloud data in lidar frame.
   * @param world_pnts The 2d point list in world (2d) frame.
   * @return int 0-> ok.
   */
    int convertCloudToScan(const Eigen::Matrix4f &tf_world_lidar,
                           const CloudT::Ptr lidar_cloud_ptr,
                           std::vector<cv::Point2f> &world_pnts);

private:
    CloudToScanParams::SharedPtr m_params_ptr = nullptr;
};

}// namespace grid_map

#endif /* A845E923_F128_4BA7_B699_E6BE991691B1 */
