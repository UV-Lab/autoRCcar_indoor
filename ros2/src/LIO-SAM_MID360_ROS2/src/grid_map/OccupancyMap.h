#ifndef A9D3FC74_29E9_4834_A148_7B3621BD6900
#define A9D3FC74_29E9_4834_A148_7B3621BD6900

#include <memory>

#include <opencv2/opencv.hpp>

//#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "Common.h"

namespace grid_map {

struct ScanFrame {
    using SharedPtr = std::shared_ptr<ScanFrame>;
    sensor_msgs::msg::LaserScan::Ptr scan_ptr;
    Eigen::Matrix3f tf_w_l;
};

class OccupancyMapParams {
public:
    using SharedPtr = std::shared_ptr<OccupancyMapParams>;
    // The resolution of grid map, unit: meter/grid.
    float resolution;
    cv::Point2f world_min_pnt;
    cv::Point2f world_max_pnt;
    std::string grid_map_name;

    int free_pix_th = 137;
    int obstacle_pix_th = 117;

    // size_t map_width;
    // size_t map_height;
    // // The origin point in Map.
    // cv::Point2f origin_pnt;
};

class OccupancyMap {
public:
    OccupancyMap(OccupancyMapParams::SharedPtr params_ptr);
    virtual ~OccupancyMap();
    int init();

    // 通过逐个逐个添加激光帧的方式生成GridMap.
    int addLidar2dFrame(ScanFrame::SharedPtr scan_frame_ptr);
    int addLidar2dFrame(const cv::Point2f &pnt_w_l,
                        const std::vector<cv::Point2f> &world_pnts);

    // 通过添加整个障碍物地图的方式来生成GridMap. 整个生成与逐帧生成这两种方式只能二选一。
    int addWholeObstacleCloud(const std::vector<cv::Point2f> &world_pnts);

    cv::Mat getOccupancyMap() const;
    cv::Mat getOccupancyMapBlackWhite() const;
    int saveGridMap(const std::string &file_path);

private:
    cv::Point2f worldToImage(const cv::Point2f &world_pnt);
    int bresenhamFilling(const cv::Point2f &start_img_pnt,
                         const cv::Point2f &end_img_pnt);
    void setPoint(const cv::Point2i &pnt, bool occupy);

private:
    OccupancyMapParams::SharedPtr m_params_ptr = nullptr;
    cv::Mat m_occupancy_map;
    cv::Point2f m_center_pnt;
    const int m_unknown_grid_value = 127;
    //   The resolution of grid map, unit: meter/grid.
    float m_res;
    cv::Point2f m_map_origin_pnt;
};

}// namespace grid_map

#endif /* A9D3FC74_29E9_4834_A148_7B3621BD6900 */
