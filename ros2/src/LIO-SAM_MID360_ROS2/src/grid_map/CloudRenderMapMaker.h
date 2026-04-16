#ifndef B44B48A0_9B3A_4E6B_997B_D3741A8915F0
#define B44B48A0_9B3A_4E6B_997B_D3741A8915F0

#include <memory>

#include <opencv2/opencv.hpp>

#include "Common.h"

namespace grid_map {

class CloudRenderMapMakerParams {
public:
    using SharedPtr = std::shared_ptr<CloudRenderMapMakerParams>;
    float resolution;
    cv::Point2f world_min_pnt;
    cv::Point2f world_max_pnt;
    std::string render_map_name;
};

class CloudRenderMapMaker {
public:
    CloudRenderMapMaker(const CloudRenderMapMakerParams::SharedPtr &params_ptr);

    virtual ~CloudRenderMapMaker() = default;

    int init();
    int addCloud(pcl::PointCloud<PointT>::Ptr lidar_cloud_ptr,
                 const Eigen::Matrix4f &tf_map_lidar);

    int saveRenderMap(const std::string &save_path);

private:
    cv::Point worldToImage(const cv::Point2f &world_pnt);

    cv::Vec3b getRenderHsvColor(const float &cloud_info_val);

    cv::Vec3b hsvToBgr(cv::Vec3f &hsv);


private:
    CloudRenderMapMakerParams::SharedPtr m_params_ptr;
    cv::Mat m_render_rgb_map;
    cv::Mat m_pnt_z_map;
    cv::Point2f m_center_pnt;
    cv::Point2f m_map_origin_pnt;
    float m_res;
};


}// namespace grid_map

#endif /* B44B48A0_9B3A_4E6B_997B_D3741A8915F0 */
