#include "CloudRenderMapMaker.h"

#include <numeric>

#include <pcl/common/transforms.h>

#include "RobLog.h"

using namespace grid_map;

CloudRenderMapMaker::CloudRenderMapMaker(
        const CloudRenderMapMakerParams::SharedPtr &params_ptr)
    : m_params_ptr(params_ptr) {
    m_res = m_params_ptr->resolution;
}

int CloudRenderMapMaker::init() {
    const cv::Point2f &world_min_pnt = m_params_ptr->world_min_pnt;
    const cv::Point2f &world_max_pnt = m_params_ptr->world_max_pnt;
    int edge_width_pix = 10;
    int map_width = std::ceil((world_max_pnt.x - world_min_pnt.x) / m_res) +
                    2 * edge_width_pix;
    int map_height = std::ceil((world_max_pnt.y - world_min_pnt.y) / m_res) +
                     2 * edge_width_pix;
    cv::Point2f origin_pnt = cv::Point2f(edge_width_pix, edge_width_pix) -
                             world_min_pnt / m_params_ptr->resolution;
    m_map_origin_pnt = origin_pnt;
    ROB_LOG_INFO("map_width = %d, map_height = %d, origin_pnt = [%f, %f]",
                 map_width, map_height, origin_pnt.x, origin_pnt.y);
    m_render_rgb_map =
            cv::Mat(map_height, map_width, CV_8UC3, cv::Scalar(255, 255, 255));
    m_pnt_z_map = cv::Mat(map_height, map_width, CV_64FC1, cv::Scalar(std::numeric_limits<double>::min()));


    return 0;
}
int CloudRenderMapMaker::addCloud(
        pcl::PointCloud<PointT>::Ptr lidar_cloud_ptr,
        const Eigen::Matrix4f &tf_map_lidar) {
    pcl::PointCloud<PointT>::Ptr map_cloud_ptr(new pcl::PointCloud<PointT>());
    pcl::transformPointCloud(*lidar_cloud_ptr, *map_cloud_ptr, tf_map_lidar);
    for (const auto &tmp_cloud_pnt : map_cloud_ptr->points) {
        cv::Point2f world_pnt(tmp_cloud_pnt.x, tmp_cloud_pnt.y);
        cv::Point img_pnt = worldToImage(world_pnt);
        if (img_pnt.x < 0 || img_pnt.x >= m_render_rgb_map.cols ||
            img_pnt.y < 0 || img_pnt.y >= m_render_rgb_map.rows) {
            ROB_LOG_WARN("Point [%f, %f] out of render map.", tmp_cloud_pnt.x, tmp_cloud_pnt.y);
            continue;
        }
        cv::Vec3f hsv_color;
        hsv_color[0] = int(tmp_cloud_pnt.intensity) * 2 % 360;
        hsv_color[1] = 1.f;
        hsv_color[2] = 1.f;
        cv::Vec3b bgr_color = hsvToBgr(hsv_color);
        float cur_z = m_pnt_z_map.at<float>(img_pnt.y, img_pnt.x);
        // if(tmp_cloud_pnt.z>cur_z){
        m_render_rgb_map.at<cv::Vec3b>(img_pnt) = bgr_color;
        m_pnt_z_map.at<float>(img_pnt) = tmp_cloud_pnt.z;
        // }
    }
    return 0;
}

int CloudRenderMapMaker::saveRenderMap(const std::string &save_path) {
    // cv::Mat bgr_render_map;
    // cv::cvtColor(m_render_rgb_map, bgr_render_map, cv::COLOR_HSV2BGR);
    std::string image_file = save_path + "/" + m_params_ptr->render_map_name + ".png";
    cv::imwrite(image_file, m_render_rgb_map);
    ROB_LOG_INFO("save bgr image map to [%s]", image_file.c_str());

    return 0;
}

cv::Point CloudRenderMapMaker::worldToImage(const cv::Point2f &world_pnt) {
    cv::Point2f map_pnt = world_pnt / m_res + m_map_origin_pnt;
    cv::Point2f float_img_pnt(map_pnt.x, m_render_rgb_map.rows - map_pnt.y);
    cv::Point img_pnt(std::round(float_img_pnt.x), std::round(float_img_pnt.y));
    return img_pnt;
}

cv::Vec3b CloudRenderMapMaker::getRenderHsvColor(const float &cloud_info_val) {
    // 定义 Intensity 的范围（可根据实际数据调整）
    // const float &I_min = cloud_info_min_val;
    // const float &I_max = cloud_info_max_val;

    // 将 Intensity 线性映射到 [0, 360]（Dip Direction 范围）
    float mapped_value = cloud_info_val;

    // 截断 mapped_value 确保在 [0, 360] 范围内
    mapped_value = std::max(0.0f, std::min(360.0f, mapped_value));

    // 转换为 HSV 颜色（OpenCV 的 Hue 范围为 [0, 180]，对应 [0, 360] 度）
    float hue = mapped_value / 2.0f;// 缩放到 [0, 180]
    float saturation = 1.0f;        // 全饱和
    float value = 1.0f;             // 最大亮度

    // 创建单像素 HSV 图像
    // cv::Mat hsv(1, 1, CV_8UC3);
    cv::Vec3b hsv_val = cv::Vec3b(static_cast<uchar>(hue * 1.0f),
                                  static_cast<uchar>(saturation * 255.0f),
                                  static_cast<uchar>(value * 255.0f));

    return hsv_val;
}

cv::Vec3b CloudRenderMapMaker::hsvToBgr(cv::Vec3f &hsv) {
    float h = hsv[0];// 色相 [0, 360]
    float s = hsv[1];// 饱和度 [0, 1]
    float v = hsv[2];// 明度 [0, 1]

    // 归一化色相到 [0, 6] 范围
    float h_norm = h / 60.0f;
    // 计算色相区间和余数
    int h_region = static_cast<int>(h_norm);
    float remainder = h_norm - h_region;

    // 计算中间值
    float p = v * (1.0f - s);
    float q = v * (1.0f - s * remainder);
    float t = v * (1.0f - s * (1.0f - remainder));

    // 根据色相区间计算 RGB 值
    float r, g, b;
    switch (h_region) {
        case 0:
            r = v;
            g = t;
            b = p;
            break;
        case 1:
            r = q;
            g = v;
            b = p;
            break;
        case 2:
            r = p;
            g = v;
            b = t;
            break;
        case 3:
            r = p;
            g = q;
            b = v;
            break;
        case 4:
            r = t;
            g = p;
            b = v;
            break;
        case 5:
            r = v;
            g = p;
            b = q;
            break;
        default:
            r = 0;
            g = 0;
            b = 0;
            break;// 默认黑色
    }

    // 将 RGB 值从 [0, 1] 映射到 [0, 255]
    cv::Vec3b bgr;
    bgr[2] = static_cast<uchar>(r * 255);// BGR 顺序：B = bgr[0], G = bgr[1], R = bgr[2]
    bgr[1] = static_cast<uchar>(g * 255);
    bgr[0] = static_cast<uchar>(b * 255);

    return bgr;
}