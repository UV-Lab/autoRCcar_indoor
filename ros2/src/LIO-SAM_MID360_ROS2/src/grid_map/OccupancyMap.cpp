#include "OccupancyMap.h"
#include <fstream>
#include <experimental/filesystem>
#include <iostream>
#include <stdlib.h>

#include "RobLog.h"

using namespace grid_map;

OccupancyMap::OccupancyMap(OccupancyMapParams::SharedPtr params_ptr)
    : m_params_ptr(params_ptr) {

    m_res = m_params_ptr->resolution;
}
OccupancyMap::~OccupancyMap() {}

int OccupancyMap::init() {
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
    m_occupancy_map =
            cv::Mat(map_height, map_width, CV_8UC1, cv::Scalar(m_unknown_grid_value));

    return 0;
}

int OccupancyMap::addLidar2dFrame(ScanFrame::SharedPtr scan_frame_ptr) {
    // Conver the point frome scan frame.
    auto scan_data_ptr = scan_frame_ptr->scan_ptr;
    const auto &tf_world_lidar = scan_frame_ptr->tf_w_l;
    const auto &angle_min = scan_data_ptr->angle_min;
    const auto &angle_max = scan_data_ptr->angle_max;
    const auto &angle_increment = scan_data_ptr->angle_increment;
    const auto &range_data = scan_data_ptr->ranges;
    std::vector<cv::Point2f> world_pnts;
    for (int idx = 0, cnt = range_data.size(); idx != cnt; ++idx) {
        float angle = angle_min + angle_increment * idx;
        float range = range_data.at(idx);
        Eigen::Vector3f lidar_pnt(range * std::cos(angle), range * std::sin(angle),
                                  1.f);
        Eigen::Vector3f world_pnt = tf_world_lidar * lidar_pnt;
        world_pnts.emplace_back(world_pnt.x(), world_pnt.y());
    }
    cv::Point2f lidar_center_pnt(tf_world_lidar(0, 2), tf_world_lidar(1, 2));

    // update the occupancy map.
    const auto &lidar_center_map_pnt = worldToImage(lidar_center_pnt);
    for (const auto &ele_world_pnt : world_pnts) {
        const auto &ele_img_pnt = worldToImage(ele_world_pnt);
        bresenhamFilling(lidar_center_map_pnt, ele_img_pnt);
    }

    return 0;
}

int OccupancyMap::addLidar2dFrame(const cv::Point2f &pnt_w_l,
                                  const std::vector<cv::Point2f> &world_pnts) {
    cv::Point2f lidar_center_pnt = pnt_w_l;

    // update the occupancy map.
    const auto &lidar_center_map_pnt = worldToImage(lidar_center_pnt);
    for (const auto &ele_world_pnt : world_pnts) {
        const auto &ele_img_pnt = worldToImage(ele_world_pnt);
        bresenhamFilling(lidar_center_map_pnt, ele_img_pnt);
    }
    return 0;
}

int OccupancyMap::addWholeObstacleCloud(const std::vector<cv::Point2f> &world_pnts) {
    // generate the obstacle region.
    for (const auto &ele_world_pnt : world_pnts) {
        const auto &ele_map_pnt_float = worldToImage(ele_world_pnt);
        cv::Point2i ele_map_pnt(std::round(ele_map_pnt_float.x), std::round(ele_map_pnt_float.y));
        if (0 <= ele_map_pnt.x && ele_map_pnt.x < m_occupancy_map.cols &&
            0 <= ele_map_pnt.y && ele_map_pnt.y < m_occupancy_map.rows) {
            m_occupancy_map.at<std::uint8_t>(ele_map_pnt.y, ele_map_pnt.x) = m_params_ptr->obstacle_pix_th;
        }
    }

    // // image processing.
    // std::vector<cv::Point2i> img_path;
    // for (const auto &ele : world_path) {
    //     cv::Point2f float_img_pnt = worldToImage(ele);
    //     img_path.push_back(cv::Point2i(std::round(float_img_pnt.x),
    //                                    std::round(float_img_pnt.y)));
    // }

    // int line_width = std::round(0.5f / m_params_ptr->resolution);
    // cv::polylines(m_occupancy_map, img_path, false, cv::Scalar(m_params_ptr->free_pix_th), line_width);


    return 0;
}

cv::Mat OccupancyMap::getOccupancyMap() const { return m_occupancy_map; }
cv::Mat OccupancyMap::getOccupancyMapBlackWhite() const {
    cv::Mat black_white_map(m_occupancy_map.rows, m_occupancy_map.cols, CV_8UC1,
                            cv::Scalar(0));
    black_white_map += (m_occupancy_map == 127) / 255 * 127;
    black_white_map += (m_occupancy_map > 127) / 255 * 255;
    //   black_white_map += (m_occupancy_map < 127) / 255 * 0;
    return black_white_map;
}
int OccupancyMap::saveGridMap(const std::string &file_path) {
    // get the grid map.
    cv::Mat black_white_map = getOccupancyMapBlackWhite();
    // create  the folder and delete the previous one.
    // int unused = std::system((std::string("exec rm -r ") + file_path).c_str());
    // unused = std::system((std::string("exec mkdir -p ") + file_path).c_str());
    if (false == std::experimental::filesystem::exists(file_path)) {
        std::experimental::filesystem::create_directories(file_path);
    }


    // save the image to the folder.
    const auto &grid_map_name = m_params_ptr->grid_map_name;
    std::string image_file = file_path + "/" + grid_map_name + ".pgm";
    cv::imwrite(image_file, black_white_map);

    // Save the info to the folder as the yaml file.
    std::string yaml_file = file_path + "/" + grid_map_name + ".yaml";
    std::ofstream outfile(yaml_file);
    outfile << "image: " << grid_map_name << ".pgm" << std::endl;
    outfile << "resolution: " << m_res << std::endl;
    cv::Point2f w_bl = (cv::Point2f(0.f, 0.f) - m_map_origin_pnt) * m_res;
    // outfile << "origin: [" << m_map_origin_pnt.x << ", " << m_map_origin_pnt.y
    //         << ", 0.0]" << std::endl;
    outfile << "origin: [" << w_bl.x << ", " << w_bl.y << ", 0.0]" << std::endl;
    outfile << "negate: 0" << std::endl;
    outfile << "occupied_thresh: 0.65" << std::endl;
    outfile << "free_thresh: 0.196" << std::endl;
    outfile.close();

    return 0;
}

cv::Point2f OccupancyMap::worldToImage(const cv::Point2f &world_pnt) {
    cv::Point2f map_pnt = world_pnt / m_res + m_map_origin_pnt;
    cv::Point2f img_pnt(map_pnt.x, m_occupancy_map.rows - map_pnt.y);

    return img_pnt;
}

int OccupancyMap::bresenhamFilling(const cv::Point2f &start_img_pnt,
                                   const cv::Point2f &end_img_pnt) {
    cv::Point2i start_pnt(std::floor(start_img_pnt.x),
                          std::floor(start_img_pnt.y));
    cv::Point2i end_pnt(std::floor(end_img_pnt.x), std::floor(end_img_pnt.y));
    int dx = end_pnt.x - start_pnt.x;
    int dy = end_pnt.y - start_pnt.y;

    if (0 == dx && 0 == dy) {
        return 0;
    }

    int ux = dx > 0 ? 1 : -1;
    int uy = dy > 0 ? 1 : -1;

    if (std::abs(dx) > std::abs(dy)) {
        float ratio_y = float(dy) / float(dx);
        for (int inc_x = 0; inc_x != dx; inc_x += ux) {
            int cur_x = start_pnt.x + inc_x;
            int cur_y = std::floor(start_pnt.y + ratio_y * inc_x);
            setPoint(cv::Point2i(cur_x, cur_y), false);
        }
    } else {
        float ratio_x = float(dx) / float(dy);
        for (int inc_y = 0; inc_y != dy; inc_y += uy) {
            int cur_y = start_pnt.y + inc_y;
            int cur_x = std::floor(start_pnt.x + ratio_x * inc_y);
            setPoint(cv::Point2i(cur_x, cur_y), false);
        }
    }

    setPoint(end_pnt, true);

    return 0;
}

void OccupancyMap::setPoint(const cv::Point2i &pnt, bool occupy) {
    const cv::Point2i &grid_pnt = pnt;
    if (grid_pnt.x < 0 || grid_pnt.x > m_occupancy_map.cols - 1 ||
        grid_pnt.y < 0 || grid_pnt.y > m_occupancy_map.rows - 1) {
        return;
    }

    std::uint8_t grid_value =
            m_occupancy_map.at<std::uint8_t>(grid_pnt.y, grid_pnt.x);
    if (true == occupy) {
        // if (m_params_ptr->obstacle_pix_th < grid_value) {
        //     m_occupancy_map.at<std::uint8_t>(grid_pnt.y, grid_pnt.x) -= 1;
        // }
        if (grid_value > 0) {
            m_occupancy_map.at<std::uint8_t>(grid_pnt.y, grid_pnt.x) -= 1;
        }
    } else {
        // if (m_params_ptr->free_pix_th > grid_value) {
        //     m_occupancy_map.at<std::uint8_t>(grid_pnt.y, grid_pnt.x) += 1;
        // }
        if (grid_value < 255) {
            m_occupancy_map.at<std::uint8_t>(grid_pnt.y, grid_pnt.x) += 1;
        }
    }
}