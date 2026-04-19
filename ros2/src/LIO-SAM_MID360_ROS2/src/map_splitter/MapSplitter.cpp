#include "MapSplitter.h"

#include <algorithm>

#include <rob_common/log_config/RobConfig.h>

#include <yaml-cpp/yaml.h>

#include <filesystem>

using namespace rob_common;

namespace std_fs = std::experimental::filesystem;

MapSplitter::MapSplitter(MapSplitterParams::SharedPtr params_ptr)
    : m_params_ptr(params_ptr) {
}
MapSplitter::~MapSplitter() {
}

int MapSplitter::init() {
    return 0;
}

int MapSplitter::splitMap(const PointCloudT::Ptr &cloud_in_ptr) {
    if (cloud_in_ptr->empty()) {
        printf("The point cloud is empty. \n");
        return -1;
    }

    pcl::PointXYZ min_pnt, max_pnt;
    pcl::getMinMax3D(*cloud_in_ptr, min_pnt, max_pnt);
    printf("min_pnt: %f, %f, %f \n", min_pnt.x, min_pnt.y, min_pnt.z);
    printf("max_pnt: %f, %f, %f \n", max_pnt.x, max_pnt.y, max_pnt.z);
    float grid_size = m_params_ptr->grid_sz;
    int grid_num_x = std::ceil((max_pnt.x - min_pnt.x) / grid_size);
    int grid_num_y = std::ceil((max_pnt.y - min_pnt.y) / grid_size);
    int grid_row_cnt = grid_num_x;
    printf("grid_num_x: %d, grid_num_y: %d \n", grid_num_x, grid_num_y);
    m_map_min_pnt.x = min_pnt.x;
    m_map_min_pnt.y = min_pnt.y;
    m_map_max_pnt.x = max_pnt.x;
    m_map_max_pnt.y = max_pnt.y;
    m_grid_col_cnt = grid_num_x;
    m_grid_row_cnt = grid_num_y;
    m_idx_map_map.clear();

    for (const auto &ele_pnt : cloud_in_ptr->points) {
        int grid_col_idx = std::floor((ele_pnt.x - min_pnt.x) / grid_size);
        int grid_row_idx = std::floor((ele_pnt.y - min_pnt.y) / grid_size);
        int cur_grid_idx = grid_row_idx * grid_row_cnt + grid_col_idx;
        auto iter = m_idx_map_map.find(cur_grid_idx);
        if (iter == m_idx_map_map.end()) {
            SubMap sub_map;
            sub_map.map_idx = cur_grid_idx;
            // sub_map.map_cloud_ptr = boost::make_shared<PointCloudT>();
            sub_map.map_cloud_ptr = PointCloudT::Ptr(new PointCloudT());
            sub_map.map_cloud_ptr->push_back(ele_pnt);
            float grid_min_x = min_pnt.x + grid_col_idx * grid_size;
            float grid_min_y = min_pnt.y + grid_row_idx * grid_size;
            sub_map.min_pnt.x = grid_min_x;
            sub_map.min_pnt.y = grid_min_y;
            sub_map.max_pnt.x = grid_min_x + grid_size;
            sub_map.max_pnt.y = grid_min_y + grid_size;
            m_idx_map_map.insert({sub_map.map_idx, sub_map});
        } else {
            iter->second.map_cloud_ptr->push_back(ele_pnt);
        }
    }
    return 0;
}

int MapSplitter::saveMap(const std::string &path) {
    // create the folder of path if the folder does not exist
    if (false == std_fs::exists(path)) {
        printf("The folder %s does not exist, create the folder of path. \n", path.c_str());
        std_fs::create_directories(path);
    }

    // save the info of sub map
    std::string sub_map_info_path = path + "/sub_map_info.yaml";

    YAML::Node root_node;
    YAML::Node min_pnt_node = root_node["map_min_pnt_xy"];
    min_pnt_node.SetStyle(YAML::EmitterStyle::Flow);
    min_pnt_node.push_back(m_map_min_pnt.x);
    min_pnt_node.push_back(m_map_min_pnt.y);
    YAML::Node max_pnt_node = root_node["map_max_pnt_xy"];
    max_pnt_node.SetStyle(YAML::EmitterStyle::Flow);
    max_pnt_node.push_back(m_map_max_pnt.x);
    max_pnt_node.push_back(m_map_max_pnt.y);

    root_node["grid_size"] = m_params_ptr->grid_sz;
    root_node["grid_col_cnt"] = m_grid_col_cnt;
    root_node["grid_row_cnt"] = m_grid_row_cnt;

    YAML::Node sub_map_idx_ls_node = root_node["sub_map_idx_ls"];
    sub_map_idx_ls_node.SetStyle(YAML::EmitterStyle::Flow);
    std::vector<SubMap> sub_map_vec;
    for (auto &ele_pair : m_idx_map_map) {
        sub_map_vec.push_back(ele_pair.second);
    }
    for (const auto &ele : sub_map_vec) {
        sub_map_idx_ls_node.push_back(ele.map_idx);
    }
    std::string sub_map_prefix("sub_map_");
    root_node["sub_map_prefix"] = sub_map_prefix;
    for (const auto &ele_map : sub_map_vec) {
        std::string cur_sub_map_name = sub_map_prefix + std::to_string(ele_map.map_idx);
        std::string sub_map_path = path + "/" + cur_sub_map_name + ".ply";
        printf("Save the sub map to %s. \n", sub_map_path.c_str());
        pcl::io::savePLYFileBinary(sub_map_path, *ele_map.map_cloud_ptr);
        YAML::Node cur_sub_map_node = root_node[cur_sub_map_name.c_str()];
        cur_sub_map_node["sub_map_idx"] = ele_map.map_idx;
        cur_sub_map_node["sub_map_file"] = cur_sub_map_name + ".ply";
        YAML::Node min_pnt_node = cur_sub_map_node["min_pnt_xy"];
        min_pnt_node.SetStyle(YAML::EmitterStyle::Flow);
        min_pnt_node.push_back(ele_map.min_pnt.x);
        min_pnt_node.push_back(ele_map.min_pnt.y);
        YAML::Node max_pnt_node = cur_sub_map_node["max_pnt_xy"];
        max_pnt_node.SetStyle(YAML::EmitterStyle::Flow);
        max_pnt_node.push_back(ele_map.max_pnt.x);
        max_pnt_node.push_back(ele_map.max_pnt.y);
    }
    std::ofstream fout(sub_map_info_path);
    fout << root_node;
    fout.close();
    printf("Save the sub map info to %s. \n", sub_map_info_path.c_str());

    printf("Save the colorful map. \n");
    PointCloudRGBT::Ptr color_cloud_ptr(new PointCloudRGBT());
    std::vector<std::array<std::uint8_t, 3>> color_ls{
            {255, 0, 0},
            {0, 255, 0},
            {0, 0, 255},
            {255, 255, 0},
            {255, 0, 255},
            {0, 255, 255},
            {128, 0, 0},
            {0, 128, 0},
            {0, 0, 128},
            {128, 128, 0}};
    // assign the color.
    for (const auto &ele_map : sub_map_vec) {
        PointCloudRGBT::Ptr cur_color_cloud_ptr(new PointCloudRGBT());
        pcl::copyPointCloud(*(ele_map.map_cloud_ptr), *cur_color_cloud_ptr);
        PointRGBT tmp_pnt;
        const auto &cur_color = color_ls[ele_map.map_idx % color_ls.size()];
        tmp_pnt.r = cur_color[0];
        tmp_pnt.g = cur_color[1];
        tmp_pnt.b = cur_color[2];
        for (auto &ele_pnt : cur_color_cloud_ptr->points) {
            ele_pnt.rgb = tmp_pnt.rgb;
        }
        printf("Merge the -th sub map to color map cloud.\n");
        *color_cloud_ptr += *cur_color_cloud_ptr;
    }
    std::string color_map_path = path + "/color_map.ply";
    printf("Save the colorful map to %s. \n", color_map_path.c_str());
    pcl::io::savePLYFileBinary(color_map_path, *color_cloud_ptr);
    printf("Save map data successfully. \n");


    return 0;
}