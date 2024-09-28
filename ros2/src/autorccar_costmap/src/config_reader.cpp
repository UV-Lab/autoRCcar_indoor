#include "config_reader.h"

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

ConfigReader::ConfigReader(const std::string &node_file_path) {
    std::ifstream fin(node_file_path);
    if (!fin) {
        std::cerr << "Failed to open file: " << node_file_path << std::endl;
        exit(-1);
    }
    fin.close();

    YAML::Node node_ = YAML::LoadFile(node_file_path);

    config_.global_resolution = node_["global"]["resolution"].as<double>();
    config_.global_width = node_["global"]["width"].as<int>();
    config_.global_height = node_["global"]["height"].as<int>();
    config_.global_update_per_lidar = node_["global"]["update_every_Nth_lidar"].as<unsigned int>();
    config_.local_width = node_["local"]["width"].as<int>();
    config_.local_height = node_["local"]["height"].as<int>();
    config_.dbscan_eps = node_["object_detection"]["dbscan"]["eps"].as<double>();
    config_.dbscan_min_samples = node_["object_detection"]["dbscan"]["min_samples"].as<int>();

    ros_config_.publish_global_costmap = node_["global"]["publish"].as<bool>();
    ros_config_.publish_local_costmap = node_["local"]["publish"].as<bool>();
    ros_config_.publish_object_detection = node_["object_detection"]["publish"].as<bool>();
    ros_config_.visualize_object_detection = node_["object_detection"]["visualize"].as<bool>();

    std::cout << "Successfully loaded the config" << std::endl;
}

struct Config ConfigReader::GetConfig() const { return config_; }

struct RosConfig ConfigReader::GetRosConfig() const { return ros_config_; }

