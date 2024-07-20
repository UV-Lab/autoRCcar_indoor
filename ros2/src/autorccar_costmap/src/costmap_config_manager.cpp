#include "costmap_config_manager.h"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>

ConfigManager::ConfigManager(const std::string &config_file_path) {
    std::ifstream fin(config_file_path);
    if (!fin) {
        std::cerr << "Failed to open file: " << config_file_path << std::endl;
        exit(-1);
    }
    fin.close();

    config_ = YAML::LoadFile(config_file_path);

    std::cout << "Successfully loaded the config" << std::endl;
}

bool ConfigManager::getPublishGlobalCostmap() const { return config_["global"]["publish"].as<bool>(); }

double ConfigManager::getGlobalResolution() const { return config_["global"]["resolution"].as<double>(); }

int ConfigManager::getGlobalWidth() const { return config_["global"]["width"].as<int>(); }

int ConfigManager::getGlobalHeight() const { return config_["global"]["height"].as<int>(); }

unsigned int ConfigManager::getGlobalUpdatePerLidar() const {
    return config_["global"]["update_every_Nth_lidar"].as<unsigned int>();
}

bool ConfigManager::getPublishLocalCostmap() const { return config_["local"]["publish"].as<bool>(); }

int ConfigManager::getLocalWidth() const { return config_["local"]["width"].as<int>(); }

int ConfigManager::getLocalHeight() const { return config_["local"]["height"].as<int>(); }

bool ConfigManager::getPublishObjectDetection() const { return config_["object_detection"]["publish"].as<bool>(); }

bool ConfigManager::getVisualizeObjectDetection() const { return config_["object_detection"]["visualize"].as<bool>(); }

double ConfigManager::getDbscanEps() const { return config_["object_detection"]["dbscan"]["eps"].as<double>(); }

int ConfigManager::getDbscanMinSamples() const {
    return config_["object_detection"]["dbscan"]["min_samples"].as<int>();
}
