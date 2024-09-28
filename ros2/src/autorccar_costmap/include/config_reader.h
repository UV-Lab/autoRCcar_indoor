#ifndef AUTORCCAR_COSTMAP__CONFIG_READER_H
#define AUTORCCAR_COSTMAP__CONFIG_READER_H

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>
#include <string>

struct Config {
    double global_resolution;
    int global_width;
    int global_height;
    int global_update_per_lidar;
    int local_width;
    int local_height;
    double dbscan_eps;
    int dbscan_min_samples;
};

struct RosConfig {
    bool publish_global_costmap;
    bool publish_local_costmap;
    bool publish_object_detection;
    bool visualize_object_detection;
};

class ConfigReader {
   public:
    ConfigReader(const std::string &config_file_path);

    struct Config GetConfig() const;
    struct RosConfig GetRosConfig() const;

   private:
    struct Config config_;
    struct RosConfig ros_config_;
};

#endif
