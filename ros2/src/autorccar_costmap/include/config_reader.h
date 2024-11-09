#ifndef AUTORCCAR_COSTMAP__CONFIG_READER_H
#define AUTORCCAR_COSTMAP__CONFIG_READER_H

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>
#include <string>

struct Config {
    // costmap config
    double global_resolution;         // costmap resolution [m/cell]
    int global_width;                 // costmap width and height [m]
    int global_height;
    int global_update_per_lidar;
    int local_width;
    int local_height;
    double dbscan_eps;
    int dbscan_min_samples;

    // ros config
    bool publish_global_costmap;
    bool publish_local_costmap;
    bool publish_object_detection;   // publish bounding box if true
    bool visualize_object_detection; // show marker
};

class ConfigReader {
   public:
    ConfigReader(const std::string &config_file_path);

    struct Config GetConfig() const;

   private:
    struct Config config_;
};

#endif
