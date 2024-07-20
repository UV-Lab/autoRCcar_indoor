#ifndef AUTORCCAR_COSTMAP__COSTMAP_CONFIG_MANAGER_H
#define AUTORCCAR_COSTMAP__COSTMAP_CONFIG_MANAGER_H

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>
#include <string>

class ConfigManager {
   public:
    ConfigManager(const std::string &config_file_path);

    bool getPublishGlobalCostmap() const;
    double getGlobalResolution() const;
    int getGlobalWidth() const;
    int getGlobalHeight() const;
    unsigned int getGlobalUpdatePerLidar() const;

    bool getPublishLocalCostmap() const;
    int getLocalWidth() const;
    int getLocalHeight() const;

    bool getPublishObjectDetection() const;
    bool getVisualizeObjectDetection() const;
    double getDbscanEps() const;
    int getDbscanMinSamples() const;

   private:
    YAML::Node config_;
};

#endif
