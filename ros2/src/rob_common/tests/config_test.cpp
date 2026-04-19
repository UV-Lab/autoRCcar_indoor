// #include <iostream>
#include <string>

#include <log_config/RobConfig.h>
#include <log_config/RobLogger.h>

using namespace rob_common;

int main(int argc, char const *argv[]) {

    // Global config file.
    std::string global_cfg_path("./resources/config_test_file.yaml");
    /**
     * Initalize the global config file, you can get the params from this config 
     * params by calling MCConfig::getNodeValue or MCConfig::getNodeArray. But at 
     * first, you should include the head file: log_config/MCConfig.h
     */
    int rc = RobConfig::initConfig(global_cfg_path, ConfigFileType::YAML);
    if (0 != rc) {
        ROB_LOG_ERROR("Fail to init the cfg file.");
        return -1;
    }

    std::string robot_name = RobConfig::getNodeValue<std::string>("robot_name");
    ROB_LOG_INFO("The robot name: {}.", robot_name);
    rc = 0;
    std::string node_path("custom_db/db_domain");
    std::string db_domain = RobConfig::getNodeValue<std::string>(node_path, &rc);
    if (0 != rc) {
        ROB_LOG_WARN("Fail to get the node of {}.", node_path);
    } else {
        ROB_LOG_INFO("The db_domain = {}.", db_domain);
    }
    auto double_arr = RobConfig::getNodeArray<double>("test_data/double_arr");
    ROB_LOG_INFO("The double arr is {}, {}, {}.",
                double_arr.at(0), double_arr.at(1), double_arr.at(2));


    // Special config file
    /**
     * Of course, in addition to global configuration, you can also read 
     * parameters through specified configuration files
     */
    std::string json_cfg_path("./resources/config_test_file.json");
    auto config_loader_ptr = RobConfig::createJsonFileLoader(json_cfg_path);
    double weight = config_loader_ptr->getNodeValue<double>("weight");
    ROB_LOG_INFO("The weight is {}.", weight);
    std::string node_path_2("real_data");
    rc = 0;
    auto real_data_arr = config_loader_ptr->getNodeArray<double>(node_path_2, &rc);
    if (0 != rc) {
        ROB_LOG_WARN("Fail to load the params of node: {}.", node_path_2);

    } else {
        ROB_LOG_INFO("The real data arr: {}, {}, {}.",
                    real_data_arr.at(0), real_data_arr.at(1), real_data_arr.at(2));
    }
    std::string person_name = config_loader_ptr->getNodeValue<std::string>("person/name");
    ROB_LOG_INFO("The Person name is {}.", person_name);


    return 0;
}
