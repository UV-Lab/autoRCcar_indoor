#include "RobConfig.h"

using namespace rob_common;


std::unique_ptr<RobConfig> RobConfig::m_config_ptr = nullptr;

rob_common::RobConfig::RobConfig()
    : m_config_type(ConfigFileType::NONE),
      m_json_ptr(nullptr),
      m_yaml_ptr(nullptr) {
    m_file_type_str_map = std::map<ConfigFileType, std::string>(
            {{ConfigFileType::NONE, "NONE"},
             {ConfigFileType::JSON, "JSON"},
             {ConfigFileType::YAML, "YAML"}});
}

int rob_common::RobConfig::init(const std::string &file_path, const ConfigFileType &file_type) {
    int rc = -1;
    m_config_type = file_type;
    if (ConfigFileType::NONE == file_type) {
        ROB_LOG_WARN("The config file type is {}, please the config file type.", m_file_type_str_map.at(file_type));
        return rc;
    }
    if (ConfigFileType::JSON == file_type) {
        ROB_LOG_INFO("The config file type is {}.", m_file_type_str_map.at(file_type));
        m_yaml_ptr.reset();
        m_json_ptr = std::make_unique<JsonConfigWrapper>();
        rc = m_json_ptr->init(file_path);
    }
    if (ConfigFileType::YAML == file_type) {
        ROB_LOG_INFO("The config file type is {}.", m_file_type_str_map.at(file_type));
        m_json_ptr.reset();
        m_yaml_ptr = std::make_unique<YamlConfigWrapper>();
        rc = m_yaml_ptr->init(file_path);
    }
    if (0 != rc) {
        m_json_ptr.reset();
        m_yaml_ptr.reset();
        m_config_type = ConfigFileType::NONE;
    }

    return rc;
}

rob_common::RobConfig::~RobConfig() {
}

int rob_common::RobConfig::initConfig(const std::string &file_path, const ConfigFileType &file_type) {
    m_config_ptr = std::unique_ptr<RobConfig>(new RobConfig());
    auto rc = m_config_ptr->init(file_path, file_type);
    return rc;
}

std::shared_ptr<YamlConfigWrapper> rob_common::RobConfig::createYamlFileLoader(const std::string &file_path) {
    std::shared_ptr<YamlConfigWrapper> yaml_config_ptr = std::make_shared<YamlConfigWrapper>();
    auto rc = yaml_config_ptr->init(file_path);
    if (0 == rc) {
        return yaml_config_ptr;
    } else {
        return nullptr;
    }
}

std::shared_ptr<JsonConfigWrapper> rob_common::RobConfig::createJsonFileLoader(const std::string &file_path) {
    std::shared_ptr<JsonConfigWrapper> json_config_ptr = std::make_shared<JsonConfigWrapper>();
    auto rc = json_config_ptr->init(file_path);
    if (0 == rc) {
        return json_config_ptr;
    } else {
        return nullptr;
    }
}
