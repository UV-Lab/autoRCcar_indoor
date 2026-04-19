#include "YamlConfigWrapper.h"

#include <experimental/filesystem>

#include "log_config/RobLogger.h"

namespace std_fs = std::experimental::filesystem;

using namespace rob_common;

rob_common::YamlConfigWrapper::YamlConfigWrapper() : m_yaml_node_ptr(nullptr) {}

rob_common::YamlConfigWrapper::~YamlConfigWrapper() {}

int rob_common::YamlConfigWrapper::init(const std::string &file_path) {
    if (false == std_fs::exists(file_path)) {
        ROB_LOG_ERROR("The file {} doesn't exist. Please check !!!", file_path);
        return -1;
    }

    m_yaml_node_ptr = std::make_unique<YAML::Node>();
    int return_num = -2;
    try {
        *m_yaml_node_ptr = YAML::LoadFile(file_path);
        ROB_LOG_INFO("Load the yaml config file {} successfully.", file_path);
        return_num = 0;
    } catch (...) {
        ROB_LOG_WARN("Fail to load the yaml config file {}.", file_path);
        return_num = -2;
    }

    return return_num;
}

std::vector<std::string>
rob_common::YamlConfigWrapper::stringSplit(const std::string &src_str,
                                          const std::string &regex_str) {
    std::regex split_regex(regex_str);
    std::vector<std::string> str_ls(std::sregex_token_iterator(src_str.begin(),
                                                               src_str.end(),
                                                               split_regex, -1),
                                    std::sregex_token_iterator());
    return str_ls;
}

void rob_common::YamlConfigWrapper::setErrCodeVal(int *err_code_ptr, int value) {
    if (nullptr != err_code_ptr) {
        *err_code_ptr = value;
    }
}
