#ifndef CB4FFD5D_9A4E_4063_B7C7_167039E1CE79
#define CB4FFD5D_9A4E_4063_B7C7_167039E1CE79

#include <regex>
#include <type_traits>

#include <yaml-cpp/yaml.h>

// #include "log_config/RobLogger.h"
#include "rob_common/log_config/RobLogger.h"

namespace rob_common {

class YamlConfigWrapper {
public:
    YamlConfigWrapper(/* args */);
    virtual ~YamlConfigWrapper();

    /**
     * @brief Initialize the yaml config.
     * 
     * @param file_path 
     * @return int 
     */
    int init(const std::string &file_path);

    /**
     * @brief Get the Node Value.
     * 
     * @tparam DataType The data type.
     * @param node_path The path of the node.
     * @param err_code_ptr The error code. 0 -> normal, not 0 -> abnormal.
     * @return DataType The value of the node.
     */
    template<typename DataType>
    DataType getNodeValue(const std::string &node_path, int *err_code_ptr = nullptr);

    /**
     * @brief Get the Node Array.
     * 
     * @tparam DataType The data type of the array.
     * @param node_path The path of the array node.
     * @param err_code_ptr The error code: 0 -> normal, not 0 -> abnormal.
     * @return std::vector<DataType> The array of data.
     */
    template<typename DataType>
    std::vector<DataType> getNodeArray(const std::string &node_path,
                                       int *err_code_ptr = nullptr);

private:
    std::vector<std::string> stringSplit(const std::string &src_str,
                                         const std::string &regex_str);

    void setErrCodeVal(int *err_code_ptr, int value);

private:
    std::unique_ptr<YAML::Node> m_yaml_node_ptr;
};


template<typename DataType>
inline DataType YamlConfigWrapper::getNodeValue(const std::string &node_path,
                                                int *err_code_ptr) {
    DataType cur_data = DataType();
    if (false == std::is_same<DataType, bool>::value &&
        false == std::is_arithmetic<DataType>::value &&
        false == std::is_same<DataType, std::string>::value) {
        ROB_LOG_ERROR("The data type should be one of [bool, num, string]. Please check the data type.");
        // err_code_ptr = -1;
        setErrCodeVal(err_code_ptr, -1);
        return cur_data;
    }

    const auto &node_ls = stringSplit(node_path, "/");
    YAML::Node cur_node = YAML::Clone(*m_yaml_node_ptr);
    for (const auto &node_str : node_ls) {
        cur_node = YAML::Clone(cur_node[node_str]);
    }
    if (true == cur_node.IsNull()) {
        ROB_LOG_WARN("The node of {} is null.", node_path);
        setErrCodeVal(err_code_ptr, -2);
        return cur_data;
    }
    try {
        cur_data = cur_node.as<DataType>();
        setErrCodeVal(err_code_ptr, 0);
    } catch (const YAML::RepresentationException &e) {
        ROB_LOG_WARN("Fail to convert the data type of node {}. What: {}.", node_path, e.what());
        setErrCodeVal(err_code_ptr, -3);
    }
    return cur_data;
}


template<typename DataType>
inline std::vector<DataType> YamlConfigWrapper::getNodeArray(const std::string &node_path,
                                                             int *err_code_ptr) {
    std::vector<DataType> cur_data_vec = std::vector<DataType>();
    if (false == std::is_same<DataType, bool>::value &&
        false == std::is_arithmetic<DataType>::value &&
        false == std::is_same<DataType, std::string>::value) {
        ROB_LOG_ERROR("The data type should be one of [bool, num, string]. Please check the data type.");
        // err_code_ptr = -1;
        setErrCodeVal(err_code_ptr, -1);
        return cur_data_vec;
    }

    const auto &node_ls = stringSplit(node_path, "/");
    YAML::Node cur_node = YAML::Clone(*m_yaml_node_ptr);
    for (const auto &node_str : node_ls) {
        cur_node = YAML::Clone(cur_node[node_str]);
    }
    if (true == cur_node.IsNull()) {
        ROB_LOG_WARN("The node of {} is null.", node_path);
        setErrCodeVal(err_code_ptr, -2);
        return cur_data_vec;
    }
    try {
        cur_data_vec = cur_node.as<std::vector<DataType>>();
        setErrCodeVal(err_code_ptr, 0);
    } catch (const YAML::RepresentationException &e) {
        ROB_LOG_WARN("Fail to convert the data type of node {}. What: {}.", node_path, e.what());
        setErrCodeVal(err_code_ptr, -3);
    }
    // err_code_ptr = 0;
    return cur_data_vec;
}

}// namespace rob_common


#endif /* CB4FFD5D_9A4E_4063_B7C7_167039E1CE79 */
