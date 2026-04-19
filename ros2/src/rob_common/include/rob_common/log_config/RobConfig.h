#ifndef B85B5087_210D_4B00_B89A_F7C2076C1E3E
#define B85B5087_210D_4B00_B89A_F7C2076C1E3E

#include <string>

#include "config_wraper/JsonConfigWrapper.h"
#include "config_wraper/YamlConfigWrapper.h"


namespace rob_common {
enum class ConfigFileType {
    NONE = 0,
    YAML = 1,
    JSON = 2
};

class RobConfig {
public:
    ~RobConfig();

    /**
     * @brief Create the global config file.
     * 
     * @param file_path The path of config file.
     * @param file_type The type of config file: yaml or json.
     * @return 0 -> normal, not 0 -> abnormal.
     */
    static int initConfig(const std::string &file_path,
                          const ConfigFileType &file_type);

    /**
     * @brief Create a Yaml File Loader.
     * 
     * @param file_path The path of json file.
     * @return std::shared_ptr<YamlConfigWraper> The pointer of yaml config wrapper. 
     */
    static std::shared_ptr<YamlConfigWrapper> createYamlFileLoader(const std::string &file_path);
    static std::shared_ptr<JsonConfigWrapper> createJsonFileLoader(const std::string &file_path);

    template<typename DataType>
    static DataType getNodeValue(const std::string &node_path, int *err_code_ptr = nullptr);

    template<typename DataType>
    static std::vector<DataType> getNodeArray(const std::string &node_path, int *err_code_ptr = nullptr);

private:
    RobConfig();
    int init(const std::string &file_path, const ConfigFileType &file_type);

private:
    static std::unique_ptr<RobConfig> m_config_ptr;

    ConfigFileType m_config_type;
    std::unique_ptr<JsonConfigWrapper> m_json_ptr;
    std::unique_ptr<YamlConfigWrapper> m_yaml_ptr;
    std::map<ConfigFileType, std::string> m_file_type_str_map;
};


template<typename DataType>
inline DataType RobConfig::getNodeValue(const std::string &node_path, int *err_code_ptr) {
    DataType cur_data;
    switch (m_config_ptr->m_config_type) {
        case ConfigFileType::NONE:
            if (nullptr != err_code_ptr) {
                *err_code_ptr = -1;
            }
            break;
        case ConfigFileType::JSON:
            cur_data = m_config_ptr->m_json_ptr->getNodeValue<DataType>(node_path, err_code_ptr);
            break;
        case ConfigFileType::YAML:
            cur_data = m_config_ptr->m_yaml_ptr->getNodeValue<DataType>(node_path, err_code_ptr);
            break;
    }
    return cur_data;
}


template<typename DataType>
inline std::vector<DataType> RobConfig::getNodeArray(const std::string &node_path, int *err_code_ptr) {
    std::vector<DataType> cur_data_vec;
    switch (m_config_ptr->m_config_type) {
        case ConfigFileType::NONE:
            if (nullptr != err_code_ptr) {
                *err_code_ptr = -1;
            }
            break;
        case ConfigFileType::JSON:
            cur_data_vec = m_config_ptr->m_json_ptr->getNodeArray<DataType>(node_path, err_code_ptr);
            break;
        case ConfigFileType::YAML:
            cur_data_vec = m_config_ptr->m_yaml_ptr->getNodeArray<DataType>(node_path, err_code_ptr);
            break;
    }
    return cur_data_vec;
}

}// namespace rob_common


#endif /* B85B5087_210D_4B00_B89A_F7C2076C1E3E */
