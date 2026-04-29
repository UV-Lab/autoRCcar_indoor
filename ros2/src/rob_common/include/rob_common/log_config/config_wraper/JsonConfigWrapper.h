#ifndef A96514AC_BCA5_4CD0_AF01_93B301B3B77E
#define A96514AC_BCA5_4CD0_AF01_93B301B3B77E

#include <json/json11.hpp>

#include <any>
#include <regex>
#include <type_traits>

// #include "log_config/RobLogger.h"
#include "rob_common/log_config/RobLogger.h"

namespace rob_common {

class JsonConfigWrapper {
public:
    JsonConfigWrapper(/* args */);
    virtual ~JsonConfigWrapper();

    /**
     * @brief Initialize the json config.
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
     * @param err_code_ptr The pointer of error code. 0 -> normal, not 0 -> abnormal.
     * @return DataType The value of the node.
     */
    template<typename DataType>
    DataType getNodeValue(const std::string &node_path, int *err_code_ptr = nullptr);

    /**
     * @brief Get the Node Array.
     * 
     * @tparam DataType The data type of the array.
     * @param node_path The path of the array node.
     * @param err_code The error code: 0 -> normal, not 0 -> abnormal.
     * @return std::vector<DataType> The array of data.
     */
    template<typename DataType>
    std::vector<DataType> getNodeArray(const std::string &node_path,
                                       int *err_code_ptr = nullptr);

    // std::string getNodeStr(const std::string &node_path, int *err_code_ptr = nullptr);

    // std::vector<std::string> getNodeStrVec(const std::string &node_path, int *err_code_ptr = nullptr);

private:
    std::vector<std::string> stringSplit(const std::string &src_str,
                                         const std::string &regex_str);
    void setErrCodeVal(int *err_code_ptr, int value);

    template<typename DataType>
    DataType cvtAnytoDataType(const std::any &any_obj, int err_code = 0, int *err_code_ptr = nullptr);

private:
    std::unique_ptr<json11::Json> m_json_ptr;
};

template<typename DataType>
inline DataType JsonConfigWrapper::getNodeValue(const std::string &node_path, int *err_code_ptr) {
    DataType cur_value = DataType();
    if (false == std::is_same<DataType, bool>::value &&
        false == std::is_arithmetic<DataType>::value &&
        false == std::is_same<DataType, std::string>::value) {
        ROB_LOG_ERROR("The DataType should be one of the [bool, number, string]. Node path: {}. Please check the DataType.", node_path);
        setErrCodeVal(err_code_ptr, -1);
        return cur_value;
    }

    auto str_ls = stringSplit(node_path, "/");
    json11::Json json_node = *m_json_ptr;
    for (const auto &tmp_node : str_ls) {
        json_node = json_node[tmp_node];
    }

    if (true == json_node.is_null()) {
        setErrCodeVal(err_code_ptr, -2);
        ROB_LOG_WARN("The node {} doesn't exist.", node_path);
        return cur_value;
    }

    ROB_LOG_WARN_IF(true == json_node.is_null(), "The json node {} is null.", node_path);
    setErrCodeVal(err_code_ptr, 0);
    std::any json_any_val;
    if (true == std::is_same<DataType, bool>::value) {
        json_any_val = json_node.bool_value();
    } else if (true == std::is_arithmetic<DataType>::value) {
        json_any_val = json_node.number_value();
    } else if (true == std::is_same<DataType, std::string>::value) {
        json_any_val = json_node.string_value();
    }
    DataType tmp_value = cvtAnytoDataType<DataType>(json_any_val, -2, err_code_ptr);
    return tmp_value;
}

template<typename DataType>
inline std::vector<DataType> JsonConfigWrapper::getNodeArray(
        const std::string &node_path, int *err_code_ptr) {
    std::vector<DataType> cur_data_vec;
    if (false == std::is_same<DataType, bool>::value &&
        false == std::is_arithmetic<DataType>::value &&
        false == std::is_same<DataType, std::string>::value) {
        ROB_LOG_ERROR("The DataType should be one of the [bool, number, string]. Node path: {}. Please check the DataType.", node_path);
        setErrCodeVal(err_code_ptr, -1);
        return cur_data_vec;
    }

    auto str_ls = stringSplit(node_path, "/");
    json11::Json json_node = *m_json_ptr;
    for (const auto &tmp_node : str_ls) {
        json_node = json_node[tmp_node];
    }
    if (true == json_node.is_null()) {
        setErrCodeVal(err_code_ptr, -4);
        return cur_data_vec;
    }

    if (false == json_node.is_array()) {
        setErrCodeVal(err_code_ptr, -2);
        ROB_LOG_WARN("The node {} is not an array.", node_path);
        return cur_data_vec;
    }

    const auto &item_ls = json_node.array_items();
    if (true == item_ls.empty()) {
        setErrCodeVal(err_code_ptr, 0);
        ROB_LOG_WARN("The node {} is an empty array.", node_path);
        return cur_data_vec;
    }
    setErrCodeVal(err_code_ptr, 0);
    for (const auto &ele_item : item_ls) {
        std::any tmp_any_val;
        if (true == std::is_same<DataType, bool>::value) {
            tmp_any_val = ele_item.bool_value();
        } else if (true == std::is_arithmetic<DataType>::value) {
            tmp_any_val = ele_item.number_value();
        } else if (true == std::is_same<DataType, std::string>::value) {
            tmp_any_val = ele_item.string_value();
        }
        DataType tmp_val = cvtAnytoDataType<DataType>(tmp_any_val, -3, err_code_ptr);
        if (0 != *err_code_ptr) {
            ROB_LOG_ERROR("Fail to load the element of array node {}.", node_path);
            return cur_data_vec;
        }
        cur_data_vec.push_back(tmp_val);
    }
    return cur_data_vec;
}

template<typename DataType>
inline DataType JsonConfigWrapper::cvtAnytoDataType(const std::any &any_obj, int err_code, int *err_code_ptr) {
    /*
    DataType dst_obj = DataType();
    try {
        dst_obj = std::any_cast<DataType>(any_obj);
        setErrCodeVal(err_code_ptr, 0);
    } catch (const std::bad_any_cast &e) {
        MC_LOG_WARN("Fail to convert the data to DataType, exception: {}.", e.what());
        setErrCodeVal(err_code_ptr, err_code);
    }
    return dst_obj;
    */

    using CurDataType = typename std::conditional<true == std::is_arithmetic<DataType>::value &&
                                                          false == std::is_same<DataType, bool>::value,
                                                  double, DataType>::type;
    CurDataType tmp_obj = CurDataType();
    try {
        tmp_obj = std::any_cast<CurDataType>(any_obj);
        setErrCodeVal(err_code_ptr, 0);
    } catch (const std::bad_any_cast &e) {
        ROB_LOG_WARN("Fail to convert the data to DataType, exception: {}.", e.what());
        setErrCodeVal(err_code_ptr, err_code);
    }
    // using DstDataType = std::conditional<true == std::is_same<DataType, std::string>::value, std::string, DataType>::value;
    // DstDataType dst_obj = static_cast<DstDataType>(tmp_obj);
    DataType dst_obj = static_cast<DataType>(tmp_obj);
    return dst_obj;
}

}// namespace rob_common


#endif /* A96514AC_BCA5_4CD0_AF01_93B301B3B77E */
