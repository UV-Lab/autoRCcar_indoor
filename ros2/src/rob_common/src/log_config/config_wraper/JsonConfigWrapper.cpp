#include "JsonConfigWrapper.h"

#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

#include "log_config/RobLogger.h"

using namespace rob_common;
namespace std_fs = std::experimental::filesystem;

rob_common::JsonConfigWrapper::JsonConfigWrapper()
    : m_json_ptr(nullptr) {
}

rob_common::JsonConfigWrapper::~JsonConfigWrapper() {
}

int rob_common::JsonConfigWrapper::init(const std::string &file_path) {
    // check the file exist or not.
    std_fs::path json_fs_path(file_path);
    if (false == std_fs::exists(json_fs_path)) {
        ROB_LOG_ERROR("The json config file {} doesn't exist. Please check !!!",
                     file_path);
        return -1;
    }

    std::ifstream json_stream(file_path);
    std::stringstream json_buf;
    json_buf << json_stream.rdbuf();
    std::string json_ctx(json_buf.str());
    json_stream.close();
    std::string json_err;
    m_json_ptr = std::make_unique<json11::Json>();
    *m_json_ptr = json11::Json::parse(json_ctx, json_err);
    if (false == json_err.empty()) {
        ROB_LOG_ERROR("Fail to load the json config file {}.", file_path);
        ROB_LOG_ERROR("The error is {}.", json_err);
        return -2;
    } else {
        ROB_LOG_INFO("Load the json config file {} successfully.", file_path);
        ROB_LOG_INFO("The context of {} is: \n {}", file_path, json_ctx);
        return 0;
    }
}

// std::string mc_common::JsonConfigWrapper::getNodeStr(const std::string &node_path, int *err_code_ptr) {
//     std::vector<std::string> node_name_ls = stringSplit(node_path, "/");
//     json11::Json cur_node = *m_json_ptr;
//     for (const auto &node_name : node_name_ls) {
//         cur_node = cur_node[node_name];
//     }
//     if (false == cur_node.is_string()) {
//         setErrCodeVal(err_code_ptr, -1);
//         return std::string();
//     } else {
//         setErrCodeVal(err_code_ptr, 0);
//         std::string cur_node_str = cur_node.string_value();
//         return cur_node_str;
//     }
// }

// std::vector<std::string> mc_common::JsonConfigWrapper::getNodeStrVec(const std::string &node_path, int *err_code_ptr) {
//     std::vector<std::string> node_name_ls = stringSplit(node_path, "/");
//     json11::Json cur_node = *m_json_ptr;
//     for (const auto &node_name : node_name_ls) {
//         cur_node = cur_node[node_name];
//     }
//     setErrCodeVal(err_code_ptr, 0);

//     std::vector<std::string> str_vec;
//     if (false == cur_node.is_array()) {
//         setErrCodeVal(err_code_ptr, -1);
//         return str_vec;
//     } else {
//         const auto &iterm_arr = cur_node.array_items();
//         if (true == iterm_arr.empty()) {
//             return str_vec;
//         } else {
//             if (false == iterm_arr.front().is_string()) {
//                 setErrCodeVal(err_code_ptr, -2);
//                 return str_vec;
//             } else {
//                 setErrCodeVal(err_code_ptr, 0);
//                 for (const auto &ele_iterm : iterm_arr) {
//                     str_vec.push_back(ele_iterm.string_value());
//                 }
//                 return str_vec;
//             }
//         }
//     }
// }

std::vector<std::string> rob_common::JsonConfigWrapper::stringSplit(
        const std::string &src_str, const std::string &regex_str) {
    std::regex split_regex(regex_str);
    std::vector<std::string> str_ls(
            std::sregex_token_iterator(src_str.begin(),
                                       src_str.end(), split_regex, -1),
            std::sregex_token_iterator());
    return str_ls;
}

void rob_common::JsonConfigWrapper::setErrCodeVal(int *err_code_ptr, int value) {
    if (nullptr != err_code_ptr) {
        *err_code_ptr = value;
    }
}
