#include "RobLogger.h"

#include <algorithm>
#include <chrono>
#include <deque>
// #include <experimental/filesystem>
#include <map>
#include <numeric>
// #include <thread>

#include <spdlog/async.h>
#include <spdlog/async_logger.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

using namespace rob_common;

std::unique_ptr<RobLogger> RobLogger::s_mc_log_ptr = nullptr;

rob_common::RobLogger::RobLogger() {
}

rob_common::RobLogger::~RobLogger() {
    m_log_rotation_flag = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

int rob_common::RobLogger::initLog(std::shared_ptr<RobLoggerParams> params_ptr) {
    s_mc_log_ptr = std::unique_ptr<RobLogger>(new RobLogger());
    auto rc = s_mc_log_ptr->init(params_ptr);
    return rc;

    // return 0;
}

std::shared_ptr<spdlog::logger> rob_common::RobLogger::createSubLogger(
        const std::string &log_name, const std::string &log_path) {
    std_fs::path log_fs_path(log_path);
    if (true == std_fs::is_directory(log_fs_path)) {
        ROB_LOG_WARN("The {} is not a log file, please check the log path.", log_fs_path.string());
        return nullptr;
    }
    std::string log_dir_path = log_fs_path.parent_path();
    if (false == std_fs::exists(log_dir_path)) {
        auto fs_rc = std_fs::create_directories(log_dir_path);
        if (false == fs_rc) {
            ROB_LOG_WARN("Fail to create the folder: {}.", log_dir_path);
            return nullptr;
        } else {
            ROB_LOG_INFO("Create the log folder {} successfully.", log_dir_path);
        }
    }
    std::shared_ptr<spdlog::logger> new_log_ptr = nullptr;
    try {

        new_log_ptr = spdlog::basic_logger_mt(log_name, log_path);
    } catch (const spdlog::spdlog_ex &ex) {
        ROB_LOG_WARN("Fail to create the logger {}. Error info: {}.", log_path, ex.what());
    }

    return new_log_ptr;
}


int rob_common::RobLogger::init(std::shared_ptr<RobLoggerParams> params_ptr) {
    m_log_rotation_flag = false;
    // check the logger folder exist or not.
    std::string logger_path = params_ptr->log_path;
    std_fs::path log_folder_path(logger_path);
    if (false == std_fs::exists(log_folder_path)) {
        ROB_LOG_INFO("The folder {} doesn't exist. Try to create it.", logger_path);
        auto fs_rc = std_fs::create_directories(logger_path);
        if (false == fs_rc) {
            ROB_LOG_INFO("Fail to create the folder: {}", logger_path);
            return -1;
        } else {
            ROB_LOG_INFO("Create the folder {} successfully.", logger_path);
        }
    } else {
        ROB_LOG_INFO("The folder {} exists.", logger_path);
    }
    std::string log_name = params_ptr->log_path +
                           params_ptr->log_prefix +
                           "_" + getCurrentTimeStr() + ".log";
    ROB_LOG_INFO("The logger is: {}", log_name);
    auto rot_log_ptr = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            log_name,
            1024 * 1024 * params_ptr->single_log_max_size_MB,
            params_ptr->log_max_cnt);
    auto console_log_ptr = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    rot_log_ptr->set_level(params_ptr->log_level);
    console_log_ptr->set_level(params_ptr->log_level);
    spdlog::sinks_init_list sink_ls;
    if (true == params_ptr->is_console_log) {
        sink_ls = spdlog::sinks_init_list{rot_log_ptr, console_log_ptr};
    } else {
        sink_ls = spdlog::sinks_init_list{rot_log_ptr};
    }
    std::shared_ptr<spdlog::logger> tmp_log_ptr = nullptr;
    if (false == params_ptr->is_asyn_log) {
        ROB_LOG_INFO("Create the sync log.");
        tmp_log_ptr = std::make_shared<spdlog::logger>(params_ptr->log_name, sink_ls);
    } else {
        ROB_LOG_INFO("Create the async log.");
        ROB_LOG_INFO("the log_que_size = {}, log_thread_cnt = {}.",
                    params_ptr->log_que_size, params_ptr->log_thread_cnt);
        spdlog::init_thread_pool(params_ptr->log_que_size, params_ptr->log_thread_cnt);
        // m_thread_pool_ptr = std::make_shared<spdlog::details::thread_pool>(params_ptr->log_que_size,
        //                                                                    params_ptr->log_thread_cnt);
        // MC_LOG_INFO("The thread pool queue size is: {}", m_thread_pool_ptr->queue_size());
        tmp_log_ptr = std::make_shared<spdlog::async_logger>(params_ptr->log_name, sink_ls, spdlog::thread_pool());
        // spdlog::register_logger(tmp_log_ptr);
    }
    if (nullptr != tmp_log_ptr) {
        ROB_LOG_INFO("The logger lever is {}.", int(params_ptr->log_level));
        tmp_log_ptr->set_level(params_ptr->log_level);
        // m_global_log_ptr->set_level(spdlog::level::debug);
        tmp_log_ptr->flush_on(params_ptr->log_level);
        tmp_log_ptr->set_pattern("[%Y-%m-%d %H:%M:%S.%e][%t][%s,%!:%#][%^%l%$] %v");
        spdlog::set_default_logger(tmp_log_ptr);
        // ROB_LOG_INFO("The current log lever is {}.", int(spdlog::get_level()));
    }
    m_global_log_ptr = tmp_log_ptr;

    // TODO: process the rotation files.
    auto rot_log_files = [params_ptr, this]() {
        ROB_LOG_INFO("Start the logger rotation thread.");
        std_fs::path log_dir_path(params_ptr->log_path);
        std::uint64_t max_dir_size_B = params_ptr->log_max_cnt * params_ptr->single_log_max_size_MB * 1024 * 1024;
        while (true == m_log_rotation_flag) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            rotateLogger(log_dir_path, max_dir_size_B);
        }
        ROB_LOG_INFO("End the logger rotation thread.");
    };
    m_log_rotation_flag = true;
    m_log_rot_thread = std::thread(rot_log_files);
    m_log_rot_thread.detach();

    return 0;
}

std::string rob_common::RobLogger::getCurrentTimeStr() {
    auto time_now = std::chrono::system_clock::now();
    time_t tt = std::chrono::system_clock::to_time_t(time_now);
    auto time_tm = localtime(&tt);
    char str_time[25] = {0};
    sprintf(str_time, "%d-%02d-%02d_%02d-%02d-%02d", time_tm->tm_year + 1900,
            time_tm->tm_mon + 1, time_tm->tm_mday, time_tm->tm_hour,
            time_tm->tm_min, time_tm->tm_sec);
    std::string time_string(str_time);
    return time_string;
}

int rob_common::RobLogger::rotateLogger(
        const std_fs::path &folder_path,
        const std::uint64_t &max_size_B) {
    if (false == std_fs::is_directory(folder_path)) {
        ROB_LOG_WARN("The {} is not a directory.", folder_path.string());
        return -1;
    }

    using StrIntPair = std::pair<std::string, std::uint64_t>;
    using StrIntPairVec = std::deque<StrIntPair>;
    std::map<std::string, std::uint64_t> file_size_map;
    // std::map<std::string, std::uint64_t> file_time_map;
    // StrIntPairVec file_size_vec;
    StrIntPairVec file_time_vec;
    for (const auto &it : std_fs::directory_iterator(folder_path)) {
        if (true == std_fs::is_directory(it.path())) {
            continue;
        }
        file_size_map.insert({it.path().string(), std_fs::file_size(it)});
        // file_size_vec.push_back({it.path().string(), std_fs::file_size(it)});
        std::uint64_t last_write_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                                   std_fs::last_write_time(it).time_since_epoch())
                                                   .count();
        // file_time_map.insert({it.path().string(), last_write_time_ms});
        file_time_vec.push_back({it.path().string(), last_write_time_ms});
    }

    std::uint64_t dir_size_B = 0;
    for (const auto &it : file_size_map) {
        dir_size_B += it.second;
    }

    if (max_size_B >= dir_size_B) {
        return 0;
    }

    std::sort(file_time_vec.begin(), file_time_vec.end(),
              [](const StrIntPair &v_l, const StrIntPair &v_r) -> bool {
                  return v_l > v_r;
              });
    std::uint64_t cur_dir_size_B = dir_size_B;
    while (cur_dir_size_B > max_size_B) {
        std::string rm_file_name = file_time_vec.front().first;
        std_fs::remove(std_fs::path(rm_file_name));
        cur_dir_size_B -= file_size_map.at(rm_file_name);
    }

    return 0;
}
