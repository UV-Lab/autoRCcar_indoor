#ifndef FCF70CAF_CE0B_42F7_9962_59D89D048ADC
#define FCF70CAF_CE0B_42F7_9962_59D89D048ADC
#include <experimental/filesystem>
#include <iostream>
#include <memory>
#include <thread>

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include <spdlog/spdlog.h>

namespace std_fs = std::experimental::filesystem;

#define ROB_LOG(level, ...) SPDLOG_LOGGER_CALL(spdlog::default_logger(), level, ##__VA_ARGS__)
#define ROB_LOG_TRACE(...) ROB_LOG(spdlog::level::trace, ##__VA_ARGS__)
#define ROB_LOG_DEBUG(...) ROB_LOG(spdlog::level::debug, ##__VA_ARGS__)
#define ROB_LOG_INFO(...) ROB_LOG(spdlog::level::info, ##__VA_ARGS__)
#define ROB_LOG_WARN(...) ROB_LOG(spdlog::level::warn, ##__VA_ARGS__)
#define ROB_LOG_ERROR(...) ROB_LOG(spdlog::level::err, ##__VA_ARGS__)
#define ROB_LOG_CRITICAL(...) ROB_LOG(spdlog::level::critical, ##__VA_ARGS__)

// Conditional log, if the cond is true, print the log, else ignore the log.
#define ROB_LOG_IF(cond, level, ...)   \
    if (true == (cond)) {              \
        ROB_LOG(level, ##__VA_ARGS__); \
    }
#define ROB_LOG_TRACE_IF(cond, ...) ROB_LOG_IF(cond, spdlog::level::trace, ##__VA_ARGS__)
#define ROB_LOG_DEBUG_IF(cond, ...) ROB_LOG_IF(cond, spdlog::level::debug, ##__VA_ARGS__)
#define ROB_LOG_INFO_IF(cond, ...) ROB_LOG_IF(cond, spdlog::level::info, ##__VA_ARGS__)
#define ROB_LOG_WARN_IF(cond, ...) ROB_LOG_IF(cond, spdlog::level::warn, ##__VA_ARGS__)
#define ROB_LOG_ERROR_IF(cond, ...) ROB_LOG_IF(cond, spdlog::level::err, ##__VA_ARGS__)
#define ROB_LOG_CRITICAL_IF(cond, ...) ROB_LOG_IF(cond, spdlog::level::critical, ##__VA_ARGS__)

// TODO: add the conditional logger here.
// # define MC_LOG_TRACE_IF(...)


namespace rob_common {
using RobLoggerLevel = spdlog::level::level_enum;
class RobLoggerParams {
public:
    // The logger folder path.
    std::string log_path;
    // The logger name
    std::string log_name;
    // The logger name prefix
    std::string log_prefix;
    // The max size of single logger.
    int single_log_max_size_MB;
    // The max count of loggers in logger directory.
    int log_max_cnt;
    // The logger level.
    spdlog::level::level_enum log_level;
    // The logger is async (True) or sync (False).
    bool is_asyn_log = true;
    // In async logger, the queue size.
    int log_que_size = 8192;
    // In async logger, the count of threas in thread-pool.
    int log_thread_cnt = 4;
    // Print the log in console or not.
    bool is_console_log = false;
};

class RobLogger {
public:
    ~RobLogger();

    /**
     * @brief Initialize the global logger.
     * 
     * @param params_ptr The params of logger.
     * @return int 0 -> normal,  others -> abnormal.
     */
    static int initLog(std::shared_ptr<RobLoggerParams> params_ptr);

    /**
     * @brief Create a Sub Logger
     * 
     * @param log_name The logger name.
     * @param log_path The full path of logger file.
     * @return std::shared_ptr<spdlog::logger> The logger object.
     */
    static std::shared_ptr<spdlog::logger> createSubLogger(const std::string &log_name,
                                                           const std::string &log_path);


private:
    RobLogger(/* args */);

    int init(std::shared_ptr<RobLoggerParams> params_ptr);
    std::string getCurrentTimeStr();
    int rotateLogger(const std_fs::path &folder_path,
                     const std::uint64_t &max_size_B);

private:
    static std::unique_ptr<RobLogger> s_mc_log_ptr;

    std::shared_ptr<spdlog::logger> m_global_log_ptr;
    std::atomic_bool m_log_rotation_flag;
    std::thread m_log_rot_thread;
    // std::shared_ptr<spdlog::details::thread_pool> m_thread_pool_ptr;
};

}// namespace rob_common

#endif /* FCF70CAF_CE0B_42F7_9962_59D89D048ADC */
