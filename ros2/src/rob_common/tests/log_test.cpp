#include <iostream>
#include <memory>

#include <log_config/RobLogger.h>


using namespace rob_common;


int main(int argc, char const *argv[]) {
    // Set the params of logger.
    std::shared_ptr<RobLoggerParams> log_params_ptr =
            std::make_shared<RobLoggerParams>();
    log_params_ptr->log_path = "./log/robot_log/";
    log_params_ptr->log_name = "robot_test_logger";
    log_params_ptr->log_prefix = "robot_test_logger_";
    log_params_ptr->single_log_max_size_MB = 10;
    log_params_ptr->log_max_cnt = 50;
    log_params_ptr->log_level = RobLoggerLevel::debug;
    log_params_ptr->is_asyn_log = true;
    log_params_ptr->log_que_size = 8192;
    log_params_ptr->log_thread_cnt = 4;
    log_params_ptr->is_console_log = true;
    // Initalize the logger.
    int rc = RobLogger::initLog(log_params_ptr);
    if (0 != rc) {
        ROB_LOG_ERROR("Fail to init the Global Logger.");
        return -2;
    }

    ROB_LOG_TRACE("This is the trace log.");
    ROB_LOG_DEBUG("This is the debug log.");
    ROB_LOG_INFO("This is the info log.");
    ROB_LOG_INFO("The logger path is {}.", log_params_ptr->log_path);
    ROB_LOG_WARN("This is the warn log.");
    ROB_LOG_ERROR("This is the error log.");
    ROB_LOG_INFO_IF(true, "The condition is true, so print the log.");


    return 0;
}