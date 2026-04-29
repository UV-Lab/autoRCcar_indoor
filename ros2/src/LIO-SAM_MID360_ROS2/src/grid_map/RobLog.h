#ifndef F879311D_3FFD_410C_9450_776DACD0B16E
#define F879311D_3FFD_410C_9450_776DACD0B16E

#include <cstdarg>
#include <cstdio>

namespace grid_map {

enum class LogLevel { TRACE,
                      DEBUG,
                      INFO,
                      WARN };

class RobLog {
public:
    static void log(LogLevel level, const char *format, ...) {
        const char *prefix = "";
        switch (level) {
            case LogLevel::TRACE:
                prefix = "[TRACE] ";
                break;
            case LogLevel::DEBUG:
                prefix = "[DEBUG] ";
                break;
            case LogLevel::INFO:
                prefix = "[INFO]  ";
                break;
            case LogLevel::WARN:
                prefix = "[WARN]  ";
                break;
        }

        printf("%s", prefix);

        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);

        printf("\n");
    }
};

}// namespace grid_map

// 日志宏定义
#define ROB_LOG_TRACE(fmt, ...) \
    grid_map::RobLog::log(grid_map::LogLevel::TRACE, fmt, ##__VA_ARGS__)
#define ROB_LOG_DEBUG(fmt, ...) \
    grid_map::RobLog::log(grid_map::LogLevel::DEBUG, fmt, ##__VA_ARGS__)
#define ROB_LOG_INFO(fmt, ...) \
    grid_map::RobLog::log(grid_map::LogLevel::INFO, fmt, ##__VA_ARGS__)
#define ROB_LOG_WARN(fmt, ...) \
    grid_map::RobLog::log(grid_map::LogLevel::WARN, fmt, ##__VA_ARGS__)

#endif /* F879311D_3FFD_410C_9450_776DACD0B16E */