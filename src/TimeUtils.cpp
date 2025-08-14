#include "TimeUtils.h"
#include <sstream>
#include <iomanip>
#include <ctime>
#include <chrono>

// 线程安全的本地时间获取（跨平台兼容）
#ifdef _WIN32
#include <windows.h>
#else
#include <ctime>
#endif

std::string TimeUtils::generateTimestampFilename() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()
    ) % 1000000;

    std::stringstream ss;
    struct tm timeinfo;

    // 线程安全的本地时间转换（避免std::localtime的线程不安全问题）
#ifdef _WIN32
    localtime_s(&timeinfo, &in_time_t); // Windows线程安全版本
#else
    localtime_r(&in_time_t, &timeinfo); // Linux线程安全版本
#endif

    ss << std::put_time(&timeinfo, "%Y%m%d_%H%M%S")
       << "_" << std::setw(6) << std::setfill('0') << microseconds.count();
    return ss.str();
}
