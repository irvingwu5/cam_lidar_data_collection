//
// Created by wxy on 8/11/25.
//

#ifndef TIMEUTILS_H
#define TIMEUTILS_H

#include <string>
#include <chrono>

namespace TimeUtils {
    /**
     * 生成线程安全的时间戳字符串（格式：YYYYMMDD_HHMMSS_uuuuuu）
     */
    std::string generateTimestampFilename();
}
#endif //TIMEUTILS_H
