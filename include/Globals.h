#ifndef CONFIG_H
#define CONFIG_H

#include <string>

class Config {
public:
    static std::string select_path;
    // 1:保存256线位置
    // 2:保存64线
    // 3：同时保存
    static int save_type;
    static std::string lidar_64_path;
    static std::string lidar_256_path;
    static std::string central_cam_path; //中间摄像头
    static std::string side_cam_path;
    static std::string usb_path;
    static bool running;
    static bool stopRun;
};

#endif
