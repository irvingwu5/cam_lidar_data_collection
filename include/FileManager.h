#ifndef FILE_MANAGER_H
#define FILE_MANAGER_H

#include <iostream>
#include <vector>
#include <string>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <regex>

#include "Globals.h"
#include "benewake_lidar_driver.h"
#include <vector>
#include <cmath> 
#include <opencv2/opencv.hpp>
// 定义单个雷达点的结构体（可根据需求扩展字段）
struct RadarPoint {
    float x;        // X 坐标（米）
    float y;        // Y 坐标（米）
    float z;        // Z 坐标（米）
    float intensity;// 回波强度（0-255 或浮点值，取决于传感器）
    float range;    // 到雷达的距离（米，可选，可由 x,y,z 计算）
    float azimuth;  // 方位角（弧度，可选）
    float velocity; // 径向速度（米/秒，可选）

    // 构造函数（方便初始化）
    RadarPoint(float x_, float y_, float z_, float intensity_, 
               float range_ = 0, float azimuth_ = 0, float velocity_ = 0)
        : x(x_), y(y_), z(z_), intensity(intensity_), 
          range(range_), azimuth(azimuth_), velocity(velocity_) {}
};
class FileManager
{
public:
    const std::string root_path = "/root/workspace/save_path/";

    std::string getRootPath();
    std::vector<std::string> getFileChildPaths();
    void setSavePath(std::string name);
    int getPathCount(std::string dir);
    std::string getSavePath();
    std::string get_256_lidar_save_path();
    std::string get_64_lidar_save_path();
    std::string get_central_cam_path(); //中间摄像头
    std::string get_side_cam_path(); //侧方摄像头
    bool createDirectory(const std::string &namefile,bool is_has_root = true);
    bool deleteFile(const std::string &filename);
    bool saveImage(const std::string& full_path, const cv::Mat& image) const;
    bool saveTimestampTxt(const std::string &txt_full_path, const std::string time) const;
    bool is_usb_inserted();
    std::string get_usb_session_folder();
    std::string get_parent_path(const std::string &path);
    bool is_directory(const std::string &path);
    bool create_directory(const std::string &path);
    bool cp_item(const std::string &src, const std::string &dst);
    std::string move_folder_contents(std::string &src_folder, const std::string &dst_folder);

    void savePointCloudAsKITTI(const std::vector<RadarPoint>& cloud, std::string oss) const;

private:
    static std::vector<std::string> get_subdirectories(const std::string &path);
};

#endif // FILE_MANAGER_H
