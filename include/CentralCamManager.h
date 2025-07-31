#pragma once
#include "FileManager.h"
#include <mutex>
#include <string>
#include <thread>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <iostream>
class CentralCamManager{
public:
    CentralCamManager(const std::string& device_path, int width, int height, FileManager& file_manager);
    ~CentralCamManager();
    bool init();
    bool startCapture();
    void stopCapture();
    bool isRunning() const;
    bool hasCentralCamera() const;
private:
    void captureLoop();
    std::string generateTimestampFilename();// 生成带时间戳的文件名

    std::string device_path_;
    int width_;
    int height_;
    FileManager file_manager_;

    cv::VideoCapture capture_; //capture_ 是共享资源，应在销毁前确保线程不再访问它
    std::thread capture_thread_; //摄像头采集线程
    bool hasCentralCam; // 是否存在中央摄像头
    std::atomic<bool> is_running_;
    mutable std::mutex mutex_; //线程安全锁
    std::mutex capture_mutex_; //用于摄像头采集的锁，防止多线程访问冲突
};
