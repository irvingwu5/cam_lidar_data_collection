#pragma once
#include "FileManager.h"
#include <mutex>
#include <string>
#include <thread>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <iostream>
#include <memory>
#include "ThreadPool.h" // 假设您的线程池实现在这个头文件

class SideCamManager{
public:
    SideCamManager(const std::string& device_path, int width, int height, FileManager& file_manager);
    ~SideCamManager();
    bool init();
    bool startCapture();
    void stopCapture();
    bool isRunning() const;
    bool hasSideCamera() const;
    bool resetFlag();
private:
    void captureLoop();
    std::string generateTimestampFilename();// 生成带时间戳的文件名
    bool isDeviceAvailable(const std::string& device_path) const;
    std::string device_path_;
    int width_;
    int height_;
    FileManager file_manager_;

    cv::VideoCapture capture_; //capture_ 是共享资源，应在销毁前确保线程不再访问它
    //capture_thread_ (生产者)：它的唯一职责是高速、不间断地从摄像头硬件读取图像帧
    std::thread capture_thread_; //摄像头采集线程
    bool hasSideCam; // 是否存在中央摄像头
    std::atomic<bool> is_running_;
    mutable std::mutex mutex_; //线程安全锁
    std::mutex capture_mutex_; //用于摄像头采集的锁，防止多线程访问冲突
    //pool_ (消费者)：它包含一组后台工作线程，负责执行耗时的文件I/O操作（保存图像和时间戳），而不会阻塞主采集线程
    std::shared_ptr<ThreadPool> pool_; // 添加线程池智能指针
};
