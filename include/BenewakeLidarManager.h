#pragma once

#include <iostream>
#include <string>
#include <memory>
#include <queue>
#include <thread>
#include <mutex>
#include <vector>
#include <condition_variable>
#include <atomic>
#include "benewake_lidar_driver.h"
#include "SocketServer.h"
#include "SocketClient.h"
#include "FileManager.h"
#include "Globals.h"
#include "ThreadPool.h"

class BenewakeLidarManager
{
public:
    BenewakeLidarManager(int client_socket,FileManager fileManager, const std::string &ip = "192.168.0.2", int port = 2469);
    ~BenewakeLidarManager();
    bool initialize();
    bool hasLidar() const;
    void start();
    void stop();
    bool isRunning() const;
private:
    void captureLoop();

    std::string lidar_ip;
    int lidar_port;
    int client_socket;

    FileManager fileManager;
    bool lidar_present = false;
    bool save_enabled = false;

    std::shared_ptr<benewake::BenewakeLidar> lidar;
    std::shared_ptr<ThreadPool> pool;
    std::atomic<bool> is_running_{false};// 替代Config::running
    std::thread capture_thread_;// 保存主线程句柄
    std::mutex capture_mutex_;
    std::mutex send_mutex_; // 发送数据的互斥锁，确保线程安全
    std::string save_dir_;
};
