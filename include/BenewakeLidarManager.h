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

private:
    std::string lidar_ip;
    int lidar_port;
    std::string dir;
    int client_socket;
    std::shared_ptr<benewake::BenewakeLidar> lidar;
    std::shared_ptr<ThreadPool> pool;
    FileManager fileManager;
    bool lidar_present = false;
    bool save_enabled = false;

    std::atomic<bool> is_running_{false};// 替代Config::running
    std::thread main_thread_;// 保存主线程句柄
    std::string generateTimestampFilename();// 生成带时间戳的文件名

    void main_loop();
};
