#pragma once

#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "ILidarDevice.h"
#include "ILidarAlgo.h"
#include "FileManager.h"
#include "ThreadPool.h"
#include <vector>

using namespace tanway;
class TanwayLidarManager : public ILidarObserver
{
public:
    TanwayLidarManager(int client_socket,FileManager fileManager,
                       const std::string &local_ip="192.168.111.204",
                       const std::string &lidar_ip="192.168.111.51",
                       const std::string &algo_config_path="/root/workspace/camera_lidar_driver/tanwaylidar/config/algo_table.json");

    ~TanwayLidarManager();

    bool initialize();
    void start();
    void stop();
    bool hasLidar() const;

protected:
    // Callbacks from ILidarObserver
    virtual void OnPointCloud(const LidarInfo &info, const UserPointCloud &cloud) override;
    virtual void OnException(const LidarInfo &info, const Exception &e) override;
    virtual void OnIMU(const LidarInfo &lidarInfo, const IMUData &imu) override;
    virtual void OnDeviceInfoFrame(const LidarInfo &info, const DeviceInfoFrame &deviceInfoFrame) override;
    virtual void OnParsePcapProcess(const LidarInfo &info, int process, uint64_t frame, uint64_t stamp) override;


private:
    std::string local_ip;
    std::string lidar_ip;
    std::string algo_config_path;
    std::string save_dir;
    int client_socket;
    int cur_frame;
    std::shared_ptr<ILidarDevice> lidar;
    std::shared_ptr<ILidarAlgo> algo;
    std::shared_ptr<ThreadPool> thread_pool;
    FileManager fileManager;

    bool lidar_ready = false;
};
