#include "BenewakeLidarManager.h"
#include <chrono>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "TimeUtils.h"
BenewakeLidarManager::BenewakeLidarManager(int client_socket,FileManager fileManager, const std::string &ip, int port)
    : lidar_ip(ip), lidar_port(port), save_enabled(false), fileManager(fileManager),client_socket(client_socket),
    is_running_(false), lidar_present(false)  // 初始化原子变量
{
}
// 添加析构函数，确保停止线程
BenewakeLidarManager::~BenewakeLidarManager() {
    stop();
}
bool BenewakeLidarManager::initialize()
{
    std::lock_guard<std::mutex> lock(capture_mutex_);
    // 检查IP地址和端口号是否有效
    lidar = std::make_shared<benewake::BenewakeLidar>(lidar_ip, lidar_port);
    std::string version, version_fpga, sn;
    int total_num, line_num, channel_num;

    std::cout << "Trying to get device info...\n";
    if (lidar->getDeviceInformation(version, version_fpga, total_num, line_num, channel_num, sn))
    {
        std::cout << "[LIDAR] version: " << version << "\nFPGA: " << version_fpga
                  << "\nTotal points: " << total_num
                  << "\nLine points: " << line_num
                  << "\nChannels: " << channel_num
                  << "\nSN: " << sn << std::endl;
        lidar_present = true;
        return true;
    }
    else
    {
        std::cerr << "[ERROR] Failed to get LIDAR info.\n";
        lidar_present = false;
        return false;
    }
}

bool BenewakeLidarManager::hasLidar() const
{
    return lidar_present;
}

bool BenewakeLidarManager::isRunning() const {
    return is_running_.load();
}

void BenewakeLidarManager::start()
{
    std::lock_guard<std::mutex> lock(capture_mutex_);

    if (is_running_.load()) {
        std::cout << "[Benewake] Already running" << std::endl;
    }

    if (!lidar_present)
    {
        std::cerr << "[ERROR] No LIDAR detected. Aborting start().\n";
        return;
    }

    unsigned int num_threads = std::thread::hardware_concurrency();
    if (num_threads == 0)
        num_threads = 4;
    pool = std::make_shared<ThreadPool>(num_threads);

    lidar->stop();
    std::this_thread::sleep_for(std::chrono::seconds(3));

    if (!lidar->start())
    {
        std::cerr << "[ERROR] Failed to start LIDAR\n";
        return;
    }

    // 启动采集线程（不再detach，保存线程句柄）
    is_running_ = true;
    capture_thread_ = std::thread(&BenewakeLidarManager::captureLoop, this);
}

void BenewakeLidarManager::stop()
{
    if (!is_running_.exchange(false)) return; // 如果已经在停止状态，直接返回
    // 停止设备
    {
        std::lock_guard<std::mutex> lock(capture_mutex_);
        if (lidar) lidar->stop();
    }

    // 等待采集线程结束
    if (capture_thread_.joinable()) {
        auto start = std::chrono::steady_clock::now();
        const auto timeout = std::chrono::seconds(5);

        while (capture_thread_.joinable()) {
            auto now = std::chrono::steady_clock::now();
            if (now - start >= timeout) {
                std::cerr << "[Benewake] Thread join timeout" << std::endl;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
    }

    // 释放线程池
    if (pool) {
        pool.reset();
        std::cout << "[Benewake] Thread pool stopped" << std::endl;
    }

    std::cout << "[Benewake] Capture stopped" << std::endl;
}

void BenewakeLidarManager::captureLoop() {
    try {
        int frame_counter = 0;
        auto last_fps_time = std::chrono::steady_clock::now();
        benewake::BwPointCloud::Ptr pointCloud;
        benewake::SYS_INFO sys_info;
        int nFrame = 0;

        while (is_running_.load()) {
            bool data_ok = false;
            {
                std::lock_guard<std::mutex> lock(capture_mutex_);
                if (lidar) {
                    data_ok = lidar->getData(pointCloud, nFrame, sys_info);
                }
            }

            if (!data_ok) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // 生成时间戳
            std::string timestamp = TimeUtils::generateTimestampFilename();
            std::string timestamp_path = save_dir_ + "/timestamp.txt";
            if (!fileManager.saveTimestampTxt(timestamp_path, timestamp)) {
                std::cerr << "[Benewake] Failed to save timestamp" << std::endl;
            }

            // 保存点云数据
            if (!pointCloud->points.empty()) {
                std::string bin_path = save_dir_ + "/" + timestamp + ".bin";
                auto cloud_copy = *pointCloud;  // 复制数据避免生命周期问题

                pool->enqueue([this, cloud_copy, bin_path]() {
                    std::vector<RadarPoint> radar_points;
                    radar_points.reserve(cloud_copy.points.size());
                    for (const auto& pt : cloud_copy.points) {
                        radar_points.emplace_back(pt.x, pt.y, pt.z, pt.intensity);
                    }
                    fileManager.savePointCloudAsKITTI(radar_points, bin_path);

                    // 发送状态信息
                    std::string response = "{status: 1, log: [Benewake] Saved: " + bin_path + "}\n";
                    std::lock_guard<std::mutex> lock(send_mutex_);
                    send(client_socket, response.c_str(), response.size(), 0);
                });

                frame_counter++;
            }

            // 计算帧率
            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_fps_time);
            if (duration.count() >= 1) {
                std::cout << "[Benewake] FPS: " << frame_counter << std::endl;
                frame_counter = 0;
                last_fps_time = now;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    } catch (const std::exception& e) {
        std::cerr << "[Benewake] Capture loop error: " << e.what() << std::endl;
    }
    is_running_ = false;
}