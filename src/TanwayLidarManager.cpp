#include "TanwayLidarManager.h"
#include <chrono>
#include <fstream>
#include <iomanip>
#include "TimeUtils.h"
using namespace tanway;

TanwayLidarManager::TanwayLidarManager(int client_socket,FileManager fileManager,
                                       const std::string &local_ip,
                                       const std::string &lidar_ip,
                                       const std::string &algo_config_path)
    : fileManager(fileManager),
      local_ip(local_ip),
      lidar_ip(lidar_ip),
      algo_config_path(algo_config_path),
      client_socket(client_socket)
{
    thread_pool = std::make_shared<ThreadPool>(1); // 可根据需要调整线程数量
}

TanwayLidarManager::~TanwayLidarManager()
{
    stop();
}

bool TanwayLidarManager::initialize()
{
    std::lock_guard<std::mutex> lock(device_mutex_);

    lidar = ILidarDevice::Create(lidar_ip.c_str(), local_ip.c_str(), 5600, 5700, this, LT_FocusB2_B3_MP);
    if (!lidar)
    {
        std::cerr << "[TanwayLidarManager] Failed to create lidar device!" << std::endl;
        return false;
    }

    algo = ILidarAlgo::Create(LT_FocusB2_B3_MP, algo_config_path);
    if (!algo)
    {
        std::cerr << "[TanwayLidarManager] Failed to create lidar algorithm!" << std::endl;
        return false;
    }

    lidar->SetLidarAlgo(algo.get());
    lidar_ready = true;
    return true;
}

void TanwayLidarManager::start()
{
    std::lock_guard<std::mutex> lock(device_mutex_);
    if (lidar_ready && !is_running_)
    {
        cur_frame = 0;
        save_dir = fileManager.get_64_lidar_save_path();
        if (save_dir.length() <= 0)
        {
            std::cout << "select save_path" << std::endl;
            return;
        }

        int number = fileManager.getPathCount(save_dir);
        if (number < 10)
            save_dir = save_dir + "/0" + std::to_string(number);
        else
            save_dir = save_dir + "/" + std::to_string(number);
        fileManager.createDirectory(save_dir,false);

        std::ofstream timestamp_file(save_dir + "/timestamp.txt");
        timestamp_file.close();

        // 初始化线程池
        thread_pool_ = std::make_shared<ThreadPool>(1);

        lidar->Start();
        // 设置运行状态并启动线程
        is_running_ = true;
        capture_thread_ = std::thread([this]() {
            // 线程主循环：保持运行直到is_running_为false
            while (is_running_.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
        std::cout << "[TanwayLidarManager] Lidar started." << std::endl;
    }
}

void TanwayLidarManager::stop()
{
    if (!is_running_.exchange(false)) return;
    // 停止设备
    {
        std::lock_guard<std::mutex> lock(device_mutex_);
        if (lidar) lidar->Stop();
    }
    // 等待采集线程结束
    if (capture_thread_.joinable()) {
        auto start = std::chrono::steady_clock::now();
        const auto timeout = std::chrono::seconds(5);

        while (capture_thread_.joinable()) {
            auto now = std::chrono::steady_clock::now();
            if (now - start >= timeout) {
                std::cerr << "[Tanway] Thread join timeout" << std::endl;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
    }

    // 释放线程池
    if (thread_pool_) {
        thread_pool_.reset();
        std::cout << "[Tanway] Thread pool stopped" << std::endl;
    }

    std::cout << "[Tanway] Capture stopped" << std::endl;
}

bool TanwayLidarManager::hasLidar() const
{
    return lidar_ready;
}

bool TanwayLidarManager::isRunning() const {
    return is_running_.load();
}

void TanwayLidarManager::OnPointCloud(const LidarInfo& info, const UserPointCloud& cloud) {
    if (is_running_.load()) {
        handlePointCloud(cloud);
    }
}

void TanwayLidarManager::handlePointCloud(const UserPointCloud& cloud)
{
    // 只处理运行状态下的数据
    if (!is_running_.load()) return;

    static int frame_counter = 0;
    static auto last_fps_time = std::chrono::steady_clock::now();

    // 生成时间戳
    std::string timestamp = TimeUtils::generateTimestampFilename();
    std::string timestamp_path = save_dir + "/timestamp.txt";
    if (!fileManager_.saveTimestampTxt(timestamp_path, timestamp)) {
        std::cerr << "[Tanway] Failed to save timestamp" << std::endl;
    }

    // 保存点云数据
    std::string bin_path = save_dir + "/" + timestamp + ".bin";
    auto cloud_copy = cloud;  // 复制数据避免生命周期问题

    thread_pool_->enqueue([this, cloud_copy, bin_path]() {
        std::vector<RadarPoint> radar_points;
        radar_points.reserve(cloud_copy.points.size());
        for (const auto& pt : cloud_copy.points) {
            radar_points.emplace_back(pt.x, pt.y, pt.z, pt.intensity);
        }
        fileManager_.savePointCloudAsKITTI(radar_points, bin_path);

        // 发送状态信息
        std::string response = "{status: 1, log: [Tanway] Saved: " + bin_path + "}\n";
        std::lock_guard<std::mutex> lock(send_mutex_);
        send(client_socket_, response.c_str(), response.size(), 0);
    });

    // 计算帧率
    frame_counter++;
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_fps_time);
    if (duration.count() >= 1) {
        std::cout << "[Tanway] FPS: " << frame_counter << std::endl;
        frame_counter = 0;
        last_fps_time = now;
    }
}

void TanwayLidarManager::OnException(const LidarInfo &info, const Exception &e)
{
    if (e.GetErrorCode() > 0)
        std::cerr << "[TanwayLidarManager] Error: " << e.GetErrorCode() << " -> " << e.ToString() << std::endl;
    if (e.GetTipsCode() > 0)
        std::cerr << "[TanwayLidarManager] Tip: " << e.GetTipsCode() << " -> " << e.ToString() << std::endl;
}

void TanwayLidarManager::OnIMU(const LidarInfo &lidarInfo, const IMUData &imu) {
// std::cout << "IMU data callback:" << std::endl
//       << "  Angular velocity [rad/s]: "
//       << "X: " << std::fixed << std::setprecision(6)
//       << imu.angular_velocity[0] << ", Y: " << std::fixed
//       << std::setprecision(6) << imu.angular_velocity[1]
//       << ", Z: " << std::fixed << std::setprecision(6)
//       << imu.angular_velocity[2] << std::endl
//       << "  Linear acceleration [g]: "
//       << "X: " << std::fixed << std::setprecision(6)
//       << imu.linear_acceleration[0] << ", Y: " << std::fixed
//       << std::setprecision(6) << imu.linear_acceleration[1]
//       << ", Z: " << std::fixed << std::setprecision(6)
//       << imu.linear_acceleration[2] << std::endl;

  }
void TanwayLidarManager::OnDeviceInfoFrame(const LidarInfo &info, const DeviceInfoFrame &deviceInfoFrame)
{
    // 可选实现或留空
}

void TanwayLidarManager::OnParsePcapProcess(const LidarInfo &info, int process, uint64_t frame, uint64_t stamp)
{
    // 可选实现或留空
}
