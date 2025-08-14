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
	if (thread_pool) {
        thread_pool.reset(); // 确保线程池正确释放
    }
}

bool TanwayLidarManager::initialize()
{
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

        lidar->Start();
        // 设置运行状态并启动线程
        is_running_ = true;
        capture_thread_ = std::thread([this]() {
            // 线程主循环：保持运行直到is_running_为false
            while (is_running_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
        std::cout << "[TanwayLidarManager] Lidar started." << std::endl;
    }
}

void TanwayLidarManager::stop()
{
    if (lidar_ready && is_running_)
    {
        is_running_ = false; // 先停止运行状态
        // 等待线程结束
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        lidar->Stop();
        std::cout << "[TanwayLidarManager] Lidar stopped." << std::endl;
    }
}

bool TanwayLidarManager::hasLidar() const
{
    return lidar_ready;
}

std::string TanwayLidarManager::generateTimestampFilename()
{
    // 生成格式：YYYYMMDD_HHMMSS_uuuuuu
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S")
       << "_" << std::setw(6) << std::setfill('0') << microseconds.count();
    return ss.str();
}

void TanwayLidarManager::OnPointCloud(const LidarInfo &info, const UserPointCloud &tanway_cloud)
{
    // 只处理运行状态下的数据
    if (!is_running_) return;

    auto task = [tanway_cloud, this]() {

        static auto last_time = std::chrono::steady_clock::now();  // 上次统计时间（静态变量保持状态）
        static int frame_count = 0;                                // 当前统计间隔内的帧数
        static float smooth_fps = 0.0f;                            // 平滑后的帧率
        constexpr float fps_update_interval = 1.0f;                // 帧率更新间隔（秒）

        // ---------------------- 帧率计算逻辑 ----------------------
        auto current_time = std::chrono::steady_clock::now();
        auto delta_time = std::chrono::duration<float>(current_time - last_time).count();
        frame_count++;  // 每处理一帧，计数加1

        // 当时间间隔超过设定值时，计算并打印帧率
        if (delta_time >= fps_update_interval) {
            // 计算当前帧率（帧数 / 时间差）
            float current_fps = frame_count / delta_time;
            // 指数平滑（避免数值剧烈波动）
            smooth_fps = smooth_fps * 0.9f + current_fps * 0.1f;
            // 打印帧率（保留2位小数）
            std::cout << "\r[Frame Rate] Current: " << std::fixed << std::setprecision(2)
                    << smooth_fps << " FPS | Last Interval: " << current_fps << " FPS"
                    << std::flush;  // \r 使光标回到行首，覆盖旧输出

            // 重置统计变量
            frame_count = 0;
            last_time = current_time;
        }

        std::ostringstream filename;
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);

        //创建时间戳空文件
        std::string timestamp = TimeUtils::generateTimestampFilename();
        std::string timestamp_path = save_dir + "/timestamp.txt";
        //向文件中写入时间戳
        if(!fileManager.saveTimestampTxt(timestamp_path, timestamp)){
            std::cerr << "[TanwayLidarManager] Failed to save timestamp." << std::endl;
        }else{
            std::cout << "[TanwayLidarManager] Saved timestamp: " << timestamp_path << std::endl;
        }

        filename << save_dir << "/" << timestamp << ".bin";

        std::vector<RadarPoint> cloud;
        cloud.reserve(tanway_cloud.points.size());
        for(const auto pt : tanway_cloud.points){
            cloud.emplace_back(pt.x, pt.y, pt.z, pt.intensity);
        }
        fileManager.savePointCloudAsKITTI(cloud, filename.str());
        cloud.clear(); // 清空点云数据，释放内存
        std::string return_info = "{status: 1, log: [TanwayLidarManager] Save path : " + filename.str() +
                                    "}\n";
        send(client_socket, return_info.c_str(), return_info.size(), 0);
        cur_frame++;
    };

    thread_pool->enqueue(task);
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
