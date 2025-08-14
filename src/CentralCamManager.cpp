#include "CentralCamManager.h"
using namespace cv;
#include <chrono>
#include "TimeUtils.h"

CentralCamManager::CentralCamManager(const std::string& device_path, int width, int height, FileManager& file_manager)
    : device_path_(device_path),
    width_(width),
    height_(height),
    file_manager_(file_manager),new
    hasCentralCam(false),
    is_running_(false){}

CentralCamManager::~CentralCamManager() {
    stopCapture();
}

bool CentralCamManager::isDeviceAvailable(const std::string& device_path) const{
    std::ifstream dev(device_path);
    return dev.good();
}

bool CentralCamManager::init() {
    // 1. 检查设备是否存在
    if (!isDeviceAvailable(device_path_)) {
        std::cerr << "Device " << device_path_ << " is not accessible." << std::endl;
        hasCentralCam = false;
        return false;
    }

    std::cout << "Trying to open central camera: " << device_path_ << std::endl;

    // 3. 直接尝试打开摄像头
    if (!capture_.open(device_path_)) {
        std::cerr << "Failed to open central camera: " << device_path_ << std::endl;
        hasCentralCam = false;
        return false;
    }

    std::cout << "Central camera opened successfully." << std::endl;
    // 延长延迟至300ms，等待UVC摄像头硬件初始化
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 4. 设置摄像头参数
    capture_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    capture_.set(cv::CAP_PROP_FPS, 30);

    // 5. 验证参数
    int actual_fps = capture_.get(cv::CAP_PROP_FPS);
    int actual_fourcc = capture_.get(cv::CAP_PROP_FOURCC);
    std::cout << "Actual FPS: " << actual_fps << ", Actual Format: "
              << (char)(actual_fourcc & 0xFF) << (char)((actual_fourcc >> 8) & 0xFF)
              << (char)((actual_fourcc >> 16) & 0xFF) << (char)((actual_fourcc >> 24) & 0xFF)
              << ", Actual Width: " << capture_.get(cv::CAP_PROP_FRAME_WIDTH)
              << ", Actual Height: " << capture_.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    hasCentralCam = true;
    return true;
}
//主要做设备检测、线程池创建、设备启动等工作，实际采集工作在captureLoop中进行
bool CentralCamManager::startCapture() {
    std::lock_guard<std::mutex> lock(mutex_);
	if (is_running_) {
        std::cout << "Central camera is already running" << std::endl;
        return true;
    }
    // 二次检查设备是否可用（关键修改）
    if (!isDeviceAvailable(device_path_)) {
        std::cerr << "Device " << device_path_ << " is occupied, cannot start" << std::endl;
        return false;
    }
    // 关键修改：如果设备未打开（可能已被release），则重新初始化
    if (!capture_.isOpened()) {
        std::cout << "Central camera is not open. Initializing..." << std::endl;
        if (!init()) {
            std::cerr << "Failed to re-initialize central camera." << std::endl;
            return false;
        }
    }
    // 初始化线程池，可以根据CPU核心数设置线程数量
    unsigned int num_threads = std::thread::hardware_concurrency();
    if (num_threads == 0) num_threads = 4; // 默认4个线程
    pool_ = std::make_shared<ThreadPool>(num_threads);
    std::cout << "Image saving thread pool created with " << num_threads << " threads." << std::endl;

    is_running_ = true;
    capture_thread_ = std::thread(&CentralCamManager::captureLoop, this);
    return true;
}

void CentralCamManager::stopCapture() {
    if (!is_running_.exchange(false)) return;

    // 主动唤醒阻塞的read()
    if (capture_.isOpened()) {
        std::lock_guard<std::mutex> lock(capture_mutex_);
        capture_.grab(); // 中断read()阻塞
    }

    // 等待线程退出（带超时检测）
    if (capture_thread_.joinable()) {
        // 使用chrono库计算超时时间
        auto start = std::chrono::steady_clock::now();
        const auto timeout = std::chrono::seconds(5); // 超时延长至5秒

        // 循环等待线程退出，同时检测超时
        while (capture_thread_.joinable()) {
            auto now = std::chrono::steady_clock::now();
            if (now - start >= timeout) {
                std::cerr << "Warning: Central capture thread did not exit in time" << std::endl;
                break; // 超时退出循环
            }
            // 短暂休眠后再次检查
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        // 如果线程仍可连接，尝试强制join（避免资源泄漏）
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
    }
    // 停止并清空线程池，确保所有排队的图像都已保存
    if (pool_) {
        pool_.reset(); // reset会等待所有任务完成
        std::cout << "Image saving thread pool stopped and all tasks finished." << std::endl;
    }

    is_running_ = false; // 确保在停止时正确标记为未运行
    std::cout << "Central Camera " << device_path_ << " stopped" << std::endl;
}

bool CentralCamManager::isRunning() const {
    return is_running_.load();
}

bool CentralCamManager::hasCentralCamera() const {
    return hasCentralCam;
}

void CentralCamManager::captureLoop() {
    try {
        // 在循环开始前，确定本次采集的唯一保存目录
        std::string img_path = file_manager_.get_central_cam_path();
        int number = file_manager_.getPathCount(img_path);
        img_path += (number < 10) ? "/0" + std::to_string(number) : "/" + std::to_string(number);

        if (!file_manager_.createDirectory(img_path, false)) {
            throw std::runtime_error("Failed to create capture directory: " + img_path);
        }

        // 初始化时间戳文件
        std::string timestamp_path = img_path + "/timestamp.txt";
        std::ofstream timestamp_file(timestamp_path);
        if (!timestamp_file.is_open()) {
            throw std::runtime_error("Failed to create timestamp file: " + timestamp_path);
        }
        timestamp_file.close();

        // 帧率统计
        int frame_counter = 0;
        auto last_time = std::chrono::steady_clock::now();

        while (is_running_.load()) {
            cv::Mat frame;
            bool read_success = false;
            {
                std::lock_guard<std::mutex> lock(capture_mutex_);
                if (!capture_.isOpened()) {
                    std::cerr << "Capture device is not open. Exiting loop." << std::endl;
                    break;
                }
                read_success = capture_.read(frame);
            }

            if (!read_success || frame.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 避免空转占用过多CPU
                continue;
            }

            // 异步处理文件保存
            // 使用 clone() 复制图像数据，因为原始的 frame 会在下一次循环中被覆盖
            cv::Mat frame_to_save = frame.clone();
            std::string timestamp = TimeUtils::generateTimestampFilename();

            pool_->enqueue([this, frame_to_save, img_path, timestamp, timestamp_path]() {
                // 1. 保存时间戳
                if (!file_manager_.saveTimestampTxt(timestamp_path, timestamp)) {
                    std::cerr << "Failed to save timestamp: " << timestamp_path << std::endl;
                }

                // 2. 保存图像
                std::string full_path = img_path + "/" + timestamp + ".png";
                if (!file_manager_.saveImage(full_path, frame_to_save)) {
                    std::cerr << "Failed to save image: " << full_path << std::endl;
                }
            });

            frame_counter++;
            // 每秒打印一次帧率
            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_time);
            if (duration.count() >= 1) {
                std::cout << "[Frame Rate] CentralCam capturing at: " << frame_counter << " FPS" << std::endl;
                frame_counter = 0;
                last_time = now;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error in central camera capture loop: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown error in central camera capture loop." << std::endl;
    }
    is_running_ = false; // 确保在异常情况下也能正确标记为未运行
}

bool CentralCamManager::resetFlag(){
    hasCentralCam = false; // 标记为未持有摄像头
    is_running_ = false; // 确保状态正确
    return true;
}