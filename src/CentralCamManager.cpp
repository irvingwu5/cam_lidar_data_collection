#include "CentralCamManager.h"
using namespace cv;

CentralCamManager::CentralCamManager(const std::string& device_path, int width, int height, FileManager& file_manager)
    : device_path_(device_path),
    width_(width),
    height_(height),
    file_manager_(file_manager),
    hasCentralCam(false),
    is_running_(false){}

CentralCamManager::~CentralCamManager() {
    stopCapture();
}

bool CentralCamManager::init() {
    std::cout << "Trying to open central camera: " << device_path_ << std::endl;
    //设置摄像头编码格式
    capture_.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
    //设置摄像头分辨率
    capture_.set(CAP_PROP_FRAME_WIDTH, width_);
    capture_.set(CAP_PROP_FRAME_HEIGHT, height_);
    // 3. 设置帧率（30fps，设备支持且稳定）
    capture_.set(CAP_PROP_FPS, 30);
    // 4. 禁用自动RGB转换（MJPG是压缩格式，无需提前转换）
    capture_.set(CAP_PROP_CONVERT_RGB, false);
    //尝试打开摄像头
    if (!capture_.open(device_path_,cv::CAP_V4L2)) {
        std::cerr << "The central camera does not exist" << std::endl;
        hasCentralCam = false;
        return false; // 摄像头打开失败
    }
    hasCentralCam = true;
    // 延长延迟至300ms，等待UVC摄像头硬件初始化（尤其高分辨率模式）
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // 验证实际参数（可选，确保设置生效）
    int actual_fps = capture_.get(CAP_PROP_FPS);
    int actual_fourcc = capture_.get(CAP_PROP_FOURCC);
    std::cout << "Actual FPS: " << actual_fps << ", Actual Format: "
              << (char)(actual_fourcc&0xFF) << (char)((actual_fourcc>>8)&0xFF)
              << (char)((actual_fourcc>>16)&0xFF) << (char)((actual_fourcc>>24)&0xFF) << std::endl;

    return true;
}
//主要做设备检测、线程池创建、设备启动等工作，实际采集工作在captureLoop中进行
bool CentralCamManager::startCapture() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!capture_.isOpened() && !init()) return false;
    is_running_ = true;
    capture_thread_ = std::thread(&CentralCamManager::captureLoop, this);
    return true;
}

void CentralCamManager::stopCapture() {
    if (!is_running_.exchange(false)) return; // 如果未在运行，则直接返回
    if (capture_thread_.joinable()) {
        capture_thread_.join(); // 等待采集线程结束
    }
    std::lock_guard<std::mutex> lock(capture_mutex_); // 确保线程安全
    capture_.release(); // 释放摄像头资源
}

bool CentralCamManager::isRunning() const {
    return is_running_.load();
}

bool CentralCamManager::hasCentralCamera() const {
    return hasCentralCam;
}

std::string CentralCamManager::generateTimestampFilename() {
    // 生成格式：YYYYMMDD_HHMMSS_uuuuuu
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S")
       << "_" << std::setw(6) << std::setfill('0') << microseconds.count();
    return ss.str();
}

//摄像头循环采集图像并保存
void CentralCamManager::captureLoop() {
    try {
        // 在循环开始前，确定本次采集的唯一保存目录
        std::string img_path = file_manager_.get_central_cam_path();
        int number = file_manager_.getPathCount(img_path);
        if (number < 10) {
            img_path = img_path + "/0" + std::to_string(number);
        } else {
            img_path = img_path + "/" + std::to_string(number);
        }
        // 创建这个唯一的目录
        file_manager_.createDirectory(img_path, false);
        //目录下创建名字为timestamp.txt的文件
        std::ofstream timestamp_file(img_path + "/timestamp.txt");
        timestamp_file.close();
        while (CentralCamManager::isRunning()) {
            Mat frame;
            std::lock_guard<std::mutex> lock(capture_mutex_); // 确保线程安全
            if (!capture_.read(frame)) {
                std::cerr << "Failed to capture frame from camera." << std::endl;
                continue; // 如果读取失败，继续下一次循环
            }
            // 生成带时间戳的文件名
            std::string filename = generateTimestampFilename();
			std::string timestamp_path = img_path + "/timestamp.txt";
            if (!file_manager_.saveTimestampTxt(timestamp_path, filename)) {
                std::cerr << "Failed to save timestamp: " << timestamp_path << std::endl;
            } else {
                std::cout << "Saved timestamp: " << timestamp_path << std::endl;
            }
            filename += ".png"; // 添加文件扩展名
            std::string full_path = img_path + "/";
            full_path += filename;
            if (!file_manager_.saveImage(full_path,frame)) {
                std::cerr << "Failed to save image: " << full_path << std::endl;
            } else {
                std::cout << "Saved image: " << full_path << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error in capture loop: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown error in capture loop." << std::endl;
    }
}