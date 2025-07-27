#include "CentralCamManager.h"
using namespace cv;

CentralCamManager::CentralCamManager(const std::string& device_path, int width, int height, FileManager& file_manager)
    : device_path_(device_path),
    width_(width),
    height_(height),
    file_manager_(file_manager),
    is_running_(false){}

CentralCamManager::~CentralCamManager() {
    stopCapture();
}

bool CentralCamManager::init() {
    //尝试打开摄像头
    if (!capture_.open(device_path_)) {
        std::cerr << "The central camera does not exist" << std::endl;
        return false;
    }
    //设置摄像头编码格式
    capture_.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
    //设置摄像头分辨率
    bool setWidth = capture_.set(CAP_PROP_FRAME_WIDTH, width_);
    bool setHeight = capture_.set(CAP_PROP_FRAME_HEIGHT, height_);
    //检查分辨率是否设置正确
    if (!setWidth || !setHeight) {
        std::cerr<<"Warning: The camera does not support" << width_ << "*" << height_ <<
            "resolution and will use the default resolution 1280*720" << std::endl;
        capture_.set(CAP_PROP_FRAME_WIDTH, 1280);
        capture_.set(CAP_PROP_FRAME_HEIGHT, 720);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));//增加短暂延迟，等待摄像头初始化完成
    //为后续图像保存创建目录
    std::string img_path = file_manager_.get_central_cam_path();
    if (!file_manager_.createDirectory(img_path, false)) {
        std::cerr << "Failed to create directory: " << img_path << std::endl;
        return false;
    }
    return true;
}
//主要做设备检测、线程池创建、设备启动等工作，实际采集工作在captureLoop中进行
bool CentralCamManager::startCapture() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_running_) return false; // 已在运行
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

std::string CentralCamManager::generateTimestampFilename() {
    // 生成格式：YYYYMMDD_HHMMSS.png
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S")
       << ".png";
    return ss.str();
}

//摄像头循环采集图像并保存
void CentralCamManager::captureLoop() {
    try {
        while (is_running_) {
            Mat frame;
            std::lock_guard<std::mutex> lock(capture_mutex_); // 确保线程安全
            if (!capture_.read(frame)) {
                std::cerr << "Failed to capture frame from camera." << std::endl;
                continue; // 如果读取失败，继续下一次循环
            }
            std::string img_path = file_manager_.get_central_cam_path();
            // 生成带时间戳的文件名
            std::string filename = generateTimestampFilename();
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