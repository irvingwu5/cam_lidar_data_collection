#include "SideCamManager.h"
using namespace cv;

SideCamManager::SideCamManager(const std::string& device_path, int width, int height, FileManager& file_manager)
    : device_path_(device_path),
    width_(width),
    height_(height),
    file_manager_(file_manager),
    hasSideCam(false),
    is_running_(false){}

SideCamManager::~SideCamManager() {
    stopCapture();
}

bool SideCamManager::init() {
    std::cout << "Trying to open side camera: " << device_path_ << std::endl;
    if (!capture_.open(device_path_,cv::CAP_V4L2)) {
        std::cerr << "Open Side camera timeout" << std::endl;
        hasSideCam = false;
        return false;
    }
    hasSideCam = true;
    // 延长延迟至300ms，等待UVC摄像头硬件初始化（尤其高分辨率模式）
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
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
    // 设置分辨率后检查实际值
    int actual_width = capture_.get(CAP_PROP_FRAME_WIDTH);
    int actual_height = capture_.get(CAP_PROP_FRAME_HEIGHT);
    if (actual_width != width_ || actual_height != height_) {
        std::cerr << "Actual resolution: " << actual_width << "*" << actual_height << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));//增加短暂延迟，等待摄像头初始化完成
    return true;
}
//主要做设备检测、线程池创建、设备启动等工作，实际采集工作在captureLoop中进行
bool SideCamManager::startCapture() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!capture_.isOpened() && !init()) return false;
    is_running_ = true;
    capture_thread_ = std::thread(&SideCamManager::captureLoop, this);
    return true;
}

void SideCamManager::stopCapture() {
    if (!is_running_.exchange(false)) return; // 如果未在运行，则直接返回
    if (capture_thread_.joinable()) {
        capture_thread_.join(); // 等待采集线程结束
    }
    std::lock_guard<std::mutex> lock(capture_mutex_); // 确保线程安全
    capture_.release(); // 释放摄像头资源
}

bool SideCamManager::isRunning() const {
    return is_running_.load();
}

bool SideCamManager::hasSideCamera() const {
    return hasSideCam;
}

std::string SideCamManager::generateTimestampFilename() {
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
void SideCamManager::captureLoop() {
    try {
        std::string img_path = file_manager_.get_side_cam_path();
        int number = file_manager_.getPathCount(img_path);
        if (number < 10)
            img_path = img_path + "/0" + std::to_string(number);
        else
            img_path = img_path + "/" + std::to_string(number);
        file_manager_.createDirectory(img_path, false); // 创建目录，如果不存在
        //目录下创建名字为timestamp.txt的文件
        std::ofstream timestamp_file(img_path + "/timestamp.txt");
        timestamp_file.close();
        while (SideCamManager::isRunning()) {
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


