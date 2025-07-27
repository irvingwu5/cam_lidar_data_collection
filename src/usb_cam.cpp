#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>

using namespace cv;
using namespace std;

int main() {
    // 打开指定摄像头设备（仅打开一次）
    VideoCapture cap("/dev/video0");
    if (!cap.isOpened()) {
        cerr << "无法打开摄像头（路径：/dev/video0）" << endl;
        return -1;
    }

    // 关键：根据v4l2-ctl查询结果，设置支持的分辨率和像素格式
    // 示例：若支持MJPG格式的1920x1080，则先设置像素格式为MJPG
    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G')); // 设置为MJPEG格式
    // 设置分辨率（必须在支持的列表中，如1920x1080）
    bool setWidth = cap.set(CAP_PROP_FRAME_WIDTH, 1920);
    bool setHeight = cap.set(CAP_PROP_FRAME_HEIGHT, 1080);

    // 检查分辨率是否设置成功
    if (!setWidth || !setHeight) {
        cerr << "警告：摄像头不支持1920x1080分辨率，将使用默认分辨率" << endl;
        // 可选：自动降级为支持的分辨率（如1280x720，需确认是否在支持列表中）
        cap.set(CAP_PROP_FRAME_WIDTH, 1280);
        cap.set(CAP_PROP_FRAME_HEIGHT, 720);
    }

    // 增加短暂延迟，等待摄像头初始化完成（避免超时）
    this_thread::sleep_for(chrono::milliseconds(500));

    // 捕获一帧图像
    Mat frame;
    bool captureSuccess = cap.read(frame); // 推荐使用read()而非>>，更稳定

    if (!captureSuccess || frame.empty()) {
        cerr << "无法捕获图像（可能分辨率/格式不兼容）" << endl;
        cap.release();
        return -1;
    }

    // 保存图像
    string filename = "hbvcam_captured.jpg";
    bool saved = imwrite(filename, frame);
    if (saved) {
        cout << "图像已成功保存为: " << filename << endl;
    } else {
        cerr << "无法保存图像" << endl;
        return -1;
    }

    cap.release();
    return 0;
}