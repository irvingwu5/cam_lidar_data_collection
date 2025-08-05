#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>

using namespace cv;
using namespace std;

int main(int argc, char* argv[]) {
    // 检查命令行参数
    if (argc < 2) {
        cerr << "用法: " << argv[0] << " <摄像头设备路径，如 /dev/video2>" << endl;
        return -1;
    }
    string device_path = argv[1];

    // 打开指定摄像头设备（仅打开一次）
    VideoCapture cap(device_path);
    if (!cap.isOpened()) {
        cerr << "无法打开摄像头（路径：" << device_path << "）" << endl;
        return -1;
    }

    // 关键：根据v4l2-ctl查询结果，设置支持的分辨率和像素格式
    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G')); // 设置为MJPEG格式
    bool setWidth = cap.set(CAP_PROP_FRAME_WIDTH, 1920);
    bool setHeight = cap.set(CAP_PROP_FRAME_HEIGHT, 1080);

    if (!setWidth || !setHeight) {
        cerr << "警告：摄像头不支持1920x1080分辨率，将使用默认分辨率" << endl;
        cap.set(CAP_PROP_FRAME_WIDTH, 1280);
        cap.set(CAP_PROP_FRAME_HEIGHT, 720);
    }

    this_thread::sleep_for(chrono::milliseconds(500));

    Mat frame;
    bool captureSuccess = cap.read(frame);

    if (!captureSuccess || frame.empty()) {
        cerr << "无法捕获图像（可能分辨率/格式不兼容）" << endl;
        cap.release();
        return -1;
    }

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