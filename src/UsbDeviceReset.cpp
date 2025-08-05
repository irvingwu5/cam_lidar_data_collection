#include "UsbDeviceReset.h"
#include <libusb-1.0/libusb.h>
#include <cstdio>
#include <cstring>
#include <iostream>

namespace usb_utils {

bool getUsbDeviceIds(const std::string& device_path, uint16_t& vendor_id, uint16_t& product_id) {
    // 初始化输出参数
    vendor_id = 0;
    product_id = 0;

    // 构建命令获取设备的ID_VENDOR_ID和ID_MODEL_ID
    std::string cmd = "udevadm info -q property -n " + device_path +
                     " | grep -E 'ID_VENDOR_ID=|ID_MODEL_ID='";

    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
        std::cerr << "无法执行命令获取设备信息: " << cmd << std::endl;
        return false;
    }

    char buffer[128];
    while (fgets(buffer, sizeof(buffer), pipe)) {
        if (strstr(buffer, "ID_VENDOR_ID=")) {
            sscanf(buffer, "ID_VENDOR_ID=%hx", &vendor_id);
        } else if (strstr(buffer, "ID_MODEL_ID=")) {
            sscanf(buffer, "ID_MODEL_ID=%hx", &product_id);
        }
    }

    int exit_status = pclose(pipe);
    if (exit_status != 0) {
        std::cerr << "执行命令失败，退出状态: " << exit_status << std::endl;
        return false;
    }

    return (vendor_id != 0 && product_id != 0);
}

bool resetUsbDevice(const std::string& device_path) {
    // 1. 获取设备的vendor_id和product_id
    uint16_t vendor_id = 0, product_id = 0;
    if (!getUsbDeviceIds(device_path, vendor_id, product_id)) {
        std::cerr << "无法获取设备 " << device_path << " 的vendor ID和product ID" << std::endl;
        return false;
    }

    std::cout << "尝试重置设备: " << device_path
              << " (vendor=0x" << std::hex << vendor_id
              << ", product=0x" << product_id << std::dec << ")" << std::endl;

    // 2. 初始化libusb
    libusb_context* ctx = nullptr;
    int r = libusb_init(&ctx);
    if (r != LIBUSB_SUCCESS) {
        std::cerr << "libusb初始化失败: " << libusb_error_name(r) << std::endl;
        return false;
    }

    // 3. 查找设备
    libusb_device_handle* dev_handle = libusb_open_device_with_vid_pid(ctx, vendor_id, product_id);
    if (!dev_handle) {
        std::cerr << "无法打开设备 (vendor=0x" << std::hex << vendor_id
                  << ", product=0x" << product_id << std::dec << ")" << std::endl;
        libusb_exit(ctx);
        return false;
    }

    // 4. 重置设备
    r = libusb_reset_device(dev_handle);
    bool success = (r == LIBUSB_SUCCESS);

    if (success) {
        std::cout << "设备 " << device_path << " 重置成功" << std::endl;
    } else {
        std::cerr << "设备重置失败: " << libusb_error_name(r) << std::endl;
    }

    // 5. 清理资源
    libusb_close(dev_handle);
    libusb_exit(ctx);

    return success;
}

} // namespace usb_utils
