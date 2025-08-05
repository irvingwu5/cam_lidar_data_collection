#ifndef USB_DEVICE_RESET_H
#define USB_DEVICE_RESET_H

#include <string>
#include <cstdint>

namespace usb_utils {
    /**
     * 通过设备路径获取USB设备的vendor_id和product_id
     * @param device_path 设备路径（如/dev/video0）
     * @param vendor_id 输出参数，用于存储获取到的vendor ID
     * @param product_id 输出参数，用于存储获取到的product ID
     * @return 成功获取返回true，否则返回false
     */
    bool getUsbDeviceIds(const std::string& device_path, uint16_t& vendor_id, uint16_t& product_id);

    /**
     * 重置指定路径的USB设备
     * @param device_path 设备路径（如/dev/video0）
     * @return 重置成功返回true，否则返回false
     */
    bool resetUsbDevice(const std::string& device_path);
}

#endif // USB_DEVICE_RESET_H
