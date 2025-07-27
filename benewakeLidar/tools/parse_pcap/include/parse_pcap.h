/**
* @file       parse_pcap.h

* @brief      Header of Benewake Horn X2 lidar's driver.

* @details 	  Parse point cloud data from PCAP. For now, it is for personal use only. 

* @author     Wang xuanbin

* @date       09/15/2021

* @version    v1.0.0

* @par Copyright (c):

*      Benewake (Beijing) Co., Ltd
*/

#ifndef INCLUDE_BENEWAKE_PARSE_PCAP_H__
#define INCLUDE_BENEWAKE_PARSE_PCAP_H__

#define NOMINMAX
#include <iostream>
#include <string>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <atomic>
#include <memory>
#include <ctime>
#include <list>
#include "benewake_common.h"
#include "benewake_tables.h"

#ifdef _WIN32
#ifdef BENEWAKE_DLL_EXPORT
#define BENEWAKE_API __declspec(dllexport)
#else
#define BENEWAKE_API __declspec(dllimport)
#endif // BENEWAKE_DLL_EXPORT
#else
#include <semaphore.h>
#include <string.h>
#define BENEWAKE_API
#endif

namespace benewake
{
    //class BENEWAKE_API MDOPProtocol
    class BENEWAKE_API ParsePcap
    {
    public:

        ParsePcap();

        ParsePcap(const char* _file_name);

        ~ParsePcap();

        void enableAngleCalibration(bool _enable) { enable_angle_calib_ = _enable; }

        void setParseFile(std::string _file_name);

        void pcap_frame_find(std::string _file_name);

        std::vector<int64_t> frame_offset_list_;   //Records the offset of each frame in the file

        void parseFrameByOffset(int64_t _offset, BwPointCloud::Ptr & _point_cloud);

        //Gets the frame interval time
        double getIntervalFromPcap();

        void closeFile();

        void setX2LongRangeMode(bool _enable) { long_range_version_X2_ = _enable; }

    private:

        std::string parse_file_ = "2.pcap";
        bool isLittleEndian_ = false; /*big endian*/

        bool parse_a_frame_flag_ = false;
        bool parse_interval_flag_ = false;  
        double frame_interval_ = 0.0;  
        double last_frame_time_ = 0.0; 

        FILE* pInFile_;

        BwPointCloud::Ptr point_cloud_buffer_;
        BwPointCloud::Ptr point_cloud_tmp_;
        int point_bytes_ = 0;
        BwPoint pt_;
        BwRGB rgb_;
        std::vector<uint8_t> gray_vector_;
        std::vector<BwRGB> rgb_vector_;
        uint32_t pkg_count_last_ = 0;
        uint16_t nFrame_last_ = 0;

        bool rgb_enable_ = false;
        bool gray_enable_ = false;

        uint8_t device_type_ = PRODUCT_ID_X2;
        bool long_range_version_X2_ = true;

        bool enable_angle_calib_ = false;

        uint32_t checksum(uint8_t* buffer, int size);

        uint32_t check_crc32(uint8_t* _buffer, size_t _size);

        uint32_t check_crc32_sb8_mode_begin(const uint8_t* _buffer, size_t _size);

        uint32_t check_sum_with_protocol_version(uint16_t _version, uint8_t* _buffer, size_t _size);

        bool isNewFrame(unsigned char* _data, int _recv_size);

        bool parse1FrameFromPackage(unsigned char* _data, int _recv_size);

        bool parseTime(unsigned char* _data, int _recv_size);

        void angleCalibration_AD2_C(float _in_azimuth, float _in_elevation, int _in_ch, float& _out_azimuth, float& _out_elevation);

        void treatEchoMode(uint8_t _echo_flag, int& _echo_mode, uint8_t& _echo_mark_1, uint8_t& _echo_mark_2);
    };
}

#endif