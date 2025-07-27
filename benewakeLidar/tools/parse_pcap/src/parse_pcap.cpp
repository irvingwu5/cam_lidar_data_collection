#include "parse_pcap.h"
#include <stdio.h>

static float kPI = 3.141592;
static float kAngRes_AD2 = 0.05; // angle resoluton of channel along vertical
static float kAngRes_G66 = 0.22; // angle resoluton of channel along vertical

static float ang2Rad(float ang)
{
	return ang * kPI / 180.0;
}

benewake::ParsePcap::ParsePcap()
{
}

benewake::ParsePcap::ParsePcap(const char *_file_name)
{
	parse_file_ = _file_name;
}

benewake::ParsePcap::~ParsePcap()
{
}

void benewake::ParsePcap::setParseFile(std::string _file_name)
{
	parse_file_ = _file_name;
}

void benewake::ParsePcap::closeFile()
{
	fclose(pInFile_);
}

// Gets the time interval between two frames in a file, which can be used as a reference for playback speed
double benewake::ParsePcap::getIntervalFromPcap()
{
	return frame_interval_;
}

// Search all frames in pCAP and place their file locations in the linked list
void benewake::ParsePcap::pcap_frame_find(std::string _file_name)
{
	parse_file_ = _file_name;
	unsigned char pcap_file_header[24] = {0}; /*file header 24 bytes*/
	unsigned char pkt_hdr[16] = {0};		  /*pkt_hdr 16 bytes*/
	unsigned char udp_pkt_data[4096] = {0};	  /*content*/
	int len = 0;
	int pkt_cnt = 0;

	const char *char_name = _file_name.data();
	pInFile_ = fopen(char_name, "rb"); /*open pcap file*/
	if (NULL == pInFile_)
	{
		std::cout << "pcap file read error!" << std::endl;
		return;
	}

	if (1 != fread(pcap_file_header, 24, 1, pInFile_)) /*read pcap file header*/
	{
		fclose(pInFile_);
		std::cout << "header read failed!" << std::endl;
		return;
	}

	if ((0xD4 == pcap_file_header[0]) && (0xC3 == pcap_file_header[1]) && (0xB2 == pcap_file_header[2]) && (0xA1 == pcap_file_header[3]))
	{
		isLittleEndian_ = true;
	}
	unsigned char lidar_data[2000] = {0};

	int frame_cnt = 0;
	int useless = 0;
	frame_offset_list_.clear();
	parse_interval_flag_ = true;
	last_frame_time_ = 0;
	nFrame_last_ = 0;
	if (rgb_enable_)
	{
		point_cloud_buffer_.reset();
		point_cloud_buffer_ = std::make_shared<BwPointCloudRGB>();
		point_cloud_tmp_ = std::make_shared<BwPointCloudRGB>();
		point_bytes_ = 15;
	}
	else if (gray_enable_)
	{
		point_cloud_buffer_.reset();
		point_cloud_buffer_ = std::make_shared<BwPointCloudGray>();
		point_cloud_tmp_ = std::make_shared<BwPointCloudGray>();
		point_bytes_ = 13;
	}
	else
	{
		point_cloud_buffer_.reset();
		point_cloud_buffer_ = std::make_shared<BwPointCloud>();
		point_cloud_tmp_ = std::make_shared<BwPointCloud>();
		point_bytes_ = 12;
	}

	int64_t first_valid_offset = -1;
	while (1)
	{
		int64_t offset;
#ifdef _WIN32
		offset = _ftelli64(pInFile_);
#else
		offset = ftell(pInFile_);
#endif
		if (offset == -1)
		{
			std::cout << "_ftelli64 error: " << strerror(errno) << std::endl;
			break;
		}
		if (1 != fread(pkt_hdr, 16, 1, pInFile_)) /*read a pkt_hdr 16 bytes*/
		{
			// std::cout << "read header failed!" << " pkt_cnt: " << pkt_cnt  << "  "  << strerror(errno) << std::endl;
			break;
		}

		if (isLittleEndian_) /*get the pktcontent's length in the file following the pkt_hdr*/
		{
			len = (int(pkt_hdr[11]) << 24) + int((pkt_hdr[10]) << 16) + int((pkt_hdr[9]) << 8) + int(pkt_hdr[8]);
		}
		else
		{
			len = (int(pkt_hdr[8]) << 24) + int((pkt_hdr[9]) << 16) + int((pkt_hdr[10]) << 8) + int(pkt_hdr[11]);
		}

		/*read the length bytes from file, this is the pktcontent*/
		if (1 != fread(udp_pkt_data, len, 1, pInFile_))
		{
			std::cout << "read data failed!"
					  << "len: " << len << " pkt_cnt: " << pkt_cnt << std::endl;
			printf("Error no.%d: %s\n", errno, strerror(errno));
			break;
		}

		pkt_cnt++;
		if (len > 2000)
		{
			std::cout << "The current packet is not available :" << pkt_cnt << std::endl;
			continue;
		}
		if (first_valid_offset == -1)
			first_valid_offset = offset;
		memcpy(lidar_data, &udp_pkt_data[42], len - 42);
		if (isNewFrame(lidar_data, len - 42))
		{
			// std::cout << "cur offset:"  << offset  << " pkt_cnt: " << pkt_cnt << std::endl;
			frame_offset_list_.push_back(offset);
			frame_cnt++;
		}
	}
	if (frame_offset_list_.size() == 0)
	{
		frame_offset_list_.push_back(first_valid_offset);
		frame_cnt++;
	}
	std::cout << "Total " << pkt_cnt << " packets found" << std::endl;
	std::cout << "point cloud frame: " << frame_cnt << std::endl;
	std::cout << "time interval:" << frame_interval_ << std::endl;

	// fclose(pInFile_);
}

// The point cloud is resolved according to offset
void benewake::ParsePcap::parseFrameByOffset(int64_t _offset, BwPointCloud::Ptr &_point_cloud)
{
	unsigned char pcap_file_header[24] = {0}; /*file header 24 bytes*/
	unsigned char pkt_hdr[16] = {0};		  /*pkt_hdr 16 bytes*/
	unsigned char udp_pkt_data[4096] = {0};	  /*content*/
	int len = 0;
	unsigned char lidar_data[2000] = {0};
	parse_a_frame_flag_ = true;
	if (rgb_enable_)
	{
		point_cloud_buffer_.reset();
		point_cloud_buffer_ = std::make_shared<BwPointCloudRGB>();
		point_cloud_tmp_ = std::make_shared<BwPointCloudRGB>();
		point_bytes_ = 15;
	}
	else if (gray_enable_)
	{
		point_cloud_buffer_.reset();
		point_cloud_buffer_ = std::make_shared<BwPointCloudGray>();
		point_cloud_tmp_ = std::make_shared<BwPointCloudGray>();
		point_bytes_ = 13;
	}
	else
	{
		point_cloud_buffer_.reset();
		point_cloud_buffer_ = std::make_shared<BwPointCloud>();
		point_cloud_tmp_ = std::make_shared<BwPointCloud>();
		point_bytes_ = 12;
	}
	// point_cloud_tmp_.reset(new BwPointCloud());
	point_cloud_tmp_->points.reserve(1100000);

	// Moves the file read location by offset
#ifdef _WIN32
	if (_fseeki64(pInFile_, _offset, SEEK_SET))
	{
		std::cout << "set offset failed : " << _offset << "cur file position: " << ftell(pInFile_) << std::endl;
		return;
	}
#else
	if (fseeko64(pInFile_, _offset, SEEK_SET) == -1)
	{
		std::cout << "set offset failed : " << _offset << "cur file position: " << ftell(pInFile_) << std::endl;
		return;
	}
#endif

	while (1)
	{

		if (1 != fread(pkt_hdr, 16, 1, pInFile_)) /*read a pkt_hdr 16 bytes*/
		{
			_point_cloud = point_cloud_tmp_;
			// std::cout << "read head failed!" << std::endl;
			break;
		}

		if (isLittleEndian_) /*get the pktcontent's length in the file following the pkt_hdr*/
		{
			len = (int(pkt_hdr[11]) << 24) + int((pkt_hdr[10]) << 16) + int((pkt_hdr[9]) << 8) + int(pkt_hdr[8]);
		}
		else
		{
			len = (int(pkt_hdr[8]) << 24) + int((pkt_hdr[9]) << 16) + int((pkt_hdr[10]) << 8) + int(pkt_hdr[11]);
		}

		/*read the length bytes from file, this is the pktcontent*/
		if (1 != fread(udp_pkt_data, len, 1, pInFile_))
		{
			_point_cloud = point_cloud_tmp_;
			// std::cout << "read data failed!" << strerror(errno) << std::endl;
			break;
		}

		if (len > 2000)
		{
			continue;
		}
		memcpy(lidar_data, &udp_pkt_data[42], len - 42);
		if (parse1FrameFromPackage(lidar_data, len - 42))
		{
			// Returning true indicates that a frame has been resolved and needs to exit
			// std::cout << "cur frame : " << nFrame_last_ << std::endl;
			_point_cloud = point_cloud_tmp_;
			if (_point_cloud != nullptr)
			{
				//std::cout << " size: " << _point_cloud->points.size() << std::endl;
			}
			return;
		}
		else
		{
			// This frame is not over yet
			// std::cout << "---------";
			/// std::cout << "cur frame : " << nFrame_last_ << std::endl;
		}
	}
}

// Parse a frame of data from a packet
bool benewake::ParsePcap::parse1FrameFromPackage(unsigned char *_data, int _recv_size)
{
	int time_offset = 0;
	uint32_t total_lost_package = 0;
	int16_t val16 = 0, val16_ang_h = 0, val16_ang_v = 0;
	uint16_t uval16 = 0;
	int16_t dist16 = 0;
	uint8_t uval8 = 0;
	int32_t dist32 = 0;
	uint8_t echo_mark_1 = 0, echo_mark_2 = 0;

	// local variable
	uint32_t check_sum = 0;
	uint32_t pkg_count = 0;
	uint16_t nFrame = 0;
	uint16_t nLine = 0;
	uint16_t nPoint = 0;
	uint64_t time_s = 0;
	uint32_t time_ns = 0;
	uint32_t diff = 0;
	uint16_t protocol_version = 0, last_pkg_mark = 0;

	int total_bytes = 0, header_length = 38;
	int data_block_size = 0, echo_mode = 1;
	float azimuth_h, azimuth_v, dist_f;
	uint16_t roi_center_x_flag = 0, roi_center_y_flag = 0;
	float roi_center_x = 0, roi_center_y = 0;
	float roi_half_x = kPI * 15.0 / 180.0, roi_half_y = kPI * 6.4 / 180.0;

	bool use_gps_src = false;

	if (_recv_size < 44)
	{
#ifdef DEBUG_INFO
		// std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		// std::cout << "Point cloud _data package size error! The package should be larger than 44 bytes but only "
		//<< _recv_size << " bytes received." << std::endl;
#endif // DEBUG_INFO
	}
	else
	{
		if (_data[0] == 0x42 && _data[1] == 0x57 && (_data[2] == PRODUCT_ID_X2 || _data[2] == PRODUCT_ID_P4 || _data[2] == PRODUCT_ID_G66) &&
			_data[3] == PROTOCOL_ID_MDOP && _data[_recv_size - 2] == 0x00 && _data[_recv_size - 1] == 0xff)
		{
			memcpy(&protocol_version, &_data[PROTOCOL_DATA_PACKAGE_VERSION_OFFSET], sizeof(uint16_t));
			memcpy(&pkg_count, &_data[PROTOCOL_DATA_PACKAGE_COUNT_OFFSET], sizeof(uint32_t));
			memcpy(&nFrame, &_data[PROTOCOL_DATA_FRAME_COUNT_OFFSET], sizeof(uint16_t));
			memcpy(&uval16, &_data[PROTOCOL_DATA_LINE_NUM_OFFSET], sizeof(uint16_t));
			nLine = uval16 & 0x7fff;
			last_pkg_mark = uval16 & 0x8000;
			memcpy(&nPoint, &_data[PROTOCOL_DATA_POINT_COUNT_OFFSET], sizeof(uint16_t));
			if (_data[2] == PRODUCT_ID_X2)
			{
				memcpy(&time_s, &_data[PROTOCOL_X_DATA_TIME_S_OFFSET], sizeof(uint32_t));
				memcpy(&time_ns, &_data[PROTOCOL_X_DATA_TIME_NS_OFFSET], sizeof(uint32_t));
				if (rgb_enable_)
					point_bytes_ = 15;
				else if (gray_enable_)
					point_bytes_ = 13;
				else
					point_bytes_ = 12;
				total_bytes = nPoint * point_bytes_ + 44;
			}
			else if (_data[2] == PRODUCT_ID_P4)
			{
				switch (protocol_version)
				{
				case PROTOCOL_VERSION_AD2_B:
					time_s = *(uint32_t*)&_data[PROTOCOL_X_DATA_TIME_S_OFFSET];
					time_ns = *(uint32_t*)&_data[PROTOCOL_X_DATA_TIME_NS_OFFSET];
					uval8 = *(uint8_t*)&_data[PROTOCOL_AD2_B_DATA_ECHO_MODE];
					roi_center_x_flag = *(uint16_t*)&_data[PROTOCOL_AD2_B_DATA_ROI_CENTER_X];
					roi_center_y_flag = *(uint16_t*)&_data[PROTOCOL_AD2_B_DATA_ROI_CENTER_Y];
					roi_center_x = (float)roi_center_x_flag / 8192.0;
					roi_center_y = (float)roi_center_y_flag / 8192.0;

					header_length = 38;
					break;

				case PROTOCOL_VERSION_AD2_C:
					time_s = *(uint64_t*)&_data[PROTOCOL_AD2_DATA_TIME_S_OFFSET];
					time_ns = *(uint32_t*)&_data[PROTOCOL_AD2_DATA_TIME_NS_OFFSET];
					uval8 = *(uint8_t*)&_data[PROTOCOL_AD2_DATA_ECHO_MODE];
					roi_center_x_flag = *(uint16_t*)&_data[PROTOCOL_AD2_DATA_ROI_CENTER_X];
					roi_center_y_flag = *(uint16_t*)&_data[PROTOCOL_AD2_DATA_ROI_CENTER_Y];
					roi_center_x = (float)roi_center_x_flag / 10.0;
					roi_center_y = (float)roi_center_y_flag / 10.0;
					uval16 = *(uint16_t*)&_data[PROTOCOL_AD2_DATA_ROI_WIDTH];
					roi_half_x = (float)uval16 / 20.0;
					uval16 = *(uint16_t*)&_data[PROTOCOL_AD2_DATA_ROI_HEIGHT];
					roi_half_y = (float)uval16 / 20.0;

					header_length = 42;
					break;

				case PROTOCOL_VERSION_AD2_HH:
					time_s = *(uint64_t*)&_data[PROTOCOL_AD2_DATA_TIME_S_OFFSET];
					time_ns = *(uint32_t*)&_data[PROTOCOL_AD2_DATA_TIME_NS_OFFSET];
					uval8 = *(uint8_t*)&_data[PROTOCOL_AD2_DATA_ECHO_MODE];
					roi_center_x_flag = *(uint16_t*)&_data[PROTOCOL_AD2_DATA_ROI_CENTER_X];
					roi_center_y_flag = *(uint16_t*)&_data[PROTOCOL_AD2_DATA_ROI_CENTER_Y];
					roi_center_x = (float)roi_center_x_flag / 10.0;
					roi_center_y = (float)roi_center_y_flag / 10.0;
					uval16 = *(uint16_t*)&_data[PROTOCOL_AD2_DATA_ROI_WIDTH];
					roi_half_x = (float)uval16 / 20.0;
					uval16 = *(uint16_t*)&_data[PROTOCOL_AD2_DATA_ROI_HEIGHT];
					roi_half_y = (float)uval16 / 20.0;

					header_length = 42;
					break;
				default:
					break;
				}
				point_bytes_ = 4;
				treatEchoMode(uval8, echo_mode, echo_mark_1, echo_mark_2);
				if (echo_mode == 1)
				{
					data_block_size = 72;
				}
				else
				{
					data_block_size = 136;
				}
				total_bytes = 12 * data_block_size / echo_mode + header_length + 6;
			}
			else if (_data[2] == PRODUCT_ID_G66)
			{
				data_block_size = *(uint16_t*)&_data[PROTOCOL_G66_DATA_BLOCK_SIZE];
				time_s = *(uint64_t*)&_data[PROTOCOL_G66_DATA_TIME_S_OFFSET];
				time_ns = *(uint32_t*)&_data[PROTOCOL_G66_DATA_TIME_NS_OFFSET];
				uval8 = *(uint8_t*)&_data[PROTOCOL_G66_DATA_ECHO_MODE];
				roi_center_x_flag = *(int16_t*)&_data[PROTOCOL_G66_DATA_ROI_CENTER_X];
				roi_center_y_flag = *(int16_t*)&_data[PROTOCOL_G66_DATA_ROI_CENTER_Y];
				roi_center_x = (float)roi_center_x_flag / 10.0;
				roi_center_y = (float)roi_center_y_flag / 10.0;
				uval16 = *(uint16_t*)&_data[PROTOCOL_G66_DATA_ROI_WIDTH];
				roi_half_x = (float)uval16 / 20.0;
				uval16 = *(uint16_t*)&_data[PROTOCOL_G66_DATA_ROI_HEIGHT];
				roi_half_y = (float)uval16 / 20.0;
				header_length = PROTOCOL_G66_DATA_HEADER_LENGHT;

				treatEchoMode(uval8, echo_mode, echo_mark_1, echo_mark_2);
				total_bytes = 30 * data_block_size / echo_mode + header_length + 6;
				point_bytes_ = 4;
			}
			if ((time_s & 0x8000000000000000) == 0x8000000000000000)
			{
				use_gps_src = true;
			}

			if (total_bytes != _recv_size)
			{
#ifdef DEBUG_INFO
				std::cout << std::endl
						  << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::cout << "Point cloud _data package size error! The package should be " << total_bytes << " bytes but "
						  << _recv_size << " bytes received." << std::endl;
				std::cout << "INFO: package size " << total_bytes << " / " << _recv_size << std::endl;
#endif // DEBUG_INFO
				return false;
			}

			check_sum = check_sum_with_protocol_version(protocol_version, _data, _recv_size - 6);
			if (memcmp(&check_sum, &_data[_recv_size - 6], 4) != 0)
			{
#ifdef DEBUG_INFO
				uint32_t recv_check_sum;
				memcpy(&recv_check_sum, &_data[_recv_size - 6], sizeof(uint32_t));
				std::cout << std::endl
						  << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::printf("Point cloud _data package check sum error! Check sum should be  0x%08x but receive 0x%08x!\n",
							check_sum, recv_check_sum);
				std::printf("INFO: point cloud _data package check sum 0x%08x<0x%08x>\n", check_sum, recv_check_sum);
#endif // DEBUG_INFO
				return false;
			}
			if (parse_a_frame_flag_)
			{
				nFrame_last_ = nFrame;
				parse_a_frame_flag_ = false;
			}
			diff = uint16_t(nFrame - nFrame_last_);
			if (diff >= 1)
			{
				if (rgb_enable_)
				{
					auto ptr = std::dynamic_pointer_cast<BwPointCloudGray>(point_cloud_tmp_);
					ptr->gray = gray_vector_;
					gray_vector_.clear();
				}
				else if (gray_enable_)
				{
					auto ptr = std::dynamic_pointer_cast<BwPointCloudRGB>(point_cloud_tmp_);
					ptr->rgb = rgb_vector_;
					rgb_vector_.clear();
				}
				// point_cloud_buffer_ = point_cloud_tmp_;
				return true;
			}

			if (_data[2] == PRODUCT_ID_X2)
			{
				for (int i = 38; i < (_recv_size - 6); i += point_bytes_)
				{
					memcpy(&uval8, &_data[i + 6], sizeof(uint8_t));
					if (long_range_version_X2_)
					{
						memcpy(&uval16, &_data[i], sizeof(uint16_t));
						pt_.x = ((float)uval16) * 0.01;
						memcpy(&uval16, &_data[i + 2], sizeof(uint16_t));
						if ((uval8 & 0x08) == 0x08)
							pt_.y = -((float)(0xffff - uval16)) * 0.01;
						else
							pt_.y = ((float)uval16) * 0.01;
					}
					else
					{
						memcpy(&dist16, &_data[i], sizeof(int16_t));
						pt_.x = dist16 * 0.01;
						memcpy(&dist16, &_data[i + 2], sizeof(int16_t));
						pt_.y = dist16 * 0.01;
					}
					memcpy(&dist16, &_data[i + 4], sizeof(int16_t));
					pt_.z = dist16 * 0.01;
					pt_.roi = uval8 & 0x03;
					uint8_t ch = uval8 & 0x04;
					if (ch == 0x00)
					{
						pt_.channel = 1;
					}
					else if (ch == 0x04)
					{
						pt_.channel = 2;
					}
					else
					{
						pt_.channel = 0;
					}

					if ((uval8 & 0x10) == 0x10)
						pt_.y = pt_.y + 0.0025;
					if ((uval8 & 0x20) == 0x20)
						pt_.y = pt_.y + 0.005;
					if ((uval8 & 0x40) == 0x40)
						pt_.z = pt_.z + 0.0025;
					if ((uval8 & 0x80) == 0x80)
						pt_.z = pt_.z + 0.005;

					memcpy(&pt_.intensity, &_data[i + 7], sizeof(uint8_t));
					memcpy(&time_offset, &_data[i + 8], sizeof(uint32_t));
					pt_.timestamp_s = time_s;
					pt_.timestamp_ns = time_ns + time_offset;
					if (pt_.timestamp_ns >= 1000000000)
					{
						pt_.timestamp_ns -= 1000000000;
						pt_.timestamp_s++;
					}
					pt_.row = nLine;

					point_cloud_tmp_->points.push_back(pt_);
					if (gray_enable_)
					{
						memcpy(&uval8, &_data[i + 12], sizeof(uint8_t));
						gray_vector_.push_back(uval8);
					}
					else if (rgb_enable_)
					{
						memcpy(&uval8, &_data[i + 12], sizeof(uint8_t));
						rgb_.r = uval8;
						memcpy(&uval8, &_data[i + 13], sizeof(uint8_t));
						rgb_.g = uval8;
						memcpy(&uval8, &_data[i + 14], sizeof(uint8_t));
						rgb_.b = uval8;
						rgb_vector_.push_back(rgb_);
					}
				}
			}
			else if (_data[2] == PRODUCT_ID_P4)
			{
				for (int i = 0; i < nPoint; i++)
				{
					// angle
					val16 = *(int16_t*)&_data[header_length + i * data_block_size];
					azimuth_h = val16 / 8192.0;

					val16 = *(int16_t*)&_data[header_length + i * data_block_size + 2];
					azimuth_v = val16 / 8192.0;
					// timestamp
					time_offset = *(int*)&_data[header_length + i * data_block_size + 4];
					if (use_gps_src)
						time_offset *= 1000;
					pt_.timestamp_s = time_s;
					if (time_offset < 0 && abs(time_offset) > time_ns)
					{
						pt_.timestamp_ns = 1000000000 + (int)time_ns + time_offset;
						pt_.timestamp_s--;
					}
					else
					{
						pt_.timestamp_ns = time_ns + time_offset;
						if (pt_.timestamp_ns >= 1000000000)
						{
							pt_.timestamp_s += pt_.timestamp_ns / 1000000000;
							pt_.timestamp_ns %= 1000000000;
						}
					}

					//sphere coord data assignment
					float cos_h = std::cos(azimuth_h);
					float sin_h = std::sin(azimuth_h);
					float cos_v, sin_v;
					// x y z intensity
					for (int j = 0; j < 16; j++)
					{
						dist32 = 0;
						if (enable_angle_calib_)
						{
							float azimuth_h_calib, azimuth_v_calib;
							switch (protocol_version)
							{
							case PROTOCOL_VERSION_AD2_C:
								angleCalibration_AD2_C(azimuth_h, azimuth_v, j, azimuth_h_calib, azimuth_v_calib);
								break;
							case PROTOCOL_VERSION_AD2_HH:
								angleCalibration_AD2_C(azimuth_h, azimuth_v, j, azimuth_h_calib, azimuth_v_calib);
								break;
							default:
								break;
							}

							cos_h = std::cos(azimuth_h_calib);
							sin_h = std::sin(azimuth_h_calib);
							cos_v = std::cos(azimuth_v_calib);
							sin_v = std::sin(azimuth_v_calib);
						}
						else
						{
							cos_v = std::cos(azimuth_v);
							sin_v = std::sin(azimuth_v);
						}

						switch (protocol_version)
						{
						case PROTOCOL_VERSION_AD2_B:
							memcpy(&dist32, &_data[header_length + i * data_block_size + 8 + j * point_bytes_ + 0], 3);
							pt_.intensity = *(uint8_t*)&_data[header_length + i * data_block_size + 8 + j * point_bytes_ + 3];
							break;
						case PROTOCOL_VERSION_AD2_C:
							memcpy(&dist32, &_data[header_length + i * data_block_size + 8 + j * point_bytes_ + 0], 2);
							pt_.intensity = *(uint8_t*)&_data[header_length + i * data_block_size + 8 + j * point_bytes_ + 2];
							pt_.confidence = (*(uint8_t*)&_data[header_length + i * data_block_size + 8 + j * point_bytes_ + 3]) & 0x03;
							break;
						case PROTOCOL_VERSION_AD2_HH:
							memcpy(&dist32, &_data[header_length + i * data_block_size + 8 + j * point_bytes_ + 0], 2);
							pt_.intensity = *(uint8_t*)&_data[header_length + i * data_block_size + 8 + j * point_bytes_ + 2];
							pt_.confidence = (*(uint8_t*)&_data[header_length + i * data_block_size + 8 + j * point_bytes_ + 3]);
							break;
						default:
							break;
						}
						dist_f = (float)dist32 / 100.0;
						pt_.x = dist_f * cos_v * cos_h;
						pt_.y = dist_f * cos_v * sin_h;
						pt_.z = dist_f * sin_v;
						pt_.row = nLine * 16 + (uint16_t)j;
						pt_.channel = (uint8_t)j;
						pt_.echo = echo_mark_1;

						// roi flag
						if (roi_center_x_flag != 0xFFFF && roi_center_y_flag != 0xFFFF)
						{
							switch (protocol_version)
							{
							case PROTOCOL_VERSION_AD2_B:
								if (std::fabs(azimuth_h - roi_center_x) < roi_half_x && std::fabs(azimuth_v - roi_center_y) < roi_half_y)
									pt_.roi = 1;
								else
									pt_.roi = 0;
								break;
							case PROTOCOL_VERSION_AD2_C:
								if (std::fabs(azimuth_h * 180.0 / kPI - roi_center_x) < roi_half_x && std::fabs(azimuth_v * 180.0 / kPI - roi_center_y) < roi_half_y)
									pt_.roi = 1;
								else
									pt_.roi = 0;
								break;
							case PROTOCOL_VERSION_AD2_HH:
								if ((pt_.confidence & 0x08) == 0x08)
									pt_.roi = 1;
								else
									pt_.roi = 0;
								break;
							default:
								break;
							}
						}
						else
							pt_.roi = 0;

						point_cloud_tmp_->points.push_back(pt_);

						if (echo_mode == 2)
						{
							switch (protocol_version)
							{
							case PROTOCOL_VERSION_AD2_B:
								memcpy(&dist32, &_data[header_length + i * data_block_size + 8 + 64 + j * point_bytes_ + 0], 3);
								pt_.intensity = *(uint8_t*)&_data[header_length + i * data_block_size + 8 + 64 + j * point_bytes_ + 3];
								break;
							case PROTOCOL_VERSION_AD2_C:
								memcpy(&dist32, &_data[header_length + i * data_block_size + 8 + 64 + j * point_bytes_ + 0], 2);
								pt_.intensity = *(uint8_t*)&_data[header_length + i * data_block_size + 8 + 64 + j * point_bytes_ + 2];
								pt_.confidence = (*(uint8_t*)&_data[header_length + i * data_block_size + 8 + 64 + j * point_bytes_ + 3]) & 0x03;
								break;
							case PROTOCOL_VERSION_AD2_HH:
								memcpy(&dist32, &_data[header_length + i * data_block_size + 8 + 64 + j * point_bytes_ + 0], 2);
								pt_.intensity = *(uint8_t*)&_data[header_length + i * data_block_size + 8 + 64 + j * point_bytes_ + 2];
								pt_.confidence = (*(uint8_t*)&_data[header_length + i * data_block_size + 8 + 64 + j * point_bytes_ + 3]);
								break;
							default:
								break;
							}
							dist_f = (float)dist32 / 100.0;
							pt_.x = dist_f * cos_v * cos_h;
							pt_.y = dist_f * cos_v * sin_h;
							pt_.z = dist_f * sin_v;
							pt_.row = nLine * 16 + (uint16_t)j;
							pt_.channel = (uint8_t)j;
							pt_.echo = echo_mark_2;

							// roi flag
							if (roi_center_x_flag != 0xFFFF && roi_center_y_flag != 0xFFFF)
							{
								switch (protocol_version)
								{
								case PROTOCOL_VERSION_AD2_B:
									if (std::fabs(azimuth_h - roi_center_x) < roi_half_x && std::fabs(azimuth_v - roi_center_y) < roi_half_y)
										pt_.roi = 1;
									else
										pt_.roi = 0;
									break;
								case PROTOCOL_VERSION_AD2_C:
									if (std::fabs(azimuth_h * 180.0 / kPI - roi_center_x) < roi_half_x && std::fabs(azimuth_v * 180.0 / kPI - roi_center_y) < roi_half_y)
										pt_.roi = 1;
									else
										pt_.roi = 0;
									break;
								case PROTOCOL_VERSION_AD2_HH:
									if ((pt_.confidence & 0x08) == 0x08)
										pt_.roi = 1;
									else
										pt_.roi = 0;
									break;
								default:
									break;
								}
							}
							else
								pt_.roi = 0;

							point_cloud_tmp_->points.push_back(pt_);
						}

						if (!enable_angle_calib_)
						{
							switch (protocol_version)
							{
							case PROTOCOL_VERSION_AD2_B:
								azimuth_v -= ang2Rad(kAngRes_AD2);
								break;
							case PROTOCOL_VERSION_AD2_C:
								azimuth_v -= ang2Rad(kAngRes_AD2 * 2);
								break;
							case PROTOCOL_VERSION_AD2_HH:
								azimuth_v -= ang2Rad(kAngRes_AD2 * 2);
								break;
							default:
								break;
							}
						}
					}
				}
			}
			else if (_data[2] == PRODUCT_ID_G66)
			{
				for (int i = 0; i < nPoint; i++)
				{
					// angle
					val16_ang_h = *(int16_t*)&_data[header_length + i * data_block_size];
					azimuth_h = val16_ang_h / 8192.0; // 2^13
					val16_ang_v = *(int16_t*)&_data[header_length + i * data_block_size + 2];
					azimuth_v = val16_ang_v / 8192.0;

					// timestamp
					time_offset = *(int*)&_data[header_length + i * data_block_size + 4];
					if (use_gps_src)
						time_offset *= 1000;
					pt_.timestamp_s = time_s;

					if (time_offset < 0 && abs(time_offset) > time_ns)
					{
						pt_.timestamp_ns = 1000000000 + (int)time_ns + time_offset;
						pt_.timestamp_s--;
					}
					else
					{
						pt_.timestamp_ns = time_ns + time_offset;
						if (pt_.timestamp_ns >= 1000000000)
						{
							pt_.timestamp_s += pt_.timestamp_ns / 1000000000;
							pt_.timestamp_ns %= 1000000000;
						}
					}

					//sphere coord data assignment
					float cos_h = std::cos(azimuth_h);
					float sin_h = std::sin(azimuth_h);
					float cos_v, sin_v;

					// x y z intensity
					for (int j = 0; j < 8; j++)
					{
						dist32 = 0;
						cos_v = std::cos(azimuth_v);
						sin_v = std::sin(azimuth_v);

						memcpy(&dist32, &_data[header_length + i * data_block_size + 8 + j * point_bytes_ + 0], 2);
						pt_.intensity = *(uint8_t*)&_data[header_length + i * data_block_size + 8 + j * point_bytes_ + 2];
						pt_.confidence = *(uint8_t*)&_data[header_length + i * data_block_size + 8 + j * point_bytes_ + 3];

						dist_f = (float)dist32 / 100.0;
						pt_.x = dist_f * cos_v * cos_h;
						pt_.y = dist_f * cos_v * sin_h;
						pt_.z = dist_f * sin_v;
						pt_.row = nLine * 8 + (uint16_t)j;
						pt_.channel = (uint8_t)j;
						pt_.echo = echo_mark_1;

						// roi flag
						if (roi_center_x_flag != 0xFFFF && roi_center_y_flag != 0xFFFF)
						{
							if ((pt_.confidence & 0x08) == 0x08)
								pt_.roi = 1;
							else
								pt_.roi = 0;
						}
						else
							pt_.roi = 0;

						point_cloud_tmp_->points.push_back(pt_);

						if (echo_mode == 2)
						{
							memcpy(&dist32, &_data[header_length + i * data_block_size + 8 + 32 + j * point_bytes_ + 0], 2);
							pt_.intensity = *(uint8_t*)&_data[header_length + i * data_block_size + 8 + 32 + j * point_bytes_ + 2];
							pt_.confidence = (*(uint8_t*)&_data[header_length + i * data_block_size + 8 + 32 + j * point_bytes_ + 3]);

							dist_f = (float)dist32 / 100.0;
							pt_.x = dist_f * cos_v * cos_h;
							pt_.y = dist_f * cos_v * sin_h;
							pt_.z = dist_f * sin_v;
							pt_.row = nLine * 8 + (uint16_t)j;
							pt_.channel = (uint8_t)j;
							pt_.echo = echo_mark_2;

							// roi flag
							if (roi_center_x_flag != 0xFFFF && roi_center_y_flag != 0xFFFF)
							{
								if ((pt_.confidence & 0x08) == 0x08)
									pt_.roi = 1;
								else
									pt_.roi = 0;
							}
							else
								pt_.roi = 0;

							point_cloud_tmp_->points.push_back(pt_);
						}
						azimuth_v -= ang2Rad(kAngRes_G66);
					}
				}
			}

			if (last_pkg_mark == 0x8000)
			{
				if (rgb_enable_)
				{
					auto ptr = std::dynamic_pointer_cast<BwPointCloudGray>(point_cloud_tmp_);
					ptr->gray = gray_vector_;
					gray_vector_.clear();
				}
				else if (gray_enable_)
				{
					auto ptr = std::dynamic_pointer_cast<BwPointCloudRGB>(point_cloud_tmp_);
					ptr->rgb = rgb_vector_;
					rgb_vector_.clear();
				}
				// point_cloud_buffer_ = point_cloud_tmp_;
				return true;
			}
		}
		else if (_data[3] != PROTOCOL_ID_DCSP_REQUEST && _data[3] != PROTOCOL_ID_DCSP_RESPONSE && _data[3] != PROTOCOL_ID_DSOP)
		{
#ifdef DEBUG_INFO
			std::cout << std::endl
					  << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::printf("Point cloud _data package header/tail error: 0x%02x 0x%02x 0x%02x 0x%02x ... 0x%02x 0x%02x!\n",
						_data[0], _data[1], _data[2], _data[3], _data[_recv_size - 2], _data[_recv_size - 1]);
#endif // DEBUG_INFO
		}
	}
	return false;
}

// Parse the packet to see if it is a new frame and determine whether it needs to be recorded
bool benewake::ParsePcap::isNewFrame(unsigned char *_data, int _recv_size)
{
	uint32_t check_sum = 0;

	// local variable
	uint32_t pkg_count = 0;
	uint16_t nFrame = 0;
	uint16_t nLine = 0;
	uint16_t nPoint = 0;
	uint64_t time_s = 0;
	uint32_t time_ns = 0;
	uint32_t diff = 0;
	uint16_t protocol_version = 0;
	uint8_t uval8 = 0, echo_mark_1 = 0, echo_mark_2 = 0;
	uint16_t uval16 = 0;
	uint16_t last_pkg_mark = 0;

	int total_bytes = 0, header_length = 38;
	int data_block_size = 0, echo_mode = 1;
	uint16_t roi_center_x_flag = 0, roi_center_y_flag = 0;
	float roi_center_x = 0, roi_center_y = 0;
	float roi_half_x = kPI * 15.0 / 180.0, roi_half_y = kPI * 6.4 / 180.0;

	if (_recv_size < 44)
	{
#ifdef DEBUG_INFO
		// std::cout << std::endl
		// 		  << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		// std::cout << "Point cloud _data package size error! The package should be larger than 44 bytes but only "
		// 		  << _recv_size << " bytes received." << std::endl;
#endif // DEBUG_INFO
		return false;
	}
	else
	{
		if (_data[0] == 0x42 && _data[1] == 0x57 && (_data[2] == PRODUCT_ID_X2 || _data[2] == PRODUCT_ID_P4 || _data[2] == PRODUCT_ID_G66) &&
			_data[3] == PROTOCOL_ID_MDOP && _data[_recv_size - 2] == 0x00 && _data[_recv_size - 1] == 0xff)
		{
			memcpy(&protocol_version, &_data[PROTOCOL_DATA_PACKAGE_VERSION_OFFSET], sizeof(uint16_t));
			memcpy(&pkg_count, &_data[PROTOCOL_DATA_PACKAGE_COUNT_OFFSET], sizeof(uint32_t));
			memcpy(&nFrame, &_data[PROTOCOL_DATA_FRAME_COUNT_OFFSET], sizeof(uint16_t));
			memcpy(&uval16, &_data[PROTOCOL_DATA_LINE_NUM_OFFSET], sizeof(uint16_t));
			nLine = uval16 & 0x7fff;
			last_pkg_mark = uval16 & 0x8000;
			memcpy(&nPoint, &_data[PROTOCOL_DATA_POINT_COUNT_OFFSET], sizeof(uint16_t));
			if (nFrame_last_ == 0)
				nFrame_last_ = nFrame;
			if (_data[2] == PRODUCT_ID_X2)
			{
				memcpy(&time_s, &_data[PROTOCOL_X_DATA_TIME_S_OFFSET], sizeof(uint32_t));
				memcpy(&time_ns, &_data[PROTOCOL_X_DATA_TIME_NS_OFFSET], sizeof(uint32_t));
				total_bytes = nPoint * point_bytes_ + 44;
			}
			else if (_data[2] == PRODUCT_ID_P4)
			{
				switch (protocol_version)
				{
				case PROTOCOL_VERSION_AD2_B:
					time_s = *(uint32_t*)&_data[PROTOCOL_X_DATA_TIME_S_OFFSET];
					time_ns = *(uint32_t*)&_data[PROTOCOL_X_DATA_TIME_NS_OFFSET];
					uval8 = *(uint8_t*)&_data[PROTOCOL_AD2_B_DATA_ECHO_MODE];
					roi_center_x_flag = *(uint16_t*)&_data[PROTOCOL_AD2_B_DATA_ROI_CENTER_X];
					roi_center_y_flag = *(uint16_t*)&_data[PROTOCOL_AD2_B_DATA_ROI_CENTER_Y];
					roi_center_x = (float)roi_center_x_flag / 8192.0;
					roi_center_y = (float)roi_center_y_flag / 8192.0;

					header_length = 38;
					break;

				case PROTOCOL_VERSION_AD2_C:
					time_s = *(uint64_t*)&_data[PROTOCOL_AD2_DATA_TIME_S_OFFSET];
					time_ns = *(uint32_t*)&_data[PROTOCOL_AD2_DATA_TIME_NS_OFFSET];
					uval8 = *(uint8_t*)&_data[PROTOCOL_AD2_DATA_ECHO_MODE];
					roi_center_x_flag = *(uint16_t*)&_data[PROTOCOL_AD2_DATA_ROI_CENTER_X];
					roi_center_y_flag = *(uint16_t*)&_data[PROTOCOL_AD2_DATA_ROI_CENTER_Y];
					roi_center_x = (float)roi_center_x_flag / 10.0;
					roi_center_y = (float)roi_center_y_flag / 10.0;
					uval16 = *(uint16_t*)&_data[PROTOCOL_AD2_DATA_ROI_WIDTH];
					roi_half_x = (float)uval16 / 20.0;
					uval16 = *(uint16_t*)&_data[PROTOCOL_AD2_DATA_ROI_HEIGHT];
					roi_half_y = (float)uval16 / 20.0;

					header_length = 42;
					break;

				case PROTOCOL_VERSION_AD2_HH:
					time_s = *(uint64_t*)&_data[PROTOCOL_AD2_DATA_TIME_S_OFFSET];
					time_ns = *(uint32_t*)&_data[PROTOCOL_AD2_DATA_TIME_NS_OFFSET];
					uval8 = *(uint8_t*)&_data[PROTOCOL_AD2_DATA_ECHO_MODE];
					roi_center_x_flag = *(uint16_t*)&_data[PROTOCOL_AD2_DATA_ROI_CENTER_X];
					roi_center_y_flag = *(uint16_t*)&_data[PROTOCOL_AD2_DATA_ROI_CENTER_Y];
					roi_center_x = (float)roi_center_x_flag / 10.0;
					roi_center_y = (float)roi_center_y_flag / 10.0;
					uval16 = *(uint16_t*)&_data[PROTOCOL_AD2_DATA_ROI_WIDTH];
					roi_half_x = (float)uval16 / 20.0;
					uval16 = *(uint16_t*)&_data[PROTOCOL_AD2_DATA_ROI_HEIGHT];
					roi_half_y = (float)uval16 / 20.0;

					header_length = 42;
					break;

				default:
					break;
				}
				point_bytes_ = 4;
				treatEchoMode(uval8, echo_mode, echo_mark_1, echo_mark_2);
				if (echo_mode == 1)
				{
					data_block_size = 72;
				}
				else
				{
					data_block_size = 136;
				}
				total_bytes = 12 * data_block_size / echo_mode + header_length + 6;
			}
			else if (_data[2] == PRODUCT_ID_G66)
			{
				data_block_size = *(uint16_t*)&_data[PROTOCOL_G66_DATA_BLOCK_SIZE];
				time_s = *(uint64_t*)&_data[PROTOCOL_G66_DATA_TIME_S_OFFSET];
				time_ns = *(uint32_t*)&_data[PROTOCOL_G66_DATA_TIME_NS_OFFSET];
				uval8 = *(uint8_t*)&_data[PROTOCOL_G66_DATA_ECHO_MODE];
				roi_center_x_flag = *(int16_t*)&_data[PROTOCOL_G66_DATA_ROI_CENTER_X];
				roi_center_y_flag = *(int16_t*)&_data[PROTOCOL_G66_DATA_ROI_CENTER_Y];
				roi_center_x = (float)roi_center_x_flag / 10.0;
				roi_center_y = (float)roi_center_y_flag / 10.0;
				uval16 = *(uint16_t*)&_data[PROTOCOL_G66_DATA_ROI_WIDTH];
				roi_half_x = (float)uval16 / 20.0;
				uval16 = *(uint16_t*)&_data[PROTOCOL_G66_DATA_ROI_HEIGHT];
				roi_half_y = (float)uval16 / 20.0;
				header_length = PROTOCOL_G66_DATA_HEADER_LENGHT;

				treatEchoMode(uval8, echo_mode, echo_mark_1, echo_mark_2);
				total_bytes = 30 * data_block_size / echo_mode + header_length + 6;
			}

			if (total_bytes != _recv_size)
			{
#ifdef DEBUG_INFO
				std::cout << std::endl
						  << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::cout << "Point cloud _data package size error! The package should be " << total_bytes << " bytes but "
						  << _recv_size << " bytes received." << std::endl;
				std::cout << "INFO: package size " << total_bytes << " / " << _recv_size << std::endl;
#endif // DEBUG_INFO
				return false;
			}

			check_sum = check_sum_with_protocol_version(protocol_version, _data, _recv_size - 6);
			if (memcmp(&check_sum, &_data[_recv_size - 6], 4) != 0)
			{
#ifdef DEBUG_INFO
				uint32_t recv_check_sum;
				memcpy(&recv_check_sum, &_data[_recv_size - 6], sizeof(uint32_t));
				std::cout << std::endl
						  << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::printf("Point cloud _data package check sum error! Check sum should be  0x%08x but receive 0x%08x!\n",
							check_sum, recv_check_sum);
				std::printf("INFO: point cloud _data package check sum 0x%08x<0x%08x>\n", check_sum, recv_check_sum);
#endif // DEBUG_INFO
				return false;
			}

			if (nFrame_last_ != nFrame)
			{
				nFrame_last_ = nFrame;
				// std::cout << "frame :" << nFrame_last_ << std::endl;

				if (parse_interval_flag_)
				{
					double now_time = time_s + (double)time_ns / 1000000000.0;

					if (last_frame_time_ > 0)
					{
						frame_interval_ = now_time - last_frame_time_;
						// printf("dur: %f\n", frame_interval_);
						parse_interval_flag_ = false;
					}
					last_frame_time_ = time_s + (double)time_ns / 1000000000.0;
				}

				return true;
			}
			else
			{
				return false;
			}
		}
		else if (_data[0] == 0x42 && _data[1] == 0x57 && _data[3] == PROTOCOL_ID_DSOP && _data[_recv_size - 2] == 0x00 && _data[_recv_size - 1] == 0xff)
		{
			if (device_type_ != _data[2])
			{
				device_type_ = _data[2];
				std::printf("switch device type to: 0x%02x\n", device_type_);
			}
		}
		else if (_data[0] == 0x42 && _data[1] == 0x57 && _data[2] == PRODUCT_ID_X2 && _data[3] == PROTOCOL_ID_DCSP_RESPONSE && _data[_recv_size - 2] == 0x00 && _data[_recv_size - 1] == 0xff)
		{
			if (_data[6] == DCSP_CMD_GET_DEVICE_INFORMATION)
			{
				std::string project_name, project_branch, project_version_major, project_version_minor, project_version_buid;
				unsigned char buf_8[9] = { 0 }, buf_2[3] = { 0 }, buf_16[17] = { 0 };

				// pl version
				memcpy(buf_8, &_data[6 + 25], 8 * sizeof(char));
				project_name = (char*)buf_8;
				int s = project_name.find_first_not_of(" ");
				s = s >= 0 ? s : 0;
				project_name = project_name.substr(s, 8 - s);

				memcpy(buf_8, &_data[6 + 33], 8 * sizeof(char));
				project_branch = (char*)buf_8;
				s = project_branch.find_first_not_of(" ");
				s = s >= 0 ? s : 0;
				project_branch = project_branch.substr(s, 8 - s);

				memcpy(buf_2, &_data[6 + 41], 2 * sizeof(char));
				project_version_major = (char*)buf_2;

				memcpy(buf_2, &_data[6 + 43], 2 * sizeof(char));
				project_version_minor = (char*)buf_2;

				memcpy(buf_2, &_data[6 + 45], 2 * sizeof(char));
				project_version_buid = (char*)buf_2;

				if (project_branch.substr(0, 3).compare("ZZX") == 0)
				{
					int major = std::stoi(project_version_major);
					int minor = std::stoi(project_version_minor);
					int build = std::stoi(project_version_buid);
					if (major > 1 || (major == 1 && minor > 5) || (major == 1 && minor == 5 && build >= 17))
					{
						long_range_version_X2_ = true;
						std::cout << std::endl << "Note: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
						std::cout << "Enable X2 long range version decoding!" << std::endl;
					}
					else
					{
						long_range_version_X2_ = false;
						std::cout << std::endl << "Note: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
						std::cout << "Disable X2 long range version decoding!" << std::endl;
					}
				}
				else
				{
					long_range_version_X2_ = false;
					std::cout << std::endl << "Note: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
					std::cout << "Disable X2 long range version decoding!" << std::endl;
				}
			}
		}
		else if (_data[0] == 0x42 && _data[1] == 0x57 && _data[3] != PROTOCOL_ID_DCSP_REQUEST && _data[3] != PROTOCOL_ID_DCSP_RESPONSE && _data[3] != PROTOCOL_ID_DSOP)
		{
#ifdef DEBUG_INFO
			std::cout << std::endl
					  << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::printf("Point cloud _data package header/tail error: 0x%02x 0x%02x 0x%02x 0x%02x ... 0x%02x 0x%02x!\n",
						_data[0], _data[1], _data[2], _data[3], _data[_recv_size - 2], _data[_recv_size - 1]);
#endif // DEBUG_INFO
		}
	}

	return false;
}

uint32_t benewake::ParsePcap::checksum(uint8_t *buffer, int size)
{
	uint32_t sum = 0;
	for (auto i = 0; i < size; ++i)
	{
		sum += buffer[i];
	}
	return sum;
}

uint32_t benewake::ParsePcap::check_crc32(uint8_t *_buffer, size_t _size)
{
	uint32_t crc;
	uint32_t i;

	i = 0;
	crc = 0;

	while (i < _size)
	{
		crc = (crc << 8) ^ const_crc_table[(crc >> 24) ^ _buffer[i]];
		i++;
	}
	return crc;
}

uint32_t benewake::ParsePcap::check_crc32_sb8_mode_begin(const uint8_t* _buffer, size_t _size)
{
	uint32_t li;
	uint32_t crc, term1, term2;
	uint32_t running_length;
	uint32_t end_bytes;
	uint32_t init_bytes = (uint32_t)_size % 4;
	crc = 0xFFFFFFFF;
	running_length = ((_size - init_bytes) / 8) * 8;
	end_bytes = _size - init_bytes - running_length;

	for (li = 0; li < init_bytes; li++)
		crc = crc_tableil8_o32[(crc ^ *_buffer++) & 0x000000FF] ^ (crc >> 8);
	for (li = 0; li < running_length / 8; li++)
	{
		crc ^= *(uint32_t*)_buffer;
		_buffer += 4;
		term1 = crc_tableil8_o88[crc & 0x000000FF] ^
			crc_tableil8_o80[(crc >> 8) & 0x000000FF];
		term2 = crc >> 16;
		crc = term1 ^
			crc_tableil8_o72[term2 & 0x000000FF] ^
			crc_tableil8_o64[(term2 >> 8) & 0x000000FF];
		term1 = crc_tableil8_o56[(*(uint32_t*)_buffer) & 0x000000FF] ^
			crc_tableil8_o48[((*(uint32_t*)_buffer) >> 8) & 0x000000FF];

		term2 = (*(uint32_t*)_buffer) >> 16;
		crc = crc ^
			term1 ^
			crc_tableil8_o40[term2 & 0x000000FF] ^
			crc_tableil8_o32[(term2 >> 8) & 0x000000FF];
		_buffer += 4;
	}
	for (li = 0; li < end_bytes; li++)
		crc = crc_tableil8_o32[(crc ^ *_buffer++) & 0x000000FF] ^ (crc >> 8);
	return crc;
}

uint32_t benewake::ParsePcap::check_sum_with_protocol_version(uint16_t _version, uint8_t *_buffer, size_t _size)
{
	switch (_version)
	{
	case PROTOCOL_CHECKSUM:
		return checksum(_buffer, _size);
		break;

	case PROTOCOL_VERSION_X:
		return check_crc32(_buffer, _size);
		break;

	case PROTOCOL_VERSION_AD2_B:
		return check_crc32_sb8_mode_begin(_buffer, _size);
		break;

	case PROTOCOL_VERSION_AD2_C:
		return check_crc32_sb8_mode_begin(_buffer, _size);
		break;

	case PROTOCOL_VERSION_AD2_HH:
		return check_crc32_sb8_mode_begin(_buffer, _size);
		break;

	case PROTOCOL_VERSION_G66:
		return check_crc32_sb8_mode_begin(_buffer, _size);
		break;

	default:
		break;
	}
	return 0;
}

bool benewake::ParsePcap::parseTime(unsigned char *_data, int _recv_size)
{
	uint32_t check_sum = 0;
	// local variable
	uint32_t pkg_count = 0;
	uint16_t nFrame = 0;
	uint16_t nLine = 0;
	uint16_t nPoint = 0;
	uint64_t time_s = 0;
	uint32_t time_ns = 0;
	uint32_t diff = 0;
	uint16_t protocol_version = 0;
	uint8_t uval8 = 0, echo_mark_1 = 0, echo_mark_2 = 0;
	uint16_t uval16 = 0;
	uint16_t last_pkg_mark = 0;

	int total_bytes = 0, header_length = 38;
	int data_block_size = 0, echo_mode = 1;
	uint16_t roi_center_x_flag = 0, roi_center_y_flag = 0;
	float roi_center_x = 0, roi_center_y = 0;
	float roi_half_x = kPI * 15.0 / 180.0, roi_half_y = kPI * 6.4 / 180.0;

	if (_recv_size < 44)
	{
#ifdef DEBUG_INFO
		// std::cout << std::endl
		// 		  << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		// std::cout << "Point cloud _data package size error! The package should be larger than 44 bytes but only "
		// 		  << _recv_size << " bytes received." << std::endl;
#endif // DEBUG_INFO
		return false;
	}
	else
	{
		if (_data[0] == 0x42 && _data[1] == 0x57 && (_data[2] == PRODUCT_ID_X2 || _data[2] == PRODUCT_ID_P4 || _data[2] == PRODUCT_ID_G66) &&
			_data[3] == PROTOCOL_ID_MDOP && _data[_recv_size - 2] == 0x00 && _data[_recv_size - 1] == 0xff)
		{
			memcpy(&protocol_version, &_data[PROTOCOL_DATA_PACKAGE_VERSION_OFFSET], sizeof(uint16_t));
			memcpy(&pkg_count, &_data[PROTOCOL_DATA_PACKAGE_COUNT_OFFSET], sizeof(uint32_t));
			memcpy(&nFrame, &_data[PROTOCOL_DATA_FRAME_COUNT_OFFSET], sizeof(uint16_t));
			memcpy(&uval16, &_data[PROTOCOL_DATA_LINE_NUM_OFFSET], sizeof(uint16_t));
			nLine = uval16 & 0x7fff;
			last_pkg_mark = uval16 & 0x8000;
			memcpy(&nPoint, &_data[PROTOCOL_DATA_POINT_COUNT_OFFSET], sizeof(uint16_t));
			if (_data[2] == PRODUCT_ID_X2)
			{
				memcpy(&time_s, &_data[PROTOCOL_X_DATA_TIME_S_OFFSET], sizeof(uint32_t));
				memcpy(&time_ns, &_data[PROTOCOL_X_DATA_TIME_NS_OFFSET], sizeof(uint32_t));
				total_bytes = nPoint * point_bytes_ + 44;
			}
			else if (_data[2] == PRODUCT_ID_P4)
			{
				switch (protocol_version)
				{
				case PROTOCOL_VERSION_AD2_B:
					time_s = *(uint32_t*)&_data[PROTOCOL_X_DATA_TIME_S_OFFSET];
					time_ns = *(uint32_t*)&_data[PROTOCOL_X_DATA_TIME_NS_OFFSET];
					uval8 = *(uint8_t*)&_data[PROTOCOL_AD2_B_DATA_ECHO_MODE];
					roi_center_x_flag = *(uint16_t*)&_data[PROTOCOL_AD2_B_DATA_ROI_CENTER_X];
					roi_center_y_flag = *(uint16_t*)&_data[PROTOCOL_AD2_B_DATA_ROI_CENTER_Y];
					roi_center_x = (float)roi_center_x_flag / 8192.0;
					roi_center_y = (float)roi_center_y_flag / 8192.0;

					header_length = 38;
					break;

				case PROTOCOL_VERSION_AD2_C:
					time_s = *(uint64_t*)&_data[PROTOCOL_AD2_DATA_TIME_S_OFFSET];
					time_ns = *(uint32_t*)&_data[PROTOCOL_AD2_DATA_TIME_NS_OFFSET];
					uval8 = *(uint8_t*)&_data[PROTOCOL_AD2_DATA_ECHO_MODE];
					roi_center_x_flag = *(uint16_t*)&_data[PROTOCOL_AD2_DATA_ROI_CENTER_X];
					roi_center_y_flag = *(uint16_t*)&_data[PROTOCOL_AD2_DATA_ROI_CENTER_Y];
					roi_center_x = (float)roi_center_x_flag / 10.0;
					roi_center_y = (float)roi_center_y_flag / 10.0;
					uval16 = *(uint16_t*)&_data[PROTOCOL_AD2_DATA_ROI_WIDTH];
					roi_half_x = (float)uval16 / 20.0;
					uval16 = *(uint16_t*)&_data[PROTOCOL_AD2_DATA_ROI_HEIGHT];
					roi_half_y = (float)uval16 / 20.0;

					header_length = 42;
					break;

				case PROTOCOL_VERSION_AD2_HH:
					time_s = *(uint64_t*)&_data[PROTOCOL_AD2_DATA_TIME_S_OFFSET];
					time_ns = *(uint32_t*)&_data[PROTOCOL_AD2_DATA_TIME_NS_OFFSET];
					uval8 = *(uint8_t*)&_data[PROTOCOL_AD2_DATA_ECHO_MODE];
					roi_center_x_flag = *(uint16_t*)&_data[PROTOCOL_AD2_DATA_ROI_CENTER_X];
					roi_center_y_flag = *(uint16_t*)&_data[PROTOCOL_AD2_DATA_ROI_CENTER_Y];
					roi_center_x = (float)roi_center_x_flag / 10.0;
					roi_center_y = (float)roi_center_y_flag / 10.0;
					uval16 = *(uint16_t*)&_data[PROTOCOL_AD2_DATA_ROI_WIDTH];
					roi_half_x = (float)uval16 / 20.0;
					uval16 = *(uint16_t*)&_data[PROTOCOL_AD2_DATA_ROI_HEIGHT];
					roi_half_y = (float)uval16 / 20.0;

					header_length = 42;
					break;

				default:
					break;
				}
				point_bytes_ = 4;
				treatEchoMode(uval8, echo_mode, echo_mark_1, echo_mark_2);
				if (echo_mode == 1)
				{
					data_block_size = 72;
				}
				else
				{
					data_block_size = 136;
				}
				total_bytes = 12 * data_block_size / echo_mode + header_length + 6;
			}
			else if (_data[2] == PRODUCT_ID_G66)
			{
				data_block_size = *(uint16_t*)&_data[PROTOCOL_G66_DATA_BLOCK_SIZE];
				time_s = *(uint64_t*)&_data[PROTOCOL_G66_DATA_TIME_S_OFFSET];
				time_ns = *(uint32_t*)&_data[PROTOCOL_G66_DATA_TIME_NS_OFFSET];
				uval8 = *(uint8_t*)&_data[PROTOCOL_G66_DATA_ECHO_MODE];
				roi_center_x_flag = *(int16_t*)&_data[PROTOCOL_G66_DATA_ROI_CENTER_X];
				roi_center_y_flag = *(int16_t*)&_data[PROTOCOL_G66_DATA_ROI_CENTER_Y];
				roi_center_x = (float)roi_center_x_flag / 10.0;
				roi_center_y = (float)roi_center_y_flag / 10.0;
				uval16 = *(uint16_t*)&_data[PROTOCOL_G66_DATA_ROI_WIDTH];
				roi_half_x = (float)uval16 / 20.0;
				uval16 = *(uint16_t*)&_data[PROTOCOL_G66_DATA_ROI_HEIGHT];
				roi_half_y = (float)uval16 / 20.0;
				header_length = PROTOCOL_G66_DATA_HEADER_LENGHT;

				treatEchoMode(uval8, echo_mode, echo_mark_1, echo_mark_2);
				total_bytes = 30 * data_block_size / echo_mode + header_length + 6;
			}

			if (total_bytes != _recv_size)
			{
#ifdef DEBUG_INFO
				std::cout << std::endl
						  << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::cout << "Point cloud _data package size error! The package should be " << total_bytes << " bytes but "
						  << _recv_size << " bytes received." << std::endl;
				std::cout << "INFO: package size " << total_bytes << " / " << _recv_size << std::endl;
#endif // DEBUG_INFO
				return false;
			}

			check_sum = check_sum_with_protocol_version(protocol_version, _data, _recv_size - 6);
			if (memcmp(&check_sum, &_data[_recv_size - 6], 4) != 0)
			{
#ifdef DEBUG_INFO
				uint32_t recv_check_sum;
				memcpy(&recv_check_sum, &_data[_recv_size - 6], sizeof(uint32_t));
				std::cout << std::endl
						  << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::printf("Point cloud _data package check sum error! Check sum should be  0x%08x but receive 0x%08x!\n",
							check_sum, recv_check_sum);
				std::printf("INFO: point cloud _data package check sum 0x%08x<0x%08x>\n", check_sum, recv_check_sum);
#endif // DEBUG_INFO
				return false;
			}

			if (nFrame_last_ != nFrame)
			{
				nFrame_last_ = nFrame;
				std::cout << "frame :" << nFrame_last_ << std::endl;
				std::cout << "time :  " << time_s << " ns: " << time_ns << std::endl;
				double now_time = time_s + (double)time_ns / 1000000000.0;

				if (last_frame_time_ > 0)
				{
					printf("dur: %f\n", (now_time - last_frame_time_));
					return true;
				}
				last_frame_time_ = now_time;
			}
			else
			{
				return false;
			}
		}
		else
		{
#ifdef DEBUG_INFO
			std::cout << std::endl
					  << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::printf("Point cloud _data package header/tail error: 0x%02x 0x%02x 0x%02x 0x%02x ... 0x%02x 0x%02x!\n",
						_data[0], _data[1], _data[2], _data[3], _data[_recv_size - 2], _data[_recv_size - 1]);
#endif // DEBUG_INFO
		}
	}

	return false;
}

void benewake::ParsePcap::angleCalibration_AD2_C(float _in_azimuth, float _in_elevation, int _in_ch, float& _out_azimuth, float& _out_elevation)
{
	_out_azimuth = _in_azimuth + (_in_elevation - 0.01309) * 0.00166491 * _in_ch;
	_out_elevation = _in_elevation - _in_ch * (0.001745 - abs((_in_elevation - 0.01309) * 0.00016));
}

void benewake::ParsePcap::treatEchoMode(uint8_t _echo_flag, int& _echo_mode, uint8_t& _echo_mark_1, uint8_t& _echo_mark_2)
{
	if (_echo_flag <= 0x02)
	{
		_echo_mode = 1;
		if (_echo_flag == 0x00)
			_echo_mark_1 = 0x01;
		else if (_echo_flag == 0x01)
			_echo_mark_1 = 0x02;
		else
			_echo_mark_1 = 0x04;
	}
	else
	{
		_echo_mode = 2;
		if (_echo_flag == 0x03)
		{
			_echo_mark_1 = 0x02;
			_echo_mark_2 = 0x04;
		}
		else if (_echo_flag == 0x04)
		{
			_echo_mark_1 = 0x01;
			_echo_mark_2 = 0x04;
		}
		else
		{
			_echo_mark_1 = 0x01;
			_echo_mark_2 = 0x02;
		}
	}
}
