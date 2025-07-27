#include "benewake_dsop.h"

benewake::DSOPProtocol::DSOPProtocol(std::string _local_ip, int _local_port, std::string _remote_ip, int _remote_port, UDPType _udp_type) :
	Protocol(_local_ip, _local_port, _remote_ip, _remote_port, _udp_type)
{
	//if ( _udp_type != benewake::UDPType::BOARDCAST)
	//{
	//	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	//	std::cout << "Heart beat receiving settings error!" << std::endl;
	//}
#ifdef _WIN32
	occup_ = CreateSemaphore(NULL, 1, 1, NULL);
	callback_occup_ = CreateSemaphore(NULL, 1, 1, NULL);
#else
	sem_init(&occup_, 0, 1);
	sem_init(&callback_occup_, 0, 1);
#endif // _WIN32
	int ret = this->open();
	if (ret == STATUS_OK)
	{
		thread_run_ = true;
		listen_thread_ = new std::thread(&DSOPProtocol::listenHeartBeat, this);
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cout << "Open heart beat listener filed!" << std::endl;
		thread_run_ = false;
	}
	callback_enable_ = false;
}

benewake::DSOPProtocol::~DSOPProtocol()
{
	if (thread_run_)
	{
		thread_run_ = false;
		listen_thread_->join();
	}
}

void benewake::DSOPProtocol::dsop_cancel_heart_beat_callback()
{
	callback_enable_ = false;
#ifdef _WIN32
	WaitForSingleObject(callback_occup_, INFINITE);
#else
	sem_wait(&callback_occup_);
#endif // _WIN32
	if (callback_pointer_ != NULL)
		callback_pointer_ = NULL;
	if (callback_func_ != NULL)
		callback_func_ = NULL;
#ifdef _WIN32
	ReleaseSemaphore(callback_occup_, 1, NULL);
#else
	sem_post(&callback_occup_);
#endif // _WIN32
}

void benewake::DSOPProtocol::dsop_disable_heart_beat_receiver(std::string _ip, int _dcsp_port, uint8_t _device_type, uint16_t _protocol_version)
{
	thread_run_ = false;
	listen_thread_->join();
	sys_info_.ip = _ip;
	sys_info_.dcsp_port = _dcsp_port;
	sys_info_.device_type = _device_type;
	sys_info_.comm_stat = benewake::COMM_STATUS::COMM_OK;
	sys_info_.sys_err = benewake::SYS_ERROR::ERR_OK;
	sys_info_.sys_stat = benewake::SYS_STATUS::STAT_STANDBY;
	sys_info_.protocol = _protocol_version;
	heart_beat_disabled_ = true;

	if (dsop_info_ != nullptr)
	{
		dsop_info_->ip = sys_info_.ip;
		dsop_info_->dcsp_port = sys_info_.dcsp_port;
		dsop_info_->device_type = sys_info_.device_type;
		dsop_info_->comm_stat = sys_info_.comm_stat;
		dsop_info_->sys_err = sys_info_.sys_err;
		dsop_info_->sys_stat = sys_info_.sys_stat;
		dsop_info_->protocol = sys_info_.protocol;
	}
}

benewake::SYS_INFO benewake::DSOPProtocol::dsop_get_system_information()
{
	SYS_INFO info;
	if (heart_beat_disabled_)
	{
		info = sys_info_;
		return info;
	}

	if (!new_pkg_)
	{
		if (!thread_run_)
		{
#ifdef DEBUG_INFO
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Heart beat receiver thread is not running!" << std::endl;
#endif // DEBUG_INFO
			return info;
		}
		std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
		std::chrono::system_clock::time_point stop;
		while (!new_pkg_ && thread_run_)
		{
#ifdef _WIN32
			Sleep(50);
#else
			usleep(50000);
#endif // _WIN32
			stop = std::chrono::system_clock::now();
			auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
			if (dur.count() > 2000)
			{
#ifdef DEBUG_INFO
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::cout << "Wait for heart beat time out!" << std::endl;
#endif // DEBUG_INFO
				return info;
			}
		}
	}
#ifdef _WIN32
	WaitForSingleObject(occup_, INFINITE);
#else
	sem_wait(&occup_);
#endif // _WIN32
	info = sys_info_;
	new_pkg_ = false;
#ifdef _WIN32
	ReleaseSemaphore(occup_, 1, NULL);
#else
	sem_post(&occup_);
#endif // _WIN32
	return info;
}

benewake::SYS_INFO benewake::DSOPProtocol::dsop_get_system_information_no_wait()
{
	SYS_INFO info;
	if (heart_beat_disabled_)
	{
		info = sys_info_;
		return info;
	}

#ifdef _WIN32
	WaitForSingleObject(occup_, INFINITE);
#else
	sem_wait(&occup_);
#endif // _WIN32
	info = sys_info_;
	new_pkg_ = false;
#ifdef _WIN32
	ReleaseSemaphore(occup_, 1, NULL);
#else
	sem_post(&occup_);
#endif // _WIN32
	return info;
}

void benewake::DSOPProtocol::dsop_regist_heart_beat_callback(HeartBeatCallbcakFunc _func, void * _pData)
{
#ifdef _WIN32
	WaitForSingleObject(callback_occup_, INFINITE);
#else
	sem_wait(&callback_occup_);
#endif // _WIN32
	callback_func_ = _func;
	callback_pointer_ = _pData;
#ifdef _WIN32
	ReleaseSemaphore(callback_occup_, 1, NULL);
#else
	sem_post(&callback_occup_);
#endif // _WIN32
	callback_enable_ = true;
}

void benewake::DSOPProtocol::dsop_search_device(std::vector<SYS_INFO> & _device_list, int _search_milliseconds)
{
	device_list_.clear();
	device_search_time_ = _search_milliseconds;
	device_search_start_ = std::chrono::system_clock::now();
	device_searching_ = true;
	while (device_searching_ && thread_run_)
	{
#ifdef _WIN32
		Sleep(100);
#else
		usleep(100000);
#endif // _WIN32
	}
	_device_list = device_list_;
}

int benewake::DSOPProtocol::reset_connection(std::string _local_ip, int _local_port, std::string _remote_ip, int _remote_port, UDPType _udp_type)
{
	//if (_udp_type != benewake::UDPType::BOARDCAST)
	//{
	//	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	//	std::cout << "Heart beat receiving settings error!" << std::endl;
	//	return STATUS_FAIL;
	//}

	int status;
	local_ip_ = _local_ip;
	local_port_ = _local_port;
	remote_ip_ = _remote_ip;
	remote_port_ = _remote_port;
	connection_type_ = _udp_type;
	if (pudp_ != nullptr)
	{
		if (thread_run_)
		{
			thread_run_ = false;
			listen_thread_->join();
		}
	}
	pudp_->disconnect();
	pudp_.reset(new Udp(local_ip_, local_port_, remote_ip_, remote_port_));
	status = pudp_->connect(connection_type_);
	if (status == STATUS_OK)
	{
		thread_run_ = true;
		listen_thread_ = new std::thread(&DSOPProtocol::listenHeartBeat, this);
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cout << "Open heart beat listener filed!" << std::endl;
		thread_run_ = false;
	}
	return STATUS_OK;
}

void benewake::DSOPProtocol::setDeviceInfoPtr(SYS_INFO* _info)
{
	dsop_info_ = _info;
	if (heart_beat_disabled_)
	{
		dsop_info_->ip = sys_info_.ip;
		dsop_info_->dcsp_port = sys_info_.dcsp_port;
		dsop_info_->device_type = sys_info_.device_type;
		dsop_info_->comm_stat = sys_info_.comm_stat;
		dsop_info_->sys_err = sys_info_.sys_err;
		dsop_info_->sys_stat = sys_info_.sys_stat;
		dsop_info_->protocol = sys_info_.protocol;
	}
}

void benewake::DSOPProtocol::listenHeartBeat()
{
	unsigned char data[PROTOCOL_DATA_PACKAGE_MAX_LENGTH];
	int status = 0, recv_size;
	unsigned char sn_char[33] = { 0 };
	SYS_INFO info;
	uint32_t pkg_count = 0, pkg_count_last = 0, time_s = 0, time_ns = 0, total_lost_package = 0, check_sum = 0;
	int year = 0, month = 0, day = 0, hour = 0, minute = 0;

	while (thread_run_)
	{
		status = pudp_->recvSomeData(data, PROTOCOL_DATA_PACKAGE_MAX_LENGTH, 3, 0);
		if (status == STATUS_FAIL)
		{
			info = SYS_INFO();
			info.comm_stat = COMM_STATUS::COMM_FAIL;
#ifdef _WIN32
			WaitForSingleObject(occup_, INFINITE);
#else
			sem_wait(&occup_);
#endif // _WIN32
			sys_info_ = info;
			new_pkg_ = true;
#ifdef _WIN32
			ReleaseSemaphore(occup_, 1, NULL);
#else
			sem_post(&occup_);
#endif // _WIN32

#ifdef DEBUG_INFO
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Receive heart beat data failed!" << std::endl;
#endif // DEBUG_INFO
			if (device_searching_)
			{
				device_search_elapsed_ = std::chrono::system_clock::now();
				auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(device_search_elapsed_ - device_search_start_);
				if (dur.count() > device_search_time_)
					device_searching_ = false;
			}
			if (callback_enable_)
			{
#ifdef _WIN32
				WaitForSingleObject(callback_occup_, INFINITE);
#else
				sem_wait(&callback_occup_);
#endif // _WIN32
				if (callback_func_ != NULL)
					callback_func_(info, callback_pointer_);
#ifdef _WIN32
				ReleaseSemaphore(callback_occup_, 1, NULL);
#else
				sem_post(&callback_occup_);
#endif // _WIN32
			}
		}
		else if (status == STATUS_TIME_OUT)
		{
			info = SYS_INFO();
			info.comm_stat = COMM_STATUS::COMM_TIMEOUT;
#ifdef _WIN32
			WaitForSingleObject(occup_, INFINITE);
#else
			sem_wait(&occup_);
#endif // _WIN32
			sys_info_ = info;
			new_pkg_ = true;
#ifdef _WIN32
			ReleaseSemaphore(occup_, 1, NULL);
#else
			sem_post(&occup_);
#endif // _WIN32

#ifdef DEBUG_INFO
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Receive heart beat data from " << remote_ip_ << " time out!" << std::endl;
#endif // DEBUG_INFO
			if (device_searching_)
			{
				device_search_elapsed_ = std::chrono::system_clock::now();
				auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(device_search_elapsed_ - device_search_start_);
				if (dur.count() > device_search_time_)
					device_searching_ = false;
			}
			if (callback_enable_)
			{
#ifdef _WIN32
				WaitForSingleObject(callback_occup_, INFINITE);
#else
				sem_wait(&callback_occup_);
#endif // _WIN32
				if (callback_func_ != NULL)
					callback_func_(info, callback_pointer_);
#ifdef _WIN32
				ReleaseSemaphore(callback_occup_, 1, NULL);
#else
				sem_post(&callback_occup_);
#endif // _WIN32
			}
		}
		else
		{
			recv_size = status;
			if (data[0] == 0x42 && data[1] == 0x57 && data[3] == PROTOCOL_ID_DSOP
				&& data[recv_size - 2] == 0x00 && data[recv_size - 1] == 0xff)
			{
				// check device type
				if (data[2] == PRODUCT_ID_X2)
					device_type_ = PRODUCT_ID_X2;
				else if (data[2] == PRODUCT_ID_P4)
					device_type_ = PRODUCT_ID_P4;
				else if (data[2] == PRODUCT_ID_G66)
					device_type_ = PRODUCT_ID_G66;
				else
				{
#ifdef DEBUG_INFO
					std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
					std::cout << "Unknown device type found: " << (int)device_type_ << std::endl;
#endif // DEBUG_INFO
				}
				info.device_type = device_type_;

				uint16_t protocol_version = 0;
				memcpy(&protocol_version, &data[PROTOCOL_DATA_PACKAGE_VERSION_OFFSET], sizeof(uint16_t));
				check_sum = check_sum_with_protocol_version(protocol_version, data, recv_size - 6);
				if (memcmp(&check_sum, &data[recv_size - 6], 4) != 0)
				{
					char check_bytes[4];
					check_bytes[0] = (check_sum >> 0) & 0xff;
					check_bytes[1] = (check_sum >> 8) & 0xff;
					check_bytes[2] = (check_sum >> 16) & 0xff;
					check_bytes[3] = (check_sum >> 24) & 0xff;
					std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
					printf("Heart beat data package check error! Check bytes should be 0x%02x 0x%02x 0x%02x 0x%02x but receive 0x%02x 0x%02x 0x%02x 0x%02x.\n",
						check_bytes[0], check_bytes[1], check_bytes[2], check_bytes[3], 
						data[recv_size - 6], data[recv_size - 5], data[recv_size - 4], data[recv_size - 3]);
					continue;
				}

				info.comm_stat = COMM_STATUS::COMM_OK;
				std::string ip = inet_ntoa(pudp_->getRecvInfo().sin_addr);
				info.ip = ip;
				memcpy(sn_char, &data[6], 32);
				info.device_sn = (char *)sn_char;
				memcpy(&info.dcsp_port, &data[38], sizeof(uint16_t));
				memcpy(&info.timestamp_s, &data[40], sizeof(uint32_t));
				memcpy(&info.timestamp_ns, &data[44], sizeof(uint32_t));
				
				if ((info.timestamp_s & 0x80000000) == 0x80000000)
				{
					year = (info.timestamp_s & 0x3f000000) >> 24;
					month = (info.timestamp_s & 0x00fc0000) >> 18;
					day = (info.timestamp_s & 0x0003f000) >> 12;
					hour = (info.timestamp_s & 0x00000fc0) >> 6;
					minute = (info.timestamp_s & 0x0000003f);

					utc_time_t utcTime;
					utcTime.year = year + 2000;
					utcTime.month = month;
					utcTime.day = day;
					utcTime.hour = hour;
					utcTime.minute = minute;
					utcTime.second = info.timestamp_ns / 1000000 % 100;
					info.timestamp_s = covUTC2UnixTimestamp(&utcTime);

					info.timestamp_ns = info.timestamp_ns % 1000000 * 1000;
				}
				if (info.timestamp_ns >= 1000000000) {
					info.timestamp_ns -= 1000000000;
					info.timestamp_s++;
				}
				memcpy(&info.count, &data[48], sizeof(uint32_t));
				info.protocol = protocol_version;

				if (device_type_ == PRODUCT_ID_X2)
				{
					processPayloadX(info, data);
				}
				else if (device_type_ == PRODUCT_ID_P4)
				{
					processPayloadP4(info, data);
				}
				else if (device_type_ == PRODUCT_ID_G66)
				{
					processPayloadP4(info, data);
				}

				if(dsop_info_ != nullptr && dsop_info_->ip.compare(info.ip) == 0 && dsop_info_->protocol != protocol_version)
				{
					std::cout << "device ip: " << info.ip << " protocol_version changed: " << protocol_version << std::endl;
					dsop_info_->protocol = protocol_version;
				}
#ifdef _WIN32
				WaitForSingleObject(occup_, INFINITE);
#else
				sem_wait(&occup_);
#endif // _WIN32
				sys_info_ = info;
				new_pkg_ = true;
#ifdef _WIN32
				ReleaseSemaphore(occup_, 1, NULL);
#else
				sem_post(&occup_);
#endif // _WIN32
				if (device_searching_)
				{
					bool added = false;
					for (int i = 0; i < device_list_.size() && !added; i++)
					{
						if (info.ip == device_list_[i].ip)
							added = true;
					}
					if (!added)
						device_list_.push_back(info);

					device_search_elapsed_ = std::chrono::system_clock::now();
					auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(device_search_elapsed_ - device_search_start_);
					if (dur.count() > device_search_time_ || remote_ip_.size() != 0)
						device_searching_ = false;
				}

				if (callback_enable_)
				{
#ifdef _WIN32
					WaitForSingleObject(callback_occup_, INFINITE);
#else
					sem_wait(&callback_occup_);
#endif // _WIN32
					if (callback_func_ != NULL)
						callback_func_(info, callback_pointer_);
#ifdef _WIN32
					ReleaseSemaphore(callback_occup_, 1, NULL);
#else
					sem_post(&callback_occup_);
#endif // _WIN32
				}
			}
			else
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::cout << "Heart beat package header/tail error: "
					<< data[0] << " " << data[1] << " " << data[2] << " " << data[3] << " ... "
					<< data[recv_size - 2] << " " << data[recv_size - 1] << "!" << std::endl;
				if (device_searching_)
				{
					device_search_elapsed_ = std::chrono::system_clock::now();
					auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(device_search_elapsed_ - device_search_start_);
					if (dur.count() > device_search_time_)
						device_searching_ = false;
				}
			}
		}
	}
}

void benewake::DSOPProtocol::processPayloadX(SYS_INFO& _info, unsigned char* _data)
{
	uint32_t uval32;
	uint32_t temp = 0;
	float f_temp = 0.0;

	memcpy(&_info.sys_err, &_data[52], sizeof(uint32_t));

	memcpy(&uval32, &_data[64], sizeof(uint32_t));
	_info.sys_stat = SYS_STATUS(uval32 & 0x07);
	_info.sig_stat.stat_PPS = (uval32 & 0x08) > 0;
	_info.sig_stat.stat_GPRMC = (uval32 & 0x10) > 0;
	_info.sig_stat.stat_NTP = (uval32 & 0x20) > 0;
	_info.sig_stat.stat_1588 = (uval32 & 0x40) > 0;
	_info.sig_stat.stat_IMU = (uval32 & 0x80) > 0;
	_info.multicast = (uval32 & 0x0100) > 0;

	memcpy(&_info.fan_pwn, &_data[68], sizeof(uint16_t));
	memcpy(&_info.lidar_work_mode, &_data[70], sizeof(uint16_t));
	memcpy(&temp, &_data[72], sizeof(uint32_t));
	f_temp = float(temp);
	_info.temperature = f_temp / 1000.0;
}

void benewake::DSOPProtocol::processPayloadP4(SYS_INFO& _info, unsigned char* _data)
{
	uint32_t uval32;

	if (_info.protocol == PROTOCOL_VERSION_AD2_B || _info.protocol == PROTOCOL_VERSION_AD2_C)
	{
		memcpy(&_info.sys_err, &_data[52], sizeof(uint32_t));

		memcpy(&uval32, &_data[64], sizeof(uint32_t));
		_info.sys_stat = SYS_STATUS(uval32 & 0x07);
		_info.sig_stat.stat_PPS = (uval32 & 0x08) > 0;
		_info.sig_stat.stat_GPRMC = (uval32 & 0x10) > 0;
		_info.sig_stat.stat_NTP = (uval32 & 0x20) > 0;
		_info.sig_stat.stat_1588 = (uval32 & 0x40) > 0;
		_info.sig_stat.stat_IMU = (uval32 & 0x80) > 0;
		_info.multicast = (uval32 & 0x0100) > 0;

		memcpy(&_info.func_safety.FS_version, &_data[68], sizeof(uint8_t));
		memcpy(&_info.func_safety.FS_state, &_data[69], sizeof(uint8_t));
		memcpy(&_info.func_safety.fault_code_type, &_data[70], sizeof(uint8_t));
		memcpy(&_info.func_safety.fault_code, &_data[71], sizeof(uint64_t));
		memcpy(&_info.func_safety.total_fault_code_num, &_data[79], sizeof(uint16_t));
		memcpy(&_info.func_safety.fault_index, &_data[81], sizeof(uint8_t));
	}
	else if (_info.protocol == PROTOCOL_VERSION_AD2_HH || _info.protocol == PROTOCOL_VERSION_G66)
	{
		uint8_t content_type = _data[52];
		_info.msg_type = content_type;
		if (content_type == 0x00) // common heart beat
		{
			int16_t val16;
			memcpy(&_info.voltage, &_data[53], sizeof(int16_t));
			memcpy(&_info.current, &_data[55], sizeof(int16_t));
			memcpy(&val16, &_data[57], sizeof(int16_t));
			_info.temperature = (float)val16;
			memcpy(&_info.humidity, &_data[59], sizeof(int16_t));
			memcpy(&_info.duration, &_data[61], sizeof(uint32_t));

			memcpy(&uval32, &_data[65], sizeof(uint32_t));
			_info.sys_stat = SYS_STATUS(uval32 & 0x07);
			_info.sig_stat.stat_PPS = (uval32 & 0x08) > 0;
			_info.sig_stat.stat_GPRMC = (uval32 & 0x10) > 0;
			_info.sig_stat.stat_NTP = (uval32 & 0x20) > 0;
			_info.sig_stat.stat_1588 = (uval32 & 0x40) > 0;
			_info.sig_stat.stat_IMU = (uval32 & 0x80) > 0;
			_info.multicast = (uval32 & 0x0100) > 0;

			uint8_t eth_size = _data[69];
			_info.ETH_stat.resize(eth_size);
			for (size_t i = 0; i < eth_size; ++i)
			{
				_info.ETH_stat[i] = _data[70 + i];
			}

			uint8_t dtc_size = _data[70 + eth_size];
			_info.DTC_msgs.resize(dtc_size);
			for (size_t i = 0; i < dtc_size; ++i)
			{
				DTC_INFO msg;
				memcpy(msg.time, &_data[71 + eth_size + i * 11], 7 * sizeof(char));
				memcpy(&msg.DTC_code, &_data[71 + eth_size + i * 11 + 7], 3 * sizeof(char));
				memcpy(&msg.DTC_status, &_data[71 + eth_size + i * 11 + 8], sizeof(uint8_t));
				_info.DTC_msgs[i] = msg;
			}

			uint8_t did_size = _data[71 + eth_size + dtc_size * 11];
			_info.DID_msgs.resize(did_size);
			int did_offset = 0;
			uint8_t version_length = 0;
			for (size_t i = 0; i < did_size; ++i)
			{
				DID_INFO msg;
				memcpy(&msg.id, &_data[72 + eth_size + dtc_size * 11 + did_offset], sizeof(uint16_t));
				version_length = _data[72 + eth_size + dtc_size * 11 + did_offset + 2];
				for (size_t j = 0; j < version_length; ++j)
				{
					msg.version += (char)_data[72 + eth_size + dtc_size * 11 + did_offset + 3 + j];
				}
				_info.DID_msgs[i] = msg;
				did_offset += (3 + version_length);
			}
		}
		else if (content_type == 0x01) // functional safety
		{
			memcpy(&_info.func_safety.FS_version, &_data[53], sizeof(uint8_t));
			memcpy(&_info.func_safety.FS_state, &_data[54], sizeof(uint8_t));
			memcpy(&_info.func_safety.fault_code_type, &_data[55], sizeof(uint8_t));
			memcpy(&_info.func_safety.fault_code, &_data[56], sizeof(uint64_t));
			memcpy(&_info.func_safety.total_fault_code_num, &_data[64], sizeof(uint16_t));
			memcpy(&_info.func_safety.fault_index, &_data[66], sizeof(uint8_t));
		}
	}
}