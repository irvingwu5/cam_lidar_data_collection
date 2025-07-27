#include "benewake_lidar_driver.h"

namespace benewake
{
	bool BW_FIND_AVAILABLE_DEVICES(std::vector<SYS_INFO>& _device_list, int _wait_milliseconds, int _search_port)
	{
		std::shared_ptr<DSOPProtocol> dsop = std::shared_ptr<DSOPProtocol>(new DSOPProtocol("0.0.0.0", _search_port, "", 65000));
		dsop->dsop_search_device(_device_list, _wait_milliseconds);
		//dsop->~DSOPProtocol();
		dsop.reset();
		if (_device_list.size() == 0)
			return false;
		else
			return true;
	}

	int BW_GET_SYSTEM_STATUS_CODE(const benewake::SYS_INFO& _info)
	{
		if (_info.comm_stat != benewake::COMM_STATUS::COMM_OK)
			return BW_COMM_ERROR;
		//if (_info.sys_stat == benewake::SYS_STATUS::STAT_ERROR)
		//	return BW_SYS_ERROR;
		//if (_info.sys_stat == benewake::SYS_STATUS::STAT_WARNING)
		//	return BW_SYS_WARNING;
		//if (_info.sys_stat == benewake::SYS_STATUS::STAT_INIT || _info.sys_stat == benewake::SYS_STATUS::STAT_BOOTLOADER)
		//	return BW_SYS_BUSY;
		return BW_OK;
	}

	BenewakeLidar::BenewakeLidar(std::string _remote_ip, uint32_t _remote_dcsp_port, UDPType _type, std::string _local_ip, uint32_t _local_dsop_port) :
		ip_(_remote_ip), dcsp_port_(_remote_dcsp_port), connect_type_(_type), local_ip_(_local_ip), local_dsop_port_(_local_dsop_port)
	{
		dsop_ = std::shared_ptr<DSOPProtocol>(new DSOPProtocol("0.0.0.0", local_dsop_port_, ip_, dsop_port_));

		local_dcsp_port_ = dcsp_port_;

		info_transport_dsop_dcsp_.ip = ip_;
		info_transport_dsop_dcsp_.protocol = 0x01;
		dsop_->setDeviceInfoPtr(&info_transport_dsop_dcsp_);
	}

	BenewakeLidar::BenewakeLidar(benewake::SYS_INFO _info, std::string _local_ip, uint32_t _local_dsop_port) :
		ip_(_info.ip), dcsp_port_(_info.dcsp_port), local_ip_(_local_ip), local_dsop_port_(_local_dsop_port)
	{
		connect_type_ = _info.multicast ? UDPType::MULTICAST : UDPType::NORMAL;
		dsop_ = std::shared_ptr<DSOPProtocol>(new DSOPProtocol("0.0.0.0", local_dsop_port_, ip_, dsop_port_));
		if (_info.device_type == PRODUCT_ID_X2)
		{
			dcsp_ = std::shared_ptr<DCSPProtocol>(new DCSPProtocol(local_ip_, dcsp_port_, ip_, dcsp_port_));
		}
		else if (_info.device_type == PRODUCT_ID_P4)
		{
			dcsp_ = std::shared_ptr<DCSPProtocolP4>(new DCSPProtocolP4(local_ip_, dcsp_port_, ip_, dcsp_port_));
		}
		else if (_info.device_type == PRODUCT_ID_G66)
		{
			dcsp_ = std::shared_ptr<DCSPProtocolG66>(new DCSPProtocolG66(local_ip_, dcsp_port_, ip_, dcsp_port_));
		}
		if (_info.device_type == PRODUCT_ID_X2)
		{
			mdop_ = std::shared_ptr<MDOPProtocol>(new MDOPProtocol(local_ip_, mdop_port_, ip_, mdop_port_, connect_type_));
		}
		else if (_info.device_type == PRODUCT_ID_P4)
		{
			mdop_ = std::shared_ptr<MDOPProtocolP4>(new MDOPProtocolP4(local_ip_, mdop_port_, ip_, mdop_port_, connect_type_));
		}
		else if (_info.device_type == PRODUCT_ID_G66)
		{
			mdop_ = std::shared_ptr<MDOPProtocolG66>(new MDOPProtocolG66(local_ip_, mdop_port_, ip_, mdop_port_, connect_type_));
		}

		local_dcsp_port_ = dcsp_port_;

		info_transport_dsop_dcsp_ = _info;
		dcsp_->setDeviceInfoPtr(&info_transport_dsop_dcsp_);
		dsop_->setDeviceInfoPtr(&info_transport_dsop_dcsp_);

		int ret = dcsp_->open();
		if (ret != STATUS_OK)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Open command control connection filed!" << std::endl;
		}
	}

	BenewakeLidar::~BenewakeLidar()
	{
		if (dcsp_ != NULL)
		{
			dcsp_->close();
			dcsp_.reset();
		}
		if (dsop_ != NULL)
		{
			dsop_.reset();
		}
		if (mdop_ != NULL)
		{
			mdop_->close();
			mdop_.reset();
		}
	}

	bool BenewakeLidar::bindLocalAddress(std::string _local_ip, uint32_t _local_dcsp_port, uint32_t _local_mdop_port, uint32_t _local_dsop_port, 
		std::string _remote_ip, uint32_t _remote_dcsp_port, uint32_t _remote_mdop_port, uint32_t _remote_dsop_port)
	{
		ip_ = _remote_ip;
		dcsp_port_ = _remote_dcsp_port;
		mdop_port_ = _remote_mdop_port;
		dsop_port_ = _remote_dsop_port;
		local_ip_ = _local_ip;
		local_dcsp_port_ = _local_dcsp_port;
		local_mdop_port_ = _local_mdop_port;
		local_dsop_port_ = _local_dsop_port;
		connect_type_ = UDPType::LOCAL_BIND;

		dsop_->reset_connection("255.255.255.255", local_dsop_port_, ip_, dsop_port_);
		if (dcsp_ != NULL)
		{
			dcsp_.reset();
			checkDCSPProtocol();
		}
		if (mdop_ != NULL)
		{
			mdop_.reset();
			checkMDOPProtocol();
		}

		benewake::SYS_INFO info = dsop_->dsop_get_system_information();
		info = dsop_->dsop_get_system_information();
		if (info.comm_stat == benewake::COMM_STATUS::COMM_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::downloadFirmware(std::string _firmware_file)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_download_firmware(_firmware_file);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::clearConfig()
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_clear_config();
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	void BenewakeLidar::enableABFrame(bool _enable)
	{
		enable_AB_frame_ = _enable;
	}

	bool BenewakeLidar::enableAntiInterference(bool _enable)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_anti_interference(_enable);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::enableDataForwarding(bool _enable, std::string _forward_ip, int _forward_port)
	{
		if (!checkMDOPProtocol())
			return false;

		int stat = mdop_->mdop_enable_data_forwarding(_enable, _forward_ip, _forward_port);

		if (stat != STATUS_OK)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "enableDataForwarding failed!" << std::endl;
			return false;
		}

		return true;
	}

	bool BenewakeLidar::enableLog(bool _enable, std::string _path)
	{
		if (_enable)
		{
			if (!mkDir(_path))
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::cout << "Create log directory failed!" << std::endl;
				return false;
			}
			log_path_ = _path;
		}
		log_enable_ = _enable;
		return true;
	}

	bool BenewakeLidar::enablieStaticARP(bool _enable)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_static_arp(_enable);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::enableSunlightResistance(bool _enable)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_sunlight_resistance(_enable);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::enableTimeWindowMode(bool _enable, uint32_t _interval)
	{
		if (!checkMDOPProtocol())
			return false;

		bool succ = mdop_->mdop_set_time_window(_enable, _interval);
		return succ;
	}

	bool BenewakeLidar::enableWindowHeating(int _win_heater, int _voltage,
		int _temp_a, int _temp_b, int _temp_c, int _temp_d,
		int _tx_heater, int _param_x, int _param_y, int _param_z)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_window_heating((uint8_t)_win_heater, (uint16_t)_voltage, 
			(int16_t)_temp_a, (int16_t)_temp_b, (int16_t)_temp_c, (int16_t)_temp_d,
			(uint8_t)_tx_heater, (int16_t)_param_x, (int16_t)_param_y, (int16_t)_param_z);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	benewake::BwPointCloud::Ptr BenewakeLidar::timeWindowDataQueueFront()
	{
		if (!checkMDOPProtocol())
		{
			std::shared_ptr<benewake::BwPointCloud> cloud = std::shared_ptr<benewake::BwPointCloud>(new benewake::BwPointCloud());
			return cloud;
		}

		return mdop_->mdop_front_time_window_data();
	}

	void BenewakeLidar::timeWindowDataQueuePop()
	{
		if (!checkMDOPProtocol())
			return;

		mdop_->mdop_pop_time_window_data();
	}

	benewake::BwPointCloud::Ptr BenewakeLidar::timeWindowDataQueuePopFront()
	{
		if (!checkMDOPProtocol())
		{
			std::shared_ptr<benewake::BwPointCloud> cloud = std::shared_ptr<benewake::BwPointCloud>(new benewake::BwPointCloud());
			return cloud;
		}

		return mdop_->mdop_pop_front_time_window_data();
	}

	int BenewakeLidar::timeWindowDataQueueSize()
	{
		if (!checkMDOPProtocol())
			return 0;

		return mdop_->mdop_time_window_data_queue_size();
	}

	bool BenewakeLidar::getAutorunStatus(bool& _enable)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_get_autorun_status(_enable);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::getClockConfig(bool& _1588_enable, uint8_t& _1588_domain_nu, bool& _gPTP_enable, uint8_t& _gPTP_domain_nu)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_get_clock_config(_1588_enable, _1588_domain_nu, _gPTP_enable, _gPTP_domain_nu);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::getConnection(std::string& _ip, uint32_t& _port, UDPType& _type)
	{
		_ip = ip_;
		_port = dcsp_port_;
		_type = connect_type_;
		return true;
	}

	bool BenewakeLidar::getCoordiateSystem(uint8_t& _coordinate)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_get_coordinate_system(_coordinate);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::getData(benewake::BwPointCloud::Ptr & _pointcloud, int & _frame_id, int _timeout)
	{
		if (!checkMDOPProtocol())
			return false;

		int stat = mdop_->mdop_get_frame(_pointcloud, _frame_id, _timeout);
		if (stat == STATUS_OK)
		{
			if (front_view_filter_enabled_)
			{
				mdop_->mdop_front_view_filter(_pointcloud);
			}
			return true;
		}
		else
			return false;
	}

	bool BenewakeLidar::getData(benewake::BwPointCloud::Ptr & _pointcloud, int & _frame_id, SYS_INFO& _sys_info, int _timeout)
	{
		if (!checkMDOPProtocol())
			return false;

		int stat = mdop_->mdop_get_frame(_pointcloud, _frame_id, _timeout);
		_sys_info = dsop_->dsop_get_system_information_no_wait();
		if (stat == STATUS_OK)
		{
			if (front_view_filter_enabled_)
			{
				mdop_->mdop_front_view_filter(_pointcloud);
			}
			return true;
		}
		else
			return false;
	}

	bool BenewakeLidar::getData(benewake::BwPointCloud::Ptr& _pointcloud, int& _frame_id, char* _signal_message, uint16_t& _signal_time_offset, int _timeout)
	{
		if (!checkMDOPProtocol())
			return false;

		int stat = mdop_->mdop_get_frame(_pointcloud, _frame_id, _timeout);
		if (stat == STATUS_OK)
		{
			if (front_view_filter_enabled_)
			{
				mdop_->mdop_front_view_filter(_pointcloud);
			}
			mdop_->mdop_get_signal_message(_signal_message, _signal_time_offset);
			return true;
		}
		else
			return false;
	}

	void BenewakeLidar::getDataTransmissionState(uint32_t& _delay_time, float& _pkg_loss_rate, float& _frame_loss_rate)
	{
		if (!checkMDOPProtocol())
		{
			_delay_time = 0;
			_pkg_loss_rate = 0;
			_frame_loss_rate = 0;
			return;
		}

		mdop_->mdop_get_data_transmission_state(_delay_time, _pkg_loss_rate, _frame_loss_rate);
	}

	std::shared_ptr<DCSPProtocol> BenewakeLidar::getDCSP()
	{
		if (!checkDCSPProtocol())
			return NULL;
		return dcsp_;
	}

	bool BenewakeLidar::getDeviceInformation(std::string & _version, std::string & _fpga_version,
		int & _total_points_num, int & _line_points_num, int & _channel_num, std::string & _sn)
	{
		if (!checkDCSPProtocol())
			return false;

		uint32_t total_num;
		uint16_t line_num, channel_num;
		int stat = dcsp_->dcsp_get_device_information(_version, _fpga_version, total_num, line_num, channel_num, _sn);
		_total_points_num = total_num;
		_line_points_num = line_num;
		_channel_num = channel_num;
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::getDHCPStatus(bool & _enable)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_get_DHCP(_enable);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	std::shared_ptr<DSOPProtocol> BenewakeLidar::getDSOP()
	{
		return dsop_;
	}

	bool BenewakeLidar::getDSOPDestIPAndPort(std::string& _dsop_ip, uint32_t& _dsop_port)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_get_dsop_dest_ip_and_port(_dsop_ip, _dsop_port);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::getDSPVersion(std::string& _app_version, std::string& _boot_loader_version)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_get_dsp_version(_app_version, _boot_loader_version);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::getExtrinsicParameters(bool& _enable, std::vector<float>& _params)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_get_extrinsic_parameters(_enable, _params);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::getFrontViewFilterParameters(bool& _status, int& _rows, int& _cols, int& _bind, int& _n_scan, float& _h_resolution, float& _v_resolution)
	{
		if (!checkMDOPProtocol())
			return false;

		_status = front_view_filter_enabled_;
		mdop_->mdop_get_front_view_filter_parameters(_rows, _cols, _bind, _n_scan, _h_resolution, _v_resolution);
		return true;
	}

	bool BenewakeLidar::getGaze(bool& _enable)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_get_gaze_status(_enable);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	std::shared_ptr<MDOPProtocol> BenewakeLidar::getMDOP()
	{
		if (!checkMDOPProtocol())
			return NULL;
		return mdop_;
	}

	bool BenewakeLidar::getMulticastIPAndPort(std::string& _ip, uint32_t& _port)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_get_multicast_IP_port(_ip, _port);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::getMulticastStatus(bool& _enable)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_get_multicast_status(_enable);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::getMultiEchoMode(int& _mode)
	{
		int32_t val = -1;
		uint32_t addr = 0x0000120d;
		bool rt = readRegister(&val, addr, 1);
		_mode = val;
		return rt;
	}

	bool BenewakeLidar::getNetworkTimeout(int& _time)
	{
		if (!checkDCSPProtocol())
			return false;

		uint32_t t = 0;
		int stat = dcsp_->dcsp_get_network_timeout(t);
		_time = t;
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::getNTPInfo(uint8_t& _clock_source, uint8_t& _clock_status, std::string& _server_ip, uint32_t& _interval)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_get_ntp_info(_clock_source, _clock_status, _server_ip, _interval);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::getOfflineLoggerInfo(int& _file_num)
	{
		if (!checkDCSPProtocol())
			return false;

		uint8_t file_num = 0;
		int stat = dcsp_->dcsp_get_offline_logger_info(file_num);
		_file_num = file_num;
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::getOfflineLoggerParameters(int& _log_level, int& _file_size, int& _file_num, bool& _enable_authority, bool& _enable_uart, bool& _enable_udp)
	{
		if (!checkDCSPProtocol())
			return false;

		uint8_t log_level = 0, file_size = 0, file_num = 0;
		int stat = dcsp_->dcsp_get_offline_logger_parameters(log_level, file_size, file_num, _enable_authority, _enable_uart, _enable_udp);
		_log_level = log_level;
		_file_size = file_size;
		_file_num = file_num;
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::getPower(int & _power)
	{
		if (!checkDCSPProtocol())
			return false;

		int32_t mem;
		uint32_t addr = 0x00000400;
		int stat = dcsp_->dcsp_read_register(&mem, addr, 1);
		if (stat != STATUS_OK) {
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Read power register failed!" << std::endl;
			return false;
		}
		_power = mem;

		return true;
	}

	bool BenewakeLidar::getPTPInfo(uint8_t& _clock_source, uint8_t& _clock_status, int& _offset_from_master_s, int& _offset_from_master_ns, int& _offset_accumulated, int& _path_delay_s, int& _path_delay_ns)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_get_ptp_info(_clock_source, _clock_status, _offset_from_master_s, _offset_from_master_ns, _offset_accumulated, _path_delay_s, _path_delay_ns);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::getRCMode(bool& _enable)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_get_rc_mode(_enable);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::getTimestampFormat(BwClockSource & _source)
	{
		if (!checkDCSPProtocol())
			return false;

		int flag = 0;
		int stat = dcsp_->dcsp_get_timestamp_format(flag);
		if (stat == STATUS_OK)
		{
			_source = (BwClockSource)flag;
			return true;
		}
		else
			return false;
	}

	bool BenewakeLidar::getTimestampFormat(int& _source)
	{
		if (!checkDCSPProtocol())
			return false;

		int flag = 0;
		int stat = dcsp_->dcsp_get_timestamp_format(flag);
		if (stat == STATUS_OK)
		{
			_source = flag;
			return true;
		}
		else
			return false;
	}

	bool BenewakeLidar::getWorkMode(BwWorkMode& _mode, int _timeout_s, int _timeout_us)
	{
		if (!checkDCSPProtocol())
			return false;
		if (!checkMDOPProtocol())
			return false;

		uint8_t mode = 0;
		int stat = dcsp_->dcsp_get_mode(mode, _timeout_s, _timeout_us);
		_mode = BwWorkMode(mode);
		if (stat == STATUS_OK)
		{
			mdop_->mdop_set_current_work_mode(mode);
			return true;
		}
		else
			return false;
	}

	bool BenewakeLidar::getWorkMode(int& _mode, int _timeout_s, int _timeout_us)
	{
		if (!checkDCSPProtocol())
			return false;
		if (!checkMDOPProtocol())
			return false;

		uint8_t mode = 0;
		int stat = dcsp_->dcsp_get_mode(mode, _timeout_s, _timeout_us);
		_mode = (int)mode;
		if (stat == STATUS_OK)
		{
			mdop_->mdop_set_current_work_mode(mode);
			return true;
		}
		else
			return false;
	}

	bool BenewakeLidar::readPLRegister(int32_t* _value, uint32_t _address)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_get_PL_register(_value, _address);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::readRegister(int32_t* _value, uint32_t _address, int _size)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_read_register(_value, _address, _size);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::readRegister(uint32_t* _value, uint32_t _address, int _size)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_read_register(_value, _address, _size);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	void BenewakeLidar::registHeartBeatCallback(HeartBeatCallbcakFunc _func, void* _pData)
	{
		dsop_->dsop_regist_heart_beat_callback(_func, _pData);
	}

	void BenewakeLidar::cancelHeartBeatCallback()
	{
		dsop_->dsop_cancel_heart_beat_callback();
	}

	void BenewakeLidar::registPointCloudCallback(PointCloudCallbackFunc _func, void * _pData)
	{
		if (!checkMDOPProtocol())
			return;

		mdop_->mdop_regist_point_cloud_callback(_func, _pData);
	}

	bool BenewakeLidar::reboot()
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_reboot_device();
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::restoreToDefault()
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_restore_to_default();
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::searchDevice(SYS_INFO & _info, int _wait_milliseconds)
	{
		std::vector<SYS_INFO> info_list;
		dsop_->dsop_search_device(info_list, _wait_milliseconds);
		if (info_list.size() == 0)
			return false;
		else
		{
			_info = info_list[0];
			return true;
		}
	}

	bool BenewakeLidar::setAutomobileInformation(int _battery_voltage, int _speed, int _time, int _mileage, 
		int _outside_temperature, int _altitude, int _rainfall, int _vehicle_status)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_automobile_info(_battery_voltage, _speed, _time, _mileage, _outside_temperature, _altitude, _rainfall, _vehicle_status);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::setAutorunStatus(bool _enable)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_autorun_status(_enable);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::setClockConfig(bool _1588_enable, uint8_t _1588_domain_nu, bool _gPTP_enable, uint8_t _gPTP_domain_nu)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_clock_config(_1588_enable, _1588_domain_nu, _gPTP_enable, _gPTP_domain_nu);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::setConnection(std::string _ip, uint32_t _port, UDPType _type)
	{
		int stat;
		ip_ = _ip;
		dcsp_port_ = _port;
		connect_type_ = _type;
		if (dcsp_ != NULL)
		{
			stat = dcsp_->close();
			if (stat != STATUS_OK)
				return false;
			dcsp_.reset();
		}
		if (dsop_ != NULL)
		{
			dsop_.reset();
		}
		if (mdop_ != NULL)
		{
			stat = mdop_->close();
			if (stat != STATUS_OK)
				return false;
			mdop_.reset();
		}
		dsop_ = std::shared_ptr<DSOPProtocol>(new DSOPProtocol("255.255.255.255", 65000, ip_, 65000));
		mdop_.reset();
		dcsp_.reset();
		if (!checkDCSPProtocol())
			return false;
		if (!checkMDOPProtocol())
			return false;

		return true;
	}

	bool BenewakeLidar::setCoordiateSystem(uint8_t _coordinate)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_coordinate_system(_coordinate);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::setDestinationIPAndPort(std::string _ip, uint32_t _mdop_port, uint32_t _dcsp_port, uint32_t _dsop_port, 
		std::string _mask, std::string _gateway, std::string _mac, uint16_t _vlan)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_destination_ip_port(_ip, _mdop_port, _dcsp_port, _dsop_port, _mask, _gateway, _mac, _vlan);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::setDeviceIPAndPort(std::string _ip, uint32_t _mdop_port, uint32_t _dcsp_port, std::string _mask, std::string _gateway)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_device_ip_port(_ip, _mdop_port, _dcsp_port, 65000, _mask, _gateway);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::setDeviceIPAndPort(std::string _ip, uint32_t _mdop_port, uint32_t _dcsp_port, std::string _mask, std::string _gateway, bool _use_fix_mac, std::string _mac)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_device_ip_port_mac(_ip, _mdop_port, _dcsp_port, 65000, _mask, _gateway, _use_fix_mac, _mac);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::setDeviceSN(const char * _SN)
	{
		if (!checkDCSPProtocol())
			return false;

		char * sn = const_cast<char*>(_SN);
		int stat = dcsp_->dcsp_set_sn(sn);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::setDHCPStatus(bool _enable)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_DHCP(_enable);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::setDSOPDestIPAndPort(std::string _dsop_ip, uint32_t _dsop_port)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_dsop_dest_ip_and_port(_dsop_ip, _dsop_port);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::setExtrinsicParameters(bool _enable, const std::vector<float>& _params)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_extrinsic_parameters(_enable, _params);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::setFOVROILocation(float _horizontal, float _vertical)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_fov_roi_location((uint16_t)(_horizontal * 10), (uint16_t)(_vertical * 10));
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::setFOVROISize(float _width, float _height)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_fov_roi_size((uint16_t)(_width * 10), (uint16_t)(_height * 10));
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::setGaze(bool _enable)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_gaze_status(_enable);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::setLocalTransform(bool _enable, float _roll, float _pitch, float _yaw, float _x, float _y, float _z)
	{
		if (!checkMDOPProtocol())
			return false;
		mdop_->mdop_set_local_transform(_enable, _roll, _pitch, _yaw, _x, _y, _z);
		return true;
	}

	bool BenewakeLidar::setMulticastIPAndPort(std::string _ip, uint32_t _port)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_multicast_IP_port(_ip, _port);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::setMulticastStatus(bool _enable)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_multicast_status(_enable);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::setMultiEchoMode(int _mode)
	{
		int32_t val = _mode;
		uint32_t addr = 0x0000120d;
		bool rt = writeRegister(&val, addr, 1);
		return rt;
	}

	bool BenewakeLidar::setNetworkTimeout(int _time)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_network_timeout(_time);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::setNTPParameters(std::string _server_ip, uint32_t _interval)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_ntp_info(_server_ip, _interval);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::setOfflineLoggerParameters(int _log_level, int _file_size, int _file_num, bool _enable_authority, bool _enable_uart, bool _enable_udp)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_offline_logger_parameters((uint8_t)_log_level, (uint8_t)_file_size, (uint8_t)_file_num, _enable_authority, _enable_uart, _enable_udp);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::setPower(int _power)
	{
		if (_power < 0 || _power > 1000)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Illegal power value!" << std::endl;
			return false;
		}
		if (!checkDCSPProtocol())
			return false;

		int32_t mem = _power;
		uint32_t addr = 0x00000400;
		int stat = dcsp_->dcsp_write_register(&mem, addr, 1);
		if (stat != STATUS_OK)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Write power register failed!" << std::endl;
			return false;
		}
		return true;
	}

	bool BenewakeLidar::setRCMode(bool _enable)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_rc_mode(_enable);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::setROI(bool _enable, const std::vector<float>& _vertices)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_roi(_enable, _vertices);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::setFrontViewFilter(bool _status, int _rows, int _cols, int _bind, int _n_scan, float _h_resolution, float _v_resolution)
	{
		if (!checkMDOPProtocol())
			return false;

		front_view_filter_enabled_ = _status;
		mdop_->mdop_set_front_view_filter_parameters(_rows, _cols, _bind, _n_scan, _h_resolution, _v_resolution);
		return true;
	}

	bool BenewakeLidar::setTimestampFormat(BwClockSource _source)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_timestamp_format((int)_source);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}


	bool BenewakeLidar::setTimestampFormat(int _source)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_timestamp_format(_source);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::setWorkMode(BwWorkMode _mode)
	{
		if (!checkDCSPProtocol())
			return false;
		if (!checkMDOPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_mode((uint8_t)_mode);
		if (stat == STATUS_OK)
		{
			mdop_->mdop_set_current_work_mode((uint8_t)_mode);
			return true;
		}
		else
			return false;
	}

	bool BenewakeLidar::setWorkMode(int _mode)
	{
		if (!checkDCSPProtocol())
			return false;
		if (!checkMDOPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_mode((uint8_t)_mode);
		if (stat == STATUS_OK)
		{
			mdop_->mdop_set_current_work_mode((uint8_t)_mode);
			return true;
		}
		else
			return false;
	}

	bool BenewakeLidar::shutdownWarning(int _delay_sec)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_shutdown_warning((uint8_t)_delay_sec);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::start()
	{
		if (!checkDCSPProtocol())
			return false;
		if (!checkMDOPProtocol())
			return false;

		int stat;
		// check heart beat
		SYS_INFO info = dsop_->dsop_get_system_information_no_wait();
		int stat_code = BW_GET_SYSTEM_STATUS_CODE(info);
		if (stat_code != BW_OK)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "System exception detected! Status code: " << stat_code << std::endl;
			return false;
		}
		device_info_ = info;

		// check mdop port
		if (connect_type_ == UDPType::MULTICAST)
			stat = dcsp_->dcsp_get_multicast_IP_port(multicast_ip_, local_mdop_port_);
		else
			stat = dcsp_->dcsp_get_mdop_port(mdop_port_);
		if (stat != STATUS_OK)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Get point cloud data output port failed!" << std::endl;
			return false;
		}

		mdop_->reset_connection(local_ip_, local_mdop_port_, ip_, mdop_port_, connect_type_);
		if (connect_type_ == UDPType::MULTICAST)
			mdop_->mdop_set_multicast_ip(multicast_ip_, local_mdop_port_);

		stat = mdop_->open(connect_type_);
		if (stat != STATUS_OK)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Open point cloud data receive port failed!" << std::endl;
			return false;
		}

		uint8_t work_mode = 0;
		stat = dcsp_->dcsp_get_mode(work_mode);
		if (stat != STATUS_OK)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Get work mode failed!" << std::endl;
			return false;
		}
		mdop_->mdop_set_current_work_mode(work_mode);

		// check X2 long range version
		if (device_info_.device_type == PRODUCT_ID_X2)
		{
			std::string version, version_fpga, sn;
			uint32_t total_num;
			uint16_t line_num, channel_num;
			int status = dcsp_->dcsp_get_device_information(version, version_fpga, total_num, line_num, channel_num, sn);
			if (status == STATUS_OK)
			{
				std::vector<std::string> substr;
				splitString(version_fpga, substr, ".");
				if ((substr.size() == 5 && substr[1].substr(0, 3).compare("ZZX") == 0) || 
					(substr.size() == 5 && substr[1].substr(0, 3).compare("STD") == 0))
				{
					int major = std::stoi(substr[2]);
					int minor = std::stoi(substr[3]);
					int build = std::stoi(substr[4]);
					if (major > 1 || (major == 1 && minor > 5) || (major == 1 && minor == 5 && build >= 17))
					{
						mdop_->mdop_set_X2_long_range_version(true);
						std::cout << std::endl << "Note: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
						std::cout << "Enable X2 long range version decoding!" << std::endl;
					}
					else
						mdop_->mdop_set_X2_long_range_version(false);
				}
				else
				{
					mdop_->mdop_set_X2_long_range_version(false);
				}
			}
		}

		// if set AB frame separation enabled, get rows of current work mode and set mdop
		if (enable_AB_frame_)
		{
			int32_t val, addr = 0x0106;
			stat = dcsp_->dcsp_read_register(&val, addr, 1);
			if (stat != STATUS_OK)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::cout << "Read frame lines register failed!" << std::endl;
				return false;
			}
			std::cout << "Info: current work mode lines " << val << std::endl;
			mdop_->mdop_enable_AB_Frame(enable_AB_frame_, val);
		}
		else
			mdop_->mdop_enable_AB_Frame(enable_AB_frame_, 0);
		// check whether LiDAR is working
		if (info.sys_stat != benewake::SYS_STATUS::STAT_RUNNING)
		{
			stat = dcsp_->dcsp_enable(true);
			if (stat != STATUS_OK)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::cout << "Enable LiDAR work failed!" << std::endl;
				return false;
			}
		}
		
		stat = mdop_->mdop_enable_decode(true);
		if (stat != STATUS_OK)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Enable decoding thread failed!" << std::endl;
			return false;
		}
		isStarted_ = true;
		return true;
	}

	bool BenewakeLidar::startOnlyReceiver(std::string _ip, int _port, uint8_t _device_type, uint16_t _protocol_version, bool _long_range_version, bool _enable_AB_frame, int _row_num)
	{
		dsop_->dsop_disable_heart_beat_receiver(ip_, dcsp_port_, _device_type, _protocol_version);

		if (!checkDCSPProtocol())
			return false;
		if (!checkMDOPProtocol())
			return false;

		int stat;
		mdop_port_ = _port;
		mdop_->reset_connection(_ip, _port, "", 44880, benewake::UDPType::LOCAL_BIND);
		stat = mdop_->open(benewake::UDPType::LOCAL_BIND);
		if (stat != STATUS_OK)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Open point cloud data receive port failed!" << std::endl;
			return false;
		}

		mdop_->mdop_set_X2_long_range_version(_long_range_version);

		mdop_->mdop_enable_AB_Frame(_enable_AB_frame, _row_num);

		stat = mdop_->mdop_enable_decode(true);
		if (stat != STATUS_OK)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Enable decoding thread failed!" << std::endl;
			return false;
		}
		isStarted_ = true;
		return true;
	}

	bool BenewakeLidar::stop()
	{
		if (!checkDCSPProtocol())
			return false;
		if (!checkMDOPProtocol())
			return false;

		int stat = dcsp_->dcsp_enable(false);
		if (stat != STATUS_OK)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Disable LiDAR work failed!" << std::endl;
			return false;
		}
		stat = mdop_->mdop_enable_decode(false);
		if (stat != STATUS_OK)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Disable decoding thread failed!" << std::endl;
			return false;
		}
		stat = mdop_->close();
		if (stat != STATUS_OK)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Close point cloud data receive port failed!" << std::endl;
			return false;
		}

		isStarted_ = false;
		return true;
	}

	bool BenewakeLidar::switchProtocol(BwProtocolType _protocol)
	{
		if (!checkMDOPProtocol())
			return false;

		mdop_->mdop_switch_protocol(_protocol);
		return true;
	}

	bool BenewakeLidar::updateFlash()
	{
		if (!checkDCSPProtocol())
			return false;
		int stat = dcsp_->dcsp_store(0x00);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::uploadOfflineLogger(std::string _save_name, int _file_id)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_upload_offline_logger(_save_name,(uint8_t)_file_id);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::writePLRegister(int32_t* _value, uint32_t _address)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_PL_register(_value, _address);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::writeRegister(int32_t* _value, uint32_t _address, int _size)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_write_register(_value, _address, _size);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::writeRegister(uint32_t* _value, uint32_t _address, int _size)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_write_register(_value, _address, _size);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::setROI(int _x_max, int _x_min, int _y_max, int _y_min)
	{
		if (!checkDCSPProtocol())
			return false;

		uint32_t roi0_point1_x_addr = 0x00001930;

		uint32_t roi1_point1_x_addr = 0x00001940;

		uint32_t roi_data_only_addr = 0x00001948;

		int stat = STATUS_OK;
		
		int32_t roi_0[8] = { _x_max, _y_min, _x_max, _y_max, _x_min, _y_max, _x_min, _y_min};
		//roi 0
		stat = dcsp_->dcsp_write_register(roi_0, roi0_point1_x_addr, 8);
		if (stat != STATUS_OK)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Write roi 0 failed!" << std::endl;
			return false;
		}
		
		stat = dcsp_->dcsp_write_register(roi_0, roi1_point1_x_addr, 8);
		if (stat != STATUS_OK)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Write roi 1 failed!" << std::endl;
			return false;
		}
		int32_t roi_data_only_enable = 1;
		stat = dcsp_->dcsp_write_register(&roi_data_only_enable, roi_data_only_addr, 1);
		if (stat != STATUS_OK)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Set roi data only enable failed!" << std::endl;
			return false;
		}
		
		return true;
	}

	bool BenewakeLidar::getROI(int& _x_max, int& _x_min, int& _y_max, int& _y_min)
	{
		if (!checkDCSPProtocol())
			return false;

		uint32_t roi0_point1_x_addr = 0x00001930;

		uint32_t roi1_point1_x_addr = 0x00001940;

		uint32_t roi_data_only_addr = 0x00001948;

		int stat = STATUS_OK;

		int roi_0[8] = { _x_max, _y_min, _x_max, _y_max, _x_min, _y_max, _x_min, _y_min };
		//roi 0
		stat = dcsp_->dcsp_read_register(roi_0, roi0_point1_x_addr, 8);
		if (stat != STATUS_OK)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Read roi 0 failed!" << std::endl;
			return false;
		}
		_x_max = roi_0[0];
		_y_min = roi_0[1];
		_y_max = roi_0[3];
		_x_min = roi_0[4];
		return true;
	}

	bool BenewakeLidar::setROIStatus(bool _enable)
	{
		if (!checkDCSPProtocol())
			return false;

		char data_channel[4] = { 0 };
		char data_type[2] = { 0 };
		char algorithm[2] = { 0 };
		int stat = dcsp_->dcsp_get_mask_info(data_channel, data_type, algorithm);
		if (stat != STATUS_OK)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Get mask failed!" << std::endl;
			return false;
		}

		if (_enable)
		{
			algorithm[1] = algorithm[1] | 0x08;
		}
		else
		{
			algorithm[1] = algorithm[1] & 0xf7;
		}

		stat = dcsp_->dcsp_set_mask_info(data_channel, data_type, algorithm);
		if (stat != STATUS_OK)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Set mask failed!" << std::endl;
			return false;
		}
		return true;
	}

	bool BenewakeLidar::setSignalMessage(char* _msg)
	{
		if (!checkDCSPProtocol())
			return false;

		int stat = dcsp_->dcsp_set_signal_message(_msg);
		if (stat == STATUS_OK)
			return true;
		else
			return false;
	}

	bool BenewakeLidar::getROIStatus(bool& _enable)
	{
		if (!checkDCSPProtocol())
			return false;

		char data_channel[4] = { 0 };
		char data_type[2] = { 0 };
		char algorithm[2] = { 0 };
		int stat = dcsp_->dcsp_get_mask_info(data_channel, data_type, algorithm);
		if (stat != STATUS_OK)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Get mask failed!" << std::endl;
			return false;
		}

		if (algorithm[1] & 0x08)
		{
			_enable = true;
		}
		else
		{
			_enable = false;
		}
		return true;
	}

	void BenewakeLidar::splitString(const std::string & s, std::vector<std::string>& v, const std::string & c)
	{
		std::string::size_type pos1, pos2;
		pos2 = s.find(c);
		pos1 = 0;
		while (std::string::npos != pos2)
		{
			v.push_back(s.substr(pos1, pos2 - pos1));
			pos1 = pos2 + c.size();
			pos2 = s.find(c, pos1);
		}
		if (pos1 != s.length())
			v.push_back(s.substr(pos1));
	}
	
	bool BenewakeLidar::checkDCSPProtocol()
	{
		if (dcsp_ == NULL)
		{
			SYS_INFO sysInfo;
			if (device_info_.comm_stat == COMM_STATUS::COMM_OK)
				sysInfo = device_info_;
			else
				sysInfo = dsop_->dsop_get_system_information();
			if (sysInfo.comm_stat == COMM_STATUS::COMM_TIMEOUT)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::cout << "No heart beat found!" << std::endl;
				return false;
			}
			UDPType udp_type = (connect_type_ == UDPType::LOCAL_BIND) ? UDPType::LOCAL_BIND : UDPType::NORMAL;
			if (sysInfo.device_type == PRODUCT_ID_X2)
			{
				dcsp_ = std::shared_ptr<DCSPProtocol>(new DCSPProtocol(local_ip_, local_dcsp_port_, ip_, dcsp_port_, udp_type));
			}
			else if (sysInfo.device_type == PRODUCT_ID_P4)
			{
				dcsp_ = std::shared_ptr<DCSPProtocolP4>(new DCSPProtocolP4(local_ip_, local_dcsp_port_, ip_, dcsp_port_, udp_type));
			}
			else if (sysInfo.device_type == PRODUCT_ID_G66)
			{
				dcsp_ = std::shared_ptr<DCSPProtocolG66>(new DCSPProtocolG66(local_ip_, local_dcsp_port_, ip_, dcsp_port_, udp_type));
			}
			else
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::cout << "Unknown device type found!" << std::endl;
				return false;
			}
			dcsp_->setDeviceInfoPtr(&info_transport_dsop_dcsp_);
			device_info_ = sysInfo;

			int ret = dcsp_->open();
			if (ret != STATUS_OK)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::cout << "Open DCSP protocol filed!" << std::endl;
				return false;
			}
			return true;
		}
		else
			return true;
	}

	bool BenewakeLidar::checkMDOPProtocol()
	{
		if (mdop_ == NULL)
		{
			SYS_INFO sysInfo;
			if (device_info_.comm_stat == COMM_STATUS::COMM_OK)
				sysInfo = device_info_;
			else
				sysInfo = dsop_->dsop_get_system_information();
			if (sysInfo.comm_stat == COMM_STATUS::COMM_TIMEOUT)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::cout << "No heart beat found!" << std::endl;
				return false;
			}
			if (sysInfo.device_type == PRODUCT_ID_X2)
			{
				mdop_ = std::shared_ptr<MDOPProtocol>(new MDOPProtocol(local_ip_, local_mdop_port_, ip_, mdop_port_, connect_type_));
			}
			else if (sysInfo.device_type == PRODUCT_ID_P4)
			{
				mdop_ = std::shared_ptr<MDOPProtocolP4>(new MDOPProtocolP4(local_ip_, local_mdop_port_, ip_, mdop_port_, connect_type_));
			}
			else if (sysInfo.device_type == PRODUCT_ID_G66)
			{
				mdop_ = std::shared_ptr<MDOPProtocolG66>(new MDOPProtocolG66(local_ip_, local_mdop_port_, ip_, mdop_port_, connect_type_));
			}
			else
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::cout << "Unknown device type found!" << std::endl;
				return false;
			}
			device_info_ = sysInfo;

			return true;
		}
		else
			return true;
	}
}
