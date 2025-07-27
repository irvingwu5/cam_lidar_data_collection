/**
* @file       benewake_dcsp.h

* @brief      Header of Benewake lidar's driver.

* @details 	  For those who want to develop their own program based on Benewake Horn X2 driver, this header
			  should be included in the codes. Corresponding Dynamic Link Library(.dll) and Object File
			  Library(.lib) also should be contained in the project.

* @author     Fengqiang, TAN Wenquan

* @date       02/10/2023

* @version    v3.0.0

* @par Copyright (c):

*      Benewake (Beijing) Co., Ltd
*/
#ifndef INCLUDE_BENEWAKE_DCSP_H__
#define INCLUDE_BENEWAKE_DCSP_H__
#include <string>
#include <thread>
#include <sstream>
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <vector>
#include <atomic>
#include "benewake_protocol.h"
#ifdef _WIN32
	#ifdef BENEWAKE_DLL_EXPORT
		#define BENEWAKE_API __declspec(dllexport)
	#else
		#define BENEWAKE_API __declspec(dllimport)
	#endif // BENEWAKE_DLL_EXPORT
#else
#define BENEWAKE_API
#endif
namespace benewake
{
	//class BENEWAKE_API DCSPProtocol
    class BENEWAKE_API DCSPProtocol : public Protocol
	{
	public:
		/** \brief DCSPProtocol constructor
		* \param[in] _local_ip: local device ip.
		* \param[in] _local_port: local port to receive data.
		* \param[in] _remote_ip: data source ip which will be verified. When set to "", means any data received by _recv_port will be returned. 
		* \param[in] _remote_port: remote port that the data from.
		* \param[in] _udp_type: connection mode. NORMAL, MULTICAST or BOARDCAST.
		* \return none
		* \note
		*/
		DCSPProtocol(std::string _local_ip = "0.0.0.0", int _local_port = 2469, 
			std::string _remote_ip = "192.168.0.2", int _remote_port = 2469, UDPType _udp_type = UDPType::NORMAL);
		
		~DCSPProtocol();

		/** \brief Clear work time
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		virtual int dcsp_clear_work_time();

		/** \brief switch detect mode
		* \param[in] _mode: detect mode
		* @verbatim
		*			0 - overall unit mode
		*			1 - laser test / single point mode
		*			4 - prism test
		*			5 - galvo test
		* @verbatim
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_detect_mode_switch(int32_t _mode);

		/** \brief download firmware
		* \param[in] _firmware_file: fireware file location in file system
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		int dcsp_download_firmware(std::string _firmware_file);

		/** \brief download compensation tables
		* \param[in] _table_file: table file path
		* \param[in] _check_sn: whether check SN within table file
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_download_lut(std::string _table_file, bool _check_sn);

		int dcsp_download_temp_comp_table(const uint8_t* _data, const int _data_len);

		/** \brief start or stop sampling
		* \param[in] _flag: true - enable lidar; false - disable lidar
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_enable(bool _flag);

		/** \brief format file system
		* \param[in] none
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		virtual int dcsp_format_file_system();

		/** \brief get enabled algorithm and their version
		* \param[out] _versions: algorithm and version
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		virtual int dcsp_get_algorithm_version(std::vector<std::string> &_versions);

		/** \brief get the partition where the app is in
		* \param[out] _partition: 0 - A partition; 1 - B partition; 2 - boot loader
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		virtual int dcsp_get_app_partition(uint8_t& _partition);

		/** \brief get autorun status
		* \param[out] _enable: enable flag
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_get_autorun_status(bool& _enable);

		/** \brief get clock config
		* \param[out] _1588_bmc: 1588 BMC enable flag
		* \param[out] _1588_domain_nu: 1588 domain number
		* \param[out] _gptp_bmc: gptp BMC enable flag
		* \param[out] _gptp_domain_nu: gptp domain number
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		virtual int dcsp_get_clock_config(bool& _1588_bmc, uint8_t& _1588_domain_nu, bool& _gptp_bmc, uint8_t& _gptp_domain_nu);

		/** \brief get coordinate system
		* \param[out] _coordinate: 0 - spherical coordinate; 1 - Cartesian coordinate
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_get_coordinate_system(uint8_t &_coordinate);

		/** \brief Get current sdk protocol version
		* \return  protocol version
		* \note
		*/
		uint16_t dcsp_get_current_sdk_protocol_version();
		
		/** \brief Get DDR info
		* \param[out] _ddr_capacity: DDR capacity, unit 1024 Bytes
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		virtual int dcsp_get_DDR_info(uint32_t &_ddr_capacity);

		/** \brief get device information
		* \param[out] _version: PS core version
		* \param[out] _fpga_version: PL core version
		* \param[out] _total_num: total points amount of a frame
		* \param[out] _line_num: points amount of a line
		* \param[out] _channel_num: total channel amount
		* \param[out] _sn: SN code
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		int dcsp_get_device_information(std::string &_version, std::string &_fpga_version, 
			uint32_t &_total_num, uint16_t &_line_num, uint16_t &_channel_num, std::string &_sn, 
			int _timeout_s = 2, int _timeout_us = 0);

		/** \brief get DHCP enable/disable
		* \param[out] _enable: enable flag
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_get_DHCP(bool &_enable);

		/** \brief get dsop destination IP and port
		* \param[out] _ip: destination IP. If IP is 255.255.255.255, means dsop is in boardcast mode.
		* \param[out] _port: destination port
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		virtual int dcsp_get_dsop_dest_ip_and_port(std::string& _ip, uint32_t& _port);

		/** \brief get device information
		* \param[out] _app_version: APP's version
		* \param[out] _boot_loader_version: boot loader's version
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		int dcsp_get_dsp_version(std::string& _app_version, std::string& _boot_loader_version);

		/** \brief get EEPROM status
		* \param[out] _status: 0 - OK; other - ERROR
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		virtual int dcsp_get_EEPROM_status(uint8_t& _status);

		/** \brief Get extrinsic parameters of
		* | a11 a12 a13 x |
		* | a21 a22 a23 y |
		* | a31 a32 a33 z |
		* \param[in] _enable: enable transformation or not
		* \param[in] _params: parameters in order {a11, a12, a13, a21, a22, a23, a31, a32, a33, x, y, z}
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_get_extrinsic_parameters(bool& _enable, std::vector<float>& _params);

		/** \brief get fallback status
		* \param[in] _status: fallback status
		* @verbatim
		*			bit 0 - A53_0 core status, 0: disable, 1: enable
		*			bit 1 - A53_1 core status, 0: disable, 1: enable
		*			bit 2 - A53_2 core status, 0: disable, 1: enable
		*			bit 3 - A53_3 core status, 0: disable, 1: enable
		*			bit 4 - R5_0 core status, 0: disable, 1: enable
		*			bit 5 - R5_1 core status, 0: disable, 1: enable
		* @verbatim
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_get_fallback_status(uint8_t& _status);

		/** \brief Get flash information.
		* \param[out] _flash_id: flash ID
		* \param[out] _section_size: 1 section size
		* \param[out] _section_num: amount of sections
		* \param[out] _page_size: 1 page size
		* \param[out] _page_num: amount of pages
		* \param[out] _total_size: total size of flash
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		virtual int dcsp_get_flash_info(uint32_t& _flash_id, uint32_t& _section_size, uint32_t& _section_num, uint32_t& _page_size, 
			uint32_t& _page_num, uint32_t& _total_size);

		/** \brief Get current gaze status.
		* \param[out] _enable: true: gaze is open; false: gaze is close
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		int dcsp_get_gaze_status(bool& _enable);

		/** \brief get device MAC and VLAN
		* \param[out] _mac: MAC
		* \param[out] _vlan: vlan value
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		virtual int dcsp_get_mac_and_vlan(std::string& _mac, uint16_t& _vlan);

		/** \brief get mask information
		* \param[out] _data_channel: data channel mask
		* \param[out] _data_type: data type mask
		* \param[out] _algorithm: algorithm mask
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_get_mask_info(char *_data_channel, char *_data_type, char *_algorithm);

		/** \brief get mask information
		* \param[out] _mdop_port: mdop port
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_get_mdop_port(uint32_t& _mdop_port);

		/** \brief get lidar mode
		* \param[out] _mode: lidar mode
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_get_mode(uint8_t& _mode, int timeout_s = 2, int timeout_us = 0);

		/** \brief get multicast IP and port
		* \param[out] _ip: multicast group IP
		* \param[out] _port: multicast port
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_get_multicast_IP_port(std::string& _ip, uint32_t& _port);

		/** \brief get multicast enable/disable
		* \param[out] _enable: enable flag
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_get_multicast_status(bool& _enable);

		/** \brief get network timeout time
		* \param[out] _time: time in second
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_get_network_timeout(uint32_t& _time);

		/** \brief get NTP information from lidar
		* \param[out] _clock_source: current clock synchronization mode
		* @verbatim
		*			0 - no synchronous source
		*			1 - PTP(1588V2)
		*			2 - NTP
		*			3 - GPS + PPS
		*			4 - PPS
		*			5 - gPTP
		*			255 - meaningless value
		* @verbatim
		* \param[out] _clock_status: NTP status
		* @verbatim
		*			0 - NORMAL
		*			1 - NTP_NOT_ENABLE
		*			2 - NTP_SERVER_UNREACHABLE
		*			255 - meaningless value
		* @verbatim
		* \param[out] _server_ip: NTP server IP.
		* \param[out] _interval: polling interval in millisecond.
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		virtual int dcsp_get_ntp_info(uint8_t& _clock_source, uint8_t& _clock_status, std::string& _server_ip, uint32_t& _interval);
		
		/** \brief get offline logger information
		* \param[out] _file_num: logger file amount
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		virtual int dcsp_get_offline_logger_info(uint8_t& _file_num);

		/** \brief get offline logger parameters
		* \param[out] _log_level: set record logger level
		* @verbatim
		*			0 - MSG_EMERG
		*			1 - MSG_ALERT
		*			2 - MSG_CRIT
		*			3 - MSG_ERR
		*			4 - MSG_WARNING
		*			5 - MSG_NOTICE
		*			6 - MSG_INFO
		*			7 - MSG_DEBUG
		* @verbatim
		* \param[out] _file_size: max file size. range [1, 62], unit kb
		* \param[out] _file_num: max file amount. range [1, 127].
		* \param[out] _enable_authority: enabel/disable authority.
		* \param[out] _enable_uart: enabel/disable UART output.
		* \param[out] _enable_udp: enabel/disable UDP output.
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		virtual int dcsp_get_offline_logger_parameters(uint8_t& _log_level, uint8_t& _file_size, uint8_t& _file_num,
			bool& _enable_authority, bool& _enable_uart, bool& _enable_udp);

		/** \brief get PL register values
		* \param[in] _value: unsigned value PL register
		* \param[in] _address: address of register
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		int dcsp_get_PL_register(int32_t* _value, uint32_t _address);

		/** \brief get ptp/gptp information from lidar
		* \param[out] _clock_source: current clock synchronization mode
		* @verbatim
		*			0 - no synchronous source
		*			1 - PTP(1588V2)
		*			2 - NTP
		*			3 - GPS + PPS
		*			4 - PPS
		*			5 - gPTP
		* @verbatim
		* \param[out] _clock_status: PTP or gPTP status
		* @verbatim
		*			0 - PTP_INITIALIZING
		*			1 - PTP_FAULTY
		*			2 - PTP_DISABLED
		*			3 - PTP_LISTENING
		*			4 - PTP_PRE_MASTER
		*			5 - PTP_MASTER
		*			6 - PTP_PASSIVE
		*			7 - PTP_UNCALIBRATED
		*			8 - PTP_SLAVE
		*			255 - meaningless value
		* @verbatim
		* \param[out] _offset_from_master_s: difference of second part between current local clock and master clock (in second)
		* \param[out] _offset_from_master_ns: difference of nanosecond part between current local clock and master clock (in nanosecond)
		* \param[out] _offset_accumulated: accumulative difference of clock (in nanosecond)
		* \param[out] _path_delay_s: second part of mean path delay (in second)
		* \param[out] _path_delay_ns: nanosecond part of mean path delay (in nanosecond)
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		virtual int dcsp_get_ptp_info(uint8_t& _clock_source, uint8_t& _clock_status, int& _offset_from_master_s, int& _offset_from_master_ns,
			int& _offset_accumulated, int& _path_delay_s, int& _path_delay_ns);

		/** \brief get RC status
		* \param[out] _enabled: enable or disable flag
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		virtual int dcsp_get_rc_mode(bool &_enabled);

		/** \brief set time synchronous source
		* \param[out] _format: format flag
		* @verbatim
		*			0 - no synchronous source
		*			1 - PTP(1588V2)
		*			2 - NTP
		*			3 - GPS + PPS
		*			4 - PPS
		*			5 - gPTP
		* @verbatim
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_get_timestamp_format(int& _format);

		/** \brief get device information
		* \param[out] _sn: laser SN code
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		int dcsp_get_laser_sn(std::string& _sn);

		int dcsp_get_temp_comp_table(const uint8_t* _data);

		/** \brief get register values
		* \param[in] _value: value of each register
		* \param[in] _address: start address of register
		* \param[in] _size: number of register to read
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		int dcsp_read_register(int32_t *_value, uint32_t _address, int _size);

		int dcsp_read_register(uint32_t* _value, uint32_t _address, int _size);

		/** \brief reboost device
		* \param[in] none
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_reboot_device();

		/** \brief restore parameters to default settings
		* \param[in] none
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_restore_to_default();

		/** \brief Clear the factory configuration area and user configuration area
		* \param[in] none
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_clear_config();

		/** \brief Enable or disable anti-interference
		* \param[in] _enable: enable/disable flag
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		virtual int dcsp_set_anti_interference(bool _enable);

		/** \brief Set automobile information
		* \param[in] _battery_voltage: voltage of vehicle battery
		* \param[in] _speed: automobile speed
		* \param[in] _time: absolute time
		* \param[in] _mileage: mileage of vehicle
		* \param[in] _outside_temperature: outside temperature
		* \param[in] _altitude: altitude information
		* \param[in] _rainfall: rainfall monitoring information
		* \param[in] _vehicle_status: vehicle status
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		virtual int dcsp_set_automobile_info(int _battery_voltage, int _speed, int _time, int _mileage,
			int _outside_temperature, int _altitude, int _rainfall, int _vehicle_status);

		/** \brief set autorun enable/disable
		* \param[in] _enable: enable flag
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_set_autorun_status(bool _enable);

		/** \brief set clock config
		* \param[in] _1588_enable: 1588 BMC enable flag
		* \param[in] _1588_domain_nu: 1588 domain number
		* \param[in] _gptp_enable: gptp BMC enable flag
		* \param[in] _gptp_domain_nu: gptp domain number
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		virtual int dcsp_set_clock_config(bool _1588_enable, uint8_t _1588_domain_nu, bool _gptp_enable, uint8_t _gptp_domain_nu);

		/** \brief set coordinate system
		* \param[in] _coordinate: 0 - spherical coordinate; 1 - Cartesian coordinate
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_set_coordinate_system(uint8_t _coordinate);

		/** \brief Set CSRD remote tcp server IP addr & port
		* \param[in] _ip: tcp server ip
		* \param[in] _port: tcp server port
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		int dcsp_set_crsd_remote_tcp_server_ip_port(std::string _ip, int _port);

		/** \brief Set customer information
		* \param[in] _custom_id: customer name.
		* \param[in] _manufacture_date: manufacture date, should be 4 bytes long.
		* @verbatim
		*			0-1: Year
		*			2:	 Month
		*			3:	 Day
		* @verbatim
		* \param[in] _trace_code: trace code, should be 18 bytes long.
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		virtual int dcsp_set_customer_info(uint16_t _custom_id, char* _manufacture_date, char* _trace_code);

		/** \brief Set customer mode
		* \param[in] _mode: 0x00: standard   0x01: crsd
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		int dcsp_set_customer_mode(int _mode);

		/** \brief set the address that device communicate with
		* \param[in] _ip: destination ip
		* \param[in] _mdop_port: destination mdop port
		* \param[in] _dcsp_port: destination dcsp port
		* \param[in] _dsop_port: destination dsop port
		* \param[in] _mask: destination mask
		* \param[in] _gateway: destination gateway
		* \param[in] _mac: MAC address, should be like "1A:2B:3C:4D:5E:6F"
		* \param[in] _vlan: destination vlan
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_set_destination_ip_port(std::string _ip, uint32_t _mdop_port, uint32_t _dcsp_port,
			uint32_t _dsop_port, std::string _mask, std::string _gateway, std::string _mac, uint16_t _vlan);

		/** \brief set device ip and ports
		* \param[in] _ip: device ip
		* \param[in] _mdop_port: device mdop port
		* \param[in] _dcsp_port: device dcsp port
		* \param[in] _dsop_port: device dsop port
		* \param[in] _mask: mask
		* \param[in] _gateway: gateway
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note AD2-S should use dcsp_set_device_ip_port_mac()
		*/
		int dcsp_set_device_ip_port(std::string _ip = "192.168.0.2", uint32_t _mdop_port = 2468, uint32_t _dcsp_port = 2469,
			uint32_t _dsop_port = 65000, std::string _mask = "255.255.255.0", std::string _gateway = "192.168.0.1");

		/** \brief set device ip and ports
		* \param[in] _ip: device ip
		* \param[in] _mdop_port: device mdop port
		* \param[in] _dcsp_port: device dcsp port
		* \param[in] _dsop_port: device dsop port
		* \param[in] _mask: mask
		* \param[in] _gateway: gateway
		* \param[in] _use_config_mac: use configured MAC or use fix MAC
		* \param[in] _mac: MAC configured
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		virtual int dcsp_set_device_ip_port_mac(std::string _ip = "192.168.0.2", uint32_t _mdop_port = 2468, uint32_t _dcsp_port = 2469,
			uint32_t _dsop_port = 65000, std::string _mask = "255.255.255.0", std::string _gateway = "192.168.0.1",
			bool _use_config_mac = true, std::string _mac = "36:7C:7C:00:00:16");

		/** \brief set DHCP enable/disable
		* \param[in] _enable: enable flag
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_set_DHCP(bool _enable);

		/** \brief get dsop destination IP and port
		* \param[out] _ip: destination IP. If set to 255.255.255.255, dsop will be in boardcast mode.
		* \param[out] _port: destination port
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		virtual int dcsp_set_dsop_dest_ip_and_port(std::string _ip, uint32_t _port);
		
		/** \brief Set extrinsic parameters of
		* | a11 a12 a13 x |
		* | a21 a22 a23 y |
		* | a31 a32 a33 z |
		* \param[in] _enable: enable transformation or not
		* \param[in] _params: parameters in order {a11, a12, a13, a21, a22, a23, a31, a32, a33, x, y, z}
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_set_extrinsic_parameters(bool _enable, const std::vector<float>& _params);

		/** \brief set fallback status
		* \param[in] _enable: enable/disable fallback
		* \param[in] _cpu_select: set fallback cpu
		* @verbatim
		*			0 - A53_0 core
		*			1 - A53_1 core
		*			2 - A53_2 core
		*			3 - A53_3 core
		*			4 - R5_0 core
		*			5 - R5_1 core
		* @verbatim
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_set_fallback_status(bool _enable, uint8_t _cpu_select = 1);

		/** \brief Set FOV ROI center's location.
		* \param[in] _horizontal_location: horizontal angle of ROI center, in 0.1 degree.
		* \param[in] _vertical_location: vertical angle of ROI center, in 0.1 degree.
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		virtual int dcsp_set_fov_roi_location(uint16_t _horizontal_location, uint16_t _vertical_location);
		
		/** \brief Set FOV ROI area width and height.
		* \param[in] _width: angle width of ROI area, in 0.1 degree.
		* \param[in] _height: angle height of ROI area, in 0.1 degree.
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		virtual int dcsp_set_fov_roi_size(uint16_t _width, uint16_t _height);

		/** \brief Set gaze status.
		* \param[in] _enable: true: open gaze  false: close gaze
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		int dcsp_set_gaze_status(bool _enable);

		/** \brief set mask information
		* \param[in] _data_channel: data channel mask
		* \param[in] _data_type: data type mask
		* \param[in] _algorithm: algorithm mask
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_set_mask_info(const char *_data_channel, const char *_data_type, const char *_algorithm);

		/** \brief set lidar mode
		* \param[in] _mode: lidar mode
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_set_mode(uint8_t _mode);

		/** \brief set multicast IP and port
		* \param[in] _ip: multicast group IP
		* \param[in] _port: multicast port
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_set_multicast_IP_port(std::string _ip, uint32_t _port);

		/** \brief set multicast enable/disable
		* \param[in] _enable: enable flag
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_set_multicast_status(bool _enable);

		/** \brief set network timeout time
		* \param[in] _time: time in second
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_set_network_timeout(uint32_t _time);

		/** \brief Set NTP configuration.
		* \param[in] _server_ip: NTP server IP.
		* \param[in] _interval: polling interval in millisecond, valid value is [15000, 3600000].
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		virtual int dcsp_set_ntp_info(std::string _server_ip, uint32_t _interval);

		/** \brief set offline logger parameters
		* \param[in] _log_level: set record logger level
		* @verbatim
		*			0 - MSG_EMERG
		*			1 - MSG_ALERT
		*			2 - MSG_CRIT
		*			3 - MSG_ERR
		*			4 - MSG_WARNING
		*			5 - MSG_NOTICE
		*			6 - MSG_INFO
		*			7 - MSG_DEBUG
		* @verbatim
		* \param[in] _file_size: max file size. range [1, 62], unit kb
		* \param[in] _file_num: max file amount. range [1, 127].
		* \param[in] _enable_authority: enabel/disable authority.
		* \param[in] _enable_uart: enabel/disable UART output.
		* \param[in] _enable_udp: enabel/disable UDP output.
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		virtual int dcsp_set_offline_logger_parameters(uint8_t _log_level, uint8_t _file_size, uint8_t _file_num,
			bool _enable_authority, bool _enable_uart, bool _enable_udp);

		/** \brief set PL register value
		* \param[in] _value: unsigned value PL register
		* \param[in] _address: address of register
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		int dcsp_set_PL_register(const int32_t* _value, uint32_t _address);

		/** \brief set RC mode status
		* \param[in] _enable: enable or disable RC mode
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		virtual int dcsp_set_rc_mode(bool _enable);

		/** \brief set ROI
		* \param[in] _enable: enable ROI or not
		* \param[in] _vertices: vertices of ROI, in order {p1.x, p1.y, p2.x, p2.y, ..., pn.x, pn.y}. Vertices cannot be less than 3.
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_set_roi(bool _enable, const std::vector<float>& _vertices);

		/** \brief Set signal message.
		* \param[in] _msg: Signal message attached to MDOP packages, should be 4 bytes;
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_set_signal_message(char* _msg);

		/** \brief set SN code
		* \param[in] _sn: SN code, max 32 bytes;
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_set_sn(const char *_sn);

		/** \brief Enable or disable static arp.
		* \param[in] _enable: enable/disable flag. To enable static arp, set true; otherwise set false;
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		virtual int dcsp_set_static_arp(bool _enable);

		/** \brief Enable or disable sunlight resistance.
		* \param[in] _enable: enable/disable flag;
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		virtual int dcsp_set_sunlight_resistance(bool _enable);

		/** \brief set time synchronous source
		* \param[in] _format: time synchronous source type
		* @verbatim
		*			0 - no synchronous source
		*			1 - PTP(1588V2)
		*			2 - NTP
		*			3 - GPS + PPS
		*			4 - PPS
		*			5 - gPTP
		* @verbatim
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_set_timestamp_format(int _format);

		/** \brief enable or disable vlan
		* \param[in] _enable: true - enable vlan; false - disable vlan
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		virtual int dcsp_set_vlan_status(bool _enable);

		/** \brief Enable or disable window heating.
		* \param[in] _mode: enable/disable window heating and set heating mode;
		* @verbatim
		*			0x0 - shutdown
		*			0x1 - manual control
		*			0x2 - temperature control
		*			0x4 - drewing control
		*			0x6 - temperature + drewing control
		*			0x8 - dirtying control
		*			0xE - temperature + drewing + dirtying control
		* @verbatim
		* \param[in] _voltage: voltage, should be [20, 28];
		* \param[in] _temp_a: control parameter tempA;
		* \param[in] _temp_b: control parameter tempB;
		* \param[in] _temp_c: control parameter tempC;
		* \param[in] _temp_d: control parameter tempD;
		* \param[in] _tx_enable: enable/disable TX heating flag;
		* @verbatim
		*			0x0 - disable
		*			0x1 - enable
		* @verbatim
		* \param[in] _param_x: control parameter X;
		* \param[in] _param_x: control parameter Y;
		* \param[in] _param_x: control parameter Z;
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		virtual int dcsp_set_window_heating(uint8_t _win_enable, uint16_t _voltage,
			int16_t _temp_a, int16_t _temp_b, int16_t _temp_c, int16_t _temp_d, 
			uint8_t _tx_enable, int16_t _param_x, int16_t _param_y, int16_t _param_z);

		/** \brief shutdown device
		* \param[in] none
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_shutdown_device();

		/** \brief inform device that power off in several seconds
		* \param[in] _delay_sec: time before power off, in second
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		virtual int dcsp_shutdown_warning(uint8_t _delay_sec);

		/** \brief store parameters to flash
		* \param[in] _store_flag: store mode
		* @verbatim
		*			0x00 - store to user flash
		*			0x01 - store to factory flash
		*			0x03 - restore default tp user flash
		* @verbatim
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		int dcsp_store(uint8_t _store_flag = 0x00);

		/** \brief switch AB partition
		* \param[in] none
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		virtual int dcsp_switch_partition();

		/** \brief upload compensation tables
		* \param[in] _table_file: table file path
		* \param[in] _table_id: table ID which define the use of table
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		virtual int dcsp_upload_lut(std::string _table_file, uint8_t _table_id);

		/** \brief upload offline logger from LiDAR
		* \param[in] _save_name: saving file path and name
		* \param[in] _file_id: file ID of logger
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		virtual int dcsp_upload_offline_logger(std::string _save_name, uint8_t _file_id);

		/** \brief write values to registers
		* \param[in] _value: value of each register
		* \param[in] _address: start address of register
		* \param[in] _size: number of register to write
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		int dcsp_write_register(const int32_t *_value, uint32_t _address, int _size);

		int dcsp_write_register(const uint32_t* _value, uint32_t _address, int _size);

		/** \brief Set Pointers to synchronize information from different ports
		* \note
		*/
		void setDeviceInfoPtr(SYS_INFO* _info);

	protected:
		unsigned char request_[PROTOCOL_DATA_PACKAGE_MAX_LENGTH] = { 0 };
		unsigned char response_[PROTOCOL_DATA_PACKAGE_MAX_LENGTH] = { 0 };
 
		int table_id_ = 0, table_ver_ = 0, table_row_ = 0, table_col_ = 0;
		std::string table_sn_;
		std::vector<int> table_;
		SYS_INFO *dcsp_info_ = nullptr;

		unsigned char request_header_[10];
		unsigned char response_header_[10];

		int header_count_len_;
		int response_status_len_;

		/** \brief send certain byte(s) of request and check response is success or fail.
		* \param[in] tx_size: bytes to send within request_
		* \param[in] cmd_response: command ID
		* \param[in] _timeout_s: set time out time's second part
		* \param[in] _timeout_us: set time out time's less than 1 second part
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note if not succeed to receive response, the process will automatically repeat 3 times, so actual time out time is 3 * timeout.
		*/
		virtual int tx_rx_check_status(int tx_size, uint8_t cmd_response, int _timeout_s = 2, int _timeout_us = 0);

		/** \brief send certain byte(s) of request and save response to response buffer
		* \param[in] tx_size: bytes to send within request_
		* \param[in] rx_size: bytes should be responsed
		* \param[in] cmd_response: command ID
		* \param[in] _timeout_s: set time out time's second part
		* \param[in] _timeout_us: set time out time's less than 1 second part
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note if not succeed to receive response, the process will automatically repeat 3 times.
		*/
		virtual int tx_rx_data(int tx_size, int rx_size, uint8_t cmd_response, int _timeout_s = 2, int _timeout_us = 0);

		/** \brief send certain byte(s) of request and save response to response buffer
		* \param[in] tx_size: bytes to send within request_
		* \param[in] rx_size: bytes should be responsed
		* \param[in] cmd_response: command ID
		* \param[in] _timeout_s: set time out time's second part
		* \param[in] _timeout_us: set time out time's less than 1 second part
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT or the bytes amount of received data
		* \note if response's length is not equal to rx_siez, this function will also return.
		*/
		virtual int tx_rx_uncertain_data(int tx_size, int rx_size, uint8_t cmd_response, int _timeout_s = 2, int _timeout_us = 0);

		/** \brief append tail to request buffer
		* \param[in] offset: request buffer length
		* \param[in] checksum: check sum of request
		* \return none
		*/
		void padding_tail(int offset, uint32_t checksum);

		/** \brief read LUT information form file.
		* \param[in] file: LUT file path
		* \return success or failed
		*/
		bool loadTable(std::string file);

		virtual void setRequestHeader(unsigned char *_request, unsigned char *_request_header, int len);
	};

	class BENEWAKE_API DCSPProtocolP4 : public DCSPProtocol
	{
	public:
		/** \brief DCSPProtocol constructor
		* \param[in] _send_ip: target device ip.
		* \param[in] _send_port: remote port where send data to.
		* \param[in] _recv_ip: data source ip which will be verified. When set to "", means any data received by _recv_port will be returned.
		* \param[in] _recv_port: local port to receive data.
		* \param[in] _udp_type: connection mode. NORMAL, MULTICAST or BOARDCAST.
		* \return none
		* \note
		*/
		DCSPProtocolP4(std::string _send_ip = "192.168.0.2", int _send_port = 2468,
			std::string _recv_ip = "192.168.0.2", int _recv_port = 2468, UDPType _udp_type = UDPType::NORMAL);

		~DCSPProtocolP4() {};

		/** \brief Clear work time
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_clear_work_time();

		/** \brief format file system
		* \param[in] none
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_format_file_system();

		/** \brief get enabled algorithm and their version
		* \param[out] _versions: algorithm and version
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_get_algorithm_version(std::vector<std::string>& _versions);

		/** \brief get the partition where the app is in
		* \param[out] _partition: 0 - A partition; 1 - B partition; 2 - boot loader
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_get_app_partition(uint8_t& _partition);

		/** \brief get clock config
		* \param[out] _1588_bmc: 1588 BMC enable flag
		* \param[out] _1588_domain_nu: 1588 domain number
		* \param[out] _gptp_bmc: gptp BMC enable flag
		* \param[out] _gptp_domain_nu: gptp domain number
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_get_clock_config(bool& _1588_bmc, uint8_t& _1588_domain_nu, bool& _gptp_bmc, uint8_t& _gptp_domain_nu);

		/** \brief Get DDR info
		* \param[out] _ddr_capacity: DDR capacity, unit 1024 Bytes
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_get_DDR_info(uint32_t& _ddr_capacity);

		/** \brief get dsop destination IP and port
		* \param[out] _ip: destination IP. If set to 255.255.255.255, dsop will be in boardcast mode.
		* \param[out] _port: destination port
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		int dcsp_get_dsop_dest_ip_and_port(std::string& _ip, uint32_t& _port);

		/** \brief get EEPROM status
		* \param[out] _status: 0 - OK; other - ERROR
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		int dcsp_get_EEPROM_status(uint8_t& _status);

		/** \brief Get flash information.
		* \param[out] _flash_id: flash ID
		* \param[out] _section_size: 1 section size
		* \param[out] _section_num: amount of sections
		* \param[out] _page_size: 1 page size
		* \param[out] _page_num: amount of pages
		* \param[out] _total_size: total size of flash
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		int dcsp_get_flash_info(uint32_t& _flash_id, uint32_t& _section_size, uint32_t& _section_num, uint32_t& _page_size,
			uint32_t& _page_num, uint32_t& _total_size);

		/** \brief get device MAC and VLAN
		* \param[out] _mac: MAC
		* \param[out] _vlan: vlan value
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		* \note
		*/
		int dcsp_get_mac_and_vlan(std::string& _mac, uint16_t& _vlan);

		/** \brief get NTP information from lidar
		* \param[out] _clock_source: current clock synchronization mode
		* @verbatim
		*			0 - no synchronous source
		*			1 - PTP(1588V2)
		*			2 - NTP
		*			3 - GPS + PPS
		*			4 - PPS
		*			5 - gPTP
		*			255 - meaningless value
		* @verbatim
		* \param[out] _clock_status: NTP status
		* @verbatim
		*			0 - NORMAL
		*			1 - NTP_NOT_ENABLE
		*			2 - NTP_SERVER_UNREACHABLE
		*			255 - meaningless value
		* @verbatim
		* \param[out] _server_ip: NTP server IP.
		* \param[out] _interval: polling interval in millisecond.
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_get_ntp_info(uint8_t& _clock_source, uint8_t& _clock_status, std::string& _server_ip, uint32_t& _interval);

		/** \brief get offline logger information
		* \param[out] _file_num: logger file amount
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_get_offline_logger_info(uint8_t& _file_num);

		/** \brief get offline logger parameters
		* \param[out] _log_level: set record logger level
		* @verbatim
		*			0 - MSG_EMERG
		*			1 - MSG_ALERT
		*			2 - MSG_CRIT
		*			3 - MSG_ERR
		*			4 - MSG_WARNING
		*			5 - MSG_NOTICE
		*			6 - MSG_INFO
		*			7 - MSG_DEBUG
		* @verbatim
		* \param[out] _file_size: max file size. range [1, 62], unit kb
		* \param[out] _file_num: max file amount. range [1, 127].
		* \param[out] _enable_authority: enable/disable authority.
		* \param[out] _enable_uart: enable/disable UART output.
		* \param[out] _enable_udp: enable/disable UDP output.
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_get_offline_logger_parameters(uint8_t& _log_level, uint8_t& _file_size, uint8_t& _file_num,
			bool& _enable_authority, bool& _enable_uart, bool& _enable_udp);

		/** \brief get ptp/gptp information from lidar
		* \param[out] _clock_source: current clock synchronization mode
		* @verbatim
		*			0 - no synchronous source
		*			1 - PTP(1588V2)
		*			2 - NTP
		*			3 - GPS + PPS
		*			4 - PPS
		*			5 - gPTP
		* @verbatim
		* \param[out] _clock_status: PTP or gPTP status
		* @verbatim
		*			0 - PTP_INITIALIZING
		*			1 - PTP_FAULTY
		*			2 - PTP_DISABLED
		*			3 - PTP_LISTENING
		*			4 - PTP_PRE_MASTER
		*			5 - PTP_MASTER
		*			6 - PTP_PASSIVE
		*			7 - PTP_UNCALIBRATED
		*			8 - PTP_SLAVE
		*			255 - meaningless value
		* @verbatim
		* \param[out] _offset_from_master_s: difference of second part between current local clock and master clock (in second
		* \param[out] _offset_from_master_ns: difference of nanosecond part between current local clock and master clock (in nanosecond)
		* \param[out] _offset_accumulated: accumulative difference of clock (in nanosecond)
		* \param[out] _path_delay_s: second part of mean path delay (in second)
		* \param[out] _path_delay_ns: nanosecond part of mean path delay (in nanosecond)
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		 int dcsp_get_ptp_info(uint8_t& _clock_source, uint8_t& _clock_status, int& _offset_from_master_s, int& _offset_from_master_ns,
			 int& _offset_accumulated, int& _path_delay_s, int& _path_delay_ns);

		 /** \brief get RC status
		 * \param[out] _enabled: enable or disable flag
		 * \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		 * \note
		 */
		 int dcsp_get_rc_mode(bool& _enabled);
		 
		 /** \brief Enable or disable anti-interference
		 * \param[in] _enable: enable/disable flag
		 * \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		 * \note
		 */
		 int dcsp_set_anti_interference(bool _enable);

		 /** \brief Set automobile information
		 * \param[in] _battery_voltage: voltage of vehicle battery
		 * \param[in] _speed: automobile speed
		 * \param[in] _time: absolute time
		 * \param[in] _mileage: mileage of vehicle
		 * \param[in] _outside_temperature: outside temperature
		 * \param[in] _altitude: altitude information
		 * \param[in] _rainfall: rainfall monitoring information
		 * \param[in] _vehicle_status: vehicle status
		 * \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		 */
		 int dcsp_set_automobile_info(int _battery_voltage, int _speed, int _time, int _mileage,
			 int _outside_temperature, int _altitude, int _rainfall, int _vehicle_status);

		 /** \brief set clock config
		 * \param[in] _1588_enable: 1588 BMC enable flag
		 * \param[in] _1588_domain_nu: 1588 domain number
		 * \param[in] _gptp_enable: gptp BMC enable flag
		 * \param[in] _gptp_domain_nu: gptp domain number
		 * \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		 * \note
		 */
		 int dcsp_set_clock_config(bool _1588_enable, uint8_t _1588_domain_nu, bool _gptp_enable, uint8_t _gptp_domain_nu);

		 /** \brief Set customer information
		 * \param[in] _custom_id: customer name.
		 * \param[in] _manufacture_date: manufacture date, should be 4 bytes long.
		 * @verbatim
		 *			0-1: Year
		 *			2:	 Month
		 *			3:	 Day
		 * @verbatim
		 * \param[in] _trace_code: trace code, should be 18 bytes long.
		 * \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		 * \note
		 */
		 int dcsp_set_customer_info(uint16_t _custom_id, char* _manufacture_date, char* _trace_code);

		 /** \brief set device ip and ports
		 * \param[in] _ip: device ip
		 * \param[in] _mdop_port: device mdop port
		 * \param[in] _dcsp_port: device dcsp port
		 * \param[in] _dsop_port: device dsop port
		 * \param[in] _mask: mask
		 * \param[in] _gateway: gateway
		 * \param[in] _use_config_mac: use configured MAC or use fix MAC
		 * \param[in] _mac: MAC configured
		 * \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		 */
		 int dcsp_set_device_ip_port_mac(std::string _ip = "192.168.0.2", uint32_t _mdop_port = 2468, uint32_t _dcsp_port = 2469,
			 uint32_t _dsop_port = 65000, std::string _mask = "255.255.255.0", std::string _gateway = "192.168.0.1",
			 bool _use_config_mac = true, std::string _mac = "36:7C:7C:00:00:16");

		 /** \brief get dsop destination IP and port
		 * \param[out] _ip: destination IP. If set to 255.255.255.255, dsop will be in boardcast mode.
		 * \param[out] _port: destination port
		 * \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		 * \note
		 */
		 int dcsp_set_dsop_dest_ip_and_port(std::string _ip, uint32_t _port);

		 /** \brief Set FOV ROI center's location.
		 * \param[in] _horizontal_location: horizontal angle of ROI center, in 0.1 degree.
		 * \param[in] _vertical_location: vertical angle of ROI center, in 0.1 degree.
		 * \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		 * \note
		 */
		 int dcsp_set_fov_roi_location(uint16_t _horizontal_location, uint16_t _vertical_location);

		 /** \brief Set FOV ROI area width and height.
		 * \param[in] _width: angle width of ROI area, in 0.1 degree.
		 * \param[in] _height: angle height of ROI area, in 0.1 degree.
		 * \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		 * \note
		 */
		 int dcsp_set_fov_roi_size(uint16_t _width, uint16_t _height);
		 
		 /** \brief Set NTP configuration.
		 * \param[in] _server_ip: NTP server IP.
		 * \param[in] _interval: polling interval in millisecond, valid value is [15000, 3600000].
		 * \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		 * \note
		 */
		 int dcsp_set_ntp_info(std::string _server_ip, uint32_t _interval);

		 /** \brief set offline logger parameters
		 * \param[in] _log_level: set record logger level
		 * @verbatim
		 *			0 - MSG_EMERG
		 *			1 - MSG_ALERT
		 *			2 - MSG_CRIT
		 *			3 - MSG_ERR
		 *			4 - MSG_WARNING
		 *			5 - MSG_NOTICE
		 *			6 - MSG_INFO
		 *			7 - MSG_DEBUG
		 * @verbatim
		 * \param[in] _file_size: max file size. range [1, 62], unit kb
		 * \param[in] _file_num: max file amount. range [1, 127].
		 * \param[in] _enable_authority: enable/disable authority.
		 * \param[in] _enable_uart: enable/disable UART output.
		 * \param[in] _enable_udp: enable/disable UDP output.
		 * \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		 */
		 int dcsp_set_offline_logger_parameters(uint8_t _log_level, uint8_t _file_size, uint8_t _file_num,
			 bool _enable_authority, bool _enable_uart, bool _enable_udp);

		 /** \brief set RC mode status
		 * \param[in] _enable: enable or disable RC mode
		 * \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		 * \note
		 */
		 int dcsp_set_rc_mode(bool _enable);

		 /** \brief Enable or disable static arp.
		 * \param[in] _enable: enable/disable flag. To enable static arp, set true; otherwise set false;
		 * \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		 */
		 int dcsp_set_static_arp(bool _enable);

		 /** \brief Enable or disable sunlight resistance.
		 * \param[in] _enable: enable/disable flag;
		 * \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		 */
		 int dcsp_set_sunlight_resistance(bool _enable);

		 /** \brief enable or disable vlan
		 * \param[in] _enable: true - enable vlan; false - disable vlan
		 * \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		 */
		 int dcsp_set_vlan_status(bool _enable);

		 /** \brief Enable or disable window heating.
		 * \param[in] _mode: enable/disable window heating and set heating mode;
		 * @verbatim
		 *			0x0 - shutdown
		 *			0x1 - manual control
		 *			0x2 - temperature control
		 *			0x4 - drewing control
		 *			0x6 - temperature + drewing control
		 *			0x8 - dirtying control
		 *			0xE - temperature + drewing + dirtying control
		 * @verbatim
		 * \param[in] _voltage: voltage, should be [20, 28];
		 * \param[in] _temp_a: control parameter tempA, should be [-40, 105];
		 * \param[in] _temp_b: control parameter tempB, should be [-40, 105];
		 * \param[in] _temp_c: control parameter tempC, should be [-40, 105];
		 * \param[in] _temp_d: control parameter tempD, should be [-40, 105];
		 * \param[in] _tx_enable: enable/disable TX heating flag;
		 * @verbatim
		 *			0x0 - disable
		 *			0x1 - enable
		 * @verbatim
		 * \param[in] _param_x: control parameter X, should be [-40, 105];
		 * \param[in] _param_x: control parameter Y, should be [-40, 105];
		 * \param[in] _param_x: control parameter Z, should be [-40, 105];
		 * \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		 */
		 int dcsp_set_window_heating(uint8_t _win_enable, uint16_t _voltage,
			 int16_t _temp_a, int16_t _temp_b, int16_t _temp_c, int16_t _temp_d,
			 uint8_t _tx_enable, int16_t _param_x, int16_t _param_y, int16_t _param_z);

		 /** \brief inform device that power off in several seconds
		 * \param[in] _delay_sec: time before power off, in second
		 * \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		 */
		 int dcsp_shutdown_warning(uint8_t _delay_sec);

		 /** \brief switch AB partition
		 * \param[in] none
		 * \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		 */
		 int dcsp_switch_partition();

		/** \brief download compensation tables
		* \param[in] _table_file: table file path
		* \param[in] _table_id: table ID which define the use of table
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_upload_lut(std::string _table_file, uint8_t _table_id);

		/** \brief upload offline logger from LiDAR
		* \param[in] _save_name: saving file path and name
		* \param[in] _file_id: file ID of logger
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT
		*/
		int dcsp_upload_offline_logger(std::string _save_name, uint8_t _file_id);

	protected:
		std::atomic<uint32_t> dcsp_count_;

		/** \brief send certain byte(s) of request and save response to response buffer
		* \param[in] _tx_size: bytes to send within request_
		* \param[in] _rx_size: bytes should be responsed
		* \param[in] _cmd_response: command ID
		* \param[in] _timeout_s: set time out time's second part
		* \param[in] _timeout_us: set time out time's less than 1 second part
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT or the bytes amount of received data
		* \note if response's length is not equal to rx_siez, this function will also return.
		*/
		int tx_rx_uncertain_data(int _tx_size, int _rx_size, uint8_t _cmd_response, int _timeout_s = 2, int _timeout_us = 0);

		/** \brief send certain byte(s) of request and save response to response buffer
		* \param[in] _tx_size: bytes to send within request_
		* \param[in] _rx_size: bytes should be responsed
		* \param[in] _cmd_response: command ID
		* \param[in] _response_status: command's response status
		* \param[in] _timeout_s: set time out time's second part
		* \param[in] _timeout_us: set time out time's less than 1 second part
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT or the bytes amount of received data
		* \note if response's length is not equal to rx_siez, this function will also return.
		*/
		int tx_rx_uncertain_data(int _tx_size, int _rx_size, uint8_t _cmd_response, uint16_t& _response_status, int _timeout_s = 2, int _timeout_us = 0);

		/** \brief send certain byte(s) of request and save response to response buffer
		* \param[in] tx_size: bytes to send within request_
		* \param[in] rx_size: bytes should be responsed
		* \param[in] cmd_response: command ID
		* \param[in] _timeout_s: set time out time's second part
		* \param[in] _timeout_us: set time out time's less than 1 second part
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT or the status of response
		* \note if not succeed to receive response, the process will automatically repeat 3 times.
		*/
		int tx_rx_data(int _tx_size, int _rx_size, uint8_t _cmd_response, int _timeout_s = 2, int _timeout_us = 0);

		/** \brief send certain byte(s) of request and check response is success or fail.
		* \param[in] tx_size: bytes to send within request_
		* \param[in] cmd_response: command ID
		* \param[in] _timeout_s: set time out time's second part
		* \param[in] _timeout_us: set time out time's less than 1 second part
		* \return STATUS_OK   STATUS_FAIL	STATUS_TIME_OUT or the status of response
		* \note if not succeed to receive response, the process will automatically repeat 3 times, so actual time out time is 3 * timeout.
		*/
		int tx_rx_check_status(int _tx_size, uint8_t _cmd_response, int _timeout_s = 2, int _timeout_us = 0);

		void setRequestHeader(unsigned char* _request, unsigned char* _request_header, int len);
	};

	class BENEWAKE_API DCSPProtocolG66 : public DCSPProtocolP4
	{
	public:
		/** \brief DCSPProtocol constructor
		* \param[in] _send_ip: target device ip.
		* \param[in] _send_port: remote port where send data to.
		* \param[in] _recv_ip: data source ip which will be verified. When set to "", means any data received by _recv_port will be returned.
		* \param[in] _recv_port: local port to receive data.
		* \param[in] _udp_type: connection mode. NORMAL, MULTICAST or BOARDCAST.
		* \return none
		* \note
		*/
		DCSPProtocolG66(std::string _send_ip = "192.168.0.2", int _send_port = 2468,
			std::string _recv_ip = "192.168.0.2", int _recv_port = 2468, UDPType _udp_type = UDPType::NORMAL);

		~DCSPProtocolG66() {};

	protected:
	};
}


#endif