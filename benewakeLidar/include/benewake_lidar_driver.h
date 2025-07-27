/**
* @file       benewake_lidar_driver.h

* @brief      Header of Benewake lidar's driver.

* @details 	  For those who want to develop their own program based on Benewake driver, this header
			  should be included in the codes. Corresponding Dynamic Link Library(.dll) and Object File
			  Library(.lib) also should be contained in the project.

* @author     TAN Wenquan

* @date       07/13/2022

* @version    v7.0.6

* @par Copyright (c):

*      Benewake (Beijing) Co., Ltd
*/
#ifndef BENEWAKE_LIDAR_DRIVER_H__
#define BENEWAKE_LIDAR_DRIVER_H__

#include <stdio.h>
#include <string>
#include <atomic>
#include <vector>
#include <deque>
#include <deque>
#include <stdio.h>
#include <numeric>
#include <ctime>

#include <iostream>
#include <string>
#include "benewake_mdop.h"
#include "benewake_dcsp.h"
#include "benewake_dsop.h"

#define DATASIZE 826

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus
	namespace benewake
	{
		/*
		* @brief Struct for timestamp. \n
		* BwTimestamp contains 2 parts which stand for timestamp's seconds part and nanoseconds part. Specially, when set timestamp output 
		* format as GPS&PPS, BwTimestamp's element \a time_s stand for time part greater than 1 hour in form of \b YYMMDDhhmm (for example, 
		* value 2009181048 stand for 2020-09-18's 10:48), and element \a time_ns stand for time part under 1 minute and unit is microsecond.
		*/
		struct BwTimestamp
		{
			uint32_t time_s = 0; ///< seconds part
			uint32_t time_ns = 0; ///< nanoseconds part
		};

		/**
		* @brief Enum for work mode.
		*/
		enum class BwWorkMode
		{
			DEFAULT = 0,
			HIGH_FPS_LONG_RANGE,
			LARGE_FOV_LONGE_RANGE,
			HIGH_RESOLUTION_LONG_RANGE,
			HIGH_RESOLUTION_SHORT_RANGE_SMALL_FOV,
			HIGH_RESOLUTION_SHORT_RANGE_LOW_FPS,
			DEFAULT_ADAPTATION,
			CUSTOM_1,
			CUSTOM_2,
			CUSTOM_3
		};

		/**
		* @brief Enum for clock source.
		*/
		enum class BwClockSource
		{
			INTERNAL_CLOCK = 0,
			PTP,
			NTP,
			GPS_PPS,
			PPS,
			gPTP
		};

		/**
		* @brief Find all available devices within current Network.
		* @param _device_list: LiDAR's information list.
		* @param _wait_milliseconds: Time to search devices.
		* @return True if LiDAR Network searched ; otherwais, false.
		*/
		BENEWAKE_API bool BW_FIND_AVAILABLE_DEVICES(std::vector<benewake::SYS_INFO> & _device_list, int _wait_milliseconds = 7000, int _search_port = 65000);

		/**
		* @brief Analyze system information struct and return status code.
		* @param _info: LiDAR's information struct.
		* @return System status code:
		* @verbatim
		*			BW_OK			- system ok
		*			BW_COMM_ERROR	- cannot communicate with system
		*			BW_SYS_ERROR	- system error occured, see SYS_INFO.sys_err for more detail
		*			BW_SYS_WARNING	- system warning
		*			BW_SYS_BUSY		- system is busy
		* @verbatim
		*/
		BENEWAKE_API int BW_GET_SYSTEM_STATUS_CODE(const benewake::SYS_INFO& _info);

		/**
		* @brief Class to enable Lidar and get pcl point cloud.
		*/
		class BENEWAKE_API BenewakeLidar
		{
		public:

			/**
			* @brief Constructor.
			* @param _remote_ip: Lidar's IP address, default is 192.168.0.2.
			* @param _remote_dcsp_port: Control commands port.
			* @param _type: Type to receive point cloud data, NORMAL or MULTICAST.
			* @param _local_ip: Local ip. Must be set when use driver on Linux with MULTICAST mode.
			* @param _local_dsop_port: Local port to receive heart beat. Default is 65000.
			*/
			BenewakeLidar(std::string _remote_ip = "192.168.0.2", uint32_t _remote_dcsp_port = 2469, UDPType _type = benewake::UDPType::NORMAL, 
				std::string _local_ip = "0.0.0.0", uint32_t _local_dsop_port = 65000);

			/**
			* @brief Constructor.
			* @param _info: Lidar's heart beat information.
			* @param _local_ip: Local ip. Must be set when use driver on Linux with MULTICAST mode.
			* @param _local_dsop_port: Local port to receive heart beat. Default is 65000.
			*/
			BenewakeLidar(benewake::SYS_INFO _info, std::string _local_ip = "0.0.0.0", uint32_t _local_dsop_port = 65000);

			~BenewakeLidar();

			/**
			* @brief Use local binded IP and port.
			* @param _local_ip: Local IP address.
			* @param _local_dcsp_port: Local port to receive dcsp data.
			* @param _local_mdop_port: Local port to receive mdop data.
			* @param _local_dsop_port: Local port to receive dsop data.
			* @param _remote_ip: Lidar IP address.
			* @param _remote_dcsp_port: lidar port to send dcsp data.
			* @param _remote_mdop_port: lidar port to send mdop data.
			* @param _remote_dsop_port: lidar port to sned dsop data.
			*/
			bool bindLocalAddress(std::string _local_ip, uint32_t _local_dcsp_port, uint32_t _local_mdop_port, uint32_t _local_dsop_port,
				std::string _remote_ip, uint32_t _remote_dcsp_port, uint32_t _remote_mdop_port, uint32_t _remote_dsop_port);

			/**
			* @brief Download firmware to device. It will take a period of time to check data before return operation result.
			* @param _firmware_file: Firmware file name.
			* @return True if succcess; otherwise, false.
			*/
			bool downloadFirmware(std::string _firmware_file);

			/**
			* @brief Clear the factory configurations and the user configurations.
			* @return True if succcess; otherwise, false.
			*/
			bool clearConfig();

			/**
			* @brief Enable or disable separation of frame. If enabled, each frame's rows is half of normal frame.
			* @param _enable: Enable flag. Default is disabled.
			* @return None.
			*/
			void enableABFrame(bool _enable);

			/**
			* @brief Enable or disable anti-interference function.
			* @param _enable: Enable flag.
			* @return True if success; otherwise, false.
			*/
			bool enableAntiInterference(bool _enable);
			
			/**
			* @brief Set point cloud dataforwarding enable/disable status.
			* @param _enable: Enable/disable status.
			* @param _forward_ip: Forward IP address.
			* @param _forward_port: Forward port.
			* @return True if success; otherwise, false.
			*/
			bool enableDataForwarding(bool _enable, std::string _forward_ip, int _forward_port);

			/**
			* @brief Enable or disable log recording
			* @param _enable: Enable flag. To enable log recording, set it to true; otherwise, false. Default is false.
			* @param _path: Path to store log files.
			* @return True if succcess; otherwise, false.
			*/
			bool enableLog(bool _enable, std::string _path = "log");

			/**
			* @brief Enable or disable static ARP.
			* @param _enable: Enable flag. To enable static ARP, set true; otherwise, set false.
			* @return True if success; otherwise, false.
			*/
			bool enablieStaticARP(bool _enable);

			/**
			* @brief Enable or disable sunlight resistance function.
			* @param _enable: Enable flag.
			* @return True if success; otherwise, false.
			*/
			bool enableSunlightResistance(bool _enable);

			/**
			* @brief Enable or disable time window mode. In this mode, frames are separated according to time instead of the position of components. 
			* @param _enable: Flag to enable or disable time window mode.
			* @param _interval: Interval of one frame.
			* @return True if scuccess; otherwise, false.
			*/
			bool enableTimeWindowMode(bool _enable, uint32_t _interval = 144);

			/**
			* @brief Enable or disable window heating function.
			* @param _win_heater: Window heater flag:
			* @verbatim
			*			0 - shutdown
			*			1 - manual control
			*			2 - temperature control
			*			4 - drewing control
			*			6 - temperature + drewing control
			*			8 - dirtying control
			*			14 - temperature + drewing + dirtying control
			* @verbatim
			* @param _voltage: Window heating voltage, value should be [20, 28].
			* @param _temp_a: Window heating parameter A, value should be [-40, 105].
			* @param _temp_b: Window heating parameter B, value should be [-40, 105].
			* @param _temp_c: Window heating parameter C, value should be [-40, 105].
			* @param _temp_d: Window heating parameter D, value should be [-40, 105].
			* @param _tx_heater: TX heater flag:
			* @verbatim
			*			0 - disable
			*			1 - enable
			* @verbatim
			* @param _param_x: TX heating parameter X, value should be [-40, 105].
			* @param _param_y: TX heating parameter Y, value should be [-40, 105].
			* @param _param_z: TX heating parameter Z, value should be [-40, 105].
			* @return True if success; otherwise, false.
			*/
			bool enableWindowHeating(int _win_heater, int _voltage, 
				int _temp_a, int _temp_b, int _temp_c, int _temp_d, 
				int _tx_heater, int _param_x, int _param_y, int _param_z);

			/**
			* @brief Get autorun enable/disable status.
			* @param _enable: Enable/disable status.
			* @return True if scuccess; otherwise, false.
			*/
			bool getAutorunStatus(bool& _enable);

			/**
			* @brief Get clock config BMC status.
			* @param _1588_enable: 1588 BMC enable/disable status.
			* @param _1588_domain_nu: 1588 domain number.
			* @param _gPTP_enable: gPTP BMC enable/disable status.
			* @param _gPTP_domain_nu: gPTP domain number.
			* @return True if scuccess; otherwise, false.
			*/
			bool getClockConfig(bool& _1588_enable, uint8_t& _1588_domain_nu, bool& _gPTP_enable, uint8_t& _gPTP_domain_nu);

			/**
			* @brief Get ip&port sittings.
			* @param _ip: Connection IP address.
			* @param _port: Connection control commands port.
			* @param _type: Data communication type.
			* @return True if success; otherwise, false.
			*/
			bool getConnection(std::string& _ip, uint32_t& _port, UDPType& _type);

			/**
			* @brief Get coordiate system of current data.
			* @param _coordinate: Coordinate system code:
			* @verbatim
			*			0 - Spherical coordinate
			*			1 - Cartesian coordinate
			* @verbatim
			* @return True if success; otherwise, false.
			*/
			bool getCoordiateSystem(uint8_t& _coordinate);

			/**
			* @brief Try to get a frame of point cloud.
			* @param _pointcloud: Pointer to point cloud data.
			* @param _frame_id: Point cloud frame ID. Range is from 0 to 65535, and will reset to 0 when overflow.
			* @param _timeout: Time to return if not get updated point cloud.
			* @return True if succcess; otherwise, false.
			*/
			bool getData(benewake::BwPointCloud::Ptr& _pointcloud, int& _frame_id, int _timeout = 2000);

			/**
			* @brief Try to get a frame of point cloud.
			* @param _pointcloud: Pointer to point cloud data.
			* @param _frame_id: Point cloud frame ID. Range is from 0 to 65535, and will reset to 0 when overflow.
			* @param _sys_info: Struct store LiDAR current status, including communication status, work status, error code, etc.
			* @param _timeout: Time to return if not get updated point cloud.
			* @return True if succcess; otherwise, false.
			*/
			bool getData(benewake::BwPointCloud::Ptr& _pointcloud, int& _frame_id, SYS_INFO& _sys_info, int _timeout = 2000);
			
			/**
			* @brief Try to get a frame of point cloud.
			* @param _pointcloud: Pointer to point cloud data.
			* @param _frame_id: Point cloud frame ID. Range is from 0 to 65535, and will reset to 0 when overflow.
			* @param _signal_message: Signal message set by setSignalMessage(), should be 4 bytes.
			* @param _signal_time_offset: Time since signal message was set, in millisecond.
			* @param _timeout: Time to return if not get updated point cloud.
			* @return True if succcess; otherwise, false.
			*/
			bool getData(benewake::BwPointCloud::Ptr& _pointcloud, int& _frame_id, char* _signal_message, uint16_t& _signal_time_offset, int _timeout = 2000);
			
			/**
			* @brief Try to get data transmission state.
			* @param _delay_time: The interval between the moment of generating a point and the moment this SDK receiving the same point, in microsecond.
			* @param _pkg_loss_rate: Loss rate of UDP packages during last 10 seconds, in %.
			* @param _frame_loss_rate: Loss rate of frames during last 10 seconds, in %.
			* @return None.
			*/
			void getDataTransmissionState(uint32_t& _delay_time, float& _pkg_loss_rate, float& _frame_loss_rate);

			/**
			* @brief Get DCSP pointer.
			* @return Return DCSP pointer if dcsp protocol is initialized; otherwise return a NULL pointer.
			*/
			std::shared_ptr<benewake::DCSPProtocol> getDCSP();

			/**
			* @brief Try to get a LiDAR device information.
			* @param _version: Version of LiDAR firmware. 
			* @param _fpga_version: Version of LiDAR FPGA. 
			* @param _total_points_num: Total points amount of a frame (simgle channel).
			* @param _line_points_num: Points amount of a line (simgle channel).
			* @param _channel_num: Channel amount.
			* @param _sn: SN.
			* @return True if scuccess; otherwise, false.
			*/
			bool getDeviceInformation(std::string& _version, std::string& _fpga_version,
				int& _total_points_num, int& _line_points_num, int& _channel_num, std::string& _sn);

			/**
			* @brief Get DHCP enable/disable status.
			* @param _enable: Enable/disable status.
			* @return True if scuccess; otherwise, false.
			*/
			bool getDHCPStatus(bool& _enable);

			/**
			* @brief Get DSOP pointer.
			* @return Return MDOP pointer if dcsp protocol is initialized; otherwise return a NULL pointer.
			*/
			std::shared_ptr<benewake::DSOPProtocol> getDSOP();

			/**
			* @brief Get LiDAR DSOP destination IP and port.
			* @param _dsop_ip: Destination IP where LiDAR set DSOP to. 255.255.255.255 means in boardcast mode. 
			* @param _dsop_port: Destination port where LiDAR set DSOP to.
			* @return True if scuccess; otherwise, false.
			*/
			bool getDSOPDestIPAndPort(std::string& _dsop_ip, uint32_t& _dsop_port);

			/**
			* @brief Get dsp version.
			* @param _app_version: APP's version
			* @param _boot_loader_version: Boot loader's version
			* @return True if scuccess; Otherwise, false.
			*/
			bool getDSPVersion(std::string& _app_version, std::string& _boot_loader_version);

			/** 
			* @brief Get extrinsic parameters of
			*	| a11 a12 a13 x |
			*	| a21 a22 a23 y |
			*	| a31 a32 a33 z |
			* @param _enable: Enable transformation or not
			* @param _params: Parameters in order {a11, a12, a13, a21, a22, a23, a31, a32, a33, x, y, z}
			* @return True if scuccess; otherwise, false.
			*/
			bool getExtrinsicParameters(bool& _enable, std::vector<float>& _params);

			/**
			* @brief Get front view filter parameters.
			* @param _status: Enable or disable status.
			* @param _rows: Rows corrspond to scanning.
			* @param _cols: Cols corrspond to scanning.
			* @param _bind: Cell size.
			* @param _n_scan: Scanning times. Scanning down and then up is counted to 2.
			* @param _h_resolution: Horizontal resolution.
			* @param _v_resolution: Vertical resolution.
			* @return True if scuccess; otherwise, false.
			*/
			bool getFrontViewFilterParameters(bool& _status, int& _rows, int& _cols, int& _bind, int& _n_scan, 
				float& _h_resolution, float& _v_resolution);

			/**
			* @brief Get gaze status.
			* @param _enable: Enable status.
			* @return True if scuccess; otherwise, false.
			*/
			bool getGaze(bool& _enable);

			/**
			* @brief Get MDOP pointer.
			* @return Return MDOP pointer if dcsp protocol is initialized; otherwise return a NULL pointer.
			*/
			std::shared_ptr<benewake::MDOPProtocol> getMDOP();

			/**
			* @brief Get multicast IP and port.
			* @param _ip: Multicast group IP.
			* @param _port: Multicast port.
			* @return True if scuccess; otherwise, false.
			*/
			bool getMulticastIPAndPort(std::string& _ip, uint32_t& _port);

			/**
			* @brief Get multicast enable/disable status.
			* @param _enable: Enable/disable status.
			* @return True if scuccess; otherwise, false.
			*/
			bool getMulticastStatus(bool& _enable);

			/**
			* @brief Get multi echo's mode.
			* @param _mode: Multi echo mode:
			* @verbatim
			*			0 - First echo
			*			1 - Strongest echo
			*			2 - Last echo
			*			3 - Last and strongest echos
			*			4 - First and last echos
			*			5 - First and strongest echos
			* @verbatim
			* @return True if scuccess; otherwise, false.
			*/
			bool getMultiEchoMode(int& _mode);

			/**
			* @brief Get network timeout time.
			* @param _time: Timeout time, in second.
			* @return True if scuccess; otherwise, false.
			*/
			bool getNetworkTimeout(int& _time);

			/**
			* @brief Get NTP information from lidar.
			* @param _clock_source: Current clock synchronization mode
			* @verbatim
			*			0 - no synchronous source
			*			1 - PTP(1588V2)
			*			2 - NTP
			*			3 - GPS + PPS
			*			4 - PPS
			*			5 - gPTP
			*			255 - meaningless value
			* @verbatim
			* @param _clock_status:NTP status
			* @verbatim
			*			0 - NORMAL
			*			1 - NTP_NOT_ENABLE
			*			2 - NTP_SERVER_UNREACHABLE
			*			255 - meaningless value
			* @verbatim
			* @param _server_ip: NTP server IP.
			* @param _interval: Polling interval in millisecond.
			* @return True if scuccess; otherwise, false.
			*/
			bool getNTPInfo(uint8_t& _clock_source, uint8_t& _clock_status, std::string& _server_ip, uint32_t& _interval);

			/**
			* @brief Get offline logger information
			* @param _file_num: Logger file amount
			* @return True if scuccess; otherwise, false.
			*/
			bool getOfflineLoggerInfo(int& _file_num);

			/**
			* @brief Get offline logger parameters.
			* @param _log_level: Set recorded logger level.
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
			* @param _file_size: Max logger file size, range is [1, 62] in kB.
			* @param _file_num: Max file amount, range is [1, 127].
			* @param _enable_authority: Enable/disable authority.
			* @param _enable_uart: Enable/disable UART output.
			* @param _enable_udp: Enable/disable UDP output.
			* @return True if scuccess; otherwise, false.
			*/
			bool getOfflineLoggerParameters(int& _log_level, int& _file_size, int& _file_num,
				bool& _enable_authority, bool& _enable_uart, bool& _enable_udp);

			/**
			* @brief Get lidar's laser power value.
			* @param _power: Power value of current work mode, in mW. The value should be between 0 and 1000;
			* @return True if scuccess; otherwise, false.
			*/
			bool getPower(int& _power);

			/** 
			* @brief Get PTP/gPTP information from lidar
			* @param _clock_source: Current clock synchronization mode.
			* @verbatim
			*			0 - no synchronous source
			*			1 - PTP(1588V2)
			*			2 - NTP
			*			3 - GPS + PPS
			*			4 - PPS
			*			5 - gPTP
			* @verbatim
			* @param _clock_status: PTP or gPTP status.
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
			* @param[out] _offset_from_master_s: Difference of second part between current local clock and master clock (in second).
			* @param[out] _offset_from_master_ns: Difference of nanosecond part between current local clock and master clock (in nanosecond).
			* @param[out] _offset_accumulated: Accumulative difference of clock (in nanosecond).
			* @param[out] _path_delay_s: Second part of mean path delay (in second).
			* @param[out] _path_delay_ns: Nanosecond part of mean path delay (in nanosecond).
			* @return True if scuccess; otherwise, false.
			*/
			bool getPTPInfo(uint8_t& _clock_source, uint8_t& _clock_status, int& _offset_from_master_s, int& _offset_from_master_ns,
				int& _offset_accumulated, int& _path_delay_s, int& _path_delay_ns);

			/**
			* @brief Get RC mode enable/disable status.
			* @param _enable: Enable/disable status.
			* @return True if scuccess; otherwise, false.
			*/
			bool getRCMode(bool& _enable);

			/**
			* @brief Get output timestamp synchronization source.
			* @param _source: Time synchronization source:
			* @verbatim
			*			0 - no synchronous source
			*			1 - PTP(1588V2)
			*			2 - NTP
			*			3 - GPS + PPS
			*			4 - PPS
			*			5 - gPTP
			* @verbatim
			* @return True if scuccess; otherwise, false.
			*/
			bool getTimestampFormat(BwClockSource & _source);

			bool getTimestampFormat(int& _source);

			/**
			* @brief Get lidar's work mode.
			* @param _mode: Work mode.
			* @param _timeout_s: Timeout period's second part.
			* @param _timeout_us: Timeout period's micro-second part.
			* @return True if success; otherwise, false.
			*/
			bool getWorkMode(BwWorkMode& _mode, int _timeout_s = 2, int _timeout_us = 0);

			/**
			* @brief Get lidar's work mode.
			* @param _mode: Work mode.
			* @param _timeout_s: Timeout period's second part.
			* @param _timeout_us: Timeout period's micro-second part.
			* @return True if success; otherwise, false.
			*/
			bool getWorkMode(int& _mode, int _timeout_s = 2, int _timeout_us = 0);

			/**
			* @brief get ROI
			* @param _x_max: Maximum X-axis		Unit: cm
			* @param _x_min: Minimum X-axis		Unit: cm
			* @param _y_max: Maximum Y-axis		Unit: cm
			* @param _y_min: Minimum Y-axis		Unit: cm
			* @return True if success; otherwise, false.
			*/
			bool getROI(int& _x_max, int& _x_min, int& _y_max, int& _y_min);

			/**
			* @brief Get the ROI function Enable/disable status.
			* @param _enable: Enable/disable status.
			* @return True if success; otherwise, false.
			*/
			bool getROIStatus(bool& _enable);

			/**
			* @brief Read continual register(s) setting(s).
			* @param _value: Pointer to array that store data.
			* @param _address: The first register's address to be read.
			* @return True if success; otherwise, false.
			*/
			bool readPLRegister(int32_t* _value, uint32_t _address);

			/**
			* @brief Read continual register(s) setting(s).
			* @param _value: Pointer to array that store data. The array's length must be equal or longer than _size.
			* @param _address: The first register's address to be read.
			* @param _size: Amount of register(s) to be read.
			* @return True if success; otherwise, false.
			*/
			bool readRegister(int32_t* _value, uint32_t _address, int _size);

			bool readRegister(uint32_t* _value, uint32_t _address, int _size);

			/**
			* @brief Regist heart beat callback.
			* @param _func: Callback function. Format benewake::HeartBeatCallbcakFunc.
			* @param _pData: Pointer to pass data between callback function and main thread.
			* @return None.
			*/
			void registHeartBeatCallback(HeartBeatCallbcakFunc _func, void* _pData);

			/**
			* @brief cancel registed heart beat callback.
			* @return None.
			*/
			void cancelHeartBeatCallback();

			/**
			* @brief Regist point cloud data ready callback.
			* @param _func: Callback function. Format benewake::PointCloudCallbackFunc.
			* @param _pData: Pointer to pass data between callback function and main thread.
			* @return None.
			*/
			void registPointCloudCallback(PointCloudCallbackFunc _func, void* _pData);

			/**
			* @brief Reboot device.
			* @return True if scuccess; otherwise, false. Device will restart after response returned.
			*/
			bool reboot();

			/**
			* @brief Restore settings to default, including work parameters, SN, etc.
			* @return True if scuccess; otherwise, false.
			*/
			bool restoreToDefault();

			/**
			* @brief Search LiDAR, whose IP is same with class' setting, within current Network.
			* @param _info: LiDAR's information.
			* @param _wait_milliseconds: Time to search devices, in millisecond.
			* @return True if scuccess; otherwise, false.
			*/
			bool searchDevice(SYS_INFO& _info, int _wait_milliseconds = 7000);

			/**
			* @brief Inform LiDAR the automobile status.
			* @param _battery_voltage: Voltage of automobile's battery .
			* @param _speed: Current speed.
			* @param _time: Absolute time.
			* @param _mileage: Mileage of vehicle.
			* @param _outside_temperature: Outside temperature.
			* @param _altitude: Altitude information.
			* @param _rainfall: Rainfall monitoring information.
			* @param _vehicle_status: Vehicle status.
			* @return True if scuccess; otherwise, false.
			*/
			bool setAutomobileInformation(int _battery_voltage, int _speed, int _time, int _mileage,
				int _outside_temperature, int _altitude, int _rainfall, int _vehicle_status);

			/**
			* @brief Set autorun enable/disable status.
			* @param _enable: Enable/disable status.
			* @return True if scuccess; otherwise, false.
			*/
			bool setAutorunStatus(bool _enable);

			/**
			* @brief Set clock config BMC enable/disable status.
			* @param _1588_enable: 1588 BMC enable/disable status.
			* @param _1588_domain_nu: 1588 domain number.
			* @param _gPTP_enable: gPTP BMC enable/disable status.
			* @param _gPTP_domain_nu: gPTP domain number.
			* @return True if scuccess; otherwise, false.
			*/
			bool setClockConfig(bool _1588_enable, uint8_t _1588_domain_nu, bool _gPTP_enable, uint8_t _gPTP_domain_nu);

			/**
			* @brief Set connection IP&port.
			* @param _ip: Connection IP address.
			* @param _port: Connection control commands port.
			* @param _type: Data communication type.
			* @return True if success; otherwise, false.
			*/
			bool setConnection(std::string _ip, uint32_t _port, UDPType _type = UDPType::NORMAL);

			/**
			* @brief Set coordiate system of current data.
			* @param _coordinate: Coordinate system code:
			* @verbatim
			*			0 - Spherical coordinate
			*			1 - Cartesian coordinate
			* @verbatim
			* @return True if success; otherwise, false.
			*/
			bool setCoordiateSystem(uint8_t _coordinate);

			/** 
			* @brief Set the destination address that device communicate with
			* @param[in] _ip: Destination ip.
			* @param[in] _mdop_port: Destination mdop port.
			* @param[in] _dcsp_port: Destination dcsp port.
			* @param[in] _dsop_port: Destination dsop port.
			* @param[in] _mask: Destination mask.
			* @param[in] _gateway: Destination gateway.
			* \param[in] _mac: MAC address, should be like "1A:2B:3C:4D:5E:6F"
			* @param[in] _vlan: Destination vlan.
			* @return True if success; otherwise, false.
			*/
			bool setDestinationIPAndPort(std::string _ip, uint32_t _mdop_port, uint32_t _dcsp_port, uint32_t _dsop_port, 
				std::string _mask, std::string _gateway, std::string _mac, uint16_t _vlan);

			/**
			* @brief Set LiDAR IP&ports sittings.
			* @param _ip: LiDAR's IP address.
			* @param _mdop_port: LiDAR's point cloud data sending port.
			* @param _dcsp_port: LiDAR's control commands receiving&response port.
			* @param _mask: LiDAR's mask.
			* @param _gateway: LiDAR's gateway.
			* @return True if success; otherwise, false.
			* @note This is not suitable to LidAR AD2-S. AD2-S must set MAC.
			*/
			bool setDeviceIPAndPort(std::string _ip, uint32_t _mdop_port, uint32_t _dcsp_port,
				std::string _mask, std::string _gateway);

			/**
			* @brief Set LiDAR IP&ports sittings.
			* @param _ip: LiDAR's IP address.
			* @param _mdop_port: LiDAR's point cloud data sending port.
			* @param _dcsp_port: LiDAR's control commands receiving&response port.
			* @param _mask: LiDAR's mask.
			* @param _gateway: LiDAR's gateway.
			* @param _use_fix_mac: Use fix MAC configured or automatically generated MAC.
			* @param _mac: MAC, in format like "36:7C:7C:00:00:16".
			* @return True if success; otherwise, false.
			* @note This is only used to LidAR AD2-S.
			*/
			bool setDeviceIPAndPort(std::string _ip, uint32_t _mdop_port, uint32_t _dcsp_port,
				std::string _mask, std::string _gateway, bool _use_fix_mac, std::string _mac);

			/**
			* @brief Set SN code.
			* @param _SN: SN code, 16 bytes.
			* @return True if scuccess; otherwise, false.
			*/
			bool setDeviceSN(const char* _SN);

			/**
			* @brief Set DHCP enable/disable status.
			* @param _enable: Enable/disable status.
			* @return True if scuccess; otherwise, false.
			*/
			bool setDHCPStatus(bool _enable);

			/**
			* @brief Set LiDAR DSOP destination IP and port.
			* @param _dsop_ip: Destination IP where LiDAR set DSOP to. 255.255.255.255 means in boardcast mode.
			* @param _dsop_port: Destination port where LiDAR set DSOP to.
			* @return True if scuccess; otherwise, false.
			*/
			bool setDSOPDestIPAndPort(std::string _dsop_ip, uint32_t _dsop_port);

			/**
			* @brief Set extrinsic parameters of
			*	| a11 a12 a13 x |
			*	| a21 a22 a23 y |
			*	| a31 a32 a33 z |
			* @param _enable: enable transformation or not
			* @param _params: parameters in order {a11, a12, a13, a21, a22, a23, a31, a32, a33, x, y, z}
			* @return True if scuccess; otherwise, false.
			*/
			bool setExtrinsicParameters(bool _enable, const std::vector<float>& _params);

			/**
			* @brief Set location of FOV ROI center. FOV ROI is the ROI defined by the angle to FOV's center.
			* @param _horizontal: Horizontal angle of center in degree, resolution is 0.1 degree and value should be [-60.0, 60.0].
			* @param _vertical: Vertical angle of center in degree, resolution is 0.1 degree and value should be [-12.8, 12.8].
			* @return True if scuccess; otherwise, false.
			*/
			bool setFOVROILocation(float _horizontal, float _vertical);

			/**
			* @brief Set size of FOV ROI. The size is defined by the angle from one edge of ROI area to the opposite edge.
			* @param _width: Angle of ROI width in degree, resolution is 0.1 degree and value should be [10.0, 120.0].
			* @param _height: Angle of ROI height in degree, resolution is 0.1 degree and value should be [1.0, 25.6].
			* @return True if scuccess; otherwise, false.
			*/
			bool setFOVROISize(float _width, float _height);

			/**
			* @brief Front view filter for filtering noise caused by misty and rain.
			* @param _status: Enable or disable filter.
			* @param _rows: Rows corrspond to scanning.
			* @param _cols: Cols corrspond to scanning.
			* @param _bind: Cell size.
			* @param _n_scan: Scanning times. Scanning down and then up is counted to 2.
			* @param _h_resolution: Horizontal resolution.
			* @param _v_resolution: Vertical resolution.
			* @return None.
			*/
			bool setFrontViewFilter(bool _status, int _rows = 1100, int _cols = 2000, int _bind = 10, int _n_scan = 4,
				float _h_resolution = 0.045, float _v_resolution = 0.026);

			/**
			* @brief Set gaze status.
			* @param _enable: Enable status.
			* @return True if scuccess; otherwise, false.
			*/
			bool setGaze(bool _enable);

			/**
			* @brief Set point cloud transform, which is calculate in this SDK.
			* @param _enable: Enable status.
			* @param _roll: Roll angle in radian.
			* @param _pitch: Pitch angle in radian.
			* @param _yaw: Yaw angle in radian.
			* @param _x: Translation along X-axis in meter.
			* @param _y: Translation along Y-axis in meter.
			* @param _z: Translation along Z-axis in meter.
			* @return True if scuccess; otherwise, false.
			*/
			bool setLocalTransform(bool _enable, float _roll, float _pitch, float _yaw, float _x, float _y, float _z);

			/**
			* @brief Set multicast's IP and port.
			* @param _ip: Multicast's IP.
			* @param _port: Multicast's port.
			* @return True if scuccess; otherwise, false.
			*/
			bool setMulticastIPAndPort(std::string _ip, uint32_t _port);

			/**
			* @brief Set multicast enable/disable status.
			* @param _enable: Enable/disable status.
			* @return True if scuccess; otherwise, false.
			*/
			bool setMulticastStatus(bool _enable);

			/**
			* @brief Set multi echo's mode.
			* @param _mode: Multi echo mode:
			* @verbatim
			*			0 - First echo
			*			1 - Strongest echo
			*			2 - Last echo
			*			3 - Last and strongest echos
			*			4 - First and last echos
			*			5 - First and strongest echos
			* @verbatim
			* @return True if scuccess; otherwise, false.
			*/
			bool setMultiEchoMode(int _mode);

			/**
			* @brief Set network timeout time.
			* @param _time: Timeout time, in second.
			* @return True if scuccess; otherwise, false.
			*/
			bool setNetworkTimeout(int _time);

			/**
			* @brief Set NTP Configuration.
			* @param _server_ip: NTP server IP.
			* @param _interval: Polling interval in millisecond, valid value is [15000, 3600000].
			* @return True if scuccess; otherwise, false.
			*/
			bool setNTPParameters(std::string _server_ip, uint32_t _interval);

			/**
			* @brief Set offline logger parameters.
			* @param _log_level: Set recorded logger level.
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
			* @param _file_size: Max logger file size, range is [1, 62] in kB.
			* @param _file_num: Max file amount, range is [1, 127].
			* @param _enable_authority: Enable/disable authority.
			* @param _enable_uart: Enable/disable UART output.
			* @param _enable_udp: Enable/disable UDP output.
			* @return True if scuccess; otherwise, false.
			*/
			bool setOfflineLoggerParameters(int _log_level, int _file_size, int _file_num,
				bool _enable_authority, bool _enable_uart, bool _enable_udp);

			/**
			* @brief Set lidar's laser power value.
			* @param _power: value in mW, value should be between 0 and 1000.
			* @return True if scuccess; otherwise, false.
			*/
			bool setPower(int _power);

			/**
			* @brief Set RC mode enable/disable status.
			* @param _enable: Enable/disable status.
			* @return True if scuccess; otherwise, false.
			*/
			bool setRCMode(bool _enable);

			/**
			* @brief Set polygon ROI
			* @param _enable: Enable ROI or not
			* @param _vertices: Vertices of ROI, in order {p1.x, p1.y, p2.x, p2.y, ..., pn.x, pn.y}, unit meter. Vertices cannot be less than 3.
			* @return True if success; otherwise, false.
			*/
			bool setROI(bool _enable, const std::vector<float>& _vertices);

			/**
			* @brief Set rectangle ROI
			* @param _x_max: Maximum X-axis		Unit: cm
			* @param _x_min: Minimum X-axis		Unit: cm
			* @param _y_max: Maximum Y-axis		Unit: cm
			* @param _y_min: Minimum Y-axis		Unit: cm
			* @return True if success; otherwise, false.
			*/
			bool setROI(int _x_max, int _x_min, int _y_max, int _y_min);

			/**
			* @brief Set the rectangle ROI function Enable/disable status.
			* @param _enable:  Enable/disable status.
			* @return True if success; otherwise, false.
			*/
			bool setROIStatus(bool _enable);

			/**
			* @brief Set signal message attached to MDOP packages.
			* @param _msg:  Signal message. The length must be 4 bytes.
			* @return True if success; otherwise, false.
			*/
			bool setSignalMessage(char* _msg);

			/**
			* @brief Set output timestamp synchronization source.
			* @param _source: Time synchronization source:
			* @verbatim
			*			0 - no synchronous source
			*			1 - PTP(1588V2)
			*			2 - NTP
			*			3 - GPS + PPS
			*			4 - PPS
			*			5 - gPTP
			* @verbatim
			* @return True if scuccess; otherwise, false.
			*/
			bool setTimestampFormat(BwClockSource _source);
			
			bool setTimestampFormat(int _source);

			/**
			* @brief Set lidar's work mode.
			* @param _mode: Work mode.
			* @return True if success; otherwise, false.
			*/
			bool setWorkMode(BwWorkMode _mode);

			/**
			* @brief Set lidar's work mode.
			* @param _mode: Work mode.
			* @return True if success; otherwise, false.
			*/
			bool setWorkMode(int _mode);

			/**
			* @brief Inform LiDAR of power off in several seconds.
			* @param _delay_sec: Time before power off, in second.
			* @return True if success; otherwise, false.
			*/
			bool shutdownWarning(int _delay_sec);

			/**
			* @brief Enable LiDAR's work.
			* @return True if LiDAR successfully start work; otherwise, false.
			*/
			bool start();

			/**
			* @brief Enable LiDAR's work. Only receiver is enabled, command and heart beat functions are disable, so that some parameters and status must be set.
			* @param[in] _ip: local receiver IP address.
			* @param[in] _port: local receiver port.
			* @param[in] _device_type: device type.
			* @verbatim
			*			0x00	- PRODUCT_ID_X2
			*			0x01	- PRODUCT_ID_P4
			* @verbatim
			* @param[in] _protocol_version: protocol version for data checking.
			* @verbatim
			*			0x00	- check sum
			*			0x01	- check CRC32
			*			0x02	- check CRC32_SB8_MODE_BEGIN
			* @verbatim
			* @param[in] _long_range_version: is long range version Horn X2 LiDAR or not.
			* @param[in] _enable_AB_frame: enable AB frame or not.
			* @param[in] _row_num: scan rows. Must be set correctly if _enable_AB_frame is enabled.
			* @return True if LiDAR successfully start work; otherwise, false.
			*/
			bool startOnlyReceiver(std::string _ip, int _port, uint8_t _device_type, uint16_t _protocol_version, bool _long_range_version, bool _enable_AB_frame, int _row_num);

			/**
			* @brief Disable lidar.
			* @return True if processings are stopped and commands have been send to lidar; otherwise, false.
			*/
			bool stop();

			/**
			* @brief switch mdop coordinate protocol.
			* @return True if success; otherwise, false.
			*/
			bool switchProtocol(BwProtocolType _protocol);

			/**
			* @brief Get earliest frame from time window data queue. The maximum size of the data queue is 5, so if the queue is full, earliest data will be lost when new data come in.
			* If lidar is not working in time window mode, empty data would always be return.
			* @return Pointer of earliest frame of data queue.
			*/
			benewake::BwPointCloud::Ptr timeWindowDataQueueFront();

			/**
			* @brief Pop out earliest frame from time window data queue.
			*/
			void timeWindowDataQueuePop();

			/**
			* @brief Get and pop out earliest frame from time window data queue. The maximum size of the data queue is 5, so if the queue is full, earliest data will be lost when new data come in.
			* If lidar is not working in time window mode, empty data would always be return.
			* @return Pointer of earliest frame of data queue.
			*/
			benewake::BwPointCloud::Ptr timeWindowDataQueuePopFront();

			/**
			* @brief Get size of time window data queue.
			* @return Size of the data queue.
			*/
			int timeWindowDataQueueSize();

			/**
			* @brief Store lidar parameters changes to flash.
			* @return True if success; otherwise, false.
			*/
			bool updateFlash();

			/** 
			* @brief Upload offline logger from LiDAR.
			* @param _save_name: Saving file name.
			* @param _file_id: File ID of logger.
			* @return True if success; otherwise, false.
			*/
			bool uploadOfflineLogger(std::string _save_name, int _file_id);

			/**
			* @brief Write continual register(s) setting(s).
			* @param _value: Pointer to array that store data.
			* @param _address: The first register's address to be read.
			* @return True if success; otherwise, false.
			*/
			bool writePLRegister(int32_t* _value, uint32_t _address);

			/**
			* @brief Write continual register(s) setting(s).
			* @param _value: Pointer to array that store the data to be written. The array's length must be equal or longer than _size.
			* @param _address: The first register's address to be written.
			* @param _size: Amount of register(s) to be written.
			* @return True if success; otherwise, false.
			*/
			bool writeRegister(int32_t* _value, uint32_t _address, int _size);

			bool writeRegister(uint32_t* _value, uint32_t _address, int _size);

		private:
			std::shared_ptr<DCSPProtocol> dcsp_ = NULL;
			std::shared_ptr<DSOPProtocol> dsop_ = NULL;
			std::shared_ptr<MDOPProtocol> mdop_ = NULL;

			// connection
			std::string ip_;
			std::string multicast_ip_ = "224.0.0.2";
			std::string local_ip_ = "0.0.0.0";
			uint32_t dcsp_port_ = 2469;
			uint32_t mdop_port_ = 2468;
			uint32_t dsop_port_ = 65000;
			uint32_t local_dcsp_port_ = 2469;
			uint32_t local_mdop_port_ = 2468;
			uint32_t local_dsop_port_ = 65000;
			UDPType connect_type_ = UDPType::NORMAL;
			bool isStarted_ = false;

			SYS_INFO info_transport_dsop_dcsp_;
			SYS_INFO device_info_;

			// log
			bool log_enable_ = false;
			std::string log_path_;

			// front view filter
			bool front_view_filter_enabled_ = false;

			// AB frame
			bool enable_AB_frame_ = false;

			void splitString(const std::string& s, std::vector<std::string>& v, const std::string& c);

			bool checkDCSPProtocol();

			bool checkMDOPProtocol();
		};
	}

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif // BENEWAKE_LIDAR_H
